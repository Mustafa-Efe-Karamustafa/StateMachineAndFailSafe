#!/usr/bin/env python3
# =============================================================================
# DOSYA : motion_node.py
# AMAÇ  : ROS 2 düğümü. Odometriyi okur, 20 Hz'de PlanarController'ı çalıştırır, çıkan kuvvet/momenti
#         allocation.py ile 6 iticiye dağıtıp Gazebo'ya yazar. Görev bitince yol/derinlik grafiğini kaydeder.
#   mission: line | square | circle | manual   (manual: /auv/cmd_wrench Wrench mesajıyla elle sür)
# AKIŞ (her 50 ms):  odom → [x,y,ψ,z] → Supervisor.step (durum + arıza) → controller.update / FAILSAFE eylemi
#                    → allocate → 6× cmd_thrust yayını
#   Ayrıca duvar saatiyle (STEADY_TIME) çalışan watchdog: /clock durursa (köprü öldü) kontrol döngüsü de durur;
#   watchdog yine çalışır, FAILSAFE'e geçirir ve itki 0 yayınlar (Q6).
# KONULAR: /auv/state (std_msgs/String, mevcut durum)   /auv/failsafe_reset (std_msgs/Empty, operatör onayı)
# MATEMATİK ÖZETİ:
#   dt = t_k - t_{k-1} (simülasyon saati). Yaw: ψ = atan2(2(wz + xy), 1 - 2(y² + z²))  (kuaterniyondan).
#   Roll φ = atan2(2(wx + yz), 1 - 2(x² + y²)),  pitch θ = asin(clamp(2(wy - zx), -1, 1))  (Q8).
#   İtki komutu: F_cmd,i = thrust_sign · F_i   (thrust_sign = ±1: itici montaj yönü düzeltmesi)
# =============================================================================
import csv
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, String

from .allocation import allocate
from .controller import PlanarController
from .state_machine import State, Supervisor

# İtici eklem adları: sıra allocate() çıktısıyla aynı olmalı [FR, FL, RL, RR, VL, VR].
THRUSTERS = ['thruster_fr', 'thruster_fl', 'thruster_rl', 'thruster_rr',
             'thruster_vl', 'thruster_vr']
VALID_MISSIONS = ('line', 'square', 'circle', 'manual')
# Test için arıza enjeksiyonu (parametre inject_fault, çalışırken `ros2 param set` ile değiştirilebilir).
VALID_INJECT = ('none', 'nan_odom', 'stale_odom', 'tilt')

# Varsayılan parametreler. gains.yaml aynı adlarla bunların üzerine yazar.
# DİKKAT: ROS parametre tipi sabittir; ondalık parametreler yaml'da 0 değil 0.0 yazılmalıdır.
DEFAULTS = dict(
    mission='line', start_delay=3.0, thrust_sign=1.0, max_thrust=20.0, model_name='auv',
    line_length=5.0, square_side=4.0, circle_radius=1.0,
    cruise_speed=0.4, accel=0.2, dwell=2.0, yaw_mode='tangent',
    yaw_rate_max=0.5, yaw_tol=0.087, d_tau=0.1,
    dive_depth=0.5, dive_rate=0.15, dive_tol=0.05,
    pos_kp=20.0, pos_ki=1.0, pos_kd=35.0, pos_limit=30.0, pos_i_limit=8.0,
    yaw_kp=3.0, yaw_ki=0.2, yaw_kd=2.5, yaw_limit=8.0, yaw_i_limit=1.5,
    z_kp=40.0, z_ki=2.0, z_kd=50.0, z_limit=30.0, z_i_limit=8.0,
    ff_lin=5.0, ff_quad=20.0, ff_lin_sway=8.0, ff_quad_sway=35.0,
    ff_yaw_lin=2.0, ff_yaw_quad=4.0, save_plot=True,
    # --- FAILSAFE eşikleri (gerekçeler: docs/ODEV2_STATE_MACHINE_FAILSAFE.md) ---
    fs_odom_timeout=0.5, fs_watchdog_period=0.05,
    fs_tilt_max=0.785, fs_z_min=-2.6, fs_z_max=-0.2,
    fs_err_max=0.6, fs_sat_time=3.0,
    fs_nan_n=1, fs_tilt_n=5, fs_depth_n=3, fs_err_n=20, fs_sat_n=1,
    fs_clear_time=2.0,
    fs_ascend_force=20.0, fs_ascend_stop_z=-1.0, fs_ascend_max_time=10.0,
    inject_fault='none', log_dir='~/auv_logs',
)


# quat_to_yaw: birim kuaterniyon q=(w,x,y,z) → yaw ψ (z ekseni etrafında dönüş).
#   ψ = atan2(2(w·z + x·y), 1 - 2(y² + z²)). Roll/pitch olsa bile doğru yaw verir.
def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# quat_to_roll_pitch: φ = atan2(2(wx + yz), 1 - 2(x² + y²)),  θ = asin(2(wy - zx)).
# Sayısal hatayla |2(wy - zx)| = 1.0000001 olabilir → asin ValueError fırlatır (düğüm çöker). [-1, 1]'e kırpılır (Q8a).
def quat_to_roll_pitch(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    sp = 2.0 * (q.w * q.y - q.z * q.x)
    return roll, math.asin(max(-1.0, min(1.0, sp)))


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        # Parametreleri bildir: her anahtar DEFAULTS değeriyle açılır, yaml/launch değeri üstüne yazar.
        self.p = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        self.mission = self.p['mission']
        # Yazım hatalarını görev başlamadan yakala (aksi halde start_delay sonrası çöker).
        if self.mission not in VALID_MISSIONS:
            raise ValueError(f"mission {VALID_MISSIONS} içinden biri olmalı, gelen: {self.mission}")
        if self.p['yaw_mode'] not in ('tangent', 'hold', 'auto'):
            raise ValueError("yaw_mode 'tangent' | 'hold' | 'auto' olmalı")
        if self.p['inject_fault'] not in VALID_INJECT:
            raise ValueError(f"inject_fault {VALID_INJECT} içinden biri olmalı")

        # 6 itici yayıncısı + odometri aboneliği + elle sürüş (Wrench) aboneliği.
        name = self.p['model_name']
        self.pubs = [self.create_publisher(Float64, f'/model/{name}/joint/{j}/cmd_thrust', 10)
                     for j in THRUSTERS]
        self.create_subscription(Odometry, f'/model/{name}/odometry', self._odom_cb, 10)
        self.create_subscription(Wrench, '/auv/cmd_wrench', self._wrench_cb, 10)
        # Operatör onayı: olay (event) olduğu için konu; parametre olsaydı "true" kalıp sonraki arızaları da onaylardı (Q13).
        self.create_subscription(Empty, '/auv/failsafe_reset', self._reset_cb, 10)
        self.state_pub = self.create_publisher(String, '/auv/state', 10)

        self.pose = None                    # son odometri: (x, y, ψ, z)
        self.rp = (0.0, 0.0)                # son roll, pitch [rad]
        self.t_start = None                 # ilk geçerli odometri anı (görev saati sıfırı)
        self.t_init = None                  # INIT'e son giriş anı (start_delay buradan sayılır)
        self.ctrl = None                    # görev başlayınca kurulan PlanarController
        self.last_t = None                  # önceki döngü zamanı (dt için)
        self.manual = (0.0, 0.0, 0.0, 0.0)  # elle komut: X, Y, Z, N
        self.manual_t = None
        self.trace = []                     # grafik için kayıt
        self.reported = False

        # --- State machine + FAILSAFE ---
        self.sup = Supervisor(self.p, self._sup_log)
        self.t_mission = 0.0                # son görev saati [s] (watchdog logu için)
        self.err = None                     # son takip hatası ‖p_ref - p‖ [m]
        self.last_sat = False               # son yayında bir itici sınırda mıydı
        self.sat_time = 0.0                 # kesintisiz doyma süresi [s]
        self.z_hold = None                  # 'hold' eyleminde tutulacak derinlik
        self.fs_t0 = 0.0                    # FAILSAFE'e giriş anı (ascend süre sınırı için)
        # Q6: watchdog DUVAR saatiyle çalışır. Simülasyon saati (/clock) köprüyle birlikte ölür; o saatle kurulan
        # zamanlayıcı da durur. STEADY_TIME monoton: NTP/elle saat değişiminden etkilenmez.
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.odom_wall = None               # son odometri alındığı an (duvar) — Q5 zaman damgası
        self.odom_wall0 = None              # ilk odometri anı (duvar)
        self.loop_wall = None               # son kontrol döngüsü çalıştığı an (duvar)
        self.wall0 = self._wall()

        # CSV kaydı: her adımda durum, poz, hata (T1–T6 grafikleri `ros2 run sualti_sim_son plot_run` ile çizilir).
        self.csv = None
        if self.mission != 'manual' and self.p['log_dir']:
            d = os.path.expanduser(self.p['log_dir'])
            os.makedirs(d, exist_ok=True)
            self.csv_path = os.path.join(d, f"auv_{self.mission}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['t', 'wall', 'state', 'mode', 'fault', 'x', 'y', 'z', 'roll', 'pitch',
                               'xs', 'ys', 'rx', 'ry', 'err', 'sat_time', 'age'])
            self.get_logger().info(f"Kayıt: {self.csv_path}")

        # Kontrol döngüsü 20 Hz (T = 0.05 s, simülasyon saatiyle).
        self.create_timer(0.05, self._loop)
        # Watchdog: aynı executor'da ama duvar saatli zamanlayıcı.
        self.create_timer(self.p['fs_watchdog_period'], self._watchdog, clock=self.steady)
        self.get_logger().info(f"Hareket düğümü hazır, görev: {self.mission}")

    def _wall(self):
        return self.steady.now().nanoseconds * 1e-9

    # _sup_log: Supervisor logu. rclpy aynı satırdan farklı seviyede log yazılmasına izin vermez → iki ayrı çağrı.
    def _sup_log(self, msg, important):
        if important:
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)

    # _odom_cb: odometri mesajından poz çıkar: x, y, z doğrudan; ψ, φ, θ kuaterniyondan.
    # Q5: alındığı an duvar saatiyle saklanır (yaş = şimdi - bu an). Test için inject_fault burada uygulanır.
    def _odom_cb(self, msg):
        inject = self.get_parameter('inject_fault').value
        if inject == 'stale_odom':
            return                                      # mesaj "gelmemiş" sayılır → yaş büyür
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, quat_to_yaw(p.orientation), p.position.z)
        rp = quat_to_roll_pitch(p.orientation)
        if inject == 'nan_odom':
            pose = (float('nan'),) * 4
        elif inject == 'tilt':
            rp = (rp[0] + 1.2, rp[1])                   # ≈ 69° roll eklenir
        self.pose, self.rp = pose, rp
        self.odom_wall = self._wall()
        if self.odom_wall0 is None:
            self.odom_wall0 = self.odom_wall

    # _wrench_cb: elle komut (X = force.x, Y = force.y, Z = force.z, N = torque.z) ve alındığı an.
    def _wrench_cb(self, msg):
        self.manual = (msg.force.x, msg.force.y, msg.force.z, msg.torque.z)
        self.manual_t = self.get_clock().now()

    # _reset_cb: operatör onayı. Supervisor kabul edip etmeyeceğine (T_clear) kendisi karar verir.
    def _reset_cb(self, _msg):
        self.get_logger().warn("Operatör FAILSAFE onayı alındı")
        self.sup.acknowledge()

    # _publish: kuvvet/moment → 6 itici itkisi → yayın. NaN/inf (örn. 0/0) Gazebo'yu çökertebilir;
    # girdide sonsuz/NaN varsa tüm itkiler 0'a çekilir.
    def _publish(self, surge, sway, heave, torque):
        if not all(math.isfinite(v) for v in (surge, sway, heave, torque)):
            self.get_logger().error("Komut geçersiz (nan/inf), itki sıfırlandı", throttle_duration_sec=1.0)
            surge = sway = heave = torque = 0.0
        f = allocate(surge, sway, heave, torque, self.p['max_thrust'])
        # Q10: herhangi bir itici sınırdaysa doyma sayılır.
        self.last_sat = max(abs(v) for v in f) >= 0.999 * self.p['max_thrust']
        for pub, v in zip(self.pubs, f):
            pub.publish(Float64(data=float(self.p['thrust_sign'] * v)))

    # _fault_inputs: Supervisor'ın arıza girdileri (Q5, Q7–Q10).
    def _fault_inputs(self, finite):
        age = None if self.odom_wall is None else self._wall() - self.odom_wall
        s = dict(age=age, finite=finite)
        if finite:
            s.update(roll=self.rp[0], pitch=self.rp[1], z=self.pose[3], err=self.err, sat_time=self.sat_time)
        return s

    # _loop: 20 Hz ana döngü.
    def _loop(self):
        now = self.get_clock().now()
        self.loop_wall = self._wall()
        if self.pose is None:
            return
        finite = all(math.isfinite(v) for v in self.pose) and all(math.isfinite(v) for v in self.rp)
        if finite and self.t_start is None:
            self.t_start = self.t_init = now
        # dt = t_k - t_{k-1} [s]. İlk adımda yok; dt ≤ 0 (duraklatılmış sim) veya dt > 0.5 s (kopukluk) → kontrol adımı atlanır.
        dt = None if self.last_t is None else (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        dt_ok = dt is not None and 0.0 < dt <= 0.5

        # Elle mod: state machine dışında. Son komut 1 s'den tazeyse uygula, değilse 0 (komut kesilirse araç durur).
        if self.mission == 'manual':
            if not finite:
                self.stop()
            elif dt_ok:
                fresh = self.manual_t is not None and (now - self.manual_t).nanoseconds < 1e9
                self._publish(*(self.manual if fresh else (0.0, 0.0, 0.0, 0.0)))
            return
        if self.t_start is None:
            self.stop()
            return

        # --- Supervisor adımı: arıza tespiti dt/başlangıç return'lerinden ÖNCE (Q6: hiçbir yol denetimi atlamasın) ---
        t = (now - self.t_start).nanoseconds * 1e-9
        self.t_mission = t
        ready = (now - self.t_init).nanoseconds * 1e-9 >= self.p['start_delay']
        # INIT'ten çıkış anı: başlangıç pozunu (x0, y0, ψ0, z0) orijin olarak kaydet, taze PID'lerle yeni kontrolcü.
        if self.sup.state is State.INIT and ready and finite and self.ctrl is None:
            self.ctrl = PlanarController(self.mission, self.p)
            self.ctrl.set_origin(*self.pose)
            self.err, self.sat_time = 0.0, 0.0
            self.get_logger().info("Görev başladı")
            z_goal = self.pose[3] - self.p['dive_depth']
            if not self.p['fs_z_min'] <= z_goal <= self.p['fs_z_max']:
                self.get_logger().warn(f"Hedef derinlik z* = {z_goal:.2f} m güvenli aralık dışında "
                                       f"[{self.p['fs_z_min']}, {self.p['fs_z_max']}]; FAILSAFE bekleniyor")
        m = None
        if self.ctrl is not None and finite:
            m = self.ctrl.phase_inputs(*self.pose)
            m['ready'] = ready
        elif self.sup.state is State.INIT:
            m = dict(ready=False)
        prev = self.sup.state
        state, mode = self.sup.step(t, self._fault_inputs(finite), m)
        if state is not prev:
            self._on_transition(state, now, t)

        # --- Eylem a(S) ---
        x, y, yaw, z = self.pose
        info = None
        if mode == 'normal':
            if dt_ok:
                surge, sway, heave, torque, info = self.ctrl.update(x, y, yaw, z, dt, self.sup.advance())
                self._publish(surge, sway, heave, torque)
                self.err = math.hypot(info['rx'] - info['xs'], info['ry'] - info['ys'])
                self.sat_time = self.sat_time + dt if self.last_sat else 0.0
        elif mode == 'hold':
            # Q12 (B): yatay itki 0, derinlik PID'i girişteki derinliği tutar.
            if finite and dt_ok and self.ctrl is not None and self.z_hold is not None:
                self._publish(0.0, 0.0, self.ctrl.hold_depth(self.z_hold, z, dt), 0.0)
            else:
                self.stop()
        elif mode == 'ascend':
            # Q12 (C): sabit F_yukarı. ascend_stop_z'ye varınca veya süre dolunca 0 (nötr yüzer, orada kalır).
            up = finite and z < self.p['fs_ascend_stop_z'] and t - self.fs_t0 < self.p['fs_ascend_max_time']
            self._publish(0.0, 0.0, self.p['fs_ascend_force'] if up else 0.0, 0.0)
        else:
            self.stop()

        self.state_pub.publish(String(data=state.name))
        self._log_row(t, mode, info, finite)
        # Kayıt görev bitene kadar tutulur (bitişten sonra liste sınırsız büyümesin).
        if info is not None and not self.reported:
            self.trace.append((t, info['xs'], info['ys'], info['rx'], info['ry'],
                               info['z'], info['z_ref']))

        # Görev bitti: bir kez raporla, grafiği ayrı thread'de çiz (kontrol döngüsünü bloklamasın).
        if state is State.DONE and not self.reported:
            self.reported = True
            self.get_logger().info("Görev bitti, son noktada tutuluyor")
            if self.p['save_plot']:
                threading.Thread(target=self._save_plot, args=(list(self.trace),), daemon=True).start()

    # _on_transition: duruma GİRİŞTE bir kez yapılacaklar.
    def _on_transition(self, state, now, t):
        if state is State.FAILSAFE:
            self.fs_t0 = t
            ok = self.pose is not None and all(math.isfinite(v) for v in self.pose)
            self.z_hold = self.pose[3] if ok else None
            self.sat_time = 0.0
        elif state is State.INIT:
            # FAILSAFE'den onayla çıkış: kontrolcü atılır (PID integralleri ve kaymış referans tehlikeli, Q13);
            # start_delay yeniden sayılır, görev o anki pozdan baştan başlar.
            self.ctrl, self.t_init, self.err, self.reported = None, now, None, False
            self.trace = []

    # _watchdog: duvar saatiyle 20 Hz. Kontrol döngüsü T_stale'den uzun süredir çalışmıyorsa (/clock durdu, köprü öldü):
    # FAILSAFE + itki 0. Döngü canlıysa odometri yaşını döngü kendisi denetler (iki yerden çelişen komut gitmesin).
    def _watchdog(self):
        if self.mission == 'manual' or self.odom_wall is None:
            return
        w = self._wall()
        # Döngü henüz hiç çalışmadıysa (açılışta /clock odometriden geç gelebilir) ilk odometri anı referanstır.
        ref = self.loop_wall if self.loop_wall is not None else self.odom_wall0
        if w - ref <= self.p['fs_odom_timeout']:
            return
        if self.sup.state is not State.FAILSAFE:
            self.get_logger().error(f"Watchdog: kontrol döngüsü {w - ref:.2f} s, son odometri {w - self.odom_wall:.2f} s "
                                    f"önce çalıştı (duvar saati) → /clock durdu, köprü/simülasyon yanıt vermiyor")
            self.sup.trip('odom_stale', self.t_mission)
            self._on_transition(State.FAILSAFE, None, self.t_mission)
        self.stop()                     # köprü geri gelirse Gazebo'ya ilk ulaşan komut 0 olsun
        self.state_pub.publish(String(data=self.sup.state.name))
        self._log_row(self.t_mission, 'zero', None, False, age=w - self.odom_wall)

    def _log_row(self, t, mode, info, finite, age=None):
        if self.csv is None:
            return
        if age is None and self.odom_wall is not None:
            age = self._wall() - self.odom_wall
        x, y, _, z = self.pose if finite else (float('nan'),) * 4
        r = info or {}
        self.csv.writerow([f'{t:.3f}', f'{self._wall() - self.wall0:.3f}', self.sup.state.name, mode,
                           self.sup.fault or '', x, y, z, self.rp[0], self.rp[1],
                           r.get('xs', ''), r.get('ys', ''), r.get('rx', ''), r.get('ry', ''),
                           '' if self.err is None else self.err, self.sat_time,
                           '' if age is None else f'{age:.3f}'])
        self.csv_file.flush()

    # stop: tüm itkileri 0 yap. Gazebo son komutu tutar; kapanırken ve hata anında çağrılır.
    def stop(self):
        self._publish(0.0, 0.0, 0.0, 0.0)

    # _save_plot: sol grafik yol (referans kesikli, araç düz), sağ grafik derinlik farkı z(t) - z0.
    def _save_plot(self, trace):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn("matplotlib yok, grafik kaydedilmedi")
            return
        t, xs, ys, rx, ry, z, zr = zip(*trace)
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 5))
        a1.plot(rx, ry, 'r--', label='referans')
        a1.plot(xs, ys, 'b-', label='araç')
        a1.axis('equal')
        a1.grid(True)
        a1.set_xlabel('x [m] (ileri)')
        a1.set_ylabel('y [m] (sol)')
        a1.legend()
        a2.plot(t, zr, 'r--', label='referans')
        a2.plot(t, z, 'b-', label='araç')
        a2.grid(True)
        a2.set_xlabel('zaman [s]')
        a2.set_ylabel('yükseklik farkı [m]')
        a2.legend()
        path = os.path.expanduser(f"~/auv_{self.mission}.png")
        fig.savefig(path, dpi=120)
        self.get_logger().info(f"Grafik kaydedildi: {path}")


def main(args=None):
    # Ctrl+C'de rclpy kendi sinyal işleyicisiyle bağlamı hemen kapatırsa son "itki = 0" yayını gidemez.
    # İşleyiciyi kapatınca Ctrl+C Python'da KeyboardInterrupt olur, bağlam açık kalır, stop() çalışır.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Q14: yalnız "düzgün" kapanışı kapsar (Ctrl+C, spin'den çıkan istisna). SIGKILL, segfault, donma, köprü
        # ölümü burada yakalanamaz → gerçek araçta itici sürücüsünde komut zaman aşımı gerekir.
        try:
            node.stop()          # Gazebo son itkiyi tutar; kapanırken sıfırla
            time.sleep(0.1)      # mesajın DDS'e çıkması için kısa bekleme
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
