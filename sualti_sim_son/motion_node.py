#!/usr/bin/env python3
# =============================================================================
# DOSYA : motion_node.py
# AMAÇ  : ROS 2 düğümü. Odometriyi okur, 20 Hz'de PlanarController'ı çalıştırır, çıkan kuvvet/momenti
#         allocation.py ile 6 iticiye dağıtıp Gazebo'ya yazar. Görev bitince yol/derinlik grafiğini kaydeder.
#   mission: line | square | circle | manual   (manual: /auv/cmd_wrench Wrench mesajıyla elle sür)
# AKIŞ (her 50 ms):  odom → [x,y,ψ,z] → controller.update → allocate → 6× cmd_thrust yayını
# MATEMATİK ÖZETİ:
#   dt = t_k - t_{k-1} (simülasyon saati). Yaw: ψ = atan2(2(wz + xy), 1 - 2(y² + z²))  (kuaterniyondan).
#   İtki komutu: F_cmd,i = thrust_sign · F_i   (thrust_sign = ±1: itici montaj yönü düzeltmesi)
# =============================================================================
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from .allocation import allocate
from .controller import PlanarController

# İtici eklem adları: sıra allocate() çıktısıyla aynı olmalı [FR, FL, RL, RR, VL, VR].
THRUSTERS = ['thruster_fr', 'thruster_fl', 'thruster_rl', 'thruster_rr',
             'thruster_vl', 'thruster_vr']
VALID_MISSIONS = ('line', 'square', 'circle', 'manual')

# Varsayılan parametreler. gains.yaml aynı adlarla bunların üzerine yazar.
# DİKKAT: ROS parametre tipi sabittir; ondalık parametreler yaml'da 0 değil 0.0 yazılmalıdır.
DEFAULTS = dict(
    mission='line', start_delay=3.0, thrust_sign=1.0, max_thrust=20.0, model_name='auv',
    line_length=5.0, square_side=4.0, circle_radius=1.0,
    cruise_speed=0.4, accel=0.2, dwell=2.0, yaw_mode='tangent',
    yaw_rate_max=0.5, yaw_tol=0.087, d_tau=0.1,
    dive_depth=0.5, dive_rate=0.15,
    pos_kp=20.0, pos_ki=1.0, pos_kd=35.0, pos_limit=30.0, pos_i_limit=8.0,
    yaw_kp=3.0, yaw_ki=0.2, yaw_kd=2.5, yaw_limit=8.0, yaw_i_limit=1.5,
    z_kp=40.0, z_ki=2.0, z_kd=50.0, z_limit=30.0, z_i_limit=8.0,
    ff_lin=5.0, ff_quad=20.0, ff_lin_sway=8.0, ff_quad_sway=35.0,
    ff_yaw_lin=2.0, ff_yaw_quad=4.0, save_plot=True,
)


# quat_to_yaw: birim kuaterniyon q=(w,x,y,z) → yaw ψ (z ekseni etrafında dönüş).
#   ψ = atan2(2(w·z + x·y), 1 - 2(y² + z²)). Roll/pitch olsa bile doğru yaw verir.
def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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

        # 6 itici yayıncısı + odometri aboneliği + elle sürüş (Wrench) aboneliği.
        name = self.p['model_name']
        self.pubs = [self.create_publisher(Float64, f'/model/{name}/joint/{j}/cmd_thrust', 10)
                     for j in THRUSTERS]
        self.create_subscription(Odometry, f'/model/{name}/odometry', self._odom_cb, 10)
        self.create_subscription(Wrench, '/auv/cmd_wrench', self._wrench_cb, 10)

        self.pose = None                    # son odometri: (x, y, ψ, z)
        self.t_start = None                 # ilk geçerli odometri anı (görev saati sıfırı)
        self.ctrl = None                    # görev başlayınca kurulan PlanarController
        self.last_t = None                  # önceki döngü zamanı (dt için)
        self.manual = (0.0, 0.0, 0.0, 0.0)  # elle komut: X, Y, Z, N
        self.manual_t = None
        self.trace = []                     # grafik için kayıt
        self.reported = False
        # Kontrol döngüsü 20 Hz (T = 0.05 s, simülasyon saatiyle).
        self.create_timer(0.05, self._loop)
        self.get_logger().info(f"Hareket düğümü hazır, görev: {self.mission}")

    # _odom_cb: odometri mesajından poz çıkar: x, y, z doğrudan; ψ kuaterniyondan.
    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, quat_to_yaw(p.orientation), p.position.z)

    # _wrench_cb: elle komut (X = force.x, Y = force.y, Z = force.z, N = torque.z) ve alındığı an.
    def _wrench_cb(self, msg):
        self.manual = (msg.force.x, msg.force.y, msg.force.z, msg.torque.z)
        self.manual_t = self.get_clock().now()

    # _publish: kuvvet/moment → 6 itici itkisi → yayın. NaN/inf (örn. 0/0) Gazebo'yu çökertebilir;
    # girdide sonsuz/NaN varsa tüm itkiler 0'a çekilir.
    def _publish(self, surge, sway, heave, torque):
        if not all(math.isfinite(v) for v in (surge, sway, heave, torque)):
            self.get_logger().error("Komut geçersiz (nan/inf), itki sıfırlandı", throttle_duration_sec=1.0)
            surge = sway = heave = torque = 0.0
        f = allocate(surge, sway, heave, torque, self.p['max_thrust'])
        for pub, v in zip(self.pubs, f):
            pub.publish(Float64(data=float(self.p['thrust_sign'] * v)))

    # _loop: 20 Hz ana döngü.
    def _loop(self):
        now = self.get_clock().now()
        if self.pose is None:
            return
        # Odometri sonlu mu? (nan/inf → itkiyi kes)
        if not all(math.isfinite(v) for v in self.pose):
            self.get_logger().error("Odometri geçersiz (nan/inf), itki sıfırlandı", throttle_duration_sec=1.0)
            self.stop()
            return
        if self.t_start is None:
            self.t_start = now
        if self.last_t is None:
            self.last_t = now
            return
        # dt = t_k - t_{k-1} [s]. dt ≤ 0 (duraklatılmış sim) veya dt > 0.5 s (kopukluk) → bu adımı atla.
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0 or dt > 0.5:
            return

        # Elle mod: son komut 1 s'den tazeyse uygula, değilse 0 (komut kesilirse araç durur).
        if self.mission == 'manual':
            fresh = self.manual_t is not None and (now - self.manual_t).nanoseconds < 1e9
            self._publish(*(self.manual if fresh else (0.0, 0.0, 0.0, 0.0)))
            return

        # Görev saati t = şimdi - t_start. start_delay dolana kadar itki 0 (araç oturur).
        t = (now - self.t_start).nanoseconds * 1e-9
        if t < self.p['start_delay']:
            self._publish(0.0, 0.0, 0.0, 0.0)
            return
        # İlk görev adımı: başlangıç pozunu (x0, y0, ψ0, z0) orijin olarak kaydet.
        if self.ctrl is None:
            self.ctrl = PlanarController(self.mission, self.p)
            self.ctrl.set_origin(*self.pose)
            self.get_logger().info("Görev başladı")

        # Kontrol: (X, Y, Z, N) hesapla ve iticilere yaz.
        x, y, yaw, z = self.pose
        surge, sway, heave, torque, info = self.ctrl.update(x, y, yaw, z, dt)
        self._publish(surge, sway, heave, torque)
        # Kayıt görev bitene kadar tutulur (bitişten sonra liste sınırsız büyümesin).
        if not self.reported:
            self.trace.append((t, info['xs'], info['ys'], info['rx'], info['ry'],
                               info['z'], info['z_ref']))

        # Görev bitti: bir kez raporla, grafiği ayrı thread'de çiz (kontrol döngüsünü bloklamasın).
        if self.ctrl.done and not self.reported:
            self.reported = True
            self.get_logger().info("Görev bitti, son noktada tutuluyor")
            if self.p['save_plot']:
                threading.Thread(target=self._save_plot, args=(list(self.trace),), daemon=True).start()

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
