# =============================================================================
# DOSYA : controller.py
# AMAÇ  : Aracın konum (x,y), yönelim (yaw) ve derinlik (z) kontrolü. Girdi: ölçülen poz (x,y,yaw,z).
#         Çıktı: gövde çerçevesinde surge/sway/heave kuvveti [N] ve yaw momenti [N·m].
# "YENGEÇ YÜRÜYÜŞÜ" ÇÖZÜMÜ: Araç yalnız ileri (surge) itki ile verimli gider; yan (sway) itki zayıf ve sürtünmeli.
#   Bu yüzden yaw, yolun teğet yönüne kilitlenir (ψ_ref = ψ_yol). Köşede araç önce döner, hizalanınca ilerler.
# MATEMATİK ÖZETİ:
#   Başlangıç çerçevesi: p_s = R(-ψ0)·(p - p0)           (başlangıç noktası orijin, başlangıç yönü +x)
#   Konum PID (başlangıç çerçevesi) → F_s;  gövdeye: F_b = R(-ψ_s)·F_s
#   Sürtünme ileri beslemesi: F_ff = (d_lin + d_quad·|v|)·v   (hidrodinamik sürtünmeyi baştan karşılar)
# =============================================================================
import math

from .pid import PID, wrap_angle
from .trajectories import ReferenceGenerator, make_path


class PlanarController:
    # mission: 'line' | 'square' | 'circle';  p: parametre sözlüğü (gains.yaml / DEFAULTS).
    def __init__(self, mission, p):
        self.p = p
        # Yaw modu: 'auto' → 'tangent'. Bilinmeyen yazım (örn. 'Tangent') sessizce 'hold' olmasın diye reddedilir.
        mode = p['yaw_mode']
        if mode == 'auto':
            mode = 'tangent'
        if mode not in ('tangent', 'hold'):
            raise ValueError(f"yaw_mode 'tangent' | 'hold' | 'auto' olmalı, gelen: {mode}")
        self.yaw_mode = mode
        # Daire koşulu: yaw referansı en fazla ω_max = yaw_rate_max hızla döner, dairede gereken ψ̇ = v/r.
        # v/r > ω_max olursa yaw geride kalır → araç yan yan gider (yengeç). Güvenlik: v ≤ 0.9·ω_max·r.
        v_cruise = p['cruise_speed']
        if mission == 'circle' and mode == 'tangent':
            v_cruise = min(v_cruise, 0.9 * p['yaw_rate_max'] * p['circle_radius'])
        self.gen = ReferenceGenerator(
            make_path(mission, p['line_length'], p['square_side'], p['circle_radius']),
            v_cruise, p['accel'], p['dwell'])
        # Dört PID (aynı d_tau türev filtresi): x, y konum; yaw; z derinlik.
        dt_ = p['d_tau']
        self.pid_x = PID(p['pos_kp'], p['pos_ki'], p['pos_kd'], p['pos_limit'], p['pos_i_limit'], dt_)
        self.pid_y = PID(p['pos_kp'], p['pos_ki'], p['pos_kd'], p['pos_limit'], p['pos_i_limit'], dt_)
        self.pid_yaw = PID(p['yaw_kp'], p['yaw_ki'], p['yaw_kd'], p['yaw_limit'], p['yaw_i_limit'], dt_)
        self.pid_z = PID(p['z_kp'], p['z_ki'], p['z_kd'], p['z_limit'], p['z_i_limit'], dt_)
        self.origin = None                 # (x0, y0, ψ0): görev başındaki dünya pozu
        self.origin_z = 0.0                # z0
        self.z_ref = 0.0                   # kayan derinlik referansı
        self.z_target = 0.0                # hedef derinlik
        # Aşama bilgisi (dalış / hizalanma / bekleme) artık burada değil: state_machine.Supervisor tutar.
        # Son referans: (x, y, yön ψ, vx, vy). Başlangıç yönü yolun s=0'daki teğeti.
        self.ref = (0.0, 0.0, self.gen.path.at(0.0)[2], 0.0, 0.0)
        self.yaw_ref = 0.0                 # hız sınırlı yaw referansı ψ_ref

    # done: referans son durağa vardı ve bekleme bitti.
    @property
    def done(self):
        return self.gen.done

    # set_origin: başlangıç çerçevesini kur. Hedef derinlik z* = z0 - dive_depth (z yukarı pozitif, dalış aşağı).
    def set_origin(self, x, y, yaw, z):
        self.origin = (x, y, yaw)
        self.origin_z = z
        self.z_ref = z
        self.z_target = z - self.p['dive_depth']

    # to_start_frame: dünya → başlangıç çerçevesi (2B döndürme).
    #   [xs]   [ cosψ0  sinψ0] [x - x0]          ψs = wrap(ψ - ψ0)
    #   [ys] = [-sinψ0  cosψ0] [y - y0]
    def to_start_frame(self, x, y, yaw):
        x0, y0, yaw0 = self.origin
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        return c * dx + s * dy, -s * dx + c * dy, wrap_angle(yaw - yaw0)

    # _heading_target: istenen yön. 'tangent' → referans noktasının yol teğeti ψ_yol; 'hold' → 0 (başlangıç yönü).
    def _heading_target(self):
        return self.ref[2] if self.yaw_mode == 'tangent' else 0.0

    # phase_inputs: Supervisor'ın geçiş koşulları için gereken büyüklükler (Q2 tablosu).
    #   dz      = |z* - z|                       (DIVE → ALIGN:  dz < dive_tol)
    #   yaw_err = |wrap(ψ_hedef - ψ_s)|          (ALIGN → TRACK/DWELL: yaw_err < yaw_tol)
    #   v, dwell_left, done: referans üretecinin durumu (TRACK ↔ DWELL ↔ DONE)
    def phase_inputs(self, x, y, yaw, z):
        _, _, yaws = self.to_start_frame(x, y, yaw)
        return dict(dz=abs(self.z_target - z),
                    yaw_err=abs(wrap_angle(self._heading_target() - yaws)),
                    v=self.gen.v, dwell_left=self.gen.dwell_left, done=self.gen.done)

    # update: bir kontrol adımı (dt [s]). advance: yol referansı bu adımda ilerlesin mi (durum TRACK/DWELL ise
    # Supervisor True verir). Dönen: surge, sway, heave [N], torque [N·m], info (kayıt için).
    def update(self, x, y, yaw, z, dt, advance):
        # dt sıfır/negatif gelirse w_ref = dψ/dt bölmesi sonsuz olurdu → alt sınır.
        dt = max(dt, 1e-6)
        xs, ys, yaws = self.to_start_frame(x, y, yaw)

        # --- DERİNLİK ---
        # Referans hedefe sabit hızla kayar: z_ref ← z_ref + clamp(z* - z_ref, ±dive_rate·dt) (basamak yok).
        # Heave = PID(z_ref - z) (nötr yüzdürme: ağırlık telafisi yok). Dalışın bitişi (|z* - z| < dive_tol) Supervisor'da.
        step = self.p['dive_rate'] * dt
        self.z_ref += max(-step, min(step, self.z_target - self.z_ref))
        heave = self.pid_z.update(self.z_ref - z, dt)

        # --- REFERANS NOKTASI ---
        # Eski kod: "if not diving and (aligned or v > 0): ilerlet". Bu karar artık durumdan gelir (advance).
        if advance:
            self.ref = self.gen.step(dt)
        rx, ry, _, rvx, rvy = self.ref

        # --- YAW ---
        # Yaw referansı hedef yöne en çok ω_max hızla ilerler: Δψ = clamp(wrap(ψ_hedef - ψ_ref), ±ω_max·dt).
        # Böylece 90°'lik köşe "basamak" olmaz (basamak → büyük moment → doyma). w_ref = Δψ/dt.
        dpsi = wrap_angle(self._heading_target() - self.yaw_ref)
        lim = self.p['yaw_rate_max'] * dt
        dpsi = max(-lim, min(lim, dpsi))
        self.yaw_ref = wrap_angle(self.yaw_ref + dpsi)
        w_ref = dpsi / dt
        # Yaw sürtünme ileri beslemesi: N_ff = (n_lin + n_quad·|ω|)·ω;  toplam moment N = PID(wrap(ψ_ref - ψ)) + N_ff
        ff_yaw = (self.p['ff_yaw_lin'] + self.p['ff_yaw_quad'] * abs(w_ref)) * w_ref
        torque = self.pid_yaw.update(wrap_angle(self.yaw_ref - yaws), dt) + ff_yaw

        # --- KONUM ---
        # Başlangıç çerçevesinde kuvvet: F_s = PID(r - p_s)  (x ve y ayrı PID).
        fx = self.pid_x.update(rx - xs, dt)
        fy = self.pid_y.update(ry - ys, dt)
        # Gövdeye döndür: surge = cosψ·fx + sinψ·fy ;  sway = -sinψ·fx + cosψ·fy
        c, s = math.cos(yaws), math.sin(yaws)
        surge = c * fx + s * fy
        sway = -s * fx + c * fy

        # --- SÜRTÜNME İLERİ BESLEMESİ ---
        # Referans hızı gövde eksenine çevir (aynı döndürme): v_bx = c·vx + s·vy, v_by = -s·vx + c·vy.
        # Hidrodinamik sürtünme F_d = (d1 + d2·|v|)·v (pool.sdf: xU,xUU / yV,yVV) → aynı kuvveti baştan ekle.
        # Surge ve sway katsayıları farklıdır (gövde şekli).
        vbx = c * rvx + s * rvy
        vby = -s * rvx + c * rvy
        surge += (self.p['ff_lin'] + self.p['ff_quad'] * abs(vbx)) * vbx
        sway += (self.p['ff_lin_sway'] + self.p['ff_quad_sway'] * abs(vby)) * vby

        info = dict(xs=xs, ys=ys, yaw=yaws, rx=rx, ry=ry, yaw_ref=self.yaw_ref,
                    z=z - self.origin_z, z_ref=self.z_ref - self.origin_z)
        return surge, sway, heave, torque, info

    # hold_depth: FAILSAFE 'hold' eylemi. Yatay kuvvet ve moment 0; heave = PID(z_tut - z) ile olduğun derinlikte kal.
    # z_tut FAILSAFE'e girişte bir kez alınır (referans kaymaz). Dönen: heave [N].
    def hold_depth(self, z_hold, z, dt):
        return self.pid_z.update(z_hold - z, max(dt, 1e-6))
