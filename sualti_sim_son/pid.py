# =============================================================================
# DOSYA : pid.py
# AMAÇ  : Hata sinyalinden (referans - ölçüm) kuvvet/moment komutu üreten PID denetleyicisi
#         ve açıları (-π, π] aralığına saran yardımcı fonksiyon.
# KULLANIM: controller.py içinde 4 PID vardır: x, y (konum), yaw (yönelim), z (derinlik).
# MATEMATİK ÖZETİ:
#   e(t) = r(t) - y(t)                        (hata = referans - ölçüm)
#   u(t) = Kp·e + Ki·∫e dt + Kd·de/dt         (PID)
#   Kapalı çevrim (kütle m, doğrusal sürtünme d):  m·ẍ = u - d·ẋ
#       → m·s² + (d + Kd)·s + Kp = 0   →   ωn = √(Kp/m),  ζ = (d + Kd) / (2·√(Kp·m))
#   ζ ≈ 1 kritik sönüm: taşmasız en hızlı yanıt. ζ < 1 salınır, ζ > 1 yavaşlar.
# =============================================================================
import math


# wrap_angle: a'yı (-π, π] aralığına indirger:  a_wrap = atan2(sin a, cos a).
# sin²a + cos²a = 1 olduğundan atan2'nin iki argümanı aynı anda 0 olamaz → atan2(0,0) belirsizliği oluşmaz.
# Örnek: 359° - 1° = 358° hatası yerine -2° döner (hep kısa yönden dön).
def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    # Parametreler: Kp, Ki, Kd kazançları; out_limit: |u| üst sınırı [N veya N·m];
    # i_limit: integral terimi Ki·∫e için üst sınır; d_tau: türev filtresi zaman sabiti τ [s].
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, out_limit=None, i_limit=None, d_tau=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.d_tau = d_tau
        self.reset()

    # reset: durum değişkenlerini sıfırlar: I = 0 (biriken hata yok), e_{k-1} yok, filtreli türev d_f = 0.
    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.d_filt = 0.0

    # update: bir örnekleme adımı (dt [s]) için u_k üretir. Her çağrıda e_k yeni hatadır.
    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        # Türev (sonlu fark): d_raw = (e_k - e_{k-1}) / dt. İlk örnekte e_{k-1} yok → 0 (ilk adımda türev patlamasın).
        d_raw = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        # Düşük geçiren filtre: d_f ← d_f + α·(d_raw - d_f), α = dt/(τ + dt)  [sürekli hali: τ·ḋ_f + d_f = d_raw].
        # Sonlu fark 1/dt ile gürültüyü büyütür; filtre bu yüksek frekansı sönümler.
        self.d_filt += dt / (self.d_tau + dt) * (d_raw - self.d_filt)
        self.prev_error = error

        # Integral (dikdörtgen kuralı): I_new = I + e·dt. |Ki·I| ≤ i_limit olsun diye |I| ≤ i_limit/|Ki| ile kırp.
        i_new = self.integral + error * dt
        if self.i_limit is not None and self.ki != 0.0:
            lim = self.i_limit / abs(self.ki)
            i_new = max(-lim, min(lim, i_new))
        # PD kısmı ve ham çıkış: u = Kp·e + Kd·d_f + Ki·I
        pd = self.kp * error + self.kd * self.d_filt
        u = pd + self.ki * i_new
        # Anti-windup (koşullu integral): çıkış doymuşsa (|u| > limit) ve e·u > 0 (hata doymayı daha da artırıyorsa)
        # integrali dondur; aksi halde I, doymuş çıkış yüzünden sınırsız şişer (windup) ve büyük taşma yapar.
        saturated = self.out_limit is not None and abs(u) > self.out_limit
        if not (saturated and error * u > 0.0):
            self.integral = i_new
        # Son çıkış: kabul edilen integralle yeniden hesapla, itici sınırına kırp: u ∈ [-limit, +limit].
        u = pd + self.ki * self.integral
        if self.out_limit is not None:
            u = max(-self.out_limit, min(self.out_limit, u))
        return u
