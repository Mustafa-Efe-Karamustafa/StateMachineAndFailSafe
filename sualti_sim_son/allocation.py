# =============================================================================
# DOSYA : allocation.py
# AMAÇ  : Kontrolcünün istediği gövde kuvvet/momentini (surge X, sway Y, heave Z, yaw momenti N)
#         6 iticinin tek tek itki değerine [N] çevirir (itki dağıtımı / thrust allocation).
# MATEMATİK ÖZETİ:  τ = B·F   (τ: istenen [X,Y,N]; F: 4 yatay itki; B: 3x4 yerleşim matrisi)
#   Çözüm: 4 bilinmeyen, 3 denklem → simetrik (minimum norm) çözüm seçilir.
#   Doyma (|F_i| ≤ F_max) olursa yaw önceliklidir; öteleme (surge/sway) ölçeklenerek kısılır.
# =============================================================================
import math

# Yatay 4 itici (X dizilimi), sıra: FR, FL, RL, RR.
# Konumlar (x,y): FR(+a,-b) FL(+a,+b) RL(-a,+b) RR(-a,-b);  itki açıları θ: +45°, -45°, +45°, -45°.
# Dikey 2 itici: VL (y>0), VR (y<0), itki +z (yukarı).  a = THRUSTER_X, b = THRUSTER_Y [m].
THRUSTER_X = 0.18
THRUSTER_Y = 0.19
# Yaw moment kolu: L = a + b  (Nz = c·L·(F1 - F2 - F3 + F4), c = 1/√2; aşağıda türetilmiştir).
L = THRUSTER_X + THRUSTER_Y


# allocate: (X, Y, Z [N], N [N·m]) → [FR, FL, RL, RR, VL, VR] [N].
# İleri model (her itki F_i yönü d_i = (cosθ_i, sinθ_i), konumu r_i = (x_i, y_i)):
#   X  = ΣF_i·cosθ_i             = c·(F1 + F2 + F3 + F4)
#   Y  = ΣF_i·sinθ_i             = c·(F1 - F2 + F3 - F4)
#   Nz = Σ(x_i·F_i,y - y_i·F_i,x) = c·L·(F1 - F2 - F3 + F4)         (c = cos45° = 1/√2)
# Ters çözüm (simetrik): F_i = k·(X ± Y) + s_i·k·N/L,  k = 1/(2√2),  s = (+1, -1, -1, +1)
#   Doğrulama: yalnız X → F_i = k·X; Fx = c·4k·X = X ✓.  Yalnız N → Nz = c·L·4k·N/L = N ✓.
def allocate(surge, sway, heave, yaw_torque, max_thrust, arm=L):
    k = 1.0 / (2.0 * math.sqrt(2.0))
    # Yaw payı: y_i = s_i·k·N/L; en büyük |y_i| ≤ F_max olacak şekilde orantılı kırp (yaw öncelikli).
    yaw = [k * yaw_torque / arm * s for s in (1.0, -1.0, -1.0, 1.0)]
    m = max(abs(v) for v in yaw)
    if m > max_thrust:
        yaw = [v * max_thrust / m for v in yaw]
    # Öteleme payı: t_i = k·(X±Y). FR, RL: (X+Y); FL, RR: (X-Y).
    trn = [k * (surge + sway), k * (surge - sway), k * (surge + sway), k * (surge - sway)]
    # Öteleme ölçeği α ∈ [0,1]: her itici için |α·t_i + y_i| ≤ F_max koşulundan
    #   t_i > 0 → α ≤ (F_max - y_i)/t_i ;   t_i < 0 → α ≤ (-F_max - y_i)/t_i
    alpha = 1.0
    for t, y in zip(trn, yaw):
        if t > 1e-9:
            alpha = min(alpha, (max_thrust - y) / t)
        elif t < -1e-9:
            alpha = min(alpha, (-max_thrust - y) / t)
    # Yatay itkiler: F_i = α·t_i + y_i
    h = [alpha * t + y for t, y in zip(trn, yaw)]
    # Dikey: Fz = VL + VR, simetrik dağıtım → her biri Z/2 (roll momenti oluşmaz); ±F_max'a kırp.
    v = max(-max_thrust, min(max_thrust, heave / 2.0))
    return h + [v, v]
