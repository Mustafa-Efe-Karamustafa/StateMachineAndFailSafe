# =============================================================================
# DOSYA : trajectories.py
# AMAÇ  : Aracın izleyeceği YOLU (doğru / kare / daire) ve yol üzerinde zamanla ilerleyen
#         REFERANS NOKTASINI (trapez hız profili) üretir. Hepsi aracın BAŞLANGIÇ çerçevesindedir:
#         başlangıç noktası (0,0), başlangıç yönü +x (ileri), +y sol.
# FİZİK/MATEMATİK ÖZETİ:
#   Yol, yay uzunluğu s ile parametrelenir: P(s) = (x(s), y(s)), teğet yönü ψ(s) = atan2(dy/ds, dx/ds).
#   Referans noktası s(t) ile kayar: ṡ = v(t), v trapez profil izler (ivme a, tepe hız v_max).
#   Hız bileşenleri: ẋ = v·cosψ, ẏ = v·sinψ  (ileri besleme için kullanılır).
# =============================================================================
import bisect
import math


class PolylinePath:
    """Kırık çizgi: (0,0)'dan başlayıp verilen köşelerden geçer. Her köşede durulur."""

    # points: köşe listesi [(x1,y1), (x2,y2), ...]. Önce başa (0,0) eklenir.
    # Ardışık çakışan noktalar atılır: |P_{k+1} - P_k| = 0 olsa at() içinde (s - s_k)/0 = 0/0 bölmesi çıkardı.
    def __init__(self, points):
        pts = [(0.0, 0.0)]
        for p in points:
            if math.dist(pts[-1], p) > 1e-9:
                pts.append((float(p[0]), float(p[1])))
        if len(pts) < 2:
            raise ValueError('yol uzunluğu sıfır olamaz')
        self.pts = pts
        # Birikimli yay uzunluğu: s_0 = 0, s_{k+1} = s_k + |P_{k+1} - P_k| (Öklid uzaklığı √(Δx²+Δy²)).
        self.cum = [0.0]
        for a, b in zip(self.pts[:-1], self.pts[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.length = self.cum[-1]
        # Duraklar: her köşenin yay uzunluğu s_k (k ≥ 1). Referans her durakta s_k'ya oturup bekler.
        self.stops = self.cum[1:]

    # at(s): yay uzunluğu s'deki nokta ve teğet yönü.
    # Doğrusal interpolasyon: P(s) = A + u·(B - A),  u = (s - s_i)/(s_{i+1} - s_i) ∈ [0,1]  (payda > 0 garantili).
    # Yön: ψ = atan2(By - Ay, Bx - Ax); A≠B olduğundan atan2(0,0) oluşmaz.
    def at(self, s):
        s = min(max(s, 0.0), self.length)
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2)
        a, b = self.pts[i], self.pts[i + 1]
        u = (s - self.cum[i]) / (self.cum[i + 1] - self.cum[i])
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]),
                math.atan2(b[1] - a[1], b[0] - a[0]))


class CirclePath:
    """Başlangıçta +x yönüne teğet daire. ccw=True: merkez solda, (0, r)."""

    # radius: yarıçap r [m]; laps: tur sayısı; ccw: saat yönünün tersi mi.
    # Toplam yay uzunluğu L = 2π·r·laps. r = 0 olsa phi = s/r bölmesi patlardı → reddet.
    def __init__(self, radius, laps=1.0, ccw=True):
        if radius <= 0.0 or laps <= 0.0:
            raise ValueError('daire yarıçapı ve tur sayısı > 0 olmalı')
        self.r, self.sign = radius, (1.0 if ccw else -1.0)
        self.length = 2.0 * math.pi * radius * laps
        self.stops = [self.length]

    # at(s): açısal konum φ = s/r. Merkez (0, σr), σ = ±1 (ccw/cw):
    #   x = r·sinφ,   y = σ·r·(1 - cosφ)   (φ=0'da (0,0) ve +x'e teğet)
    #   Teğet: (dx/dφ, dy/dφ) = (r·cosφ, σ·r·sinφ) → yön ψ = σ·φ. Yaw hızı ψ̇ = v/r.
    def at(self, s):
        phi = min(max(s, 0.0), self.length) / self.r
        return (self.r * math.sin(phi),
                self.sign * self.r * (1.0 - math.cos(phi)),
                self.sign * phi)


# make_path: görev adından yol nesnesi seçer. Uzunluk/kenar/yarıçap pozitif olmalı (aksi halde sıfır uzunluklu yol).
#   line   : (0,0) → (L,0)
#   square : (0,0) → (a,0) → (a,a) → (0,a) → (0,0)   (4 köşe, a = kenar)
#   circle : yarıçap r çember
def make_path(mission, length=5.0, side=4.0, radius=1.0):
    if mission == 'line':
        return PolylinePath([(length, 0.0)])
    if mission == 'square':
        return PolylinePath([(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)])
    if mission == 'circle':
        return CirclePath(radius)
    raise ValueError(f'bilinmeyen görev: {mission}')


class ReferenceGenerator:
    """Yol üzerinde hareket eden referans noktası (trapez hız profili, duraklarda bekleme)."""

    # v_cruise: tepe hız [m/s]; accel: ivme/yavaşlama a [m/s²]; dwell: her durakta bekleme [s].
    # a ≤ 0 veya v ≤ 0 ise referans hiç ilerlemez → baştan reddet.
    def __init__(self, path, v_cruise=0.4, accel=0.2, dwell=2.0):
        if v_cruise <= 0.0 or accel <= 0.0 or dwell < 0.0:
            raise ValueError('cruise_speed > 0, accel > 0, dwell >= 0 olmalı')
        self.path, self.v_cruise, self.a, self.dwell = path, v_cruise, accel, dwell
        self.s = 0.0                 # yay üzerindeki konum [m]
        self.v = 0.0                 # yay boyunca hız ṡ [m/s]
        self.stop_idx = 0            # sıradaki durak numarası
        self.dwell_left = 0.0        # kalan bekleme süresi [s]
        self.done = False            # tüm duraklar bitti ve son bekleme doldu

    # step(dt): referansı dt kadar ilerletir. Dönen: (x, y, yön ψ, vx, vy).
    def step(self, dt):
        stops = self.path.stops
        if not self.done:
            if self.dwell_left > 0.0:
                # Beklemedeki durak: sayaç t_w ← t_w - dt.
                self.dwell_left -= dt
            elif self.stop_idx < len(stops):
                # Trapez profil: kalan mesafe rem = s_stop - s.
                #   hızlanma  : v ← v + a·dt
                #   yavaşlama : v ≤ √(2·a·rem)  (v² = 2·a·rem ile tam durağa oturur)
                #   tepe hız  : v ≤ v_max;  alt sınır 0.03 m/s (son santimetrede takılıp kalmasın)
                target = stops[self.stop_idx]
                rem = target - self.s
                v_allow = max(math.sqrt(2.0 * self.a * max(rem, 0.0)), 0.03)
                self.v = min(self.v + self.a * dt, self.v_cruise, v_allow)
                # Konum integrali: s ← s + v·dt (durağı aşma).
                self.s = min(self.s + self.v * dt, target)
                # Durağa 0.1 mm kala: tam oturt, hızı sıfırla, bekleme sayacını kur.
                if target - self.s < 1e-4:
                    self.s, self.v = target, 0.0
                    self.stop_idx += 1
                    self.dwell_left = self.dwell
            # Bitiş koşulu: tüm duraklar geçildi VE bekleme ≤ 0.
            # (dwell = 0 iken de burada bitirilir; eski kodda bu durumda stops[stop_idx] taşıp IndexError veriyordu.)
            if self.dwell_left <= 0.0 and self.stop_idx >= len(stops):
                self.done = True
        # Referans konum, yön ve hız bileşenleri: vx = v·cosψ, vy = v·sinψ.
        x, y, hd = self.path.at(self.s)
        return x, y, hd, self.v * math.cos(hd), self.v * math.sin(hd)
