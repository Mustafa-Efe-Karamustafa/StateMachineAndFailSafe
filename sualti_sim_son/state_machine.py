# =============================================================================
# DOSYA : state_machine.py
# AMAÇ  : Aracın "şu an ne yapıyorum?" bilgisini TEK bir durum değişkeninde toplar (state machine) ve
#         arızaları algılayıp FAILSAFE durumuna geçirir. motion_node her 20 Hz adımda Supervisor.step() çağırır;
#         dönen komut moduna göre ya kontrolcünün çıktısını ya da güvenli eylemi (itki 0 / derinlik tut / yüksel) uygular.
# DURUMLAR (docs/ODEV2_STATE_MACHINE_FAILSAFE.md, Q4 tablosu):
#   INIT     : start_delay dolana kadar itki 0                       → DIVE
#   DIVE     : derinlik referansı kayar, yol referansı DURUR          → ALIGN   (|z* - z| < dive_tol)
#   ALIGN    : araç yerinde döner, yol referansı DURUR                → TRACK / DWELL (|e_ψ| < yaw_tol)
#   TRACK    : referans trapez profille ilerler                       → DWELL / ALIGN / DONE (durağa varıldı)
#   DWELL    : durakta bekleme sayacı akar                            → TRACK / ALIGN / DONE
#   DONE     : son noktada konum tutulur                              (son durum)
#   FAILSAFE : arıza; kilitli (latch). Çıkış: arıza T_clear s yok + operatör onayı → INIT
# ARIZALAR (Q5-Q11): odom_stale, nan, tilt, depth_low, depth_high, track_error, saturation
# =============================================================================
from enum import Enum


class State(Enum):
    INIT = 0
    DIVE = 1
    ALIGN = 2
    TRACK = 3
    DWELL = 4
    DONE = 5
    FAILSAFE = 6


# Durumdan yol referansının ilerleyip ilerlemeyeceği (controller.update(advance=...)).
# Eski koddaki "if not diving and (aligned or v > 0): gen.step()" satırının durum karşılığı.
ADVANCE = {State.TRACK: True, State.DWELL: True}

# Arıza → FAILSAFE eylemi (Q12 tablosu).
#   zero   : tüm itkiler 0 (araç nötr yüzer, sürtünmeyle durur)
#   hold   : yatay itki 0, derinlik PID'i olduğu derinliği tutar (odometri sağlam olmalı)
#   ascend : dikey iticilerle sabit kuvvetle yüksel, ascend_stop_z'ye varınca / süre dolunca 0
FAULT_ACTION = {
    'odom_stale': 'zero',      # konum bilinmiyor → kapalı çevrim yok; kör itki vermek yerine dur
    'nan': 'zero',             # ölçüm anlamsız → kapalı çevrim yok
    'tilt': 'zero',            # itki yönleri yanlış; pasif geri getirme momenti düzeltsin
    'depth_low': 'ascend',     # zemine yaklaşıyor → aktif fren + yüksel
    'depth_high': 'zero',      # yüzeye çıkmış / derinlik kontrolü kaybolmuş
    'track_error': 'hold',     # odometri sağlam, araç sıkışmış/sürükleniyor → iterek zorlama, derinliği tut
    'saturation': 'hold',
}

# Her arıza için kaç ardışık örnek gerekir (debounce, Q11). Eşikler gains.yaml'dan gelir (fs_*_n).
DEBOUNCE_KEYS = {
    'nan': 'fs_nan_n', 'tilt': 'fs_tilt_n', 'depth_low': 'fs_depth_n', 'depth_high': 'fs_depth_n',
    'track_error': 'fs_err_n', 'saturation': 'fs_sat_n',
}

# Takip/doyma denetiminin anlamlı olduğu durumlar (INIT'te referans yok, kontrol kapalı).
MISSION_STATES = (State.DIVE, State.ALIGN, State.TRACK, State.DWELL, State.DONE)


class Supervisor:
    # p: parametre sözlüğü (gains.yaml); log: tek argümanlı yazdırma fonksiyonu (node logger).
    def __init__(self, p, log=print):
        self.p = p                      # eşikler gains.yaml'dan gelir
        self.log = log
        self.state = State.INIT
        self.fault = None               # arıza adı (str) ya da None
        self.t_enter = 0.0              # mevcut duruma giriş zamanı
        self.count = {}                 # her arıza için ardışık örnek sayacı (Q11)
        self.clean_since = None         # FAILSAFE içinde: arızasız geçen sürenin başlangıcı (Q13)
        self.ack_pending = False        # operatör onayı geldi mi (std_msgs/Empty)
        self.raw_active = False         # son adımda ham (debounce'suz) arıza koşulu var mıydı

    # _check_faults: s sözlüğünden ham koşulları değerlendirir, debounce uygular. İlk onaylanan arızayı döndürür.
    #   s = {age, finite, roll, pitch, z, err, sat_time}; yoksa (None) o denetim atlanır.
    def _check_faults(self, s):
        p = self.p
        raw = {}
        # Q5: odometri yaşı (duvar saatiyle ölçülür). Zaten bir süre eşiği → ek debounce yok.
        if s.get('age') is not None and s['age'] > p['fs_odom_timeout']:
            self.raw_active = True
            return 'odom_stale'
        # Q7: geçersiz sayı. Tek NaN bile PID integralini kalıcı bozar → N = fs_nan_n (1).
        raw['nan'] = not s.get('finite', True)
        if not raw['nan']:
            # Q8: devrilme |φ| veya |θ| > fs_tilt_max
            if s.get('roll') is not None:
                raw['tilt'] = max(abs(s['roll']), abs(s['pitch'])) > p['fs_tilt_max']
            # Q9: güvenli derinlik aralığı [z_min, z_max]
            if s.get('z') is not None:
                raw['depth_low'] = s['z'] < p['fs_z_min']
                raw['depth_high'] = s['z'] > p['fs_z_max']
            # Q10: takip hatası ve doyma (yalnız görev durumlarında)
            if self.state in MISSION_STATES:
                if s.get('err') is not None:
                    raw['track_error'] = s['err'] > p['fs_err_max']
                if s.get('sat_time') is not None:
                    raw['saturation'] = s['sat_time'] > p['fs_sat_time']
        # FAILSAFE çıkışı için: herhangi bir ham koşul şu an doğru mu (debounce'suz).
        self.raw_active = any(raw.values())
        # Q11: debounce — koşul N ardışık örnek doğruysa arıza onaylanır; bir kez yanlışsa sayaç sıfırlanır.
        found = None
        for name, active in raw.items():
            self.count[name] = self.count.get(name, 0) + 1 if active else 0
            if found is None and self.count[name] >= p[DEBOUNCE_KEYS[name]]:
                found = name
        return found

    # _set: durum geçişi. Her geçiş loglanır (eski -> yeni, neden).
    def _set(self, new, t, why=''):
        if new is self.state:
            return
        old = self.state
        self.state, self.t_enter = new, t
        if new is State.FAILSAFE:
            self.log(f"FAILSAFE: sebep={why}, t={t:.2f} s, önceki durum={old.name}, eylem={self.action()}", True)
        else:
            self.log(f"Durum: {old.name} -> {new.name} ({why}) t={t:.2f} s", False)

    # trip: dışarıdan (watchdog) zorla FAILSAFE. Watchdog kontrol döngüsü ölse bile çalıştığı için step'ten ayrıdır.
    def trip(self, fault, t):
        if self.state is not State.FAILSAFE:
            self.fault = fault
            self.clean_since = None
            self.ack_pending = False
            self._set(State.FAILSAFE, t, fault)

    # acknowledge: operatör onayı (/auv/failsafe_reset). Yalnız FAILSAFE içinde anlamlıdır.
    def acknowledge(self):
        if self.state is State.FAILSAFE:
            self.ack_pending = True

    # action: mevcut durumun komut modu.
    def action(self):
        if self.state is State.FAILSAFE:
            return FAULT_ACTION.get(self.fault, 'zero')
        if self.state is State.INIT:
            return 'zero'
        return 'normal'

    # advance: bu adımda yol referansı ilerlesin mi (controller.update(advance=...)).
    def advance(self):
        return ADVANCE.get(self.state, False)

    # step: her 20 Hz'de çağrılır. t: görev saati [s]; s: arıza girdileri; m: görev girdileri
    #   m = {ready: start_delay doldu mu, dz: |z*-z|, yaw_err: |e_ψ|, v: referans hızı, dwell_left, done}
    # Dönen: (state, komut_modu)  komut_modu: 'normal' | 'zero' | 'hold' | 'ascend'
    def step(self, t, s, m=None):
        p = self.p
        fault = self._check_faults(s)

        # --- FAILSAFE'e giriş: her durumdan, ilk onaylanan arızayla (Q13) ---
        if self.state is not State.FAILSAFE:
            if fault is not None:
                self.trip(fault, t)
                return self.state, self.action()
        else:
            # --- Kilitleme (latch) ve çıkış (Q13) ---
            # Arıza sürerken temiz sayaç sıfırlanır; operatör onayı da yok sayılır (önceden basılmış onay geçmez).
            if fault is not None or self.raw_active:
                self.clean_since = None
                if self.ack_pending:
                    self.log("FAILSAFE onayı reddedildi: arıza hâlâ var", True)
                self.ack_pending = False
            elif self.clean_since is None:
                self.clean_since = t
            clean_for = 0.0 if self.clean_since is None else t - self.clean_since
            if self.ack_pending:
                self.ack_pending = False
                if clean_for >= p['fs_clear_time']:
                    self.fault = None
                    self.count.clear()
                    self._set(State.INIT, t, f'operatör onayı, {clean_for:.1f} s arızasız')
                else:
                    self.log(f"FAILSAFE onayı reddedildi: arızasız süre {clean_for:.1f} s < "
                             f"{p['fs_clear_time']:.1f} s", True)
            return self.state, self.action()

        # --- Normal görev geçişleri (Q2/Q4 tablosu). Tabloda olmayan geçiş kodda yok. ---
        # Aynı adımda zincirleme geçiş olabilir (örn. DIVE→ALIGN→TRACK): eski kodda dalış bittiği adımda
        # referans da ilerliyordu; bunu korumak için durum oturana kadar (en çok 4 kez) değerlendirilir.
        if m is None:
            return self.state, self.action()
        aligned = m.get('yaw_err', 0.0) < p['yaw_tol']
        for _ in range(4):
            st = self.state
            if st is State.INIT:
                if m['ready']:
                    self._set(State.DIVE, t, 'start_delay doldu')
            elif st is State.DIVE:
                if m['dz'] < p['dive_tol']:
                    self._set(State.ALIGN, t, f"|z*-z| < {p['dive_tol']}")
            elif st is State.ALIGN:
                if aligned:
                    self._set(State.DWELL if m['dwell_left'] > 0.0 else State.TRACK, t, '|e_ψ| < yaw_tol')
            elif st is State.TRACK:
                if m['done']:
                    self._set(State.DONE, t, 'son durak + bekleme bitti')
                elif m['v'] <= 0.0 and m['dwell_left'] > 0.0:
                    self._set(State.DWELL if aligned else State.ALIGN, t, 'durağa varıldı')
                elif m['v'] <= 0.0 and not aligned:
                    self._set(State.ALIGN, t, 'durağa varıldı (dwell=0), yön hatalı')
            elif st is State.DWELL:
                if m['done']:
                    self._set(State.DONE, t, 'son durak + bekleme bitti')
                elif not aligned:
                    self._set(State.ALIGN, t, '|e_ψ| ≥ yaw_tol, bekleme durdu')
                elif m['dwell_left'] <= 0.0:
                    self._set(State.TRACK, t, 'bekleme bitti')
            if self.state is st:
                break
        return self.state, self.action()
