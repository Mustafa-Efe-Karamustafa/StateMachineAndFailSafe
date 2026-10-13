<!-- =============================================================================
     DOSYA : docs/OGRENCI_ODEV_STATE_MACHINE_FAILSAFE.md
     AMAÇ  : Öğrenciler sorulara kendi cevaplarını yazarak sualti_sim_son koduna bir STATE MACHINE (durum makinesi)
             ve FAILSAFE (arıza güvenliği) ekler. Cevap anahtarı yoktur: cevaplar sizin tasarımınızdır.
     ============================================================================= -->
# Ödev: Sualtı aracına State Machine + FAILSAFE ekle

**Kural:** Önce soruları cevapla (kağıt/bu dosya), sonra kod yaz. Kodu anlamadan değil, soruları cevaplayarak ilerle.
Her adımın sonunda simülasyonu çalıştır ve grafiği kaydet. Elle bitirilmiş çalışan Seviye 1, yarım kalmış Seviye 2'den iyidir.

## 0. Neden state machine?
Araç şu an tek bir sürekli kontrol döngüsü çalıştırıyor; "ne yapıyorum?" bilgisi kodda **dağınık bayraklar** olarak saklı. Arıza çıkınca (sensör kesildi, araç devrildi, zemine çarptı) kod bunu bilmiyor ve son komutu uygulamaya devam ediyor. **Gazebo son itki komutunu tutar**: kimse sıfır göndermezse itici dönmeye devam eder.

State machine = (durumlar S, olaylar/koşullar g, geçişler S_i --g--> S_j, her durumda yapılacak eylem a(S)). Her anda **tam bir** durum aktiftir.

**Mini örnek (sizin çözümünüz değil):** iticiyi arm/disarm eden makine.
```python
from enum import Enum
class Arm(Enum):
    DISARMED = 0
    ARMED = 1
# geçiş tablosu: durum -> [(koşul fonksiyonu, yeni durum), ...]  (ilk doğru koşul kazanır)
GECIS = {
    Arm.DISARMED: [(lambda v: v['arm_istegi'] and v['pil'] > 11.0, Arm.ARMED)],
    Arm.ARMED:    [(lambda v: not v['arm_istegi'] or v['pil'] < 10.0, Arm.DISARMED)],
}
def adim(durum, v):
    for kosul, yeni in GECIS[durum]:
        if kosul(v):
            return yeni
    return durum            # koşul yoksa durumda kal
```
Dikkat: pil için giriş 11.0 V, çıkış 10.0 V. Aradaki 1 V = **histerezis** (titreşen geçişi önler).

## 1. Kod haritası (hepsini oku, yalnız * olanlara dokunacaksın)
| Dosya | Ne yapar | Dokun? |
|---|---|---|
| `motion_node.py` | odom oku → kontrol → itki yaz (`_odom_cb`, `_loop`, `_publish`, `stop`) | * |
| `controller.py` | PID'ler; örtük durumlar: `diving`, `aligned`, `gen.dwell_left`, `done` | okunacak |
| `trajectories.py` | referans noktası, duraklar | okunacak |
| `allocation.py`, `pid.py` | itki dağıtımı, PID | dokunma |
| `config/gains.yaml` | eşikler buraya | * |
| `sualti_sim_son/state_machine.py` | **yeni dosya, sen yazacaksın** | * |

## 2. Seviye 1: Mevcut koddaki örtük durumları bul (≈30 dk)
**Q1.** `controller.py` ve `motion_node.py` içinde "araç şu an hangi aşamada" bilgisini taşıyan her değişkeni listele (en az 5). Her biri için: hangi değerler hangi aşamayı gösterir?
> Cevabın:

**Q2.** Bu aşamalardan hangisinden hangisine geçilir? Geçiş koşulunu **matematiksel** yaz (örnek biçim: `|z* - z| < 0.05`). Kodda koşulun yazıldığı satırı bul.
> Cevabın (tablo: kaynak durum → koşul → hedef durum):

**Q3.** Karenin 3. köşesinde araç hangi sırayla hangi aşamalardan geçer? Zaman ekseni çiz: dönüş süresi `Δψ/ω_max = (π/2)/0.5 =` ? s, bekleme `dwell` = ? s. Simülasyon grafiğiyle karşılaştır.
> Cevabın:

**Q4.** Şu an aşama tek bir yerde değil, `if` zincirlerinde dağınık. Bunu tek bir `state` değişkeninde toplarsan hangi 6-8 durumu tanımlarsın? (Öneri başlangıcı: INIT, DIVE, ALIGN, TRACK, DWELL, DONE.) Her durumda **yapılan eylemi** yaz: hangi PID aktif, referans ilerliyor mu?
> Cevabın:

## 3. Seviye 1: FAILSAFE tasarımı (≈60 dk) ← asıl ödev
Önce **ne bozulabilir** sorusu. Aşağıdaki arızaların her biri için bir **algılama koşulu** ve bir **eşik** seç, nedenini yaz.

**Q5. Odometri bayatladı.** `age = t_şimdi - t_son_odom`. `_odom_cb` şu an zaman damgası saklıyor mu? Eşik T_stale için denge: çok küçük → yanlış alarm; çok büyük → kör gidiş mesafesi `v·T_stale = 0.4·T_stale`. Odometri 50 Hz, kontrol 20 Hz: T_stale en az kaç periyot olmalı? Seçimin ve gerekçen:
> Cevabın:

**Q6. Saat tuzağı.** Düğüm `use_sim_time` kullanıyor. Köprü ölürse `/clock` da durur. O zaman 20 Hz zamanlayıcıya ne olur? Watchdog'un (bekçi) kendisi hangi saatle ve hangi zamanlayıcıyla çalışmalı? Öte yandan Gazebo kullanıcı tarafından **duraklatılırsa** duvar saatli watchdog ne yapar; bu istenen davranış mı? İpucu: `rclpy.clock.Clock`, `ClockType.SYSTEM_TIME`, `create_timer(..., clock=...)`.
> Cevabın:

**Q7. Geçersiz sayı.** Hangi hesapta NaN/inf doğabilir? (`dt`, `atan2`, bölme.) `_publish` bunu şu an nasıl karşılıyor, bu yeterli mi, neden bir de odometri girişini denetliyoruz?
> Cevabın:

**Q8. Devrilme.** Odometri kuaterniyonu `q = (w, x, y, z)`. Roll ve pitch:
`φ = atan2(2(w·x + y·z), 1 - 2(x² + y²))`,  `θ = asin(2(w·y - z·x))`.
(a) `asin` argümanı sayısal hatayla 1.0000001 olursa ne olur? Nasıl önlersin (kırpma)? (b) Aracın geri getirme momenti `M ≈ m·g·h·sinφ` (pool.sdf yorumuna bak): hangi açıdan sonra devrilmiş sayarsın? Eşik ve gerekçe.
> Cevabın:

**Q9. Derinlik sınırı.** Havuz zemini z = -3.0 m, yüzey z = 0, araç z = -1.5'te başlıyor, `dive_depth` 0.5. Güvenli aralık `[z_min, z_max]` nedir? Araç yarı yüksekliği 0.1 m ve hızlı inişte durma mesafesi `w²/(2a)` payı eklenmeli mi? Hesapla.
> Cevabın:

**Q10. Takip hatası ve doyma.** Referans ile araç arası `e = ‖p_ref - p‖`. Normal çalışmada simülasyondaki en büyük e nedir (grafikten ölç)? Eşik `e_max` bunun kaç katı olmalı? İtkiler `T` saniye boyunca sürekli `|F| = F_max` ise ne anlama gelir (araç sıkışmış / sürükleniyor)? T'yi nasıl seçersin?
> Cevabın:

**Q11. Yanlış alarm.** Bir koşulun tek örnekte doğru olması yeterli mi? Gürültüde tek örnek yanlış alarm olasılığı `p` ise `N` ardışık örnekte `pᴺ`. Her arıza için "kaç ardışık örnek / kaç saniye" (debounce) seç. Bedeli: algılama gecikmesi `N·0.05 s`.
> Cevabın:

**Q12. Arıza eylemi (en önemli fizik sorusu).** Arıza olunca ne yapılacak? Seçenekler: (A) tüm itki = 0, (B) olduğun derinlikte tut, (C) açık çevrim yukarı çık. Araç **nötr yüzer**:
- (A) için: `m·v̇ = -(d₁ + d₂|v|)·v`. v₀ = 0.4 m/s'den durma süresini ve mesafeyi doğrusal sürtünmeyle (`τ = m/d₁`, `x = v₀·τ`) ve ikinci dereceden terimle ayrı ayrı hesapla. Simülasyonla karşılaştır.
- (C) için: sabit `F_yukarı` ile terminal hız `F = d₁w + d₂w²` denkleminden w'yi çöz (heave: d₁ = 10, d₂ = 70). 1.5 m çıkış süresi?
Hangi arıza için hangi eylem? (Örn. odometri yokken kapalı çevrim derinlik tutulabilir mi?) Tablo yap.
> Cevabın:

**Q13. Kilitleme (latch) ve çıkış.** FAILSAFE'den otomatik çıkılmalı mı? Arıza geçince hemen DIVE'a dönmek neden tehlikeli olabilir? "Arıza T_temiz s boyunca yok + operatör onayı" gibi bir kural yaz. Onay `std_msgs/Empty` konusu ile mi, parametre ile mi gelir? Neden?
> Cevabın:

**Q14. Düğüm ölürse.** `motion_node` çökerse kimse FAILSAFE yürütemez ve Gazebo son itkiyi tutar. `main()` içinde `finally: node.stop()` bunu nereye kadar çözüyor? Gerçek araçta bunu çözen **bağımsız** (aracın üstündeki) mekanizma ne olurdu? (Kısa tasarım notu yeterli.)
> Cevabın:

## 4. Seviye 1: Kodlama (≈60 dk)
`sualti_sim_son/state_machine.py` dosyasını oluştur. İskelet (TODO'ları Q cevaplarınla doldur):
```python
from enum import Enum

class State(Enum):
    INIT = 0
    DIVE = 1
    TRACK = 2
    DONE = 3
    FAILSAFE = 4          # TODO(Q4): ALIGN, DWELL gibi durumları Seviye 2'de ekle

class Supervisor:
    def __init__(self, p):
        self.p = p                      # eşikler gains.yaml'dan gelir
        self.state = State.INIT
        self.fault = None               # arıza adı (str) ya da None
        self.t_enter = 0.0              # mevcut duruma giriş zamanı
        self.count = {}                 # her arıza için ardışık örnek sayacı (Q11)

    def _check_faults(self, s):
        """s: sözlük {age, finite, roll, pitch, z, err, sat_time ...}. Arıza adı döndür ya da None."""
        # TODO(Q5,Q7,Q8,Q9,Q10): her koşulu yaz; Q11'deki debounce'u uygula
        return None

    def _set(self, new, t):
        # TODO(Q13): geçişte log yaz (eski -> yeni, neden); self.t_enter = t
        self.state, self.t_enter = new, t

    def step(self, t, s):
        """Her 20 Hz'de çağrılır. Dönen: (state, komut_modu) komut_modu: 'normal' | 'zero' | 'ascend' ..."""
        fault = self._check_faults(s)
        # TODO(Q13): FAILSAFE'e giriş (her durumdan) ve latch kuralı
        # TODO(Q2,Q4): diğer geçişler: bir tablo/dict ya da if-zinciri; her geçişin koşulu Q2 tablosundan
        # TODO(Q12): FAILSAFE içinde hangi komut modu?
        return self.state, 'normal'
```
Bağlama noktaları (**hepsi `motion_node.py` içinde**, önce kendin bul):
1. `_odom_cb`: zaman damgasını sakla (Q5).
2. `_loop`: `dt` hesaplandıktan sonra `Supervisor.step` çağır; durum `FAILSAFE` ise `ctrl.update` sonucunu **kullanma**, Q12'deki eylemi uygula. Arıza tespiti için `age` ve poz denetimi `_loop` başındaki `return`'lerden **önce** olmalı (neden? Q6).
3. `gains.yaml`: tüm eşikleri buraya taşı (kod içinde sihirli sayı yok).

## 5. Seviye 2: Aşamaları state machine'e taşı (süre kalırsa)
`controller.py` içindeki `diving`, `aligned`, `gen.dwell_left` mantığını `Supervisor` durumlarına taşı: DIVE → ALIGN → TRACK → DWELL → ALIGN ... → DONE. Kural: **her durumun giriş koşulu, çıkış koşulu ve eylemi** Q2/Q4 tablosunda yazılı olmalı; tabloda olmayan geçiş kodda olmayacak. Taşıdıktan sonra kare görevinin grafiği **öncekiyle aynı** olmalı (regresyon testi).

## 6. Test senaryoları (hepsi için durum-zaman grafiği çiz)
| # | Arıza nasıl oluşturulur | Beklenen durum dizisi | Geçti ölçütü |
|---|---|---|---|
| T1 | arıza yok, `mission:=square` | INIT→DIVE→TRACK→DONE | grafik eski koddan sapmıyor, FAILSAFE'e girmiyor |
| T2 | görev sırasında başka terminalde `pkill -f parameter_bridge` | TRACK→FAILSAFE | tespit süresi ≈ T_stale; itki sıfırlanıyor (Q6 yüzünden watchdog yalnız duvar saatiyle çalışır) |
| T3 | `gains.yaml`: `dive_depth: 2.8` | DIVE→FAILSAFE (derinlik) | araç zemine çarpmadan durur/çıkar |
| T4 | parametre `inject_fault: nan_odom` (kendin ekle) | →FAILSAFE (geçersiz sayı) | Gazebo çökmüyor |
| T5 | `max_thrust: 3.0` (kontrol yetmesin) | →FAILSAFE (takip/doyma) | Q10 eşiğine göre |
| T6 | FAILSAFE'den sonra arıza giderildi | FAILSAFE'te kal, onayla çık | Q13 kuralı |
Her test için: log satırı ("FAILSAFE: sebep=..., t=...") ve grafik.

## 7. Teslim
1. Doldurulmuş bu dosya (Q1-Q14 cevapları, durum diyagramı, geçiş tablosu).
2. `state_machine.py` + değişen `motion_node.py` + `gains.yaml`.
3. T1-T6 grafikleri ve logları.
4. Yarım sayfa: "Hangi arızayı hâlâ yakalayamıyoruz ve gerçek araçta ne eklerdik?"
