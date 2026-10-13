<!-- =============================================================================
     DOSYA : README.md
     AMAÇ  : Paketin ne yaptığı, nasıl kurulup çalıştırıldığı, hangi dosyanın ne işe yaradığı ve
             ödevin çözümünde yapılan düzeltmeler. (Matematik: ilgili .py dosyalarının yorumlarında.)
     ============================================================================= -->
# sualti_sim_son

Gazebo'da fizikle yüzen takım aracı (55 x 48 x 20,5 cm): nötr yüzdürme, hidrodinamik sürtünme, 4 yatay itici (X dizilimi) + 2 dikey itici. PID ile derinlik tutma ve doğru / kare / daire sürüş.

## Kurulum (Ubuntu 22.04, ROS 2 Humble)
```bash
sudo apt install ros-humble-ros-gz python3-matplotlib
# Gazebo Harmonic kullanıyorsanız ros-humble-ros-gz yerine ros-humble-ros-gzharmonic
cd ~/ros2_ws/src && unzip sualti_sim_son.zip
cd ~/ros2_ws && colcon build --packages-select sualti_sim_son --symlink-install
source install/setup.bash
```

## Çalıştırma
```bash
ros2 launch sualti_sim_son sim.launch.py mission:=line      # 5 m dümdüz
ros2 launch sualti_sim_son sim.launch.py mission:=square    # 4 m kare
ros2 launch sualti_sim_son sim.launch.py mission:=circle    # r = 1 m daire
```
Araç önce 0,5 m dalar (`dive_depth`), sonra görevi yapar. Görev bitince `~/auv_<görev>.png` (yol ve derinlik grafiği) kaydedilir.
Gazebo sürümü otomatik seçilir; zorlamak için `gz:=fortress` veya `gz:=harmonic`.

## İlk kontrol
Araç geri giderse: `thrust_sign:=-1.0`.
Elle sürüş: `mission:=manual`, sonra
`ros2 topic pub -r 10 /auv/cmd_wrench geometry_msgs/msg/Wrench "{force: {x: 10.0}}"` (force.z yukarı/aşağı, torque.z dönüş)
Köşe iticileri 45° (X dizilimi) varsayıldı; gerçek aracınızda farklıysa `worlds/pool.sdf` içindeki itici açıları ve `allocation.py` birlikte değişmeli.

## Dosyalar
- `worlds/pool.sdf` havuz, araç, itici ve hidrodinamik eklentileri
- `config/gains.yaml` PID kazançları, hız, kare kenarı, daire yarıçapı
- `sualti_sim_son/pid.py` PID sınıfı ve açı sarma
- `sualti_sim_son/trajectories.py` yollar (doğru, kare, daire) ve trapez hızlı referans noktası
- `sualti_sim_son/controller.py` konum + yönelim + derinlik kontrolü
- `sualti_sim_son/allocation.py` istenen kuvvet/moment → 6 itici
- `sualti_sim_son/motion_node.py` ROS düğümü
- `docs/OGRENCI_ODEV_STATE_MACHINE_FAILSAFE.md` öğrenci ödevi: bu koda state machine + FAILSAFE ekleme

## Davranış
Yaw her görevde yola hizalanır (`yaw_mode: tangent`); köşede araç önce döner (`yaw_tol` içine girene kadar bekler), sonra ilerler.
Görev bitince grafik ayrı thread'de kaydedilir; düğüm kapanırken itki sıfırlanır.

## Ödevin çözümü: ne düzeltildi, neden
| Sorun | Çözüm (dosya) |
|---|---|
| Yengeç yürüyüşü: araç yan gidiyordu | yaw yolun teğetine kilitli, köşede önce dön sonra ilerle (`controller.py`) |
| Yola varınca atan2(0/0) / 0'a bölme | yön yoldan analitik üretilir; sıfır uzunluklu segment atılır, yarıçap/hız/ivme > 0 denetlenir (`trajectories.py`) |
| `dwell: 0` iken son durakta `IndexError` | bitiş koşulu yeniden yazıldı (`trajectories.py`) |
| Daire: `yaw_rate_max` < v/r ise araç yan yan gidiyordu (sway 0,43 m/s) | hız v ≤ 0,9·ω_max·r'ye kısılır (`controller.py`, sway 0,01 m/s) |
| NaN/inf komut Gazebo'yu çökertebilir | `_publish` girdiyi denetler, sonsuz/NaN ise itki 0 (`motion_node.py`) |
| Ctrl+C'de "itki = 0" gitmeyebiliyordu | sinyal işleyicisi kapatıldı, `stop()` bağlam açıkken çalışır (`motion_node.py`; ROS'ta test edilmedi) |
| Yanlış `mission`/`yaw_mode` yazımı gecikmeli çöküyordu | düğüm açılışında denetlenir (`motion_node.py`) |
| Görev bitince kayıt listesi büyüyordu | bitişten sonra kayıt durur (`motion_node.py`) |

Doğrulama: 3 serbestlik dereceli basit model (pool.sdf'teki kütle, eklenen kütle ve sürtünme katsayılarıyla) üzerinde line/square/circle tamamlanır, en büyük sway ≤ 0,05 m/s, iz hatası ≤ 0,12 m. Gazebo/ROS içinde çalıştırılmadı.

## Sonraki adımlar
State machine + FAILSAFE (öğrenci ödevi), IMU ve basınç sensörü, kamera.
