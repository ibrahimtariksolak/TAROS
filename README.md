# TAROS — Anti-Drone Savunma Sistemi Simülatörü

ROS 2 tabanlı, modüler bir **C-UAS (Counter-Unmanned Aerial Systems)** test yatağı.
Çoklu sensör füzyonu, çoklu hedef takibi, tehdit değerlendirme ve müdahale
zincirinin tamamını Gazebo üzerinde simüle eder.

> **Kapsam notu:** Bu proje tamamen bir simülasyon ve araştırma test yatağıdır.
> Gerçek donanım kontrolü içermez. Müdahale zinciri tasarım gereği
> **man-in-the-loop**'tur: operatör yetkisi olmadan hiçbir efektör devreye girmez.

---

## Neden bu proje

Düşük maliyetli İHA'lar modern hava savunmasının en hızlı büyüyen tehdit
sınıfı. Buna karşı geliştirilen sistemlerin test edilmesi pahalı ve riskli.
TAROS, algılama → takip → sınıflandırma → karar → müdahale zincirini
tekrarlanabilir, ölçülebilir senaryolar altında çalıştırmayı amaçlıyor.

Projenin ölçülebilir iddiası: **EKF tabanlı çoklu sensör füzyonunun, tek
sensöre kıyasla takip doğruluğunu ve yanlış alarm oranını ne kadar
iyileştirdiğini sayısal olarak göstermek.**

---

## Sistem mimarisi

```
          ┌──────────────── Hedef üretimi ────────────────┐
          │  scenario_manager → drone_spawner → drone_pilot │
          └───────────────────────┬─────────────────────────┘
                                  │ ground truth (referans)
                                  ▼
  ┌─ Algılama ─┐    ┌─ Füzyon ─┐    ┌─ Karar ─┐    ┌─ Müdahale ─┐
  │ radar      │    │          │    │ tehdit  │    │ RF jammer  │
  │ EO/IR      ├───►│   EKF    ├───►│ skoru   ├───►│ GPS spoof  │
  │ akustik    │    │ + MTT    │    │ + WTA   │    │ interceptor│
  └────────────┘    └──────────┘    └─────────┘    └────────────┘
                                  │
                        ┌─────────┴─────────┐
                        │  C2 arayüzü / AAR │
                        └───────────────────┘
```

### Paketler

| Paket | Sorumluluk | Durum |
|---|---|---|
| `taros_msgs` | Ortak mesaj / servis / action tanımları | ✅ Hazır |
| `taros_gazebo` | Dünya, hedef modelleri, spawner, uçuş kontrolü | ✅ Hazır |
| `taros_sensor` | Radar, EO/IR, akustik sensör simülasyonu | 🚧 Planlandı |
| `taros_tracker` | EKF füzyonu, çoklu hedef takibi (MTT) | 🚧 Planlandı |
| `taros_classifier` | Sınıflandırma, yol tahmini, niyet analizi | 🚧 Planlandı |
| `taros_effector` | Soft kill (EW) + hard kill (interceptor) | 🚧 Planlandı |
| `taros_navigation` | UGV otonom intikal, mevzi seçimi | 🚧 Planlandı |
| `taros_iads` | Çoklu birlik koordinasyonu, kapsama optimizasyonu | 🚧 Planlandı |
| `taros_sigint` | RF analizi, operatör konumlandırma (TDOA) | 🚧 Planlandı |
| `taros_terrain` | Arazi analizi, radar görüş hattı, kör alanlar | 🚧 Planlandı |
| `taros_environment` | Hava koşulları, sensör performans degradasyonu | 🚧 Planlandı |
| `taros_aar` | Muharebe sonrası analiz, replay, metrikler | 🚧 Planlandı |
| `taros_c2` | Komuta kontrol arayüzü köprüsü | 🚧 Planlandı |
| `taros_bringup` | Sistem genelinde launch | 🚧 Planlandı |

---

## Kurulum

**Gereksinimler:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Çalıştırma

```bash
ros2 launch taros_gazebo taros_sim.launch.py scenario:=swarm_saturate
```

### Senaryolar

| Senaryo | Açıklama | Hedef sayısı |
|---|---|---|
| `single_recon` | Tek quadrotor keşif uçuşu | 1 |
| `dual_probe` | İki farklı yönden sızma | 2 |
| `swarm_wave` | Dalga saldırısı | 10 |
| `swarm_saturate` | Her yönden doyurma saldırısı | 20 |
| `mixed_loiter` | Keşif + loiter + saldırı karışık | 3 |

### Launch argümanları

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `scenario` | `single_recon` | Tehdit senaryosu |
| `gui` | `true` | Gazebo arayüzü (`false` = headless) |
| `spawn_delay` | `2.0` | Hedefler arası spawn gecikmesi [s] |
| `update_rate` | `30.0` | Uçuş kontrol döngüsü frekansı [Hz] |
| `verbose` | `false` | Gazebo ayrıntılı günlüğü |

### Çıktıyı izleme

```bash
# Aktif hedeflerin ground truth verisi
ros2 topic echo /taros/ground_truth/tracks

# Yayın frekansı
ros2 topic hz /taros/ground_truth/tracks
```

---

## Doğrulanmış davranış

Aşağıdaki ölçümler izole bir ROS domain'inde, headless Gazebo ile alındı:

| Ölçüm | Sonuç |
|---|---|
| 20 hedef eşzamanlı spawn | 20/20 başarılı, 0 erken düşme |
| Ground truth yayın frekansı (20 hedef) | 30.2 Hz (hedef 30 Hz, degradasyon yok) |
| Mesaj başına hedef sayısı | Sabit 20 |
| Durgun hedefte irtifa tutma | 8 s boyunca 0.00000 m sapma |
| Sınıf dağılımı (`swarm_saturate`) | 10 quad / 5 askeri / 5 fixedwing (senaryo tanımıyla birebir) |
| Karışık davranış (`mixed_loiter`) | 3/3 hedef, RECON + LOITER + ATTACK eşzamanlı |
| `colcon test` | 72 test, 0 hata, 0 başarısızlık |

---

## Girdi doğrulama

`taros/spawn_drone` servisi harici girdi kabul ettiği için
[`validation.py`](taros_gazebo/taros_gazebo/validation.py) kapıda iki risk
sınıfını kapatır:

1. **Yol kaçışı.** `drone_type` dosya yoluna giriyor; whitelist dışı bir
   değer (`../../etc/passwd` gibi) dosya sistemine erişim açardı.
2. **NaN / sonsuz sayısal değer.** Bunlar sessizce Gazebo'ya ve ground truth
   akışına sızar, oradan da takip/füzyon zincirini geri dönülemez şekilde
   zehirler.

Doğrulama modülü bilinçli olarak ROS'tan bağımsızdır, böylece kurallar
simülasyon ayakta olmadan test edilebilir:

```bash
colcon test --packages-select taros_gazebo
```

---

## Tasarım kararları

**Hedefler kinematik olarak sürülür.** Hedef drone'lar tam aerodinamik modelle
değil, `drone_pilot_node` tarafından teleport ile hareket ettirilir. Amaç
hedeflerin kendisini doğrulamak değil, algılama/takip zincirini beslemek.
Modeller `<gravity>false</gravity>` ve `<kinematic>true</kinematic>` taşır;
aksi halde fizik motoru komut edilen poza karşı çalışıp irtifa kaybettiriyordu.

**Pilot otoritatif iç durum tutar.** Pozisyon Gazebo'dan geri okunmaz. Geri
okuma denendiğinde yerçekimi sapması bir geri besleme döngüsüne girip
hedefleri yavaşça batırıyordu (6 saniyede 0.4 m). İç durum entegrasyonuna
geçildiğinde sapma tam olarak sıfırlandı.

**Uçuş planları latched topic ile devredilir.** Spawner `transient_local`
QoS ile `taros/drone_flight_plan` yayınlar, böylece pilot daha sonra başlasa
bile planları alır.

**Hedefler ancak bir kez görüldükten sonra düşürülebilir.** `model_states`
güncellemesi spawn'dan birkaç milisaniye geride kaldığı için, yeni doğmuş
hedefi anında imha edilmiş sayan bir race condition oluşuyordu. Artık bir
hedef `model_states`'te en az bir kez teyit edilene kadar imha kontrolüne
girmez.

**`gazebo.launch.py` yerine `gzserver`/`gzclient` doğrudan include edilir.**
`gazebo.launch.py` alt launch dosyalarına `launch_arguments` iletmez, bu
yüzden `world` gibi argümanların geçtiğine güvenmek kırılgandır.

**State plugin dünya dosyasında tanımlıdır.** `libgazebo_ros_state.so` bir
`WorldPlugin`'dir; `gzserver`'a `-s` ile verilirse `SystemPlugin` olarak
aranır ve sessizce yüklenmez. `/gazebo/model_states` ve
`/gazebo/set_entity_state` arayüzleri bu plugin'e bağlı.

---

## Yol haritası

- **Faz 1 — Temel ve algılama:** dünya, hedef üretimi, uçuş kontrolü ✅ · sensör simülasyonu 🚧 · EKF füzyonu 🚧
- **Faz 2 — Zeka:** görsel sınıflandırma, yol tahmini, niyet analizi, SIGINT
- **Faz 3 — Müdahale:** soft/hard kill, swarm'a karşı WTA, UGV intikali, arazi analizi
- **Faz 4 — Entegrasyon:** IADS koordinasyonu, C2 arayüzü, AAR ve metrikler

---

## Lisans

MIT — ayrıntılar için [LICENSE](LICENSE).
