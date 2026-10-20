# TAROS

**T**estbed for **A**utonomous **R**esponse on **ROS 2** — insansız hava aracı
savunma sistemleri (C-UAS) için modüler bir simülasyon test yatağı.

**Anahtar kelimeler:** counter-UAS, çoklu sensör füzyonu, çoklu hedef takibi,
genişletilmiş Kalman filtresi, hedef-silah ataması, ROS 2, Gazebo

---

## Özet

Düşük maliyetli insansız hava araçları, hava savunmasında hızla büyüyen bir
tehdit sınıfı oluşturmaktadır. Bu tehdide karşı geliştirilen algılama ve
müdahale sistemlerinin saha koşullarında denenmesi maliyetli ve kısıtlıdır.
TAROS, algılama–takip–sınıflandırma–karar–müdahale zincirinin tamamını
tekrarlanabilir senaryolar altında çalıştırmayı amaçlayan açık kaynaklı bir
simülasyon altyapısıdır.

Çalışmanın sınanabilir savı şudur: *radar, elektro-optik/kızılötesi ve akustik
ölçümlerin genişletilmiş Kalman filtresi (EKF) ile birleştirilmesi, tek
sensörlü duruma kıyasla takip doğruluğunu ve yanlış alarm oranını ölçülebilir
biçimde iyileştirir.* Altyapı, bu savı karşılaştırmalı deneylerle sınamak
üzere tasarlanmıştır.

### Kapsam ve amaç sınırı

Bu depo yalnızca bir simülasyon ve araştırma test yatağıdır; gerçek donanım
denetimi içermez ve herhangi bir fiziksel sisteme komut üretmez. Müdahale
zinciri tasarım gereği operatör onayına bağlıdır (*man-in-the-loop*): yetki
verilmeden hiçbir efektör devreye girmez. Bu kısıt, mimarinin bir yapılandırma
seçeneği değil, durum makinesinin değiştirilemez bir aşamasıdır.

---

## 1. Sistem mimarisi

İşlem hattı dört katmandan oluşur. Hedef üretim katmanı, takip başarımını
ölçmek için referans (*ground truth*) veri üretir; bu veri algılama
katmanından yalıtılmıştır.

```
Hedef üretimi
  scenario_manager -> drone_spawner -> drone_pilot
                                          |
                                          +--> referans iz verisi
                                          v
Algılama            Füzyon              Karar               Müdahale
  radar               EKF                 tehdit skoru        RF karıştırma
  EO/IR        -->    çoklu hedef  -->    niyet analizi -->   GPS aldatma
  akustik             takibi              hedef-silah         kinetik önleyici
                                          ataması
                                          |
                                          v
                              Komuta kontrol arayüzü
                              Muharebe sonrası analiz
```

### 1.1 Paket yapısı

| Paket | Sorumluluk | Durum |
|---|---|---|
| `taros_msgs` | Ortak mesaj, servis ve eylem tanımları | Uygulandı |
| `taros_gazebo` | Simülasyon dünyası, hedef modelleri, uçuş denetimi | Uygulandı |
| `taros_sensor` | Radar, EO/IR ve akustik sensör modelleri | Uygulandı |
| `taros_tracker` | EKF füzyonu, çoklu hedef takibi | Uygulandı |
| `taros_classifier` | Sınıflandırma, yörünge kestirimi, niyet analizi | Uygulandı |
| `taros_effector` | Elektronik harp ve kinetik önleme efektörleri | Planlandı |
| `taros_navigation` | Kara aracı özerk intikali, mevzi seçimi | Planlandı |
| `taros_iads` | Çoklu birlik eşgüdümü, kapsama eniyilemesi | Planlandı |
| `taros_sigint` | RF sinyal analizi, operatör konumlandırma | Planlandı |
| `taros_terrain` | Arazi analizi, radar görüş hattı, kör bölgeler | Planlandı |
| `taros_environment` | Hava koşulları, sensör başarım düşümü | Uygulandı |
| `taros_aar` | Kayıt, yeniden oynatma, başarım ölçütleri | Planlandı |
| `taros_c2` | Komuta kontrol arayüzü köprüsü | Planlandı |
| `taros_bringup` | Sistem genelinde başlatma yapılandırması | Planlandı |

---

## 2. Kurulum

Gereksinimler: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 3. Kullanım

```bash
ros2 launch taros_gazebo taros_sim.launch.py scenario:=swarm_saturate
```

### 3.1 Senaryolar

| Senaryo | Tanım | Hedef sayısı |
|---|---|---|
| `single_recon` | Tek döner kanatlı keşif uçuşu | 1 |
| `dual_probe` | İki ayrı eksenden eşzamanlı yaklaşma | 2 |
| `swarm_wave` | Tek eksenli dalga saldırısı | 10 |
| `swarm_saturate` | Çok eksenli doyurma saldırısı | 20 |
| `mixed_loiter` | Keşif, bekleme ve saldırı profillerinin bileşimi | 3 |

### 3.2 Başlatma parametreleri

| Parametre | Varsayılan | Tanım |
|---|---|---|
| `scenario` | `single_recon` | Çalıştırılacak tehdit senaryosu |
| `gui` | `true` | Gazebo arayüzü; `false` başsız çalıştırma |
| `spawn_delay` | `2.0` | Hedefler arası üretim gecikmesi [s] |
| `update_rate` | `30.0` | Uçuş denetim döngüsü frekansı [Hz] |
| `verbose` | `false` | Ayrıntılı Gazebo günlüğü |

### 3.3 Çıktının izlenmesi

```bash
ros2 topic echo /taros/ground_truth/tracks
ros2 topic hz   /taros/ground_truth/tracks

ros2 topic echo /taros/sensor/radar/returns
ros2 topic echo /taros/sensor/eoir/detections
ros2 topic echo /taros/sensor/acoustic/detections
ros2 topic echo /taros/environment/weather

ros2 topic echo /taros/tracker/tracks
ros2 topic hz   /taros/tracker/tracks
```

---

## 4. Yöntem

### 4.1 Hedeflerin kinematik modellenmesi

Hedef hava araçları tam aerodinamik modelle değil, kinematik olarak
sürülmektedir. Bu bilinçli bir modelleme kararıdır: çalışmanın doğrulama
nesnesi hedeflerin uçuş dinamiği değil, algılama ve takip zinciridir.
Hedeflere öngörülebilir ve tekrarlanabilir yörüngeler verilmesi, takip
hatasının kaynağının sensör ve füzyon katmanına yalıtılmasını sağlar.

Model tanımlarında `<gravity>false</gravity>` ve `<kinematic>true</kinematic>`
kullanılır. Aksi durumda fizik çözücü, komut edilen konuma karşı çalışarak
irtifa kaybına yol açmaktadır.

### 4.2 Durum bütünlüğü

Uçuş denetleyicisi konumu simülatörden geri okumaz; kendi durumunu yetkili
kaynak olarak tutar ve ileri entegre eder. Geri okumaya dayalı ilk tasarımda
yerçekimi sapması bir geri besleme döngüsüne girerek hedeflerin yavaşça
alçalmasına neden olmuştur (6 saniyede 0.4 m). İç durum entegrasyonuna
geçildiğinde sapma ölçüm çözünürlüğünde sıfırlanmıştır (bkz. Bölüm 5).

### 4.3 Görev planlarının devri

Hedef üretici, `transient_local` dayanıklılık politikasıyla
`taros/drone_flight_plan` konusunu yayımlar. Böylece uçuş denetleyicisi
üreticiden sonra başlatılsa bile mevcut görev planlarını alır; başlatma
sırası bir bağımlılık oluşturmaz.

### 4.4 Sensör modelleri

Üç bağımsız sensör aynı referans iz verisini tüketir; her biri kendi fizik
modeline göre tespit üretir veya üretmez.

**Radar.** X-band darbeli Doppler model. Radar denklemi SNR hesabı, Swerling-I
tespit olasılığı, menzil/açı gürültüsü (Cramér-Rao alt sınırına orantılı) ve
radyal hız (Doppler) ölçümü içerir. Yağmur zayıflaması ITU-R P.838
basitleştirilmiş modeliyle uygulanır.

**EO/IR.** Pan-tilt gimbal üzerinde dar görüşlü kamera. En yakın hedefe
otomatik kilit, görüş alanı dışı hedefleri tespit etmez. Yönelim tabanlı
(*bearing-only*) ölçüm üretir; menzil tahmini yalnızca 1 km altında verilir.
Görünürlük, yağmur ve gece koşulları etkili menzili düşürür.

**Akustik.** 4 elemanlı mikrofon dizisi. Küresel yayılım kaybı, rüzgâr gürültü
modeli, geliş yönü (DOA) kestirimi ve frekans imzası eşleştirmesi içerir.
Etkili menzil ~800 m ile sınırlıdır.

**Hava koşulları.** Tüm sensörlerin başarımı `taros_environment/weather_node`
tarafından yayımlanan meteorolojik koşullara bağlıdır. `SetWeather` servisi
ile senaryo içinde dinamik olarak değiştirilebilir.

### 4.5 Genişletilmiş Kalman filtresi füzyonu

Üç sensörün asenkron ölçümleri tek bir 6-durumlu (`x, y, z, vx, vy, vz`)
genişletilmiş Kalman filtresiyle birleştirilir. Radar ölçüm modeli dört
boyutludur (menzil, azimut, yükseliş, Doppler hızı); EO/IR ve akustik
modeller iki boyutludur (yalnızca yönelim). Her ölçüm modeli için analitik
Jacobian hesaplanır.

Süreç gürültüsü sabit beyaz gürültü ivme modeline dayanır (Bar-Shalom ve
ark., 2001). Bu model, kısa vadeli manevralarda yeterli adaptasyon sağlar;
uzun süreli yüksek manevra durumlarında IMM'e (Interactive Multiple Model)
geçiş planlanmaktadır.

### 4.6 Çoklu hedef takibi

Veri ilişkilendirmede Mahalanobis mesafesi ile ki-kare kapılaması
uygulanır; kapı dışı ölçümler eşleştirme dışında bırakılır. Kapılama
sonrası Macar algoritması (Kuhn, 1955) ile küresel en yakın komşu (GNN)
ataması yapılır.

İz yaşam döngüsü: bir ölçüm mevcut izlerden hiçbiriyle eşleşmezse geçici
(*tentative*) iz olarak başlatılır. Belirli sayıda (`M/N`) eşleşmeden sonra
onaylı (*confirmed*) duruma geçer. Eşleşme kesildiğinde izleme (*coasting*)
dönemine girer; zaman aşımında silinir.

### 4.7 Hedef yaşam döngüsü

Bir hedef, simülatör model listesinde en az bir kez doğrulanmadan imha
denetimine girmez. İlk tasarımda model listesi güncellemesi hedef üretiminin
birkaç milisaniye gerisinde kaldığı için, yeni üretilmiş hedefler imha
edilmiş sayılmaktaydı. Bu durum, çok hedefli senaryolarda hedeflerin önemli
bir bölümünün sessizce kaybına yol açan bir yarış koşuluydu.

---

## 5. Doğrulama

### 5.1 Ölçüm yöntemi

Ölçümler, çalışan diğer süreçlerden yalıtılmış bir ortamda alınmıştır: ayrı
`ROS_DOMAIN_ID`, ayrı Gazebo ana süreç portu, `ROS_LOCALHOST_ONLY=1` ve başsız
(`gui:=false`) çalıştırma. İz verisi doğrudan `taros/ground_truth/tracks`
konusundan örneklenmiş, konum farkları ölçüm penceresi süresine bölünerek
bağımsız hız kestirimi yapılmış ve bildirilen hızla karşılaştırılmıştır.

### 5.2 Sonuçlar

| Ölçüt | Sonuç |
|---|---|
| Eşzamanlı hedef üretimi | 20/20; erken kayıp yok |
| İz verisi yayın frekansı (20 hedef) | 30.2 Hz; hedef 30 Hz, başarım düşümü gözlenmedi |
| İleti başına hedef sayısı | 181 örnekte değişmez 20 |
| Durgun hedefte irtifa tutma | 8 s boyunca 0.00000 m sapma |
| Sınıf dağılımı (`swarm_saturate`) | 10 / 5 / 5; senaryo tanımıyla birebir |
| Çoklu profil (`mixed_loiter`) | 3/3 hedef; keşif, bekleme, saldırı eşzamanlı |
| Radar SNR azalışı (menzil) | 5x menzil artışında SNR monoton azalır |
| Yağmur: radar SNR düşümü | 20 mm/h yağışta ölçülebilir zayıflama |
| EO/IR menzil düşümü (sis) | 2 km görünürlükte %80 menzil kaybı |
| EO/IR menzil düşümü (gece) | %40 menzil kaybı |
| Akustik SNR düşümü (rüzgâr) | 15 m/s rüzgârda ölçülebilir gürültü artışı |
| DOA gürültüsü (düşük SNR) | Sonlu; sayısal taşma yok |
| EKF radar yakınsaması | 30 iterasyonda 50 m içine yakınsar |
| Üç sensör füzyonu: kovaryans | Tekli sensöre kıyasla kovaryans izi azalır |
| Kovaryans simetrisi (50 döngü) | Simetri hatası < 1e-6 |
| Macar algoritması eşleştirmesi | 2 iz / 2 ölçüm doğru eşlenir; kapı dışı reddedilir |
| İz yaşam döngüsü | TENTATIVE → CONFIRMED → COASTING → DELETED geçişleri doğrulandı |
| Niyet sınıflandırması | ATTACK, LOITER, RECON, TRANSIT doğru atanır |
| Tehdit skoru: yakın vs uzak | Yakın hedef yüksek skor alır |
| Tehdit seviyeleri | GREEN/YELLOW/ORANGE/RED sınırları doğrulandı |
| En yakın yaklaşma | Analitik çözüm; karşıdan gelen hedefte ~0 m |
| Birim ve biçim testleri | 178 test; hata ve başarısızlık yok |

### 5.3 Girdi doğrulama

`taros/spawn_drone` servisi harici girdi kabul ettiğinden,
[`validation.py`](taros_gazebo/taros_gazebo/validation.py) iki risk sınıfını
giriş noktasında kapatır:

1. **Dizin kaçışı.** `drone_type` alanı dosya yolu oluşturmada kullanılır.
   İzin listesi dışı bir değer, dosya sistemine yetkisiz erişime imkân
   verebilirdi.
2. **Tanımsız sayısal değer.** `NaN` ve sonsuz değerler sessizce simülatöre ve
   referans iz akışına yayılır; oradan da kestirim zincirine geri dönüşsüz
   biçimde bulaşır.

Doğrulama modülü ROS'tan bağımsız tutulmuştur; kurallar simülasyon
çalıştırılmadan sınanabilir.

```bash
colcon test --packages-select taros_gazebo
```

---

## 6. Geçerlilik sınırları

Çalışmanın sonuçları aşağıdaki kısıtlar çerçevesinde yorumlanmalıdır.

- **Hedef dinamiği.** Kinematik hedef modeli aerodinamik olarak geçerli
  değildir. Rüzgâr etkisi, manevra sınırları ve itki kısıtları
  modellenmemiştir; dolayısıyla manevra kabiliyeti yüksek hedeflere karşı
  takip başarımı üzerine çıkarım yapılamaz (Bölüm 4.1).
- **Füzyon karşılaştırması.** EKF füzyon katmanı uygulanmıştır ve birim
  testleri çoklu sensörün tekli sensöre kıyasla kovaryansı düşürdüğünü
  doğrulamaktadır. Ancak kapsamlı Monte Carlo karşılaştırması henüz
  yapılmamıştır; izleme doğruluğu ile yanlış alarm oranındaki sayısal
  iyileşme henüz raporlanmamıştır.
- **RF yayılımı.** Sinyal istihbaratı ve karıştırma katmanları, gerçek
  elektromanyetik yayılım çözümü içermeyecek; menzil ve engel etkileri
  basitleştirilmiş modellerle temsil edilecektir.
- **Fizik çözücü.** Gazebo Classic ODE çözücüsü 250 Hz güncelleme hızıyla
  yapılandırılmıştır. Yüksek hızlı çarpışma olaylarının çözünürlüğü bu adım
  boyutuyla sınırlıdır.
- **Ölçek.** Bildirilen başarım ölçümleri 20 eşzamanlı hedefe kadar
  geçerlidir; bu sınırın üzerindeki davranış sınanmamıştır.

---

## 7. Yol haritası

| Aşama | İçerik |
|---|---|
| 1 | Simülasyon altyapısı ve hedef üretimi; sensör modelleri; EKF füzyonu |
| 2 | Görsel sınıflandırma, yörünge kestirimi, niyet analizi, sinyal istihbaratı |
| 3 | Efektör modelleri, hedef-silah ataması, kara aracı intikali, arazi analizi |
| 4 | Çoklu birlik eşgüdümü, komuta kontrol arayüzü, muharebe sonrası analiz |

Birinci ve ikinci aşamalar büyük ölçüde tamamlanmıştır: hedef üretimi,
sensör modelleri, EKF füzyon/takip katmanı, sınıflandırma ve tehdit
değerlendirmesi. Sırada sinyal istihbaratı, efektör modelleri ve hedef-silah
ataması bulunmaktadır.

---

## 8. Kaynaklar

Aşağıdaki çalışmalar, planlanan katmanların yöntemsel temelini oluşturur.

1. Bar-Shalom, Y., Li, X. R., Kirubarajan, T. (2001). *Estimation with
   Applications to Tracking and Navigation: Theory, Algorithms and Software.*
   Wiley.
2. Blackman, S., Popoli, R. (1999). *Design and Analysis of Modern Tracking
   Systems.* Artech House.
3. Kuhn, H. W. (1955). The Hungarian method for the assignment problem.
   *Naval Research Logistics Quarterly*, 2(1–2), 83–97.
4. Manne, A. S. (1958). A target-assignment problem. *Operations Research*,
   6(3), 346–351.
5. Zarchan, P. *Tactical and Strategic Missile Guidance.* AIAA Progress in
   Astronautics and Aeronautics.
6. Koenig, N., Howard, A. (2004). Design and use paradigms for Gazebo, an
   open-source multi-robot simulator. *IEEE/RSJ International Conference on
   Intelligent Robots and Systems (IROS)*.
7. Macenski, S., Foote, T., Gerkey, B., Lalancette, C., Woodall, W. (2022).
   Robot Operating System 2: Design, architecture, and uses in the wild.
   *Science Robotics*, 7(66).

---

## 9. Atıf

```bibtex
@software{solak_taros,
  author  = {Solak, Ibrahim Tarik},
  title   = {{TAROS}: Testbed for Autonomous Response on {ROS 2}},
  year    = {2026},
  url     = {https://github.com/ibrahimtariksolak/TAROS},
  license = {MIT}
}
```

---

## Lisans

MIT. Ayrıntılar için [LICENSE](LICENSE) dosyasına bakınız.
