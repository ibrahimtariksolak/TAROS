std_msgs/Header header
uint32 detection_id
float64 azimuth             # hedef yonelimi [rad]
float64 elevation           # yukselis acisi [rad]
float64 pixel_x             # normalize goruntu x [0-1]
float64 pixel_y             # normalize goruntu y [0-1]
float64 bbox_width          # normalize sinirlayici kutu genisligi
float64 bbox_height         # normalize sinirlayici kutu yuksekligi
float64 estimated_range     # menzil tahmini [m], 0 = sadece yonelim
string classification       # gorsel siniflandirma
float64 confidence          # tespit guveni [0-1]
