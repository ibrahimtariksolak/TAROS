std_msgs/Header header
uint32 track_id
float64 threat_score         # 0-100
uint8 threat_level           # 0=GREEN 1=YELLOW 2=ORANGE 3=RED
string intent                # TRANSIT / LOITERING / RECONNAISSANCE / ATTACK / UNKNOWN
string drone_type            # siniflandirma sonucu
float64 time_to_arrival      # [s]
float64 closest_approach     # tahmin edilen en yakin yaklasma [m]
geometry_msgs/Point predicted_impact  # tahmin edilen hedef noktasi
