std_msgs/Header header
geometry_msgs/Point position   # tahmini konum ENU [m]
float64 confidence             # 0.0-1.0
float64 cep                    # dairesel hata olasiligi [m]
string method                  # TDOA / AOA / HYBRID
uint32 associated_track_id     # hangi drone'un operatoru
