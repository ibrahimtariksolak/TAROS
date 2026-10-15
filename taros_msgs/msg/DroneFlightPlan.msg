# Spawner -> pilot ucus plani devri.
# Spawner bir drone'u Gazebo'ya yerlestirdikten sonra bunu yayinlar;
# pilot abone olup drone'u ucurmaya baslar.
std_msgs/Header header
string drone_id                  # Gazebo entity adi
string drone_type                # quadrotor / fixedwing / hexarotor
geometry_msgs/Point spawn_position  # pilotun kinematik durumunu baslattigi poz
geometry_msgs/Point[] waypoints  # takip edilecek rota (ENU)
float64 speed                    # seyir hizi [m/s]
string behavior                  # TRANSIT / LOITER / ATTACK / RECON
