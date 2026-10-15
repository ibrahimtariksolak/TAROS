uint32 track_id
uint8 status                          # 0=TENTATIVE 1=CONFIRMED 2=COASTING 3=DELETED
string classification                 # COMMERCIAL_QUAD / COMMERCIAL_FIXEDWING / MILITARY / BIRD / UNKNOWN
float64 threat_score                  # 0.0 - 100.0

geometry_msgs/Point position          # ENU frame [m]
geometry_msgs/Vector3 velocity        # [m/s]
geometry_msgs/Vector3 acceleration    # [m/s²]

float64 speed                         # |velocity| [m/s]
float64 heading                       # [rad]
float64 altitude                      # AGL [m]
float64 range_to_base                 # korunan alana mesafe [m]
float64 time_to_arrival               # TTA [s]

float64[3] sensor_contributions       # [radar, eoir, acoustic] 0.0-1.0
float64 confidence                    # genel guvenilirlik 0.0-1.0

builtin_interfaces/Time first_seen
builtin_interfaces/Time last_update
uint32 update_count
