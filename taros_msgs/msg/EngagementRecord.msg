std_msgs/Header header
uint32 engagement_id
uint32 track_id
string unit_id
uint8 effector_type          # 0=RF_JAMMER 1=GPS_SPOOFER 2=KINETIC_INTERCEPTOR
uint8 result                 # 0=PENDING 1=SUCCESS 2=MISS 3=ABORTED
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
float64 sensor_to_shoot_time # tespitten atese gecen sure [s]
float64 intercept_range      # karsilasma mesafesi [m]
float64 miss_distance        # isabet mesafesi [m]
