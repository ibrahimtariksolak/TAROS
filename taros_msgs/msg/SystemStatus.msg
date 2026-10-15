std_msgs/Header header
string unit_id
bool operational
uint8 readiness_level        # 0=OFFLINE 1=STANDBY 2=ALERT 3=ENGAGED
uint32 interceptor_count     # kalan munisynon
float64[3] sensor_health     # [radar, eoir, acoustic] 0.0-1.0
geometry_msgs/Point position # birlik konumu ENU
float64 effective_range      # efektif menzil [m]
