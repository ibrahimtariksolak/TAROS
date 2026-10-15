std_msgs/Header header
uint32 track_id
uint8 effector_type          # 0=RF_JAMMER 1=GPS_SPOOFER 2=KINETIC_INTERCEPTOR
uint8 authorization          # 0=PENDING 1=AUTHORIZED 2=DENIED
uint8 priority               # 1(en yuksek) - 10(en dusuk)
string unit_id               # hangi AEGIS birligi
