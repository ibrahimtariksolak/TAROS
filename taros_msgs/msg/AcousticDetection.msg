std_msgs/Header header
float64 doa_azimuth         # yön tahmini [rad]
float64 doa_elevation       # [rad]
float64[] frequency_spectrum  # FFT çıktısı [dB]
float64 dominant_frequency   # baskın frekans [Hz]
float64 confidence           # 0.0-1.0
string signature_match       # veritabanı eşleşmesi (ör. "DJI_MAVIC_3")
