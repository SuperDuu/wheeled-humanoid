# FOC Diagnostic & Calibration Tools Suite

Thư mục tập hợp các công cụ Python phục vụ chẩn đoán, cân chỉnh bù góc, nhận diện thông số động cơ và quét đặc tính tần số cho driver FOC.

## Danh mục công cụ:
* **Nhận diện thông số động cơ:**
  * `auto_identify_motor.py`: Tự động nhận diện điện trở $R$, độ tự cảm $L$, và từ thông $\lambda$.
  * `foc_auto_tune.py`: Tính toán ma trận thông số PI vòng dòng và bộ lọc.
* **Cân chỉnh bù sai số phi tuyến Encoder (LUT Calibration):**
  * `calibrate_encoder_lut.py` & `calibrate_lut_smooth.py`: Đo và sinh bảng tra 128 điểm bù phi tuyến tính từ trường nam châm.
* **Đo kiểm đáp ứng tần số & Quét thông số (Bode / Chirp / Sweep):**
  * `chirp_sweep_bode.py`: Phát tín hiệu chirp quét đáp ứng tần số Bode của khớp.
  * `grid_search_tuner.py`: Tìm kiếm lưới tham số tối ưu.
* **Chẩn đoán chuyên sâu:**
  * `foc_diagnose.py`, `debug_root_cause.py`, `raw_debug.py`.
