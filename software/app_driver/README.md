# ⚡ STM32 FOC Telemetry Studio & Oscilloscope Visualizer

Ứng dụng hiển thị dao động ký (Oscilloscope) thời gian thực và chẩn đoán điều khiển FOC cho động cơ STM32G473, chạy trực tiếp trên hệ điều hành Ubuntu.

---

## 🌟 Tính Năng Nổi Bật

1. **Chọn Cổng USB Tự Động (`/dev/ttyUSB*`, `/dev/ttyACM*`)**:
   - Tự động quét và liệt kê tất cả các cổng USB-UART / ST-Link VCP.
   - Hỗ trợ chọn baudrate cao (**115,200**, **460,800**, **921,600**, **2,000,000** baud) để truyền stream 100Hz–500Hz không giật lag.

2. **Scope 1: Dạng Sóng 3 Pha Hình Sin Chuẩn MATLAB ($I_a, I_b, I_c$)**:
   - Hiển thị trực quan 3 pha dòng điện $I_a$ (Đỏ/Cam), $I_b$ (Xanh lục), $I_c$ (Xanh dương) với tốc độ 60 FPS Canvas.
   - Tự động tính toán dòng đỉnh ($I_{peak}$), tổng 3 pha ($I_a + I_b + I_c$) và kiểm tra độ lệch pha $120^\circ$ đối xứng.
   - Có nút chuyển đổi xem Dòng điện 3 pha (A) hoặc Duty PWM 3 pha (%).

3. **Scope 2: Dòng Vector Không Gian ($I_d, I_q, I_{q\_target}$)**:
   - Đánh giá khả năng bám dòng moment $I_q$ và kiểm tra dòng từ thông $I_d \approx 0\text{A}$.

4. **Scope 3: Góc Điện $\theta_e$ & Góc Cơ Học**:
   - Quan sát răng cưa góc điện $-\pi \to +\pi$ đồng bộ với dòng điện pha.

5. **Scope 4: Đáp Ứng Vận Tốc (RPM Thực Tế vs Mục Tiêu)**:
   - Theo dõi bám tốc độ, độ vọt lố (overshoot) và sai số RPM.

6. **Log Chẩn Đoán Định Kỳ (Throttled Log)**:
   - Chỉ in tóm tắt ngắn gọn 1 dòng mỗi 500ms (hoặc khi có lỗi) vào console và terminal, giúp theo dõi mà không làm ngập màn hình.

7. **Ghi & Xuất Dữ Liệu Ra File CSV cho MATLAB**:
   - Nút "Start Recording" ghi lại dữ liệu telemetry tốc độ cao và "Download CSV" để import vào MATLAB (`readmatrix`) hoặc Python (`pandas`).

8. **Bảng Điều Khiển Động Cơ Trực Tiếp**:
   - Chuyển chế độ: `IDLE (0)`, `CURRENT (1)`, `BRAKE (2)`, `SPEED (3)`, `POSITION (4)`.
   - Thanh trượt và nút bấm đặt tốc độ tức thời.
   - Nút **EMERGENCY STOP** dừng khẩn cấp.

---

## 🚀 Hướng Dẫn Chạy Ứng Dụng Trên Ubuntu

### Cách 1: Chạy bằng file script (Khuyên dùng)
Mở terminal tại thư mục dự án và chạy:
```bash
cd software/app_driver
./run_visualizer.sh
```
Trình duyệt web sẽ tự động mở trang giao diện tại: `http://localhost:8080`

### Cách 2: Chạy trực tiếp bằng Python
```bash
cd software/app_driver
python3 server.py 8080
```
Sau đó mở trình duyệt (Chrome/Firefox) truy cập `http://localhost:8080`.

---

## 🔌 Cấp Quyền Truy Cập Cổng USB trên Ubuntu

Nếu gặp lỗi **Permission Denied** khi mở `/dev/ttyUSB0` hoặc `/dev/ttyACM0`:
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*
```
*(Sau đó khởi động lại hoặc đăng xuất/đăng nhập lại để cập nhật quyền group dialout).*

---

## 📊 Phân Tích Dạng Sóng Chuẩn FOC Trên Scope

1. **Hình sin đẹp (Good FOC)**: 
   - 3 đường $I_a, I_b, I_c$ có dạng sóng sin tròn trịa, lệch pha nhau đúng $120^\circ$.
   - Tổng dòng tức thời $I_a + I_b + I_c \approx 0$.
   - Dòng $I_d$ dao động nhỏ quanh $0\text{A}$.
2. **Hình sin bị méo / răng cưa**:
   - Nếu dạng sóng bị nhọn đỉnh hoặc méo: Offset dòng ADC hoặc góc zero encoder chưa thật chuẩn.
   - Nếu $I_d$ lệch lớn khỏi 0A: Cần kiểm tra lại góc zero electric angle (`Run_EncoderAlignment`).
