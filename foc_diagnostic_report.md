# 📊 Báo Cáo Chẩn Đoán FOC Closed-Loop (Joint Driver 8115)
*Thời gian tạo:* `2026-08-18 10:27:24` | *Cổng kết nối:* `/dev/ttyACM0`

---

### 1. Kết Luận Nhanh (Executive Verdict)
🔴 **FAIL TẠI GÓC ENCODER (Sai Chiều Quay, Offset hoặc Thứ Tự Pha)**

---

### 2. Bảng Thông Số Đo Đạc Thực Tế (Telemetry Snapshot)

| Thông số | Giá trị đo được | Đánh giá / Trạng thái |
| :--- | :--- | :--- |
| **Điện áp Bus (VBUS)** | `23.16 V` | ✅ Bình thường (≥12V) |
| **Chiều quay Encoder (`encoder_dir`)** | `1` | ✅ Hợp lệ |
| **Zero Electrical Offset ($	heta_{offset}$)** | `1.3568 rad` (`77.74°`) | ✅ Đã căn chỉnh |
| **Test Voltage-Mode (`VQ 1.5V`)** | `0.57 RPM` | ❌ Không quay / Giật khục |
| **Test Current-Mode (`IQ 0.4A`)** | Sai số bám `9.981 A` | ❌ Lỗi đo dòng / Quá dòng |
| **Lỗi phần cứng (Fault Code)** | `0` | ✅ Không có lỗi |

---

### 3. Vấn Đề Được Phát Hiện (Detected Issues)
- ⚠️ **Ở chế độ VQ (Voltage-Mode), tốc độ động cơ gần như bằng 0 (0.57 RPM).**
- ⚠️ **Dòng điện đo về Iq BỊ NGƯỢC DẤU so với dòng mục tiêu (Target > 0 nhưng Measured < 0).**

---

### 4. Hướng Dẫn Khắc Phục Chính Xác (Action Items)
1. **Kiểm tra xem động cơ có bị kẹt cơ khí hộp số 1:17 hay Zero Offset bị lệch 90 độ (ép áp vào trục D thay vì trục Q).**
2. **Đổi dấu dòng trong main.c (dòng 1643-1644): Đổi `current_b = -((float)raw_ib ...)` thành `+`.**
