# Kịch Bản Kiểm Thử Phần Cứng FOC (HIL Test Suite)

Thư mục chứa các kịch bản kiểm thử tự động phần cứng trong vòng lặp (Hardware-in-the-Loop - HIL) thông qua REST API của FOC Studio OSS (`http://127.0.0.1:1111`).

## 1. Danh sách kịch bản chính (`tests/hil/`)

| Script | Mục đích | Tiêu chuẩn đánh giá |
| :--- | :--- | :--- |
| `benchmark_robot_joint.py` | Kiểm thử toàn diện các chế độ (Torque, Speed, Position Trajectory) | Đo đồng bộ Target vs Actual, kiểm tra sai số xác lập |
| `test_position_repeatability_3trials.py` | Kiểm tra độ lặp lại vị trí (3 lần liên tiếp) | Sai số xác lập $\le 0.5^\circ$, không overshoot |
| `test_repeatability_3trials.py` | Kiểm tra độ lặp lại bám tốc độ (3 lần liên tiếp) | Sai số vận tốc $\le 5\text{ RPM}$, độ lệch chuẩn $\sigma \le 5\text{ RPM}$ |
| `test_speed_ab.py` | Thử nghiệm so sánh A/B các bộ thông số lọc và feedforward | Đánh giá độ êm và hiện tượng rung cộng hưởng 23 Hz |
| `test_align_verification.py` | Xác thực chu trình cân chỉnh cực động cơ (`ALIGN`) | Kiểm tra tính nhất quán góc zero và chiều encoder |

## 2. Cách chạy
Đảm bảo FOC Studio OSS đang chạy trên port 1111 (`./run_foc_studio_oss.sh`):
```bash
python3 tests/hil/test_position_repeatability_3trials.py
python3 tests/hil/test_repeatability_3trials.py
```
