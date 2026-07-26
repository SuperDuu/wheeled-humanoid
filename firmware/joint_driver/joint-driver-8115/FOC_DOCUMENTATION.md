# BÁO CÁO PHÂN TÍCH KỸ THUẬT VÀ LÝ THUYẾT ĐIỀU KHIỂN FOC VESC
## DÀNH CHO HỘP SỐ CYCLOID & KHỚP ROBOT HUMANOID (JOINT DRIVER 8115)

**Dự án:** Wheeled Humanoid Robot Joint Actuator Driver  
**Phần cứng:** STM32G473RET6 + DRV8353RS + AS5048A + Động cơ GB8115-4 (21 Pole Pairs) + Hộp số Cycloid 1:17  

---

## 1. PHÂN TÍCH ĐỘNG LỰC HỌC VÀ LÝ THUYẾT ĐIỀU KHIỂN (CONTROL THEORY & PHYSICAL DYNAMICS)

### 1.1 Phương trình vi phân trạng thái Động cơ PMSM trong Tọa độ $d-q$
Động cơ BLDC/PMSM nam ma sát mặt (Surface PMSM - GB8115-4) được mô tả toán học trong hệ tọa độ đồng bộ $d-q$ bằng hệ phương trình vi phân phi tuyến:

$$\begin{aligned}
\frac{di_d}{dt} &= -\frac{R_s}{L_d} i_d + \omega_e \frac{L_q}{L_d} i_q + \frac{1}{L_d} v_d \\
\frac{di_q}{dt} &= -\frac{R_s}{L_q} i_q - \omega_e \frac{L_d}{L_q} i_d - \omega_e \frac{\psi_m}{L_q} + \frac{1}{L_q} v_q
\end{aligned}$$

Trong đó:
* $R_s = 0.090\,\Omega$: Điện trở cuộn dây pha.
* $L_d = L_q = L_s = 0.000120\,\text{H}$ ($120\,\mu\text{H}$): Độ tự cảm pha.
* $\psi_m = 0.0045\,\text{Wb}$: Từ thông liên kết nam ma sát Neodymium.
* $\omega_e = P_{pairs} \cdot \omega_m = 21 \cdot \omega_m$: Vận tốc góc điện.

#### Phân tích thành phần chéo (Cross-coupling terms):
Thành phần $\omega_e \frac{L_q}{L_d} i_q$ và $-\omega_e \frac{L_d}{L_q} i_d - \omega_e \frac{\psi_m}{L_q}$ gây ra sự tương tác chéo giữa 2 trục $d$ và $q$. Khi tốc độ quay $\omega_e$ tăng cao:
1. Dòng $i_q$ tăng sẽ kéo theo suất điện động phản hồi dội ngược làm thay đổi điện áp $v_d$.
2. Sức điện động ngược $\omega_e \psi_m$ (Back-EMF) trên trục $q$ chống lại điện áp bơm $v_q$, làm sụt giảm khả năng điều khiển dòng momen.

Để triệt tiêu hiện tượng phi tuyến này, bộ điều khiển VESC áp dụng **Khử tương tác chéo tiền định (Feedforward Decoupling)** trong `foc_control.c`:

$$v_d^{final} = v_d^{PI} - \omega_e L_s i_q$$
$$v_q^{final} = v_q^{PI} + \omega_e L_s i_d + \omega_e \psi_m$$

---

### 1.2 Phân tích Đáp ứng Tần số & Thiết kế Bộ điều khiển PI Dòng điện (Pole-Zero Cancellation)
Mạch vòng dòng điện (Current Loop) là mạch vòng trong cùng, quyết định độ ổn định và đáp ứng động học của toàn bộ khớp robot.

Mô hình hàm truyền của cuộn dây động cơ (Plant):
$$G_{plant}(s) = \frac{I(s)}{V(s)} = \frac{1}{L_s s + R_s} = \frac{1/R_s}{\frac{L_s}{R_s} s + 1}$$

Hằng số thời gian cuộn dây:
$$\tau_m = \frac{L_s}{R_s} = \frac{0.000120}{0.090} = 1.333\,\text{ms}$$

Bộ điều khiển PI có dạng:
$$C_{PI}(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s} = K_p \frac{s + \frac{K_i}{K_p}}{s}$$

Áp dụng phương pháp **Triệt tiêu Cực-Không (Pole-Zero Cancellation)**, ta đặt điểm Zero của bộ PI triệt tiêu đúng điểm Pole của động cơ:
$$\frac{K_i}{K_p} = \frac{R_s}{L_s} \Rightarrow K_i = K_p \cdot \frac{R_s}{L_s}$$

Hàm truyền vòng kín của mạch vòng dòng điện trở thành hệ bậc 1 đơn giản:
$$G_{CL}(s) = \frac{C_{PI}(s) G_{plant}(s)}{1 + C_{PI}(s) G_{plant}(s)} = \frac{\frac{K_p}{L_s}}{s + \frac{K_p}{L_s}}$$

Tần số cắt (Bandwidth) của mạch vòng dòng điện $\omega_{bw} = \frac{K_p}{L_s}$ (rad/s). Cho dải thông mục tiêu $f_{bw} = 1000\,\text{Hz}$ ($\omega_{bw} = 2\pi \times 1000 = 6283.18\,\text{rad/s}$):
$$K_p = L_s \cdot \omega_{bw} = 0.000120 \times 6283.18 = 0.754$$
$$K_i = R_s \cdot \omega_{bw} = 0.090 \times 6283.18 = 565.48$$

*Trong thực tế triển khai trên VESC, để đảm bảo độ dự trữ ổn định (Phase Margin $> 60^\circ$) khi có sai số đo thông số motor, hệ số được chọn an toàn là $K_p = 0.25$, $K_i = 150.0$.*

---

### 1.3 Kỹ thuật Điều chế Vector Không gian SVPWM 6 Sectors & Khống chế Biên độ Điện áp Vòng tròn

#### Lý thuyết Vector Không gian:
8 trạng thái van đóng cắt của cầu H 3 pha tạo ra 6 vector điện áp tích cực ($V_1 \rightarrow V_6$) có biên độ $\frac{2}{3}V_{bus}$ và 2 vector không ($V_0, V_7$).

Vector điện áp tổng $V_s = V_\alpha + j V_\beta$ được tổng hợp theo quy tắc đòn bẩy thời gian trong 1 chu kỳ $T_s$:
$$V_s \cdot T_s = V_A \cdot t_1 + V_B \cdot t_2 + V_0 \cdot t_0$$

Biên độ điện áp cực đại không bị bão hòa sóng hài (Modulation Index $m \le 1.0$):
$$V_{max} = \frac{V_{bus}}{\sqrt{3}} \approx 0.57735 \cdot V_{bus}$$

#### Khống chế biên độ điện áp vòng tròn (Circle Limitation & Anti-Windup):
Khi bộ điều khiển PI yêu cầu tổng điện áp $\sqrt{v_d^2 + v_q^2} > V_{max}$, hệ thống rơi vào vùng bão hòa điện áp. VESC giải quyết bằng cách **Ưu tiên trục D (Field Weakening / Magnetizing Priority)**:

1. Kẹp cứng $v_d$: $|v_d| \le V_{max} \cdot \text{foc\_mag\_vd\_max}$ (với $\text{foc\_mag\_vd\_max} = 0.2$).
2. Điện áp còn lại dành cho trục Q (Momen):
   $$v_{q,max} = \sqrt{V_{max}^2 - v_d^2}$$
3. Kẹp cứng $v_q$ và tích phân $v_{q,int}$ theo $v_{q,max}$ để chống tràn tích phân (Integrator Windup).

---

## 2. KIẾN TRÚC FIRMWARE THỜI GIAN THỰC & LUỒNG DỮ LIỆU (REAL-TIME ARCHITECTURE)

### 2.1 Timeline Ngắt phần cứng @ 20kHz (High-Speed ISR Timing Diagram)
Vòng lặp ngắt FOC chạy ở tần số **20 kHz** (chu kỳ $50\,\mu s$), đồng bộ hóa tuyệt đối bằng phần cứng STM32G473:

```text
TIM1 Up-Down Counter:   0 ------> ARR (Top) ------> 0 (Bottom)
TIM1 TRGO Event:                    || Trigger ADC Conversion
ADC1/ADC2 Sample:                   ||--------> [4.0 MSPS Conversion: 0.25µs]
ADC End-of-Conversion:                         || Trigger Interrupt ISR
FOC_Control_Current_ISR():                     ||=================> [Execution: ~12µs]
TIM1 CCR Compare Update:                                           || Latch Duty Cycles
```

#### Quy trình xử lý trong 12 microgiây ($12\,\mu s$) ngắt:
1. **Lấy mẫu dòng ($0.25\,\mu s$):** ADC1 và ADC2 đọc đồng bộ dòng pha A và pha B tại điểm giữa của xung PWM (nơi nhiễu đóng cắt MOSFET bằng 0).
2. **Trừ Offset ($0.1\,\mu s$):** $i_\alpha = I_a - \text{offset}_a$, $i_\beta = (i_\alpha + 2(I_b - \text{offset}_b))/\sqrt{3}$.
3. **Đọc Encoder SPI ($1.5\,\mu s$):** Đọc góc 14-bit từ AS5048A qua SPI3 @ 5.31MHz, tính góc điện $\theta_e = (21 \cdot \theta_m) - \theta_0$.
4. **Biến đổi Park & PI Dòng ($3.0\,\mu s$):** Tính $i_d, i_q$, chạy 2 bộ PI dòng điện có anti-windup.
5. **Bù Decoupling & Circle Limit ($2.0\,\mu s$):** Tính $v_d, v_q$ đã bù BEMF và kẹp vòng tròn điện áp.
6. **Park ngược & SVPWM ($4.0\,\mu s$):** Tính $v_\alpha, v_\beta$, xác định Sector và nạp giá trị so sánh vào `TIM1->CCR1`, `CCR2`, `CCR3`.

---

### 2.2 Sơ đồ Liên kết Dữ liệu giữa các Module Code
Mô hình phần mềm được chia thành các lớp trừu tượng hóa rõ ràng (Clean Layered Architecture):

```mermaid
graph TD
    Main[main.c - Application Main Loop] -->|motor_set_position / motor_get_position| API[motor_interface.c/h]
    API -->|g_foc_controller handle| CTRL[foc_control.c/h - ISR & Safety Supervisor]
    CTRL -->|Calculates FOC Transforms| MATH[foc_math.c/h - Core VESC Algorithms]
    MATH -->|Fast Math & Filters| UTILS[vesc_utils.c/h & vesc_filter.c/h]
    CTRL -->|Config Parameters| CONF[vesc_conf.c/h & vesc_datatypes.h]
    CTRL -->|SPI Control| DRV[drv8353.c/h - Gate Driver]
    CTRL -->|SPI Angle Read| ENC[as5048a.c/h - Magnetic Encoder]
```

---

## 3. CƠ HỌC HỘP SỐ CYCLOID & XỬ LÝ ĐẶC THỤ (CYCLOIDAL GEARBOX KINEMATICS & PROTECTION)

### 3.1 Động học Hộp số Cycloid & Thuật toán Đếm vòng Đa vòng (Multi-turn Accumulator)
Hộp số Cycloid biến đổi chuyển động quay lệch tâm của đĩa Cycloid thành chuyển động quay đồng tâm đầu ra thông qua các con lăn (Pins/Rollers).

#### Phương trình Động học:
$$i = \frac{N_p - N_c}{N_c} = \frac{18 - 17}{17} = \frac{1}{17} \Rightarrow \text{Gear Ratio} = 17:1$$

Trong đó $N_p = 18$ là số chốt stator (Housing pins), $N_c = 17$ là số răng đĩa Cycloid.

#### Xử lý điểm tràn vạch biên Encoder 0 $\leftrightarrow$ $2\pi$:
Encoder AS5048A chỉ trả về góc cơ đơn vòng $\theta_m \in [0, 2\pi)$. Khi động cơ quay liên tục qua ranh giới $2\pi \rightarrow 0$ hoặc $0 \rightarrow 2\pi$, hàm `foc_update_cycloidal_joint_angle()` xử lý như sau:

$$\Delta\theta = \theta_m(k) - \theta_m(k-1)$$
$$\text{Nếu } \Delta\theta < -\pi \Rightarrow N_{turns} = N_{turns} + 1$$
$$\text{Nếu } \Delta\theta > +\pi \Rightarrow N_{turns} = N_{turns} - 1$$
$$\theta_{motor\_total} = N_{turns} \cdot 2\pi + \theta_m(k)$$
$$\theta_{joint\_output} = \frac{\theta_{motor\_total}}{17.0} \cdot \text{encoder\_direction}$$

---

### 3.2 Bộ điều khiển Soft Joint Limits ($\pm 180^\circ$) & Triệt tiêu Vọt áp Derivative Kick
Khớp quay robot dáng người có giới hạn chuyển động cơ học (ví dụ khớp gối, khớp háng). Để bảo vệ tuyệt đối:

1. **Soft Position Clamping:** Khi nhận lệnh vị trí đặt $\theta_{target}$, giá trị được kẹp cứng:
   $$\theta_{target}^{clamped} = \text{clamp}(\theta_{target}, -180^\circ, +180^\circ)$$
2. **Derivative Kick Elimination ($D$-on-Measurement):** Trong bộ PID Vị trí, nếu tính khâu $D$ trên tín hiệu sai số $e(t) = \theta_{target} - \theta_{joint}$, mỗi khi người dùng thay đổi $\theta_{target}$ đột ngột, đạo hàm $\frac{de}{dt} \rightarrow \infty$ sẽ tạo ra một cú sốc dòng điện (Derivative Kick) làm mẻ răng hộp số Cycloid.

VESC giải quyết bằng cách tính khâu $D$ trực tiếp trên góc biến đo $\theta_{joint}$:
$$D_{proc} = -K_{d,proc} \cdot \frac{d\theta_{joint}}{dt}$$
kết hợp với bộ lọc Low-pass Biquad giúp đáp ứng vị trí cực kỳ mịn màng, không vọt hằng số (Zero Overshoot).

---

### 3.3 Chuỗi Giám sát An toàn Khẩn cấp 5 Tầng (Safety Supervisor)

```mermaid
graph TD
    ISR[FOC_Control_Current_ISR @ 20kHz] --> Check1{I_mag > 25A?}
    Check1 -->|Yes| Trip1[MC_FAULT_OVER_CURRENT -> Trip PWM]
    Check1 -->|No| Check2{VBUS > 50V?}
    Check2 -->|Yes| Trip2[MC_FAULT_OVER_VOLTAGE -> Trip PWM]
    Check2 -->|No| Check3{VBUS < 12V?}
    Check3 -->|Yes| Trip3[MC_FAULT_UNDER_VOLTAGE -> Trip PWM]
    Check3 -->|No| Check4{Temp > 85C?}
    Check4 -->|Yes| Trip4[MC_FAULT_OVER_TEMP_MOS -> Derate/Trip]
    Check4 -->|No| Check5{Joint Angle out of +-180 deg?}
    Check5 -->|Yes| Trip5[MC_FAULT_POS_LIMIT -> Trip PWM]
    Check5 -->|No| Normal[Execute FOC Loop Normally]
```

---

## 4. BẢNG THÔNG SỐ VÀ DANH SÁCH MODULE CODE TRONG DỰ ÁN

### 4.1 Bảng Tham số Cấu hình Hệ thống (System Configuration Matrix)

| Tham số | Giá trị | Đơn vị | Ý nghĩa kỹ thuật |
|:---|:---|:---|:---|
| `foc_f_zv` | 20000.0 | Hz | Tần số phát xung PWM TIM1 |
| `foc_motor_pole_pairs` | 21 | - | Số cặp cực động cơ GB8115-4 |
| `foc_motor_r` | 0.090 | $\Omega$ | Điện trở pha cuộn dây ($90\,\text{m}\Omega$) |
| `foc_motor_l` | 0.000120 | H | Độ tự cảm pha cuộn dây ($120\,\mu\text{H}$) |
| `foc_motor_flux_linkage` | 0.0045 | Wb | Từ thông nam ma sát Neodymium |
| `gear_ratio` | 17.0 | - | Tỉ số truyền Hộp số Cycloid (17:1) |
| `joint_pos_min` | -3.14159 | rad | Giới hạn góc khớp mềm tối thiểu ($-180^\circ$) |
| `joint_pos_max` | +3.14159 | rad | Giới hạn góc khớp mềm tối đa ($+180^\circ$) |
| `foc_current_kp` | 0.25 | V/A | Hệ số Kp bộ điều khiển PI dòng điện |
| `foc_current_ki` | 150.0 | V/(A·s) | Hệ số Ki bộ điều khiển PI dòng điện |
| `p_pid_kp` | 15.0 | (A/rad) | Hệ số Kp bộ điều khiển Vị trí góc khớp |
| `p_pid_kd` | 0.03 | A/(rad/s) | Hệ số Kd sai số vị trí |
| `p_pid_kd_proc` | 0.02 | A/(rad/s) | Hệ số Kd trên biến đo vị trí ($D$-on-measurement) |
| `l_current_max` | 25.0 | A | Giới hạn dòng điện motor tối đa |
| `l_voltage_max` | 50.0 | V | Ngắt bảo vệ quá áp VBUS (OVP) |
| `l_voltage_min` | 12.0 | V | Ngắt bảo vệ áp thấp VBUS (UVP) |

---

### 4.2 Danh sách Module Code C trong Dự án (`joint-driver-8115`)

1. **[vesc_datatypes.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/vesc_datatypes.h):** Định nghĩa cấu trúc `motor_state_t` (66 trường trạng thái FOC) và các Enums điều khiển/mã lỗi.
2. **[vesc_conf.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/vesc_conf.h) / [vesc_conf.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/vesc_conf.c):** Cấu trúc `mc_configuration` và hàm nạp thông số mặc định GB8115-4 + Hộp số 1:17.
3. **[vesc_utils.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/vesc_utils.h) / [vesc_utils.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/vesc_utils.c):** Thư viện toán học số thực tối ưu (`utils_fast_atan2`, `utils_fast_sincos`, `saturate_vector_2d`, LP Filter).
4. **[vesc_filter.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/vesc_filter.h) / [vesc_filter.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/vesc_filter.c):** Bộ lọc số Biquad Direct Form II Filter.
5. **[foc_math.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/foc_math.h) / [foc_math.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/foc_math.c):** Thuật toán lõi VESC (`foc_observer_update`, `foc_pll_run`, `foc_svm` 6-Sector, PID Position/Speed, Field Weakening, Multi-turn Accumulator).
6. **[foc_control.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/foc_control.h) / [foc_control.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/foc_control.c):** Vòng lặp ngắt ngắt 20kHz `FOC_Control_Current_ISR()`, Decoupling, Circle Limitation, ADC Offset Calibration, Alignment.
7. **[motor_interface.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/motor_interface.h) / [motor_interface.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/motor_interface.c):** API giao diện cấp cao (`motor_init`, `motor_set_position`, `motor_get_position`, `motor_set_speed`, `motor_set_current`).
8. **[drv8353.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/drv8353.h) / [drv8353.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/drv8353.c):** Driver IC Lái cổng TI DRV8353RS qua SPI1 (CSA Gain 20V/V).
9. **[as5048a.h](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Inc/as5048a.h) / [as5048a.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/as5048a.c):** Driver Cảm biến góc 14-bit AS5048A qua SPI3 Mode 1 (Parity Check).
10. **[main.c](file:///home/du/Desktop/wheeled-humanoid/firmware/joint_driver/joint-driver-8115/Core/Src/main.c):** Tích hợp STM32 HAL Main & TIM1 PWM Driver.

---

## 5. QUY TRÌNH CALIBRATION & HƯỚNG DẪN DEBUG THỰC ĐỊA (FIELD GUIDE)

### 5.1 Quy trình 5 Bước Căn chỉnh Hệ thống (Commissioning Procedure)
1. **Bước 1: Kiểm tra phần cứng không tải (Static HW Check):**
   - Cấp nguồn $24\text{V}$ VBUS, kiểm tra dòng tĩnh bo mạch $< 100\text{mA}$.
   - Đọc thanh ghi DRV8353RS `FAULT_STATUS1` qua SPI1 để đảm bảo không có lỗi nạp cổng.
2. **Bước 2: Hiệu chuẩn Offset dòng ADC (ADC Calibration):**
   - Không phát xung PWM. Chạy `FOC_Control_AdcCalibrate()` lấy mẫu 2048 điểm. Giá trị mong muốn: $1.65\text{V} \pm 20\text{mV}$ (tương ứng $\approx 2048$ LSB).
3. **Bước 3: Căn góc 0 điện Encoder (Encoder Zero Alignment):**
   - Gọi `FOC_Control_AlignEncoder()`. Bơm $V_d = 2.0\text{V}, V_q = 0.0\text{V}$ trong $500\,\text{ms}$.
   - Đọc góc cơ AS5048A $\theta_{m\_align}$ và lưu góc offset điện $\theta_0 = (21 \cdot \theta_{m\_align}) \pmod{2\pi}$.
4. **Bước 4: Kiểm tra vòng lặp dòng điện (Open-loop Current Test):**
   - Đặt $I_q^* = 1.0\text{A}$, kiểm tra động cơ quay mịn màng, dòng pha là sóng sin chuẩn.
5. **Bước 5: Kiểm tra vòng lặp vị trí góc khớp (Closed-loop Position Test):**
   - Đặt lệnh vị trí `motor_set_position(45.0f)`. Kiểm tra khớp quay đến đúng $45.0^\circ$ và khóa cứng vị trí mà không bị rung giật.

---
*Báo cáo phân tích kỹ thuật này phản ánh đúng kiến trúc điều khiển FOC thực chiến được tích hợp trong dự án Joint Driver 8115.*
