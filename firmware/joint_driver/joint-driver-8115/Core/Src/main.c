/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_control.h"
#include "motor_interface.h"
#include "comm_can.h"
#include "comm_telemetry.h"
#include "encoder_cal_store.h"
#include "vesc_utils.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ================================================================
 * ADC Regular Channel Definitions (từ file .ioc)
 * Tất cả đều nằm trên ADC1, đọc bằng polling trong slow loop
 * ================================================================ */

/* Tỷ lệ bộ chia áp VBUS trên PCB: Vbus_real = Vadc × VBUS_DIVIDER_RATIO
 * Đã hiệu chỉnh: cấp 24.6V thực tế, ratio cũ 12.915 đọc ra 11.85V
 * → ratio đúng = 12.915 × (24.6 / 11.85) ≈ 26.81
 * TODO: Đo chính xác hơn bằng nguồn chuẩn nếu cần */
#define VBUS_DIVIDER_RATIO    26.81f

/* Hệ số chuyển đổi NTC thermistor cho FET_TEMP
 * TODO: SỬA THEO MẠCH THẬT (phụ thuộc NTC + mạch phân áp) */
#define FET_TEMP_OFFSET       0.0f
#define FET_TEMP_SCALE        1.0f   /* °C per ADC volt, placeholder */

/* Tỷ lệ bộ chia áp VSENSE pha A/B/C (Back-EMF sensing)
 * TODO: SỬA THEO SCHEMATIC THẬT */
#define VSENSE_DIVIDER_RATIO  12.915f

/* ADC Reference voltage và resolution */
#define ADC_VREF              3.3f
#define ADC_MAX_VALUE         4096.0f
#define ADC_TO_VOLT           (ADC_VREF / ADC_MAX_VALUE)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float current_joint_deg = 0.0f;
float current_joint_rpm = 0.0f;
float current_iq_amps = 0.0f;
float current_vbus = 0.0f;

/* Slow loop is scheduled from measured ISR elapsed time, not an assumed IRQ rate. */
static float slow_loop_elapsed_s = 0.0f;

/* ================================================================
 * Kết quả đọc ADC Regular Channels + DRV8353 SPI Status
 * ================================================================ */


volatile ADC_Readings_t g_adc_readings = {
    .vbus = 24.0f,
    .fet_temp = 25.0f
};

/*
 * Hằng số chuyển đổi ADC → Ampere
 * Công thức: ADC_TO_AMPS = (V_ref / ADC_resolution) / (R_shunt × CSA_Gain)
 *
 * TODO: ĐIỀU CHỈNH THEO GIÁ TRỊ R_SHUNT VÀ CSA GAIN THẬT TRÊN PCB CỦA BẠN
 *       R_shunt = 1mΩ (0.001Ω), CSA_Gain = 20V/V → ADC_TO_AMPS ≈ 40.28
 *       R_shunt = 0.5mΩ, CSA_Gain = 20V/V → ADC_TO_AMPS ≈ 80.57
 *       R_shunt = 2mΩ, CSA_Gain = 20V/V → ADC_TO_AMPS ≈ 20.14
 */
#define CURRENT_SENSE_R_SHUNT 0.002f /* Ohm - SỬA THEO PCB THẬT (R002) */
#define CURRENT_SENSE_CSA_GAIN 20.0f /* V/V - SỬA THEO PCB THẬT */
#define ADC_TO_AMPS                                                            \
  ((3.3f / 4096.0f) / (CURRENT_SENSE_R_SHUNT * CURRENT_SENSE_CSA_GAIN))

/* Low-side shunt current must be sampled inside the all-low-side-on window.
 * PWM mode 1 + center-aligned counter gives that window near CNT ~= ARR. */
#define ADC_INJECTED_SAMPLE_TICKS_FROM_TOP 300U
#define CURRENT_IDLE_OFFSET_ALPHA 0.001f
#define CURRENT_RUN_FILTER_ALPHA 0.25f
#define CURRENT_DEADBAND_A 0.04f

/* Biến phục vụ test chiều quay động cơ và encoder */
volatile int run_direction_test = 0;  /* Set = 1 trong Live Expressions để chạy */
volatile int test_result = 0;         /* 0: Idle, 1: Đang test, 2: OK, 3: Ngược chiều, -1: Lỗi */

/* Cấu trúc lưu log nhích động cơ từng bước */
typedef struct {
  float angle;
  uint32_t ta;
  uint32_t tb;
  uint32_t tc;
  uint16_t raw_angle;
  uint16_t ccr1;      /* Giá trị thực tế TIM1->CCR1 sau khi ghi */
  uint16_t ccr2;      /* Giá trị thực tế TIM1->CCR2 sau khi ghi */
  uint16_t ccr3;      /* Giá trị thực tế TIM1->CCR3 sau khi ghi */
  uint32_t isr_count; /* Số lần ADC ISR đã chạy tại thời điểm log */
} Step_Debug_t;

/* Cấu trúc debug cho bài test chiều quay */
typedef struct {
  float start_angle;
  float end_angle;
  float diff_angle;
  uint16_t raw_start;
  uint16_t raw_end;
  float vbus;
  uint8_t moe;
  uint8_t break_flag;
  uint8_t step_reached;
  uint8_t calibrated;   /* ADC offset calibration hoàn tất? */
  uint16_t ccer;
  uint16_t cnt;         /* TIM1->CNT snapshot - timer có đang đếm? */
  uint16_t bdtr;        /* TIM1->BDTR snapshot */
  uint16_t drv_reg_00;
  uint16_t drv_reg_01;
  uint16_t drv_reg_02;
  uint16_t drv_reg_03;
  uint16_t drv_reg_04;
  uint16_t drv_reg_05;
  uint16_t drv_reg_06;
  uint32_t isr_at_start;  /* ADC ISR count khi bắt đầu test */
  uint32_t isr_at_end;    /* ADC ISR count khi kết thúc test */
  Step_Debug_t steps[10]; /* Mảng lưu log 10 lần trong vòng lặp test */
} Debug_Test_t;

volatile Debug_Test_t g_dbg_test = {0};
volatile uint32_t g_adc_isr_counter = 0; /* Đếm số lần ADC ISR callback chạy */
volatile float g_foc_isr_dt_s = 0.000100f;
volatile uint32_t g_foc_isr_missed_periods = 0;
volatile uint32_t g_adc_calib_wait_ms = 0;
volatile uint8_t g_adc_calib_timeout = 0;

/* ===== ENCODER ALIGNMENT & CALIBRATION DEBUG ===== */
volatile int run_alignment = 0;   /* Set = 1 trong Live Expressions để chạy alignment */
volatile int run_calibration = 0; /* Set = 1 để chạy 1-turn Ben Katz 128-point LUT calibration */
volatile int align_result = 0;    /* 0: Idle, 1: Đang chạy, 2: OK, -1: Lỗi */
volatile int8_t g_encoder_calibration_result = 0;

typedef struct {
  float zero_electric_angle;  /* Góc điện offset (rad) */
  float encoder_rad;          /* Góc encoder tại thời điểm lock (rad) */
  float vbus;                 /* VBUS khi alignment */
  float concentration;        /* Circular concentration of static zero samples */
  int32_t forward_counts;     /* Encoder travel during forward sweep */
  int32_t backward_counts;    /* Encoder travel during backward sweep */
  float current_a;            /* Dòng pha A khi lock (A) */
  float current_b;            /* Dòng pha B khi lock (A) */
  float current_c;            /* Dòng pha C khi lock (A) */
  float vd_applied;           /* Điện áp Vd đã áp dụng (V) */
  uint16_t raw_angle;         /* Raw encoder angle */
  uint8_t aligned;            /* 1 = alignment thành công */
  int8_t enc_dir;             /* encoder_direction đang dùng */
} Align_Debug_t;

volatile Align_Debug_t g_dbg_align = {0};

/* ===== CURRENT SENSING DEBUG (Phase 2) ===== */
volatile float g_dbg_ia = 0, g_dbg_ib = 0, g_dbg_ic = 0; /* Dòng điện realtime */
volatile float g_dbg_offset_ia = 0, g_dbg_offset_ib = 0;  /* ADC offsets */

/* ===== PHASE 3 & 4: FOC CLOSED-LOOP DEBUG ===== */
volatile int run_foc_mode = 0;       /* 0: Dừng, 1: Torque/Current Mode, 2: Position Mode, 3: Speed Mode */
volatile float iq_target_dbg = 0.0f; /* Dòng Iq mục tiêu (A) - ví dụ 0.5A, 1.0A */
volatile float id_target_dbg = 0.0f; /* Dòng Id mục tiêu (A) - thường = 0A */
volatile float pos_target_dbg = 0.0f;/* Vị trí khớp mục tiêu (Radian) - ví dụ 0.0, 0.5, -0.5 */
volatile float speed_target_dbg = 0.0f; /* Tốc độ mục tiêu (Motor Shaft Mechanical RPM) - ví dụ 100, 500, -500 */

typedef struct {
  float id;
  float iq;
  float id_target;
  float iq_target;
  float vd;
  float vq;
  float phase_elec;
  float speed_est_rpm;   /* Tốc độ cơ học thực tế trục động cơ (Mechanical RPM) */
  float speed_target_rpm;/* Tốc độ cơ học mục tiêu trục động cơ (Mechanical RPM) */
} FOC_Debug_t;

volatile FOC_Debug_t g_dbg_foc = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t ADC_PollSingleChannel(ADC_HandleTypeDef *hadc, uint32_t channel);
void ADC_ReadAllChannels(void);
void DRV8353_ReadStatus(void);
void Run_MotorDirectionTest(void);
void Run_EncoderAlignment(void);
void TIM1_EnsureMoeEnabled(void);
static void Apply_SvmVector(float vd, float theta_e, float vbus, uint32_t period);
static void Ramp_SvmVector(float vd_from, float vd_to, float theta_e,
                           float vbus, uint32_t period, int steps, int delay_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Xóa cờ Break (nếu có) và bật lại MOE (Main Output Enable) cho Timer1.
  * Việc này giúp giải phóng PWM ra khỏi chế độ bảo vệ nếu chân fault bị sụt áp tạm thời lúc boot.
  */
void TIM1_EnsureMoeEnabled(void)
{
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK2);
  TIM1->BDTR |= TIM_BDTR_MOE;
}

/**
  * @brief  Hàm test chiều quay động cơ so với chiều tăng của Encoder.
  * Quay open-loop bằng cách xoay vector điện áp và đo sự thay đổi của Encoder.
  */
void Run_MotorDirectionTest(void)
{
  if (run_direction_test != 1) return;
  test_result = 1; // Đang chạy test
  g_dbg_test.step_reached = 0;

  // 1. Lưu trạng thái motor hiện tại và chuyển sang trạng thái dừng an toàn tạm thời
  mc_state old_state = g_foc_controller.motor.m_state;
  g_foc_controller.motor.m_state = MC_STATE_DETECTING;

  // Đảm bảo PWM giải phóng hoàn toàn khỏi trạng thái Break
  TIM1_EnsureMoeEnabled();

  // Đọc các giá trị phần cứng tại thời điểm bắt đầu test
  g_dbg_test.vbus = g_adc_readings.vbus;
  g_dbg_test.moe = (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0;
  g_dbg_test.break_flag = (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) ? 1 : 0;
  g_dbg_test.calibrated = g_foc_controller.calibrated_offsets ? 1 : 0;
  g_dbg_test.cnt = TIM1->CNT;
  g_dbg_test.bdtr = TIM1->BDTR;
  g_dbg_test.isr_at_start = g_adc_isr_counter;

  // Đợi dòng điện ổn định - GHI TRỰC TIẾP vào TIM1->CCR
  uint32_t period = htim1.Init.Period; // = 4249
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2);
  g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.5f;
  HAL_Delay(100);

  // 2. Đọc góc encoder ban đầu
  uint16_t raw_start = 0;
  AS5048A_ReadRawAngle(&g_foc_controller.encoder, &raw_start);
  g_dbg_test.raw_start = raw_start;
  g_dbg_test.start_angle = g_foc_controller.encoder.angle_rad;

  float v_test_max = 14.0f; // Áp thử nghiệm 14V - Đủ mạnh để nhích motor qua hộp số 1:17 nhằm đo chính xác encoder_direction
  float vbus_test = (g_adc_readings.vbus > 6.0f) ? g_adc_readings.vbus : 24.0f;
  float angle = 0.0f;

  for (int step = 0; step < 200; step++) {
    // Ramp up v_test dần trong 50 bước đầu để tránh sốc dòng kích hoạt bảo vệ Overcurrent của DRV8353
    float v_test = (step < 50) ? (v_test_max * (float)step / 50.0f) : v_test_max;

    angle += 0.03f; // Nhích dần góc điện áp
    float valpha = v_test * cosf(angle);
    float vbeta  = v_test * sinf(angle);

    // Tính toán SVPWM (sử dụng vbus_test an toàn)
    uint32_t ta, tb, tc, sector;
    foc_svm(valpha / vbus_test, vbeta / vbus_test,
            g_foc_controller.conf.l_max_duty, 1000, &ta, &tb, &tc, &sector);

    // *** GHI TRỰC TIẾP vào TIM1->CCR thay vì chờ ADC callback ***
    uint32_t ccr1 = (ta * period) / 1000;
    uint32_t ccr2 = (tb * period) / 1000;
    uint32_t ccr3 = (tc * period) / 1000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);

    // Cập nhật duty cho ADC callback (nếu callback cũng chạy)
    g_foc_controller.duty_a = (float)ta / 1000.0f;
    g_foc_controller.duty_b = (float)tb / 1000.0f;
    g_foc_controller.duty_c = (float)tc / 1000.0f;

    // Lưu log 10 lần (mỗi 20 bước)
    if (step % 20 == 0 && step / 20 < 10) {
      int idx = step / 20;
      g_dbg_test.steps[idx].angle = angle;
      g_dbg_test.steps[idx].ta = ta;
      g_dbg_test.steps[idx].tb = tb;
      g_dbg_test.steps[idx].tc = tc;
      // Đọc lại giá trị thực tế từ TIM1 registers
      g_dbg_test.steps[idx].ccr1 = TIM1->CCR1;
      g_dbg_test.steps[idx].ccr2 = TIM1->CCR2;
      g_dbg_test.steps[idx].ccr3 = TIM1->CCR3;
      g_dbg_test.steps[idx].isr_count = g_adc_isr_counter;
      uint16_t r_enc = 0;
      AS5048A_ReadRawAngle(&g_foc_controller.encoder, &r_enc);
      g_dbg_test.steps[idx].raw_angle = r_enc;
    }

    // Đọc liên tục để cập nhật UI Web App
    AS5048A_ReadRawAngle(&g_foc_controller.encoder, &g_foc_controller.encoder.raw_angle);
    Comm_Telemetry_Process(&g_foc_controller);

    HAL_Delay(8); // Tổng cộng test chạy trong ~1.6 giây
    g_dbg_test.step_reached = step + 1;
  }

  // 4. Đọc góc encoder sau khi quay
  uint16_t raw_end = 0;
  AS5048A_ReadRawAngle(&g_foc_controller.encoder, &raw_end);
  g_dbg_test.raw_end = raw_end;
  g_dbg_test.end_angle = g_foc_controller.encoder.angle_rad;
  g_dbg_test.isr_at_end = g_adc_isr_counter;

  // 5. Tắt cấp nguồn motor (đưa về safe state) - ghi trực tiếp
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2);
  g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.5f;

  // 6. Tính toán chênh lệch góc và xử lý điểm tràn góc (wrap around)
  float diff = g_dbg_test.end_angle - g_dbg_test.start_angle;
  while (diff > 3.14159265f) diff -= 2.0f * 3.14159265f;
  while (diff < -3.14159265f) diff += 2.0f * 3.14159265f;
  g_dbg_test.diff_angle = diff;

  // 7. Direction test chỉ kiểm tra motor có quay hay không — chiều thật do ALIGN Step 5 xác định
  // KHÔNG hardcode encoder_direction ở đây nữa (ALIGN open-loop handoff sẽ auto-detect chính xác)
  if (diff > 0.05f || diff < -0.05f) {
    test_result = 2; // OK
  } else {
    test_result = -1; // Lỗi (Động cơ không nhúc nhích đủ hoặc kẹt)
  }

  // Khôi phục lại trạng thái cũ
  g_foc_controller.motor.m_state = old_state;
  run_direction_test = 0; // Kết thúc test
}

void Run_EncoderAlignment(void)
{
  if (run_alignment != 1) return;
  align_result = 1;

  mc_state old_state = g_foc_controller.motor.m_state;
  g_foc_controller.motor.m_state = MC_STATE_DETECTING;
  TIM1_EnsureMoeEnabled();

  uint32_t period = htim1.Init.Period;
  float vbus = g_adc_readings.vbus;
  if (vbus < 6.0f) vbus = 24.0f;

  /* A continuously rotating voltage vector gave an 83.6 degree electrical
   * zero error on this high-pole-count motor. The rotor lagged the field, so
   * the measured q-axis was almost the physical d-axis. Move the field in
   * small increments, let it settle at every point, and circular-average the
   * static relation zero = pole_pairs * encoder - field_angle instead. */
  const float vd_align = 8.0f;
  const float sample_dt = 0.005f;
  float encoder_scale = (float)(g_foc_controller.conf.encoder_direction *
                                g_foc_controller.conf.foc_motor_pole_pairs);
  int pole_pairs = g_foc_controller.conf.foc_motor_pole_pairs;
  if (pole_pairs < 1) pole_pairs = 21;
  const int points_per_elec_rev = 8;
  const int points_per_mech_rev = pole_pairs * points_per_elec_rev;
  const int microsteps_per_point = 8;
  const int settle_samples = 4;
  const float point_delta = (2.0f * (float)M_PI) /
                            (float)points_per_elec_rev;

  /* Lock to a known stator field before beginning the quasi-static sweep. */
  for (int i = 0; i <= 80; i++) {
    if (run_alignment != 1 || g_foc_controller.fault != MC_FAULT_NONE)
      goto alignment_abort;
    Apply_SvmVector(vd_align * (float)i / 80.0f, 0.0f, vbus, period);
    HAL_Delay(5);
  }
  for (int i = 0; i < 40; i++) {
    Apply_SvmVector(vd_align, 0.0f, vbus, period);
    HAL_Delay(5);
    AS5048A_Sample(&g_foc_controller.encoder, sample_dt);
  }

  float theta = 0.0f;
  float zero_sin_sum = 0.0f;
  float zero_cos_sum = 0.0f;
  uint32_t zero_count = 0U;
  int32_t sweep_progress[2] = {0, 0};

  for (int pass = 0; pass < 2; pass++) {
    float direction = (pass == 0) ? 1.0f : -1.0f;
    int32_t pass_start = g_foc_controller.encoder.count_buff[0];

    for (int point = 0; point < points_per_mech_rev; point++) {
      if (run_alignment != 1 || g_foc_controller.fault != MC_FAULT_NONE)
        goto alignment_abort;

      for (int micro = 0; micro < microsteps_per_point; micro++) {
        theta += direction * point_delta / (float)microsteps_per_point;
        utils_norm_angle_rad(&theta);
        Apply_SvmVector(vd_align, theta, vbus, period);
        HAL_Delay(2);
        AS5048A_Sample(&g_foc_controller.encoder, 0.002f);
      }

      for (int sample = 0; sample < settle_samples; sample++) {
        Apply_SvmVector(vd_align, theta, vbus, period);
        HAL_Delay(5);
        AS5048A_Sample(&g_foc_controller.encoder, sample_dt);
      }

      float zero_sample = encoder_scale *
                          g_foc_controller.encoder.angle_singleturn - theta;
      utils_norm_angle_rad(&zero_sample);
      zero_sin_sum += sinf(zero_sample);
      zero_cos_sum += cosf(zero_sample);
      zero_count++;
      Comm_Telemetry_Process(&g_foc_controller);
    }

    sweep_progress[pass] = g_foc_controller.conf.encoder_direction *
        (g_foc_controller.encoder.count_buff[0] - pass_start);
  }

  int32_t min_progress = (AS5048A_CPR * 3) / 4;
  int32_t max_progress = (AS5048A_CPR * 5) / 4;
  bool forward_valid = sweep_progress[0] >= min_progress &&
                       sweep_progress[0] <= max_progress;
  bool backward_valid = sweep_progress[1] <= -min_progress &&
                        sweep_progress[1] >= -max_progress;
  float concentration = (zero_count > 0U)
      ? sqrtf(zero_sin_sum * zero_sin_sum + zero_cos_sum * zero_cos_sum) /
        (float)zero_count
      : 0.0f;
  if (!forward_valid || !backward_valid || concentration < 0.95f)
    goto alignment_abort;

  float elec_offset = atan2f(zero_sin_sum, zero_cos_sum);
  AS5048A_Sample(&g_foc_controller.encoder, sample_dt);
  float enc_zero = g_foc_controller.encoder.angle_singleturn;
  g_foc_controller.zero_electric_angle = elec_offset;
  g_foc_controller.aligned = true;

  g_dbg_align.vbus = vbus;
  g_dbg_align.enc_dir = g_foc_controller.conf.encoder_direction;
  g_dbg_align.zero_electric_angle = elec_offset;
  g_dbg_align.encoder_rad = enc_zero;
  g_dbg_align.concentration = concentration;
  g_dbg_align.forward_counts = sweep_progress[0];
  g_dbg_align.backward_counts = sweep_progress[1];
  g_dbg_align.aligned = 1;

  /* Ramp down gently */
  for (int i = 40; i >= 0; i--) {
    Apply_SvmVector(vd_align * (float)i / 40.0f, theta, vbus, period);
    Comm_Telemetry_Process(&g_foc_controller);
    HAL_Delay(5);
  }

  align_result = 2;
  goto alignment_done;

alignment_abort:
  align_result = -1;
  g_foc_controller.aligned = false;

alignment_done:
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2);
  g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.5f;

  HAL_Delay(50);
  /* Reset velocity after the field has ramped down and the drivetrain has
   * relaxed. Resetting before ramp-down turns that elastic motion into a
   * false high-speed sample at the closed-loop handoff. */
  AS5048A_Sample(&g_foc_controller.encoder, 0.001f);
  int32_t aligned_count = g_foc_controller.encoder.count_buff[0];
  for (int i = 1; i < AS5048A_N_POS_SAMPLES; i++) {
    g_foc_controller.encoder.count_buff[i] = aligned_count;
  }
  g_foc_controller.encoder.velocity_rad_s = 0.0f;
  g_foc_controller.encoder.velocity_rpm = 0.0f;
  g_foc_controller.motor.m_speed_est_fast = 0.0f;
  g_foc_controller.motor.m_speed_d_filter = 0.0f;
  g_foc_controller.motor.m_speed_d_filter_proc = 0.0f;
  g_foc_controller.motor.m_state = old_state;
  run_alignment = 0;
}

static void Apply_SvmVector(float vd, float theta_e, float vbus, uint32_t period)
{
  float sin_a, cos_a;
  utils_fast_sincos(theta_e, &sin_a, &cos_a);
  float valpha = (vd / vbus) * cos_a;
  float vbeta  = (vd / vbus) * sin_a;
  uint32_t ta, tb, tc, sector;
  foc_svm(valpha, vbeta, g_foc_controller.conf.l_max_duty, 1000, &ta, &tb, &tc, &sector);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (ta * period) / 1000);
  if (g_foc_controller.phase_swap_bc) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (tc * period) / 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (tb * period) / 1000);
    g_foc_controller.duty_a = (float)ta / 1000.0f;
    g_foc_controller.duty_b = (float)tc / 1000.0f;
    g_foc_controller.duty_c = (float)tb / 1000.0f;
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (tb * period) / 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (tc * period) / 1000);
    g_foc_controller.duty_a = (float)ta / 1000.0f;
    g_foc_controller.duty_b = (float)tb / 1000.0f;
    g_foc_controller.duty_c = (float)tc / 1000.0f;
  }
}

static void Ramp_SvmVector(float vd_from, float vd_to, float theta_e, float vbus, uint32_t period, int steps, int delay_ms)
{
  for (int i = 0; i <= steps; i++) {
    float vd = vd_from + (vd_to - vd_from) * ((float)i / (float)steps);
    Apply_SvmVector(vd, theta_e, vbus, period);
    Comm_Telemetry_Process(&g_foc_controller);
    HAL_Delay(delay_ms);
  }
}

/**
 * @brief  Ben Katz MIT Mini Cheetah 128-point Full 1-Revolution Encoder Linearization
 */
void Run_EncoderCalibration(void)
{
  if (run_calibration != 1) return;
  align_result = 1; // Running
  g_encoder_calibration_result = 1;

  mc_state old_state = g_foc_controller.motor.m_state;
  g_foc_controller.motor.m_state = MC_STATE_DETECTING;

  TIM1_EnsureMoeEnabled();

  uint32_t period = htim1.Init.Period; // 4249
  float vbus = g_adc_readings.vbus;
  if (vbus < 6.0f) vbus = 24.0f;

  const float vd_cal = 8.0f;
  float pole_pairs = (float)g_foc_controller.conf.foc_motor_pole_pairs;
  if (pole_pairs < 1.0f) {
    pole_pairs = 21.0f;
  }

  g_foc_controller.conf.encoder_direction = 1;
  if (g_foc_controller.motor.m_conf != NULL) {
    g_foc_controller.motor.m_conf->encoder_direction = 1;
  }
  g_foc_controller.encoder.use_lut = 0; // Disable LUT during calibration

  // STEP 1: Lock rotor firmly at electrical zero for settling.
  Ramp_SvmVector(0.0f, vd_cal, 0.0f, vbus, period, 80, 5);
  for (int i = 0; i < 80; i++) {
    if (run_calibration != 1 || g_foc_controller.fault != MC_FAULT_NONE) goto calibration_abort;
    Apply_SvmVector(vd_cal, 0.0f, vbus, period);
    Comm_Telemetry_Process(&g_foc_controller);
    HAL_Delay(10);
  }

  // STEP 2: Sweep at 5.9 RPM. At 2 V the measured pull-in limit is about 10 RPM.
  int32_t err_fwd[128];
  int32_t err_bwd[128];
  int16_t candidate_lut[128];
  int32_t forward_progress = 0;
  int32_t backward_progress = 0;
  bool calibration_valid = false;

  uint16_t raw_start = 0;
  AS5048A_ReadRawAngle(&g_foc_controller.encoder, &raw_start);
  AS5048A_ReadRawAngle(&g_foc_controller.encoder, &raw_start);
  uint16_t previous_raw = raw_start;

  for (int k = 0; k < 128; k++) {
    // 10 microsteps per calibration point for buttery-smooth continuous rotation
    for (int s = 0; s < 10; s++) {
      if (run_calibration != 1 || g_foc_controller.fault != MC_FAULT_NONE) goto calibration_abort;
      float theta_mech = (2.0f * (float)M_PI * ((float)k + (float)s / 10.0f)) / 128.0f;
      float theta_elec = theta_mech * pole_pairs;
      Apply_SvmVector(vd_cal, theta_elec, vbus, period);
      Comm_Telemetry_Process(&g_foc_controller);
      HAL_Delay(8);
    }
    
    uint16_t r_val = 0;
    AS5048A_ReadRawAngle(&g_foc_controller.encoder, &r_val);
    AS5048A_ReadRawAngle(&g_foc_controller.encoder, &r_val);

    int32_t step_counts = (int32_t)r_val - (int32_t)previous_raw;
    while (step_counts > 8192) step_counts -= 16384;
    while (step_counts < -8192) step_counts += 16384;
    forward_progress += step_counts;
    previous_raw = r_val;
    
    int32_t count_ideal = ((int32_t)raw_start + ((k * 16384) / 128)) % 16384;
    int32_t diff = (int32_t)r_val - count_ideal;
    while (diff > 8192)  diff -= 16384;
    while (diff < -8192) diff += 16384;
    err_fwd[k] = diff;
  }

  // STEP 3: Sweep 1 full mechanical revolution in Backward direction (128 sample points with 10 microsteps each)
  for (int k = 127; k >= 0; k--) {
    for (int s = 0; s < 10; s++) {
      if (run_calibration != 1 || g_foc_controller.fault != MC_FAULT_NONE) goto calibration_abort;
      float theta_mech = (2.0f * (float)M_PI * ((float)k + (float)(10 - s) / 10.0f)) / 128.0f;
      float theta_elec = theta_mech * pole_pairs;
      Apply_SvmVector(vd_cal, theta_elec, vbus, period);
      Comm_Telemetry_Process(&g_foc_controller);
      HAL_Delay(8);
    }
    
    uint16_t r_val = 0;
    AS5048A_ReadRawAngle(&g_foc_controller.encoder, &r_val);
    AS5048A_ReadRawAngle(&g_foc_controller.encoder, &r_val);

    int32_t step_counts = (int32_t)r_val - (int32_t)previous_raw;
    while (step_counts > 8192) step_counts -= 16384;
    while (step_counts < -8192) step_counts += 16384;
    backward_progress += step_counts;
    previous_raw = r_val;
    
    int32_t count_ideal = ((int32_t)raw_start + ((k * 16384) / 128)) % 16384;
    int32_t diff = (int32_t)r_val - count_ideal;
    while (diff > 8192)  diff -= 16384;
    while (diff < -8192) diff += 16384;
    err_bwd[k] = diff;
  }

  // STEP 4: Compute combined offset_lut
  int32_t sum_err = 0;
  for (int k = 0; k < 128; k++) {
    int32_t avg_err = (err_fwd[k] + err_bwd[k]) / 2;
    sum_err += avg_err;
  }
  int32_t ezero_offset = sum_err / 128;

  int32_t max_abs_comp = 0;
  for (int k = 0; k < 128; k++) {
    int32_t avg_err = (err_fwd[k] + err_bwd[k]) / 2;
    int32_t comp = -(avg_err - ezero_offset);
    int32_t abs_comp = (comp < 0) ? -comp : comp;
    if (abs_comp > max_abs_comp) max_abs_comp = abs_comp;

    int lut_idx = ((((int32_t)raw_start + (k * 128)) % 16384) >> 7) & 0x7F;
    candidate_lut[lut_idx] = (int16_t)comp;
  }

  calibration_valid = forward_progress > 14745 && forward_progress < 18022 &&
                      backward_progress < -14745 && backward_progress > -18022 &&
                      max_abs_comp <= 256;
  if (calibration_valid) {
    for (int k = 0; k < 128; k++) {
      g_foc_controller.encoder.offset_lut[k] = candidate_lut[k];
    }
    g_foc_controller.encoder.use_lut = 1U;
  } else {
    /* A failed retry must not destroy a previously validated calibration. */
    EncoderCalStore_Load(&g_foc_controller.encoder);
  }

  // STEP 5: Return near electrical zero before removing calibration voltage.
  Ramp_SvmVector(vd_cal, vd_cal, 0.0f, vbus, period, 40, 5);
  for (int i = 0; i < 60; i++) {
    if (run_calibration != 1 || g_foc_controller.fault != MC_FAULT_NONE) goto calibration_abort;
    Apply_SvmVector(vd_cal, 0.0f, vbus, period);
    Comm_Telemetry_Process(&g_foc_controller);
    HAL_Delay(10);
  }

  // STEP 6: Ramp down to 0V
  Ramp_SvmVector(vd_cal, 0.0f, 0.0f, vbus, period, 40, 5);

  // Safe state
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2);
  g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.5f;

  HAL_Delay(50);
  if (calibration_valid) {
    g_encoder_calibration_result =
        EncoderCalStore_Save(candidate_lut) ? 2 : -2;
  } else {
    g_encoder_calibration_result = -1;
  }
  g_foc_controller.fault = MC_FAULT_NONE;
  g_foc_controller.motor.m_state = old_state;
  /* A long rotating calibration can finish with the rotor lagging the field.
   * A fresh zero-voltage-to-lock alignment is required before closed-loop FOC. */
  g_foc_controller.aligned = false;
  align_result = calibration_valid ? 1 : -1;
  run_calibration = 0;
  run_alignment = 1;
  return;

calibration_abort:
  EncoderCalStore_Load(&g_foc_controller.encoder);
  g_encoder_calibration_result = -1;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2);
  g_foc_controller.duty_a = g_foc_controller.duty_b = g_foc_controller.duty_c = 0.5f;
  g_foc_controller.motor.m_state = MC_STATE_OFF;
  align_result = -1;
  run_calibration = 0;
}


/**
 * @brief  Đọc 1 kênh ADC regular bằng polling (reconfigure + start + wait + read)
 * @param  hadc  : Handle ADC (dùng &hadc1)
 * @param  channel: Kênh ADC, ví dụ ADC_CHANNEL_8
 * @retval Giá trị raw 12-bit (0..4095)
 */
static uint16_t ADC_PollSingleChannel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_TypeDef *ADCx = hadc->Instance;

  /* KHÔNG dùng HAL_ADC_ConfigChannel() vì nó gọi HAL_ADC_Disable() bên trong,
   * sẽ giết chết Injected Conversions đang chạy 20kHz từ TIM1 TRGO.
   *
   * Thay vào đó: ghi trực tiếp vào SQR1 để chọn kênh regular,
   * tạm tắt External Trigger (EXTEN=0) để software start hoạt động,
   * rồi trigger ADSTART và đợi EOC. */

  /* Chờ nếu có regular conversion đang chạy (bounded) */
  uint32_t timeout = 5000;
  while ((ADCx->CR & ADC_CR_ADSTART) && --timeout);
  if (timeout == 0) return 0;

  /* Lưu CFGR gốc và tạm tắt external trigger cho regular conversion */
  uint32_t cfgr_backup = ADCx->CFGR;
  ADCx->CFGR = (cfgr_backup & ~ADC_CFGR_EXTEN);  /* EXTEN = 00 = Software start */

  /* Clear any pending EOC/OVR flags */
  ADCx->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;

  /* Write channel to SQR1: L[3:0]=0 (1 conversion), SQ1[4:0]=channel number */
  uint32_t ch_num = __LL_ADC_CHANNEL_TO_DECIMAL_NB(channel);
  ADCx->SQR1 = (ch_num << ADC_SQR1_SQ1_Pos) | (0 << ADC_SQR1_L_Pos);

  /* Start regular conversion (software trigger vì EXTEN=0) */
  ADCx->CR |= ADC_CR_ADSTART;

  /* Wait for End-Of-Conversion with bounded timeout (~50µs max) */
  timeout = 10000;
  while (!(ADCx->ISR & ADC_ISR_EOC) && --timeout);

  /* Read result */
  uint16_t raw = (uint16_t)(ADCx->DR & 0x0FFF);

  /* Khôi phục CFGR gốc (external trigger cho regular conversion) */
  ADCx->CFGR = cfgr_backup;

  return raw;
}

/**
 * @brief  Đọc VBUS và FET_TEMP thông qua ADC polling an toàn.
 *         LƯU Ý QUAN TRỌNG: Hàm này CHỈ được phép gọi từ bên trong ngắt Injected
 *         (sau khi ngắt Injected đã đo xong dòng điện pha), nếu không việc ghi
 *         SQR1 sẽ làm nhiễu loạn ADC1 Injected trigger (gây kẹt dòng ảo 50A).
 */
void ADC_ReadAllChannels(void)
{
  /* Đọc VBUS thực qua ADC1 kênh IN8 (PC2/VBUS_SENSE) với hệ số chia áp 26.81.
   * AN TOÀN vì:
   *   (1) Được gọi trong slow loop ISR (sau khi Injected xong, trước TRGO tiếp theo)
   *   (2) ADC1 Regular đã cấu hình SOFTWARE_START (Fix 1) -> không còn xung đột T1_TRGO */
  uint16_t raw_vbus = ADC_PollSingleChannel(&hadc1, ADC_CHANNEL_8); /* PC2 = VBUS_SENSE */
  if (raw_vbus > 200) { /* Sanity: >200 LSB ~ 0.16V -> VBUS > 4.3V thực */
    g_adc_readings.vbus_raw = raw_vbus;
    g_adc_readings.vbus = (float)raw_vbus * ADC_TO_VOLT * VBUS_DIVIDER_RATIO;
  }
  /* Nếu raw_vbus <= 200 (lỗi ADC hoặc VBUS quá thấp): giữ giá trị cũ (init=24.0f) */

  g_adc_readings.fet_temp = 25.0f; /* TODO: NTC thermistor via ADC_CHANNEL_4 (PA3) */

  /* Cập nhật cờ lỗi từ chân nFAULT (DRV_BKIN) của DRV8353 */
  g_adc_readings.drv_has_fault = (HAL_GPIO_ReadPin(DRV_BKIN_GPIO_Port, DRV_BKIN_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

/**
 * @brief  Đọc trạng thái DRV8353RS qua SPI1
 *         Lấy Fault Status 1 & 2, trích xuất cờ cảnh báo nhiệt
 *
 * FAULT_STATUS_1 (0x00) bit map:
 *   Bit 10: FAULT  (có lỗi chung)
 *   Bit 9 : VDS_OCP (quá dòng VDS)
 *   Bit 8 : GDF    (Gate Driver Fault)
 *   Bit 7 : UVLO   (Under-Voltage Lock-Out)
 *   Bit 6 : OTSD   (Over-Temperature Shutdown)
 *   Bit 5 : VDS_HA / Bit 4: VDS_LA ... (chi tiết từng pha)
 *
 * VGS_STATUS_2 (0x01) bit map:
 *   Bit 10: SA_OC  (Sense Amp Overcurrent)
 *   Bit 9 : SB_OC
 *   Bit 8 : SC_OC
 *   Bit 7 : OTW    (Over-Temperature Warning, ~150°C)
 *   Bit 6 : CPUV   (Charge Pump Under-Voltage)
 *   Bit 5 : VGS_HA ... (chi tiết từng pha)
 */
void DRV8353_ReadStatus(void)
{
  uint16_t fault1 = 0, fault2 = 0;

  if (DRV8353_ReadFaults(&g_foc_controller.drv8353, &fault1, &fault2) == HAL_OK) {
    g_adc_readings.drv_fault1    = fault1;
    g_adc_readings.drv_fault2    = fault2;
    g_adc_readings.drv_otsd      = (fault1 >> 6) & 0x01;  /* FAULT_STATUS_1 bit 6 */
    g_adc_readings.drv_otw       = (fault2 >> 7) & 0x01;  /* VGS_STATUS_2   bit 7 */
    g_adc_readings.drv_has_fault = (fault1 >> 10) & 0x01;  /* FAULT_STATUS_1 bit 10 */
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_TIM1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  /* ================================================================
   * STARTUP SEQUENCE - Joint Driver 8115
   * Motor: GB8115-4 (21PP), Gearbox: Cycloid 1:17, PWM: 20kHz
   * ================================================================ */

  /* 1. Initialize FOC Engine, DRV8353RS Gate Driver & AS5048A Encoder */
  motor_init(&hspi1, &hspi3);
  g_encoder_calibration_result =
      EncoderCalStore_Load(&g_foc_controller.encoder) ? 3 : 0;

  /* 2. Calibrate ADC hardware (internal offset calibration) */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* Cycle-accurate dt keeps the 10 kHz ADC/FOC math independent of PWM rate. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 3. Start PWM on all 6 channels (3 High-side + 3 Low-side complementary) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  /* 4. Set initial duty to 50% (zero differential voltage = safe state) */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, htim1.Init.Period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, htim1.Init.Period / 2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, htim1.Init.Period / 2);

  /* 5. Start Injected ADC conversions (triggered by TIM1 CC4 inside the PWM low-side window) */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                        htim1.Init.Period - ADC_INJECTED_SAMPLE_TICKS_FROM_TOP);
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  HAL_ADCEx_InjectedStart_IT(&hadc2);  // Slave ADC2 start first
  HAL_ADCEx_InjectedStart_IT(&hadc1);  // Master ADC1 start (triggers both)

  /* 6. Wait for ADC zero-current offset calibration to truly complete.
   * Calibration timeline at 10 kHz: 10000 ISR warmup + 2048 samples = ~1.205 s.
   * HAL_Delay(150) cũ KHÔNG ĐỦ -> offset chưa xong khi main loop chạy.
   * Poll flag thực để đảm bảo offset đúng trước khi gửi telemetry. */
  g_adc_calib_wait_ms = 0;
  g_adc_calib_timeout = 0;
  while (!g_foc_controller.calibrated_offsets && g_adc_calib_wait_ms < 1500U) {
      HAL_Delay(10);
      g_adc_calib_wait_ms += 10U;
  }
  if (!g_foc_controller.calibrated_offsets) {
      g_adc_calib_timeout = 1;
      g_foc_controller.motor.m_state = MC_STATE_OFF;
      g_foc_controller.fault |= MC_FAULT_ENCODER;
  } else {
      HAL_Delay(10); /* Buffer thêm 10ms sau khi offset xong */
  }

  /* 7. Đọc VBUS thực và cập nhật g_adc_readings sau khi offset calibration hoàn tất
   * AN TOÀN gọi từ main loop: ADC1 Regular = SOFTWARE_START (Fix 1) -> không xung đột T1_TRGO */
  ADC_ReadAllChannels();

  /* 8. Set initial target joint position */
  motor_set_position(0.0f);

  /* 9. Initialize VESC Protocol CAN Communication (Node ID = 1) */
  comm_can_init(&hfdcan2, DEFAULT_CAN_NODE_ID);

  /* 10. Initialize High-Speed Native USB Telemetry Stream */
  Comm_Telemetry_Init();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ================================================================
     * MAIN LOOP - Đọc tất cả cảm biến (~2Hz)
     * ================================================================ */

    /* 0. Chạy test chiều quay động cơ nếu được kích hoạt */
    Run_MotorDirectionTest();

    /* 0b. Chạy encoder alignment nếu được kích hoạt */
    Run_EncoderAlignment();

    /* 0c. Chạy 1-turn Ben Katz LUT calibration nếu được kích hoạt */
    Run_EncoderCalibration();

    /* Đảm bảo giải phóng khóa Break PWM ở Timer1 (tránh kẹt lúc khởi động) */
    TIM1_EnsureMoeEnabled();

    /* 0c. Chế độ điều khiển FOC Closed-Loop (Đồng bộ giữa Live Expressions và USB App) */
    if (g_foc_controller.fault != MC_FAULT_NONE) {
      // Nếu có lỗi an toàn (Overcurrent/Overvoltage), tự động reset mode về 0 để không bị vòng lặp ON/OFF
      run_foc_mode = 0;
      g_foc_controller.motor.m_state = MC_STATE_OFF;
    } else if (run_foc_mode == 1 || run_foc_mode == 2 || run_foc_mode == 3 || run_foc_mode == 4) {
      // Tự động Căn chỉnh Góc Encoder (Align) Lần đầu nếu chưa được căn chỉnh
      if (!g_foc_controller.aligned && run_alignment != 1) {
        run_alignment = 1;
        Run_EncoderAlignment();
      }

      if (run_foc_mode == 1) { // Current/Torque Control Mode từ Live Expressions
        g_foc_controller.motor.m_state = MC_STATE_RUNNING;
        g_foc_controller.motor.m_control_mode = CONTROL_MODE_CURRENT;
        g_foc_controller.motor.m_iq_set = iq_target_dbg;
        g_foc_controller.motor.m_motor_state.iq_target = iq_target_dbg;
        g_foc_controller.motor.m_motor_state.id_target = id_target_dbg;
      } else if (run_foc_mode == 2) { // Position Control Mode từ Live Expressions
        g_foc_controller.motor.m_state = MC_STATE_RUNNING;
        g_foc_controller.motor.m_control_mode = CONTROL_MODE_POS;
        if (!g_foc_controller.motor.m_traj_active) {
          g_foc_controller.motor.m_pos_pid_set = pos_target_dbg;
        }
        g_foc_controller.motor.m_motor_state.id_target = id_target_dbg;
      } else if (run_foc_mode == 3) { // Speed/Velocity Control Mode từ Live Expressions / Web App
        g_foc_controller.motor.m_state = MC_STATE_RUNNING;
        g_foc_controller.motor.m_control_mode = CONTROL_MODE_SPEED;
        g_foc_controller.motor.m_speed_command_rpm = speed_target_dbg * (float)g_foc_controller.conf.foc_motor_pole_pairs;
        g_foc_controller.motor.m_motor_state.id_target = id_target_dbg;
      } else if (run_foc_mode == 4) { // Direct Voltage Vq Mode
        g_foc_controller.motor.m_state = MC_STATE_RUNNING;
        g_foc_controller.motor.m_control_mode = CONTROL_MODE_DUTY;
      }
    } else if (run_direction_test != 1 && run_alignment != 1 &&
               run_calibration != 1 && run_open_loop != 1) {
      g_foc_controller.motor.m_state = MC_STATE_OFF;
      g_foc_controller.motor.m_iq_set = 0.0f;
      g_foc_controller.motor.m_motor_state.iq_target = 0.0f;
      g_foc_controller.motor.m_motor_state.id_target = 0.0f;
      pos_target_dbg = g_foc_controller.motor.m_joint_angle;
      speed_target_dbg = 0.0f;
      g_foc_controller.motor.m_speed_command_rpm = 0.0f;
      g_foc_controller.motor.m_speed_pid_set_rpm = 0.0f;
      g_foc_controller.duty_a = 0.5f;
      g_foc_controller.duty_b = 0.5f;
      g_foc_controller.duty_c = 0.5f;
      /* Keep safety faults latched for telemetry and diagnosis. STOP/CLEAR or
       * a new explicit run command clears the fault. */
    }

    /* 1. Đọc VBUS qua ADC1 Regular (SOFTWARE_START, an toàn) + DRV status ở 2Hz
     * Giữ main loop chạy nhanh cho Comm_Telemetry_Process (~100Hz+) */
    {
      static uint32_t last_slow_sensor_ms = 0;
      uint32_t now_ms = HAL_GetTick();
      if (now_ms - last_slow_sensor_ms >= 500) {
        last_slow_sensor_ms = now_ms;

        ADC_ReadAllChannels();

        /* Đọc trạng thái DRV8353 qua SoftSPI (Fault, OTW, OTSD) */
        DRV8353_ReadStatus();



        /* LED Heartbeat (chớp mỗi 500ms = 1Hz, thay vì chớp cuồng ở tốc độ main loop) */
        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
      }
    }

    /* Cập nhật live debug expressions (rất nhanh, chỉ đọc RAM) */
    g_dbg_test.raw_start = g_foc_controller.encoder.raw_angle;
    g_dbg_test.start_angle = g_foc_controller.encoder.angle_rad;
    g_dbg_test.vbus = g_adc_readings.vbus;
    g_dbg_test.moe = (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0;
    g_dbg_test.break_flag = (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) ? 1 : 0;
    g_dbg_test.calibrated = g_foc_controller.calibrated_offsets ? 1 : 0;
    g_dbg_test.cnt = TIM1->CNT;
    g_dbg_test.bdtr = TIM1->BDTR;
    g_dbg_test.ccer = TIM1->CCER;

    /* Fault indicator on LED2 (Active-Low: RESET = ON, SET = OFF) */
    if (g_adc_readings.drv_has_fault) {
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    }

    /* Truyền Telemetry 100Hz qua USB CDC - PHẢI gọi nhanh nhất có thể */
    Comm_Telemetry_Process(&g_foc_controller);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  /* FIX: Regular channel KHÔNG tự động trigger theo T1_TRGO nữa.
   * Trước đây, cả Regular và Injected cùng triggered bởi T1_TRGO và cùng đọc IN7 (PC1),
   * gây ra xung đột kênh và khuếch đại nhiễu ADC (~±25 LSB = ±0.5A).
   * Regular channel chỉ dùng bởi ADC_PollSingleChannel() (software start) khi cần thiết. */
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_30MV;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_VREFINT;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 20;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 3 ;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = 4249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP3;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = htim1.Init.Period - ADC_INJECTED_SAMPLE_TICKS_FROM_TOP;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DRV_CS_Pin|DRV_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC_EN_GPIO_Port, ENC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_CS_Pin DRV_EN_Pin */
  GPIO_InitStruct.Pin = DRV_CS_Pin|DRV_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_EN_Pin */
  GPIO_InitStruct.Pin = ENC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENC_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_CS_Pin */
  GPIO_InitStruct.Pin = ENC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ENC_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  ADC Injected Conversion Complete Callback
 *         CHẠY Ở 20 kHz - Được kích hoạt bởi TIM1 TRGO (Update Event)
 *         Đây là trái tim của toàn bộ hệ thống điều khiển FOC.
 *
 *         Luồng phần cứng:
 *         TIM1 Counter đếm tới ARR → TRGO Update Event → Kích hoạt ADC1+ADC2
 *         → ADC Injected lấy mẫu đồng thời Ib (PA7) và Ic (PC1)
 *         → ADC End-of-Injected-Conversion → Nhảy vào callback này
 *         → Chạy FOC transforms + PI + SVPWM → Cập nhật TIM1 CCR1/2/3
 *
 * @param  hadc: ADC handle pointer (ADC1 hoặc ADC2)
 * @note   Chỉ xử lý khi ADC1 (Master) báo xong conversion
 * @note   Hardware configuration generates this ADC/FOC callback at 10 kHz.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1)
    return; // Chỉ xử lý Master ADC1

  static uint32_t previous_isr_cycle = 0U;
  uint32_t isr_cycle = DWT->CYCCNT;
  float dt = 1.0f / 10000.0f;
  if (previous_isr_cycle != 0U && SystemCoreClock > 0U) {
    uint32_t elapsed_cycles = isr_cycle - previous_isr_cycle;
    float measured_dt = (float)elapsed_cycles / (float)SystemCoreClock;
    if (measured_dt >= 0.000020f && measured_dt <= 0.000500f) {
      dt = measured_dt;
      uint32_t nominal_cycles = SystemCoreClock / 10000U;
      if (nominal_cycles > 0U && elapsed_cycles > (nominal_cycles * 3U) / 2U) {
        uint32_t elapsed_periods = (elapsed_cycles + nominal_cycles / 2U) /
                                   nominal_cycles;
        if (elapsed_periods > 1U) {
          g_foc_isr_missed_periods += elapsed_periods - 1U;
        }
      }
    }
  }
  previous_isr_cycle = isr_cycle;
  g_foc_isr_dt_s = dt;
  g_adc_isr_counter++; // Đếm số lần ISR chạy để debug

  // ===== 1. ĐỌC GIÁ TRỊ DÒNG ĐIỆN THẬT TỪ ADC =====
  // ADC2 Injected Rank 1 = PA7 (IB) - Dòng pha B từ DRV8353 SOB
  // ADC1 Injected Rank 1 = PC1 (IC) - Dòng pha C từ DRV8353 SOC
  uint32_t raw_ib = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  uint32_t raw_ic = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

  // ===== 2. CALIBRATE OFFSET (2048 lần đầu khi motor OFF) =====
  if (!g_foc_controller.calibrated_offsets) {
    FOC_Control_AdcCalibrate(&g_foc_controller, (uint16_t)raw_ic,
                             (uint16_t)raw_ib);
    return; // Không chạy FOC khi đang calibrate
  }

  bool motor_driven = (run_foc_mode != 0) || (run_open_loop == 1) ||
                      (run_direction_test == 1) ||
                      (run_alignment == 1) || (run_calibration == 1);

  if (!motor_driven) {
    g_foc_controller.offset_ia += CURRENT_IDLE_OFFSET_ALPHA *
                                  ((float)raw_ic - g_foc_controller.offset_ia);
    g_foc_controller.offset_ib += CURRENT_IDLE_OFFSET_ALPHA *
                                  ((float)raw_ib - g_foc_controller.offset_ib);
  }

  float current_b, current_c;
  if (g_foc_controller.phase_swap_bc) {
    current_b = ((float)raw_ic - g_foc_controller.offset_ia) * ADC_TO_AMPS;
    current_c = ((float)raw_ib - g_foc_controller.offset_ib) * ADC_TO_AMPS;
  } else {
    current_b = ((float)raw_ib - g_foc_controller.offset_ib) * ADC_TO_AMPS;
    current_c = ((float)raw_ic - g_foc_controller.offset_ia) * ADC_TO_AMPS;
  }

  if (!motor_driven) {
    current_b = 0.0f;
    current_c = 0.0f;
  }
  float current_a = -(current_b + current_c); // Kirchhoff: Ia + Ib + Ic = 0

  // Debug: cập nhật dòng điện realtime cho monitoring
  g_dbg_ia = current_a;
  g_dbg_ib = current_b;
  g_dbg_ic = current_c;
  g_dbg_offset_ia = g_foc_controller.offset_ia;
  g_dbg_offset_ib = g_foc_controller.offset_ib;

  // ===== 4. ĐỌC VBUS VÀ FET_TEMP (từ slow loop ADC polling) =====
  float vbus = g_adc_readings.vbus;
  if (vbus < 6.0f) vbus = 24.0f;  /* Fallback nếu chưa có giá trị hợp lệ */
  float temp_fet = g_adc_readings.fet_temp;
  if (temp_fet < -20.0f || temp_fet > 150.0f) temp_fet = 25.0f;

  /* ALIGN/CALIB drive PWM from the main loop, so the normal FOC ISR below is
   * intentionally skipped. Still publish real phase currents and run the
   * safety supervisor instead of showing stale zeros during those tests. */
  if (run_open_loop == 1 || run_direction_test == 1 || run_alignment == 1 ||
      run_calibration == 1) {
    motor_state_t *state_m = &g_foc_controller.motor.m_motor_state;
    state_m->i_alpha = current_a;
    state_m->i_beta = (current_a + 2.0f * current_b) * ONE_BY_SQRT3;
    if (run_open_loop != 1) {
      state_m->id_filter = current_a;
      state_m->iq_filter = state_m->i_beta;
    }
    if (!FOC_Control_CheckSafety(&g_foc_controller, current_a, current_b,
                                 vbus, temp_fet)) {
      return;
    }
  }

  // ===== 5. CHẠY FOC CURRENT CONTROL ISR =====
  if (run_open_loop == 1) {
    TIM1_EnsureMoeEnabled();

    AS5048A_Sample(&g_foc_controller.encoder, dt);
    foc_update_cycloidal_joint_angle(&g_foc_controller.motor,
                                     g_foc_controller.encoder.angle_singleturn);

    /* === 1. SMOOTH ACCELERATION RAMP (80 RPM/s) === */
    if (fabsf(open_loop_target_rpm) > 0.1f) {
      utils_step_towards((float*)&open_loop_current_rpm, open_loop_target_rpm,
                         80.0f * dt);
    } else {
      open_loop_current_rpm = 0.0f;
    }

    /* === 2. TÍNH GÓC ĐIỆN (Electrical Angle) === */
    float pole_pairs = (float)g_foc_controller.conf.foc_motor_pole_pairs; /* 21PP */
    float elec_rad_s = (open_loop_current_rpm * pole_pairs * 2.0f * 3.14159265f) / 60.0f;
    open_loop_angle += elec_rad_s * dt;
    /* Wrap angle [-PI, +PI] mỗi ISR cycle để tránh drift/overflow float */
    utils_norm_angle_rad((float*)&open_loop_angle);
    motor_state_t *state_m = &g_foc_controller.motor.m_motor_state;
    state_m->phase = open_loop_angle;
    float sin_current, cos_current;
    sincos_lut(open_loop_angle, &sin_current, &cos_current);
    state_m->id = cos_current * state_m->i_alpha +
                  sin_current * state_m->i_beta;
    state_m->iq = cos_current * state_m->i_beta -
                  sin_current * state_m->i_alpha;
    UTILS_LP_FAST(state_m->id_filter, state_m->id,
                  g_foc_controller.conf.foc_current_filter_const);
    UTILS_LP_FAST(state_m->iq_filter, state_m->iq,
                  g_foc_controller.conf.foc_current_filter_const);
    /* Report measured rotor speed; elec_rad_s is only the field command. */
    g_foc_controller.motor.m_speed_est_fast =
        (float)(g_foc_controller.conf.encoder_direction *
                g_foc_controller.conf.foc_motor_pole_pairs) *
        g_foc_controller.encoder.velocity_rad_s;

    /* === 3. TÍNH ĐIỆN ÁP V/f CHUẨN (Mô-men Giữ Đồng Bộ Vững) ===
     * Duy trì V_boost = open_loop_voltage (Mặc định 9.0V = ~2.31A / ~25Nm ngõ ra hộp số)
     * kết hợp bù sức điện động BEMF để kéo tải nặng đĩa Cycloid 1:17 mượt mà không trượt bước. */
    float v_boost = (open_loop_voltage > 1.0f) ? open_loop_voltage : 9.0f;
    float v_bemf  = g_foc_controller.conf.foc_motor_flux_linkage * fabsf(elec_rad_s);
    float v_open  = v_boost + v_bemf;

    /* Clamp tổng điện áp ≤ Vbus/√3 × max_duty */
    float v_max = ONE_BY_SQRT3 * g_foc_controller.conf.l_max_duty * vbus;
    if (v_open > v_max) v_open = v_max;
    state_m->vd = v_open;
    state_m->vq = 0.0f;

    /* === 4. SINH SÓNG SIN CHUẨN → SVPWM === */
    float sin_val, cos_val;
    utils_fast_sincos(open_loop_angle, &sin_val, &cos_val);

    float valpha = (v_open / vbus) * cos_val;
    float vbeta  = (v_open / vbus) * sin_val;

    uint32_t ta, tb, tc, sector;
    foc_svm(valpha, vbeta, g_foc_controller.conf.l_max_duty, 1000, &ta, &tb, &tc, &sector);

    /* === 5. GHI PWM TRỰC TIẾP VÀO TIMER (Đồng bộ theo phase_swap_bc) === */
    uint32_t period = htim1.Init.Period;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (ta * period) / 1000);
    g_foc_controller.duty_a = (float)ta / 1000.0f;
    if (g_foc_controller.phase_swap_bc) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (tc * period) / 1000);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (tb * period) / 1000);
      g_foc_controller.duty_b = (float)tc / 1000.0f;
      g_foc_controller.duty_c = (float)tb / 1000.0f;
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (tb * period) / 1000);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (tc * period) / 1000);
      g_foc_controller.duty_b = (float)tb / 1000.0f;
      g_foc_controller.duty_c = (float)tc / 1000.0f;
    }
    return;
  }

  if (run_direction_test != 1 && run_alignment != 1 && run_calibration != 1) {
    FOC_Control_Current_ISR(&g_foc_controller, current_a, current_b, vbus, temp_fet, dt);

    // ===== 6. CẬP NHẬT PWM DUTY CYCLE TRỰC TIẾP VÀO TIMER =====
    // CHỈ ghi khi FOC ISR đang chạy. Khi alignment/direction test, main loop ghi trực tiếp.
    uint32_t period = htim1.Init.Period;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)(g_foc_controller.duty_a * (float)period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          (uint32_t)(g_foc_controller.duty_b * (float)period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          (uint32_t)(g_foc_controller.duty_c * (float)period));

    // ===== 7. SLOW LOOP 1kHz, scheduled from real elapsed time =====
    slow_loop_elapsed_s += dt;
    if (slow_loop_elapsed_s >= 0.001f) {
      float slow_dt = slow_loop_elapsed_s;
      slow_loop_elapsed_s = 0.0f;
      FOC_Control_SlowLoop(&g_foc_controller, slow_dt);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
