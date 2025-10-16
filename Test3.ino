#include <QTRSensors.h>

// =====================================================
// --- CẤU HÌNH CHÂN ĐỘNG CƠ ---
#define PWM_PIN_L_A 2  
#define PWM_PIN_L_B 10 
#define PWM_PIN_R_A 6  
#define PWM_PIN_R_B 5  

// --- CẤU HÌNH CẢM BIẾN DÒ LINE ---
#define SENSOR_1_PIN 4 
#define SENSOR_2_PIN 3 
#define SENSOR_3_PIN 1 
#define SENSOR_4_PIN 0 

// =====================================================
// --- THÔNG SỐ HIỆU CHỈNH ---
// Robot chạy nền trắng - vạch đen

const int TOC_DO_CO_BAN = 150;  // tốc độ cơ bản khi vào cua gắt
const int TOC_DO_MAX = 255;     // tốc độ tối đa trên đường thẳng
const int TOC_DO_VUA = 200;     // tốc độ trung bình cho cua vừa

// Hệ số PID
const float Kp = 0.25;
const float Ki = 0.001;
const float Kd = 1.0;

// Giới hạn I (chống tràn tích phân)
const int I_MAX = 400;
const int I_MIN = -400;

// =====================================================

QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

float P, I = 0, D, previousError = 0;
int toc_do_trai, toc_do_phai;
int toc_do_hien_tai = TOC_DO_CO_BAN;  // để làm mượt tốc độ

// =====================================================
void setup() {
  Serial.begin(115200);

  // --- cấu hình động cơ ---
  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // --- cấu hình cảm biến ---
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN}, SensorCount);
  delay(500);

  Serial.println("=== BAT DAU CALIBRATE TRONG 5S ===");
  digitalWrite(LED_BUILTIN, LOW);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("CALIBRATE XONG - BAT DAU CHAY SAU 2 GIAY");
  delay(2000);
}

// =====================================================
void loop() {
  robot_control();
}

// =====================================================
void robot_control() {
  qtr.read(sensorValues);

  // Kiểm tra ngã tư (tất cả đều > 800)
  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800 && sensorValues[3] > 800) {
    Serial.println("== NGA TU - RE PHAI ==");
    re_phai_ngat_tu();
    return;
  }

  // Nếu không phải ngã tư -> chạy dò line như bình thường
  uint16_t position = qtr.readLineBlack(sensorValues);
  float error = 1500.0 - position;  // giữa 4 cảm biến (0-3000)

  PID_Linefollow(error);
}

// =====================================================
void PID_Linefollow(float error) {
  // --- PID ---
  P = error;
  I += error;

  if (I > I_MAX) I = I_MAX;
  if (I < I_MIN) I = I_MIN;

  D = error - previousError;
  previousError = error;

  float PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  // --- Tính tốc độ mục tiêu theo độ lệch ---
  int toc_do_muc_tieu;
  if (abs(error) < 300) toc_do_muc_tieu = TOC_DO_MAX;   // đường thẳng
  else if (abs(error) < 800) toc_do_muc_tieu = TOC_DO_VUA; // cua vừa
  else toc_do_muc_tieu = TOC_DO_CO_BAN;                 // cua gắt

  // --- Làm mượt tốc độ (ramp up/down) ---
  if (toc_do_hien_tai < toc_do_muc_tieu) toc_do_hien_tai += 2;
  else if (toc_do_hien_tai > toc_do_muc_tieu) toc_do_hien_tai -= 2;

  // --- Tính tốc độ từng bánh ---
  toc_do_trai = toc_do_hien_tai - PID_value;
  toc_do_phai = toc_do_hien_tai + PID_value;

  toc_do_trai = constrain(toc_do_trai, -255, 255);
  toc_do_phai = constrain(toc_do_phai, -255, 255);

  motor_drive(toc_do_trai, toc_do_phai);

  // Debug (bật nếu cần)
  // Serial.print("Err: "); Serial.print(error);
  // Serial.print("  L: "); Serial.print(toc_do_trai);
  // Serial.print("  R: "); Serial.print(toc_do_phai);
  // Serial.print("  T: "); Serial.println(toc_do_hien_tai);
}

// =====================================================
void motor_drive(int leftSpeed, int rightSpeed) {
  // --- Bánh trái ---
  if (leftSpeed >= 0) {
    analogWrite(PWM_PIN_L_A, leftSpeed);
    digitalWrite(PWM_PIN_L_B, LOW);
  } else {
    digitalWrite(PWM_PIN_L_A, LOW);
    analogWrite(PWM_PIN_L_B, -leftSpeed);
  }

  // --- Bánh phải ---
  if (rightSpeed >= 0) {
    analogWrite(PWM_PIN_R_A, rightSpeed);
    digitalWrite(PWM_PIN_R_B, LOW);
  } else {
    digitalWrite(PWM_PIN_R_A, LOW);
    analogWrite(PWM_PIN_R_B, -rightSpeed);
  }
}

// =====================================================
void motor_stop() {
  digitalWrite(PWM_PIN_L_A, LOW);
  digitalWrite(PWM_PIN_L_B, LOW);
  digitalWrite(PWM_PIN_R_A, LOW);
  digitalWrite(PWM_PIN_R_B, LOW);
}

// =====================================================
void re_phai_ngat_tu() {
  motor_drive(200, -200);   // quay phải tại chỗ
  delay(400);               // thời gian rẽ ~0.4s (chỉnh theo thực tế)
  motor_drive(180, 180);    // chạy thẳng ra khỏi ngã tư
  delay(300);
}
