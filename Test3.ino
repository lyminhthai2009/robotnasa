#include <QTRSensors.h>

// =====================================================
// --- CẤU HÌNH CHÂN ĐỘNG CƠ ---
#define PWM_PIN_L_A 2  
#define PWM_PIN_L_B 10 
#define PWM_PIN_R_A 6  
#define PWM_PIN_R_B 5  

// --- CẤU HÌNH CẢM BIẾN DÒ LINE ---
// ⚠️ CHÚ Ý: từ TRÁI → PHẢI
#define SENSOR_1_PIN 4   // trái ngoài
#define SENSOR_2_PIN 3   // trái trong
#define SENSOR_3_PIN 1   // phải trong
#define SENSOR_4_PIN 0   // phải ngoài

// =====================================================
// --- THÔNG SỐ HIỆU CHỈNH ---
const int TOC_DO_CO_BAN = 140;   // tốc độ cơ bản (vào cua gắt)
const int TOC_DO_VUA = 200;      // tốc độ trung bình (cua nhẹ)
const int TOC_DO_MAX = 255;      // tốc độ tối đa (đường thẳng)

// --- HỆ SỐ PID ---
const float Kp = 0.25;
const float Ki = 0.001;
const float Kd = 1.0;

// Giới hạn I (anti wind-up)
const int I_MAX = 400;
const int I_MIN = -400;

// =====================================================
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

float P, I = 0, D, previousError = 0;
int toc_do_trai, toc_do_phai;
int toc_do_hien_tai = TOC_DO_CO_BAN;

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

  // ⚠️ Trong lúc này hãy đưa robot qua lại để cảm biến quét line đen & nền trắng
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("CALIBRATE XONG - BAT DAU SAU 2 GIAY");
  delay(2000);
}

// =====================================================
void loop() {
  robot_control();
}

// =====================================================
void robot_control() {
  qtr.read(sensorValues);

  // --- Phát hiện ngã tư (mọi cảm biến đều nhận line đen) ---
  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] > 800 && sensorValues[3] > 800) {
    Serial.println("== NGA TU - RE PHAI ==");
    re_phai_ngat_tu();
    return;
  }

  // --- Đọc vị trí line đen (nền trắng) ---
  uint16_t position = qtr.readLineBlack(sensorValues);

  // --- Tính sai số ---
  // Nếu cảm biến bị đảo hướng, đổi dấu dòng dưới thành: position - 1500
  float error = 1500.0 - position;  // 0–3000 (4 cảm biến → trung tâm 1500)

  PID_Linefollow(error);
}

// =====================================================
void PID_Linefollow(float error) {
  P = error;
  I += error;
  if (I > I_MAX) I = I_MAX;
  if (I < I_MIN) I = I_MIN;
  D = error - previousError;
  previousError = error;

  float PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  // --- Xác định tốc độ mục tiêu ---
  int toc_do_muc_tieu;
  if (abs(error) < 300) toc_do_muc_tieu = TOC_DO_MAX;    // thẳng
  else if (abs(error) < 800) toc_do_muc_tieu = TOC_DO_VUA; // cua nhẹ
  else toc_do_muc_tieu = TOC_DO_CO_BAN;                  // cua gắt

  // --- Làm mượt tốc độ ---
  if (toc_do_hien_tai < toc_do_muc_tieu) toc_do_hien_tai += 2;
  else if (toc_do_hien_tai > toc_do_muc_tieu) toc_do_hien_tai -= 2;

  toc_do_trai = toc_do_hien_tai - PID_value;
  toc_do_phai = toc_do_hien_tai + PID_value;

  toc_do_trai = constrain(toc_do_trai, -255, 255);
  toc_do_phai = constrain(toc_do_phai, -255, 255);

  motor_drive(toc_do_trai, toc_do_phai);
}

// =====================================================
void motor_drive(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    analogWrite(PWM_PIN_L_A, leftSpeed);
    digitalWrite(PWM_PIN_L_B, LOW);
  } else {
    digitalWrite(PWM_PIN_L_A, LOW);
    analogWrite(PWM_PIN_L_B, -leftSpeed);
  }

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
  motor_drive(200, -200);   // quay phải
  delay(400);
  motor_drive(180, 180);    // đi thẳng ra khỏi ngã tư
  delay(300);
}
