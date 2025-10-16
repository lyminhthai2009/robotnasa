#include <QTRSensors.h>

// --- CÁC CHÂN ĐIỀU KHIỂN ĐỘNG CƠ ---
#define PWM_PIN_L_A 2  
#define PWM_PIN_L_B 10 
#define PWM_PIN_R_A 6  
#define PWM_PIN_R_B 5  

// --- CÁC CHÂN CẢM BIẾN ---
#define SENSOR_1_PIN 4 
#define SENSOR_2_PIN 3 
#define SENSOR_3_PIN 1 
#define SENSOR_4_PIN 0 

// ===================================================================
// === KHU VỰC TINH CHỈNH "PRO VIP" V5.0 - RẼ VÒNG CUNG ===
// ===================================================================
const int TOC_DO_CHAY = 255; 
const float Kp = 0.35;
const float Ki = 0;
const float Kd = 1.5;

// --- TINH CHỈNH CHO NGÃ TƯ (PHIÊN BẢN VÒNG CUNG) ---
const int NGUONG_CAM_BIEN_DEN = 800; 
const int THOI_GIAN_DI_THANG = 80;
const int TOC_DO_RE_NGOAI = 255;
const int TOC_DO_RE_TRONG = 50; 
const int THOI_GIAN_RE_VONG_CUNG = 200;

// ===================================================================

QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

float P, D, previousError = 0;
int toc_do_trai, toc_do_phai;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN_L_A, OUTPUT); pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT); pinMode(PWM_PIN_R_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN}, SensorCount);
  delay(500);
  Serial.println("Bat dau Calibrate...");
  digitalWrite(LED_BUILTIN, LOW);
  for (uint16_t i = 0; i < 400; i++) { qtr.calibrate(); }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrate hoan thanh!");
  delay(2000);
}

void loop() {
  robot_control();
}

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  if (sensorValues[0] > NGUONG_CAM_BIEN_DEN && sensorValues[1] > NGUONG_CAM_BIEN_DEN &&
      sensorValues[2] > NGUONG_CAM_BIEN_DEN && sensorValues[3] > NGUONG_CAM_BIEN_DEN) {
    re_phai_vong_cung();
  } else {
    float error = 1500.0 - position;
    PID_Linefollow(error);
  }
}

void PID_Linefollow(float error) {
  P = error;
  D = error - previousError;
  float PID_value = (Kp * P) + (Kd * D);
  previousError = error;
  toc_do_trai = constrain(TOC_DO_CHAY - PID_value, -255, 255);
  toc_do_phai = constrain(TOC_DO_CHAY + PID_value, -255, 255);
  motor_drive(toc_do_trai, toc_do_phai);
}

// *** HÀM RẼ PHẢI V5.0 - SỬ DỤNG KỸ THUẬT RẼ VÒNG CUNG ***
void re_phai_vong_cung() {
  // BƯỚC 1: Đi thẳng một chút để tâm xe vào giữa ngã tư
  motor_drive(TOC_DO_CHAY, TOC_DO_CHAY);
  delay(THOI_GIAN_DI_THANG);

  // BƯỚC 2: Thực hiện cú rẽ vòng cung trong một khoảng thời gian đã định
  motor_drive(TOC_DO_RE_NGOAI, TOC_DO_RE_TRONG);
  delay(THOI_GIAN_RE_VONG_CUNG);
  
  // BƯỚC 3: Xoay nhẹ để "khóa" vào vạch line mới
  motor_drive(200, -200);
  do {
      qtr.readLineBlack(sensorValues);
  } while (sensorValues[1] < NGUONG_CAM_BIEN_DEN && sensorValues[2] < NGUONG_CAM_BIEN_DEN);
}

void motor_drive(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) { analogWrite(PWM_PIN_L_A, leftSpeed); digitalWrite(PWM_PIN_L_B, LOW); } 
    else { digitalWrite(PWM_PIN_L_A, LOW); analogWrite(PWM_PIN_L_B, -leftSpeed); }
    if (rightSpeed >= 0) { analogWrite(PWM_PIN_R_A, rightSpeed); digitalWrite(PWM_PIN_R_B, LOW); } 
    else { digitalWrite(PWM_PIN_R_A, LOW); analogWrite(PWM_PIN_R_B, -rightSpeed); }
}

void motor_stop() {
    digitalWrite(PWM_PIN_L_A, LOW); digitalWrite(PWM_PIN_L_B, LOW);
    digitalWrite(PWM_PIN_R_A, LOW); digitalWrite(PWM_PIN_R_B, LOW);
}
