#include <QTRSensors.h>

// =====================================================
// --- CẤU HÌNH CHÂN ĐỘNG CƠ ---
#define PWM_PIN_L_A 2  
#define PWM_PIN_L_B 10 
#define PWM_PIN_R_A 6  
#define PWM_PIN_R_B 5  

// --- CẤU HÌNH CẢM BIẾN ---
#define SENSOR_1_PIN 4 
#define SENSOR_2_PIN 3 
#define SENSOR_3_PIN 1 
#define SENSOR_4_PIN 0 

// =====================================================
// === KHU VỰC TINH CHỈNH "PRO MAX" ===
// =====================================================

// --- TỐC ĐỘ ---
const int TOC_DO_CO_BAN = 220;  // Tốc độ khi vào cua
const int TOC_DO_MAX = 255;     // Tốc độ tối đa đường thẳng

// --- PID ---
const float Kp = 0.25;  
const float Ki = 0.0008;  
const float Kd = 0.85;   

// --- NGƯỠNG PHÁT HIỆN ĐƯỜNG CONG ---
const int NGUONG_VAO_CUA = 200;  

// --- GIỚI HẠN INTEGRAL ---
const int I_MAX = 400;
const int I_MIN = -400;

// --- TRUNG TÂM ĐƯỜNG ---
const int VI_TRI_TRUNG_TAM = 1500;

// --- DEADZONE CHO ERROR (giúp đi thẳng mượt hơn) ---
const int DEADZONE = 5;

// --- GIỚI HẠN PHÁT HIỆN MẤT LINE ---
const int NGUONG_MAT_LINE = 200;

// =====================================================

QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

// PID
float P, I = 0, D, previousError = 0;
uint16_t position;
int toc_do_trai, toc_do_phai;

// PWM
const int freq = 1000;         // Tần số PWM
const int resolution = 8;      // 8-bit: giá trị 0-255
const int channel_L_A = 0;
const int channel_L_B = 1;
const int channel_R_A = 2;
const int channel_R_B = 3;

// =====================================================

void setup() {
  Serial.begin(115200);

  // --- CÀI ĐẶT PWM ---
  ledcSetup(channel_L_A, freq, resolution);
  ledcSetup(channel_L_B, freq, resolution);
  ledcSetup(channel_R_A, freq, resolution);
  ledcSetup(channel_R_B, freq, resolution);

  ledcAttachPin(PWM_PIN_L_A, channel_L_A);
  ledcAttachPin(PWM_PIN_L_B, channel_L_B);
  ledcAttachPin(PWM_PIN_R_A, channel_R_A);
  ledcAttachPin(PWM_PIN_R_B, channel_R_B);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  // --- CẢM BIẾN ---
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN}, SensorCount);

  delay(500);
  Serial.println("Bat dau calibrate trong 5 giay...");

  digitalWrite(LED_BUILTIN, LOW);

  // --- Calibrate ---
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrate xong! Bat dau chay sau 2 giay.");
  delay(2000);
}

// =====================================================

void loop() {
  dieu_khien_robot();
}

// =====================================================

void dieu_khien_robot() {
  position = qtr.readLineBlack(sensorValues); // Nền trắng, vạch đen

  float error = VI_TRI_TRUNG_TAM - position;

  // --- DEADZONE ---
  if (abs(error) < DEADZONE) error = 0;

  // --- KIỂM TRA MẤT LINE ---
  bool mat_line = true;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > NGUONG_MAT_LINE) mat_line = false;
  }

  if (mat_line) {
    motor_stop();
    Serial.println("Mat line! Dung lai...");
    delay(100);
    return;
  }

  // --- PID ---
  P = error;
  I += error;
  if (I > I_MAX) I = I_MAX;
  if (I < I_MIN) I = I_MIN;

  // Lọc D cho mượt hơn (chống rung)
  D = (error - previousError) * 0.9 + D * 0.1;

  float PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // --- ĐIỀU CHỈNH TỐC ĐỘ THEO CUA ---
  int toc_do_hien_tai = (abs(error) < NGUONG_VAO_CUA) ? TOC_DO_MAX : TOC_DO_CO_BAN;

  toc_do_trai = toc_do_hien_tai - PID_value;
  toc_do_phai = toc_do_hien_tai + PID_value;

  toc_do_trai = constrain(toc_do_trai, -255, 255);
  toc_do_phai = constrain(toc_do_phai, -255, 255);

  motor_drive(toc_do_trai, toc_do_phai);
}

// =====================================================
// --- HÀM ĐIỀU KHIỂN ĐỘNG CƠ ---
// =====================================================

void motor_drive(int leftSpeed, int rightSpeed) {
  // Trái
  if (leftSpeed >= 0) {
    ledcWrite(channel_L_A, leftSpeed);
    ledcWrite(channel_L_B, 0);
  } else {
    ledcWrite(channel_L_A, 0);
    ledcWrite(channel_L_B, -leftSpeed);
  }

  // Phải
  if (rightSpeed >= 0) {
    ledcWrite(channel_R_A, rightSpeed);
    ledcWrite(channel_R_B, 0);
  } else {
    ledcWrite(channel_R_A, 0);
    ledcWrite(channel_R_B, -rightSpeed);
  }
}

void motor_stop() {
  ledcWrite(channel_L_A, 0);
  ledcWrite(channel_L_B, 0);
  ledcWrite(channel_R_A, 0);
  ledcWrite(channel_R_B, 0);
}
