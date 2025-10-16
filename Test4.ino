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
// === THIẾT LẬP PID VÀ TỐC ĐỘ ===
// =====================================================
const int TOC_DO_CO_BAN = 220; 
const int TOC_DO_MAX = 255;

const float Kp = 0.25;
const float Ki = 0.001;
const float Kd = 0.8;

const int NGUONG_VAO_CUA = 200;
const int I_MAX = 400;
const int I_MIN = -400;

// =====================================================
// --- BIẾN TOÀN CỤC ---
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

uint16_t position;
float P, D, I = 0, previousError = 0;
int toc_do_trai, toc_do_phai;

bool dang_re_phai = false;
unsigned long thoi_gian_bat_dau_re = 0;

// =====================================================
// --- HÀM KHỞI ĐỘNG ---
void setup() {
  Serial.begin(115200);
  
  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN}, SensorCount);

  delay(500);
  Serial.println("Bat dau calibrate cam bien...");
  digitalWrite(LED_BUILTIN, LOW);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrate hoan tat!");
  delay(1500);
}

// =====================================================
// --- VÒNG LẶP CHÍNH ---
void loop() {
  xu_ly_nga_tu();       // Kiểm tra và rẽ nếu gặp ngã tư
  if (!dang_re_phai) {  // Nếu không rẽ thì chạy PID bình thường
    robot_control();
  }
}

// =====================================================
// --- HÀM ĐIỀU KHIỂN DÒ LINE ---
void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  float error = 1500.0 - position;
  PID_Linefollow(error);
}

// =====================================================
// --- HÀM PID CHÍNH ---
void PID_Linefollow(float error) {
  P = error;
  I = I + error;

  if (I > I_MAX) I = I_MAX;
  if (I < I_MIN) I = I_MIN;

  D = error - previousError;

  float PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  int toc_do_hien_tai;
  if (abs(error) < NGUONG_VAO_CUA) {
    toc_do_hien_tai = TOC_DO_MAX;
  } else {
    toc_do_hien_tai = TOC_DO_CO_BAN;
  }

  toc_do_trai = toc_do_hien_tai - PID_value;
  toc_do_phai = toc_do_hien_tai + PID_value;

  toc_do_trai = constrain(toc_do_trai, -255, 255);
  toc_do_phai = constrain(toc_do_phai, -255, 255);

  motor_drive(toc_do_trai, toc_do_phai);
}

// =====================================================
// --- HÀM ĐIỀU KHIỂN ĐỘNG CƠ ---
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

void motor_stop() {
  digitalWrite(PWM_PIN_L_A, LOW);
  digitalWrite(PWM_PIN_L_B, LOW);
  digitalWrite(PWM_PIN_R_A, LOW);
  digitalWrite(PWM_PIN_R_B, LOW);
}

// =====================================================
// --- HÀM PHÁT HIỆN & RẼ PHẢI NGÃ TƯ ---
void xu_ly_nga_tu() {
  // Đọc giá trị 4 cảm biến
  qtr.read(sensorValues);

  bool s1 = sensorValues[0] > 500;
  bool s2 = sensorValues[1] > 500;
  bool s3 = sensorValues[2] > 500;
  bool s4 = sensorValues[3] > 500;

  // ======= PHÁT HIỆN NGÃ TƯ =======
  if (s1 && s2 && s3 && s4 && !dang_re_phai) {
    dang_re_phai = true;
    thoi_gian_bat_dau_re = millis();
    Serial.println("↪ Phat hien nga tu! Dang re phai...");
  }

  // ======= XỬ LÝ RẼ PHẢI =======
  if (dang_re_phai) {
    unsigned long thoi_gian_da_re = millis() - thoi_gian_bat_dau_re;

    if (thoi_gian_da_re < 400) {
      // Quay phải tại chỗ
      motor_drive(200, -200);
      return;
    } 
    else if (thoi_gian_da_re < 700) {
      // Chạy thẳng một chút để robot vào lại line
      motor_drive(200, 200);
      return;
    } 
    else {
      // Hoàn tất rẽ
      dang_re_phai = false;
      Serial.println("✅ Da re phai xong!");
    }
  }
}
