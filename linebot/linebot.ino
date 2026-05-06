/*
 * LineBot – Line Following Robot
 * ESP32 + TB6612FNG + HC4067 MUX + SSD1306 OLED + 8 IR sensors
 * v2.0 – BLE web dashboard, non-blocking calibration, fixed buttons
 *
 * BLE device name : "LineBot"
 * Requires libraries:
 *   Adafruit_GFX, Adafruit_SSD1306 (từ Library Manager)
 *   ESP32 BLE Arduino (có sẵn trong ESP32 board package)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// OLED
// =====================================================
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C
#define OLED_SDA   21
#define OLED_SCL   22

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// =====================================================
// Nút bấm – điện trở kéo XUỐNG (active HIGH)
// =====================================================
#define BTN_MODE_PIN  2    // nhấn ngắn: chuyển trang OLED
#define BTN_CAL_PIN   4    // nhấn ngắn: RUN/STOP  |  giữ lâu: CALIB

#define DEBOUNCE_MS   40UL
#define LONG_PRESS_MS 1200UL

struct Btn {
  uint8_t       pin;
  bool          lastRaw;
  bool          stable;
  unsigned long debounceAt;
  unsigned long pressAt;
};

Btn bMode = {BTN_MODE_PIN, false, false, 0, 0};
Btn bCal  = {BTN_CAL_PIN,  false, false, 0, 0};

/* Trả về: 0=không có gì, 1=nhấn ngắn, 2=giữ lâu
 * Chỉ phát sự kiện khi THẢ nút (edge detection sạch) */
int pollBtn(Btn &b) {
  bool raw = (digitalRead(b.pin) == HIGH);

  // phát hiện thay đổi tín hiệu, reset bộ lọc debounce
  if (raw != b.lastRaw) {
    b.lastRaw    = raw;
    b.debounceAt = millis();
    return 0;
  }

  // chưa qua thời gian debounce → bỏ qua
  if (millis() - b.debounceAt < DEBOUNCE_MS) return 0;

  // tín hiệu stable chưa thay đổi so với lần trước
  if (raw == b.stable) return 0;

  b.stable = raw;

  if (raw) {
    // vừa NHẤN xuống → ghi nhớ thời điểm, chờ thả
    b.pressAt = millis();
    return 0;
  } else {
    // vừa THẢ ra → tính thời gian giữ
    unsigned long held = millis() - b.pressAt;
    return (held >= LONG_PRESS_MS) ? 2 : 1;
  }
}

// =====================================================
// HC4067 – 16-kênh MUX
// =====================================================
#define MUX_S0   23
#define MUX_S1   19
#define MUX_S2   18
#define MUX_SIG  13    // chân ADC

#define NUM_SENSORS 8

uint16_t sensorRaw [NUM_SENSORS];
uint16_t sensorMin [NUM_SENSORS];
uint16_t sensorMax [NUM_SENSORS];
uint16_t sensorTh  [NUM_SENSORS];
uint16_t sensorNorm[NUM_SENSORS];   // độ đậm đen 0–1000
bool     sensorBW  [NUM_SENSORS];   // true=đen, false=trắng

const bool SENSOR_ORDER_REVERSED = false;
bool blackIsHigh = false;
bool calibrated  = false;

// =====================================================
// Motor – TB6612FNG
// =====================================================
#define AIN1 25
#define AIN2 33
#define PWMA 32
#define BIN1 26
#define BIN2 27
#define PWMB 14

#define PWM_FREQ 20000
#define PWM_RES    8
#define CH_A       0
#define CH_B       1

const bool MOTOR_A_REV = false;   // đổi true nếu motor A quay ngược
const bool MOTOR_B_REV = false;   // đổi true nếu motor B quay ngược

// =====================================================
// Tham số điều khiển – tuỳ chỉnh qua BLE web dashboard
// =====================================================
float Kp          = 0.050f;
float Kd          = 0.220f;
int   baseSpeed   = 150;
int   maxSpeed    = 255;
int   searchSpeed = 110;
int   lineDetTh   = 900;    // tổng black-strength tối thiểu để xem là thấy line

#define CENTER_POS 3500     // vị trí trung tâm (8 cảm biến × 1000 / 2)

// =====================================================
// Trạng thái robot
// =====================================================
enum RobotState { ST_STOP, ST_CAL, ST_RUN };
RobotState robotState = ST_STOP;

int oledPage = 0;  // 0–3

// =====================================================
// Biến debug runtime
// =====================================================
int  lastPos   = CENTER_POS;
int  lastErr   = 0;
int  lastLPWM  = 0;
int  lastRPWM  = 0;
bool lastFound = false;

// =====================================================
// Calibration – state machine KHÔNG CHẶN LOOP
// =====================================================
enum CalPhase { CAL_IDLE, CAL_SWEEP, CAL_PLACING, CAL_SAMPLING, CAL_DONE };
CalPhase     calPhase   = CAL_IDLE;
unsigned long calTimer  = 0;
uint32_t     calSum34   = 0;
int          calSamples = 0;

#define SWEEP_MS  5000UL
#define PLACE_MS  2000UL
#define SAMPLE_N  80

// =====================================================
// BLE – UUIDs & objects
// =====================================================
#define BLE_DEVICE_NAME "LineBot"

// Tạo UUIDs ngẫu nhiên cho service và các characteristic
#define SVC_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEL_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Notify  – telemetry
#define CFG_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // R+W     – params
#define CMD_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // Write   – lệnh

BLECharacteristic *pTelChar = nullptr;
BLECharacteristic *pCfgChar = nullptr;
bool bleConnected = false;

// Cờ giao tiếp giữa BLE callback (task riêng) và loop() chính
volatile bool bleCmdRun    = false;
volatile bool bleCmdStop   = false;
volatile bool bleCmdCal    = false;
volatile bool bleNewParams = false;

// Giá trị params tạm – được set trong callback, áp dụng trong loop()
volatile float vKp, vKd;
volatile int   vBase, vMax, vSrch, vLineTh;

// ─── BLE Server callbacks ──────────────────────────
class BleServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    bleConnected = false;
    // tự restart advertising để thiết bị khác có thể kết nối lại
    delay(500);
    pSrv->startAdvertising();
  }
};

// ─── Phân tích chuỗi config ───────────────────────
// Định dạng: "KP:0.050|KD:0.220|BASE:150|MAX:255|SRCH:110|LINETH:900"
// Từng trường là tuỳ chọn (partial update OK)
void parseConfigString(const String &s,
                       float &kp, float &kd,
                       int &base, int &mx, int &srch, int &lineTh) {
  kp=Kp; kd=Kd; base=baseSpeed; mx=maxSpeed;
  srch=searchSpeed; lineTh=lineDetTh;  // giá trị mặc định

  int p = 0;
  while (p < (int)s.length()) {
    int pipe  = s.indexOf('|', p);
    if (pipe < 0) pipe = s.length();
    String tok = s.substring(p, pipe);
    int    col  = tok.indexOf(':');
    if (col > 0) {
      String key = tok.substring(0, col);
      String val = tok.substring(col + 1);
      if      (key == "KP")     kp     = val.toFloat();
      else if (key == "KD")     kd     = val.toFloat();
      else if (key == "BASE")   base   = val.toInt();
      else if (key == "MAX")    mx     = val.toInt();
      else if (key == "SRCH")   srch   = val.toInt();
      else if (key == "LINETH") lineTh = val.toInt();
    }
    p = pipe + 1;
  }
}

// ─── Config characteristic callback ───────────────
class CfgCB : public BLECharacteristicCallbacks {
  // Laptop GHI params mới xuống ESP32
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();
    parseConfigString(v, vKp, vKd, vBase, vMax, vSrch, vLineTh);
    bleNewParams = true;
  }
  // Laptop ĐỌC params hiện tại từ ESP32
  void onRead(BLECharacteristic *c) override {
    char buf[72];
    snprintf(buf, sizeof(buf),
      "KP:%.3f|KD:%.3f|BASE:%d|MAX:%d|SRCH:%d|LINETH:%d",
      Kp, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);
    c->setValue(buf);
  }
};

// ─── Command characteristic callback ──────────────
class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String cmd = c->getValue().c_str();
    cmd.trim();
    if      (cmd == "RUN")  bleCmdRun  = true;
    else if (cmd == "STOP") bleCmdStop = true;
    else if (cmd == "CAL")  bleCmdCal  = true;
  }
};

// ─── Khởi tạo BLE ─────────────────────────────────
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);  // đủ cho cả telemetry lẫn config string

  BLEServer  *pSrv = BLEDevice::createServer();
  pSrv->setCallbacks(new BleServerCB());

  BLEService *pSvc = pSrv->createService(SVC_UUID);

  // Telemetry – chỉ NOTIFY (ESP32 → laptop)
  pTelChar = pSvc->createCharacteristic(TEL_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTelChar->addDescriptor(new BLE2902());

  // Config – READ + WRITE (hai chiều)
  pCfgChar = pSvc->createCharacteristic(CFG_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCfgChar->setCallbacks(new CfgCB());

  // Command – chỉ WRITE (laptop → ESP32)
  BLECharacteristic *pCmdChar = pSvc->createCharacteristic(CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCmdChar->setCallbacks(new CmdCB());

  pSvc->start();

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SVC_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);  // giúp iOS/Android nhận diện tốt hơn
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as \"" BLE_DEVICE_NAME "\"");
}

// ─── Gửi telemetry ─────────────────────────────────
void sendTelemetry() {
  static unsigned long lastT = 0;
  if (!bleConnected || millis() - lastT < 120) return;
  lastT = millis();

  char bw[9];
  for (int i = 0; i < 8; i++) bw[i] = sensorBW[i] ? '1' : '0';
  bw[8] = '\0';

  const char *st = (robotState == ST_RUN) ? "RUN"
                 : (robotState == ST_CAL) ? "CAL" : "STOP";

  char buf[80];
  snprintf(buf, sizeof(buf),
    "S:%s|P:%d|E:%d|L:%d|R:%d|F:%d|BW:%s",
    st, lastPos, lastErr, lastLPWM, lastRPWM, (int)lastFound, bw);

  pTelChar->setValue(buf);
  pTelChar->notify();
}

// =====================================================
// MUX – đọc cảm biến
// =====================================================
void selectMux(uint8_t ch) {
  digitalWrite(MUX_S0,  ch       & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
}

uint16_t readMuxCh(uint8_t ch) {
  selectMux(ch);
  delayMicroseconds(8);
  analogRead(MUX_SIG);          // throw-away (ổn định mux)
  delayMicroseconds(8);
  uint32_t s = 0;
  for (int i = 0; i < 4; i++) { s += analogRead(MUX_SIG); delayMicroseconds(5); }
  return (uint16_t)(s / 4);
}

void readAllRaw() {
  for (int i = 0; i < NUM_SENSORS; i++) sensorRaw[i] = readMuxCh(i);
}

// =====================================================
// Motor
// =====================================================
void driveMotor(int ch, int in1, int in2, int pwm, bool rev) {
  pwm = constrain(pwm, -255, 255);
  if (rev) pwm = -pwm;
  if (pwm > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  ledcWrite(ch,  pwm); }
  else if (pwm < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); ledcWrite(ch, -pwm); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  ledcWrite(ch,   0);  }
}

void setMotors(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  driveMotor(CH_A, AIN1, AIN2, l, MOTOR_A_REV);
  driveMotor(CH_B, BIN1, BIN2, r, MOTOR_B_REV);
  lastLPWM = l;  lastRPWM = r;
}

void stopMotors() { setMotors(0, 0); }

// =====================================================
// Sensor processing
// =====================================================
void clearCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
    sensorTh [i] = 2048;
  }
}

void computeThresholds() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] < sensorMin[i] + 10) sensorMax[i] = sensorMin[i] + 10;
    sensorTh[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }
}

void processSensors() {
  readAllRaw();
  for (int i = 0; i < NUM_SENSORS; i++) {
    int lo = sensorMin[i], hi = sensorMax[i];
    if (hi < lo + 10) hi = lo + 10;
    long m = map(sensorRaw[i], lo, hi, 0, 1000);
    m = constrain(m, 0, 1000);
    if (blackIsHigh) {
      sensorNorm[i] = (uint16_t)m;
      sensorBW  [i] = (sensorRaw[i] > sensorTh[i]);
    } else {
      sensorNorm[i] = (uint16_t)(1000 - m);
      sensorBW  [i] = (sensorRaw[i] < sensorTh[i]);
    }
  }
}

bool getLinePos(int &posOut) {
  processSensors();
  long ws = 0, total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int idx = SENSOR_ORDER_REVERSED ? (NUM_SENSORS - 1 - i) : i;
    ws    += (long)sensorNorm[i] * idx * 1000;
    total += sensorNorm[i];
  }
  if (total < lineDetTh) return false;
  posOut = (int)(ws / total);
  return true;
}

// =====================================================
// Calibration state machine – KHÔNG CHẶN loop()
// =====================================================
void startCalibration() {
  robotState = ST_CAL;
  calibrated = false;
  stopMotors();
  clearCalibration();
  calPhase   = CAL_SWEEP;
  calTimer   = millis();
  calSum34   = 0;
  calSamples = 0;
  Serial.println("[CAL] Started – Phase 1: Sweep");
}

void updateCalibration() {
  if (calPhase == CAL_IDLE) return;

  unsigned long now = millis();
  switch (calPhase) {

    /* ─── Pha 1: quét min/max ─────────────────────────
       Di chuyển xe qua TRẮNG và ĐEN trong 5 giây       */
    case CAL_SWEEP:
      readAllRaw();
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
        if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
      }
      if (now - calTimer >= SWEEP_MS) {
        computeThresholds();
        calPhase = CAL_PLACING;
        calTimer = now;
        Serial.println("[CAL] Phase 2: Place center sensors on BLACK");
      }
      break;

    /* ─── Pha 2a: chờ người đặt xe ───────────────────
       Đặt 2 cảm biến giữa (C3, C4) lên vạch đen        */
    case CAL_PLACING:
      if (now - calTimer >= PLACE_MS) {
        calPhase   = CAL_SAMPLING;
        calSum34   = 0;
        calSamples = 0;
      }
      break;

    /* ─── Pha 2b: lấy mẫu để xác định black polarity ─ */
    case CAL_SAMPLING:
      readAllRaw();
      calSum34 += sensorRaw[3] + sensorRaw[4];
      calSamples++;
      if (calSamples >= SAMPLE_N) {
        uint32_t avg  = calSum34 / (calSamples * 2);
        uint32_t th34 = ((uint32_t)sensorTh[3] + sensorTh[4]) / 2;
        blackIsHigh = (avg > th34);
        calibrated  = true;
        calPhase    = CAL_DONE;
        Serial.printf("[CAL] Done. blackIsHigh=%d\n", blackIsHigh);
      }
      break;

    /* ─── Kết thúc ────────────────────────────────── */
    case CAL_DONE:
      robotState = ST_RUN;
      calPhase   = CAL_IDLE;
      lastErr    = 0;
      break;

    default: break;
  }
}

// =====================================================
// OLED – 4 trang
// =====================================================
const char *stateStr() {
  if (robotState == ST_RUN) return "RUN";
  if (robotState == ST_CAL) return "CAL";
  return "STOP";
}

void drawCalPage() {
  display.setCursor(0, 0);
  display.print(">>> CALIBRATING <<<");
  unsigned long elapsed = millis() - calTimer;

  switch (calPhase) {
    case CAL_SWEEP: {
      int pct = (int)min(100UL, elapsed * 100 / SWEEP_MS);
      display.setCursor(0, 12); display.print("1/2 Di chuyen xe");
      display.setCursor(0, 22); display.print("qua TRANG & DEN");
      display.setCursor(0, 36); display.print("Con lai: ");
      display.print(max(0, (int)(SWEEP_MS - elapsed) / 1000));
      display.print("s");
      display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 51, pct * 126 / 100, 7, SSD1306_WHITE);
      break;
    }
    case CAL_PLACING:
      display.setCursor(0, 14); display.print("2/2 Dat C3+C4 vao");
      display.setCursor(0, 26); display.print("VACH DEN, giu yen");
      display.setCursor(0, 44); display.print("Chuan bi...");
      break;
    case CAL_SAMPLING: {
      int pct2 = calSamples * 100 / SAMPLE_N;
      display.setCursor(0, 14); display.print("2/2 Dang do mau...");
      display.setCursor(0, 28); display.print("GIU YEN!");
      display.drawRect(0, 44, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 45, pct2 * 126 / 100, 7, SSD1306_WHITE);
      break;
    }
    default: break;
  }
}

void drawPageRaw() {
  bool hi = ((millis() / 1200) % 2) == 1;
  int  s  = hi ? 4 : 0;
  display.setCursor(0, 0); display.print("RAW "); display.print(hi ? "4-7" : "0-3");
  display.print("  "); display.print(stateStr());
  for (int r = 0; r < 4; r++) {
    int ch = s + r;
    display.setCursor(0, 14 + r * 12);
    display.print("C"); display.print(ch); display.print(":"); display.print(sensorRaw[ch]);
  }
}

void drawPageBW() {
  display.setCursor(0,  0); display.print("BW  "); display.print(stateStr());
  display.setCursor(0, 12);
  for (int i = 0; i < NUM_SENSORS; i++) { display.print(sensorBW[i] ? '1' : '0'); display.print(' '); }
  display.setCursor(0, 26); display.print("POS:"); display.print(lastPos);
  display.setCursor(68, 26); display.print("ERR:"); display.print(lastErr);
  display.setCursor(0, 40); display.print("FND:"); display.print(lastFound ? "Y" : "N");
  display.setCursor(72, 40); display.print("B="); display.print(blackIsHigh ? "HI" : "LO");
  display.setCursor(0, 54); display.print("BLE:"); display.print(bleConnected ? "CONNECTED" : "--");
}

void drawPageThreshold() {
  bool hi = ((millis() / 1200) % 2) == 1;
  int  s  = hi ? 4 : 0;
  display.setCursor(0, 0); display.print("TH "); display.print(hi ? "4-7" : "0-3");
  display.print(calibrated ? " OK" : " NO");
  for (int r = 0; r < 4; r++) {
    int ch = s + r;
    display.setCursor(0, 14 + r * 12);
    display.print("C"); display.print(ch); display.print(":"); display.print(sensorTh[ch]);
  }
}

void drawPageMotor() {
  display.setCursor(0,  0); display.print("PID  "); display.print(stateStr());
  display.setCursor(0, 12); display.print("Kp:"); display.print(Kp, 3);
  display.setCursor(64, 12); display.print("Kd:"); display.print(Kd, 3);
  display.setCursor(0, 24); display.print("base:"); display.print(baseSpeed);
  display.print(" max:"); display.print(maxSpeed);
  display.setCursor(0, 36); display.print("srch:"); display.print(searchSpeed);
  display.print(" th:"); display.print(lineDetTh);
  display.setCursor(0, 48); display.print("L:"); display.print(lastLPWM);
  display.print("  R:"); display.print(lastRPWM);
  display.setCursor(0, 58); display.print("BLE:");
  display.print(bleConnected ? "ON " : "-- ");
  display.print(BLE_DEVICE_NAME);
}

void renderOLED() {
  static unsigned long lastT = 0;
  if (millis() - lastT < 80) return;
  lastT = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (calPhase != CAL_IDLE) {
    drawCalPage();
  } else {
    switch (oledPage) {
      case 0: drawPageRaw();       break;
      case 1: drawPageBW();        break;
      case 2: drawPageThreshold(); break;
      case 3: drawPageMotor();     break;
      default: drawPageRaw();      break;
    }
  }

  display.display();
}

// =====================================================
// Serial debug
// =====================================================
void renderSerial() {
  static unsigned long lastT = 0;
  if (millis() - lastT < 200) return;
  lastT = millis();

  Serial.printf("STATE=%s POS=%d ERR=%d L=%d R=%d BW=",
    stateStr(), lastPos, lastErr, lastLPWM, lastRPWM);
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(sensorBW[i] ? '1' : '0');
  Serial.print(" RAW=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorRaw[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

// =====================================================
// Xử lý nút và lệnh BLE
// =====================================================
void handleInputs() {
  int mEv = pollBtn(bMode);
  int cEv = pollBtn(bCal);

  // ── Nút MODE: nhấn ngắn = chuyển trang OLED ────
  if (mEv == 1) {
    oledPage = (oledPage + 1) % 4;
  }

  // ── Nút CAL: giữ lâu = calibration; ngắn = RUN/STOP
  if (cEv == 2) {
    // Giữ lâu → bắt đầu calibrate (bất kể trạng thái)
    startCalibration();
  } else if (cEv == 1) {
    // Nhấn ngắn
    if (!calibrated) {
      // Chưa calibrate → hiện cảnh báo trên serial
      Serial.println("[WARN] Chua calibrate! Giu nut CAL 1.2s de bat dau calib.");
    } else if (calPhase == CAL_IDLE) {
      // Toggle RUN ↔ STOP
      if (robotState == ST_RUN) {
        robotState = ST_STOP;
        stopMotors();
        Serial.println("[BTN] STOP");
      } else {
        robotState = ST_RUN;
        lastErr    = 0;
        Serial.println("[BTN] RUN");
      }
    }
  }

  // ── Lệnh BLE (được set trong BLE callback task) ─
  if (bleNewParams) {
    bleNewParams = false;
    // Áp dụng và kẹp giới hạn an toàn
    Kp          = constrain(vKp,    0.000f, 2.000f);
    Kd          = constrain(vKd,    0.000f, 5.000f);
    baseSpeed   = constrain(vBase,  0, 255);
    maxSpeed    = constrain(vMax,   0, 255);
    searchSpeed = constrain(vSrch,  0, 255);
    lineDetTh   = constrain(vLineTh, 50, 8000);
    Serial.printf("[BLE] Params: Kp=%.3f Kd=%.3f base=%d max=%d srch=%d lineTh=%d\n",
      Kp, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);
  }

  if (bleCmdRun)  { bleCmdRun  = false; if (calibrated && calPhase == CAL_IDLE) { robotState = ST_RUN;  lastErr = 0; Serial.println("[BLE] RUN"); } }
  if (bleCmdStop) { bleCmdStop = false; robotState = ST_STOP; stopMotors();  Serial.println("[BLE] STOP"); }
  if (bleCmdCal)  { bleCmdCal  = false; startCalibration(); }
}

// =====================================================
// Line following
// =====================================================
void runLineFollower() {
  int  pos   = CENTER_POS;
  bool found = getLinePos(pos);

  lastFound = found;
  lastPos   = pos;

  if (!found) {
    // Mất line → xoay về phía lỗi cuối biết
    if (lastErr < 0) setMotors(-searchSpeed,  searchSpeed);
    else             setMotors( searchSpeed, -searchSpeed);
    return;
  }

  int err  = pos - CENTER_POS;
  int dErr = err - lastErr;
  lastErr  = err;

  float corr  = Kp * err + Kd * dErr;
  corr = constrain(corr, -200.0f, 200.0f);

  int lPWM = constrain(baseSpeed + (int)corr, 0, maxSpeed);
  int rPWM = constrain(baseSpeed - (int)corr, 0, maxSpeed);

  setMotors(lPWM, rPWM);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] LineBot v2.0");

  // Nút bấm
  pinMode(BTN_MODE_PIN, INPUT);
  pinMode(BTN_CAL_PIN,  INPUT);

  // MUX
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
  digitalWrite(MUX_S0, LOW); digitalWrite(MUX_S1, LOW); digitalWrite(MUX_S2, LOW);

  // Motor
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);

  ledcSetup(CH_A, PWM_FREQ, PWM_RES); ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); ledcAttachPin(PWMB, CH_B);

  // ADC
  analogReadResolution(12);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERR] OLED init failed!");
    while (1) delay(1000);
  }

  clearCalibration();
  stopMotors();
  readAllRaw();

  // BLE
  setupBLE();

  // Splash screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(14,  2); display.print("=== LINE BOT v2 ===");
  display.setCursor(0,  16); display.print("BLE: "); display.print(BLE_DEVICE_NAME);
  display.setCursor(0,  28); display.print("IO2 short : page");
  display.setCursor(0,  38); display.print("IO4 short : RUN/STOP");
  display.setCursor(0,  48); display.print("IO4 hold  : CALIBRATE");
  display.setCursor(0,  58); display.print("CAL REQUIRED first!");
  display.display();
  delay(2500);
}

// =====================================================
// Loop
// =====================================================
void loop() {
  handleInputs();
  updateCalibration();

  if (robotState == ST_RUN && calibrated && calPhase == CAL_IDLE) {
    runLineFollower();
  } else {
    // Không chạy → dừng motor, vẫn đọc cảm biến để debug
    if (robotState != ST_CAL) stopMotors();
    if (calibrated) processSensors();
    else            readAllRaw();
  }

  renderOLED();
  renderSerial();
  sendTelemetry();
}
