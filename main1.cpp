/*
 * LineBot – Line Following Robot
 * ESP32 + TB6612FNG + HC4067 MUX + SSD1306 OLED + 8 IR sensors
 * v2.2 – BLE dashboard, 1-button WHITE/BLACK calibration 500 samples
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
// Nút bấm – chỉ dùng 1 nút cho dễ thao tác
// Mặc định: nút nối IO4 xuống GND, dùng INPUT_PULLUP => active LOW
// Nếu mạch của bạn đã mắc kiểu active HIGH với điện trở kéo xuống, đổi BUTTON_ACTIVE_LOW = 0
// Lưu ý: IO4 là chân boot strap, KHÔNG giữ nút khi reset/nạp code.
// =====================================================
#define BTN_ONE_PIN       4
#define BUTTON_ACTIVE_LOW 1

#define DEBOUNCE_MS       40UL
#define LONG_PRESS_MS   1200UL

struct Btn {
  uint8_t       pin;
  bool          lastRaw;
  bool          stable;
  unsigned long debounceAt;
  unsigned long pressAt;
};

Btn bOne = {BTN_ONE_PIN, false, false, 0, 0};

bool readButtonPressed(uint8_t pin) {
#if BUTTON_ACTIVE_LOW
  return digitalRead(pin) == LOW;
#else
  return digitalRead(pin) == HIGH;
#endif
}

/* Trả về: 0=không có gì, 1=nhấn ngắn, 2=giữ lâu
 * Chỉ phát sự kiện khi THẢ nút (edge detection sạch) */
int pollBtn(Btn &b) {
  bool raw = readButtonPressed(b.pin);

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
uint16_t sensorFilt[NUM_SENSORS];   // raw da loc nhe IIR de giam nhieu ADC
bool     filtReady = false;
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
// Calibration – 2 bước, không chặn loop
// Bước 1: đặt toàn bộ 8 mắt trên NỀN TRẮNG, lấy 500 mẫu
// Bước 2: đặt toàn bộ 8 mắt trên LINE ĐEN/mảng đen, bấm CAL, lấy 500 mẫu
// =====================================================
enum CalPhase { CAL_IDLE, CAL_WHITE_SAMPLING, CAL_WAIT_BLACK, CAL_BLACK_SAMPLING, CAL_DONE };
CalPhase      calPhase   = CAL_IDLE;
unsigned long calTimer   = 0;
int           calSamples = 0;

#define CAL_SAMPLE_N 500

uint32_t calWhiteSum[NUM_SENSORS];
uint32_t calBlackSum[NUM_SENSORS];
uint16_t sensorWhite [NUM_SENSORS];
uint16_t sensorBlack [NUM_SENSORS];

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

    // Parse vao bien thuong truoc, vi vKp/vKd/... la volatile
    float kpTmp, kdTmp;
    int baseTmp, maxTmp, srchTmp, lineThTmp;

    parseConfigString(v, kpTmp, kdTmp, baseTmp, maxTmp, srchTmp, lineThTmp);

    vKp     = kpTmp;
    vKd     = kdTmp;
    vBase   = baseTmp;
    vMax    = maxTmp;
    vSrch   = srchTmp;
    vLineTh = lineThTmp;

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
#define ADC_SETTLE_US   80    // HC4067 + ADC ESP32 can thoi gian on dinh sau khi doi kenh
#define ADC_SAMPLES      7    // lay 7 mau, loc median/trimmed mean
#define RAW_IIR_NUM      2    // loc IIR: raw = (old*2 + new)/3
#define RAW_IIR_DEN      3
#define BW_HYST         45    // hysteresis de BW khong nhay loan quanh nguong

void selectMux(uint8_t ch) {
  digitalWrite(MUX_S0,  ch       & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
}

static void sortSmall(uint16_t *a, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
}

uint16_t readMuxChRawStable(uint8_t ch) {
  selectMux(ch);

  // Sau khi doi kenh MUX, ADC ESP32 de bi dinh dien ap cua kenh truoc.
  // Vi vay cho lau hon, doc bo 2 mau dau roi moi lay mau that.
  delayMicroseconds(ADC_SETTLE_US);
  analogRead(MUX_SIG);
  delayMicroseconds(30);
  analogRead(MUX_SIG);
  delayMicroseconds(30);

  uint16_t a[ADC_SAMPLES];
  for (int i = 0; i < ADC_SAMPLES; i++) {
    a[i] = analogRead(MUX_SIG);
    delayMicroseconds(20);
  }

  sortSmall(a, ADC_SAMPLES);

  // Bo 2 mau thap nhat va 2 mau cao nhat, lay trung binh 3 mau giua.
  // Cach nay on hon average thuan khi ADC co spike.
  uint32_t sum = 0;
  for (int i = 2; i <= 4; i++) sum += a[i];
  return (uint16_t)(sum / 3);
}

void readAllRaw() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = readMuxChRawStable(i);

    if (!filtReady) {
      sensorFilt[i] = v;
    } else {
      sensorFilt[i] = (uint16_t)(((uint32_t)sensorFilt[i] * RAW_IIR_NUM + v) / RAW_IIR_DEN);
    }

    sensorRaw[i] = sensorFilt[i];
  }
  filtReady = true;
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
  filtReady = false;
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

    bool wasBlack = sensorBW[i];

    if (blackIsHigh) {
      sensorNorm[i] = (uint16_t)m;

      // Hysteresis: da den thi de thoat den can thap hon nguong mot chut,
      // chua den thi de vao den can cao hon nguong mot chut.
      if (wasBlack) sensorBW[i] = (sensorRaw[i] > sensorTh[i] - BW_HYST);
      else          sensorBW[i] = (sensorRaw[i] > sensorTh[i] + BW_HYST);
    } else {
      sensorNorm[i] = (uint16_t)(1000 - m);

      // Truong hop line den cho gia tri ADC thap hon nen dao dieu kien.
      if (wasBlack) sensorBW[i] = (sensorRaw[i] < sensorTh[i] + BW_HYST);
      else          sensorBW[i] = (sensorRaw[i] < sensorTh[i] - BW_HYST);
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
// Calibration 2 bước WHITE/BLACK – KHÔNG CHẶN loop()
// =====================================================
void resetCalSums() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calWhiteSum[i] = 0;
    calBlackSum[i] = 0;
  }
}

void startCalibration() {
  robotState = ST_CAL;
  calibrated = false;
  stopMotors();
  clearCalibration();
  resetCalSums();
  calSamples = 0;
  calTimer   = millis();
  calPhase   = CAL_WHITE_SAMPLING;

  Serial.println("[CAL] Step 1/2: dat CA 8 MAT tren NEN TRANG, giu yen. Dang lay 500 mau WHITE...");
}

void startBlackSampling() {
  if (calPhase != CAL_WAIT_BLACK) return;

  for (int i = 0; i < NUM_SENSORS; i++) calBlackSum[i] = 0;
  calSamples = 0;
  calTimer   = millis();
  calPhase   = CAL_BLACK_SAMPLING;

  Serial.println("[CAL] Step 2/2: dat CA 8 MAT tren LINE DEN / MANG DEN, giu yen. Dang lay 500 mau BLACK...");
}

void finishTwoPointCalibration() {
  uint32_t sumWhiteAll = 0;
  uint32_t sumBlackAll = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorWhite[i] = (uint16_t)(calWhiteSum[i] / CAL_SAMPLE_N);
    sensorBlack[i] = (uint16_t)(calBlackSum[i] / CAL_SAMPLE_N);

    sensorMin[i] = min(sensorWhite[i], sensorBlack[i]);
    sensorMax[i] = max(sensorWhite[i], sensorBlack[i]);

    if (sensorMax[i] < sensorMin[i] + 20) {
      // Tránh chia cho 0 nếu mắt nào đó không khác biệt nhiều giữa trắng/đen
      sensorMax[i] = sensorMin[i] + 20;
    }

    sensorTh[i] = (sensorWhite[i] + sensorBlack[i]) / 2;

    sumWhiteAll += sensorWhite[i];
    sumBlackAll += sensorBlack[i];
  }

  // Với đa số module: đen có thể cho ADC thấp hơn hoặc cao hơn trắng.
  // Tự nhận diện polarity dựa trên trung bình cả 8 mắt.
  blackIsHigh = (sumBlackAll > sumWhiteAll);

  calibrated = true;
  calPhase   = CAL_DONE;

  Serial.println("[CAL] Done 2-point calibration.");
  Serial.printf("[CAL] blackIsHigh=%d | WHITE avg=%lu | BLACK avg=%lu\n",
                (int)blackIsHigh,
                (unsigned long)(sumWhiteAll / NUM_SENSORS),
                (unsigned long)(sumBlackAll / NUM_SENSORS));

  Serial.print("[CAL] WHITE=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorWhite[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();

  Serial.print("[CAL] BLACK=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorBlack[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();

  Serial.print("[CAL] TH=");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorTh[i]);
    if (i != NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

void updateCalibration() {
  if (calPhase == CAL_IDLE) return;

  switch (calPhase) {

    case CAL_WHITE_SAMPLING:
      readAllRaw();
      for (int i = 0; i < NUM_SENSORS; i++) {
        calWhiteSum[i] += sensorRaw[i];
      }
      calSamples++;

      if (calSamples >= CAL_SAMPLE_N) {
        for (int i = 0; i < NUM_SENSORS; i++) {
          sensorWhite[i] = (uint16_t)(calWhiteSum[i] / CAL_SAMPLE_N);
        }

        calSamples = 0;
        calPhase   = CAL_WAIT_BLACK;
        calTimer   = millis();

        Serial.println("[CAL] WHITE sampled x500 OK.");
        Serial.println("[CAL] Dat CA 8 MAT len LINE DEN / MANG DEN roi BAM ngan IO4 lan nua de lay BLACK x500.");
      }
      break;

    case CAL_WAIT_BLACK:
      // Đang chờ người đặt cảm biến lên đen và bấm CAL lần nữa.
      // Vẫn đọc raw để OLED/Serial dễ quan sát.
      readAllRaw();
      break;

    case CAL_BLACK_SAMPLING:
      readAllRaw();
      for (int i = 0; i < NUM_SENSORS; i++) {
        calBlackSum[i] += sensorRaw[i];
      }
      calSamples++;

      if (calSamples >= CAL_SAMPLE_N) {
        finishTwoPointCalibration();
      }
      break;

    case CAL_DONE:
      // Calib xong thì đứng yên ở STOP, không tự lao xe.
      robotState = ST_STOP;
      stopMotors();
      calPhase = CAL_IDLE;
      lastErr = 0;
      Serial.println("[CAL] Ready. Bam RUN de chay.");
      break;

    default:
      break;
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
  display.print(">>> CALIB 2-POINT <<<");

  switch (calPhase) {
    case CAL_WHITE_SAMPLING: {
      int pct = calSamples * 100 / CAL_SAMPLE_N;
      display.setCursor(0, 12); display.print("1/2 NEN TRANG");
      display.setCursor(0, 24); display.print("Giu yen ca 8 mat");
      display.setCursor(0, 36); display.print("Sample: ");
      display.print(calSamples);
      display.print("/");
      display.print(CAL_SAMPLE_N);
      display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 51, pct * 126 / 100, 7, SSD1306_WHITE);
      break;
    }

    case CAL_WAIT_BLACK:
      display.setCursor(0, 12); display.print("WHITE OK x500");
      display.setCursor(0, 24); display.print("2/2 Dat len DEN");
      display.setCursor(0, 36); display.print("roi BAM ngan IO4");
      display.setCursor(0, 50); display.print("Can DEN rong ca 8 mat");
      break;

    case CAL_BLACK_SAMPLING: {
      int pct2 = calSamples * 100 / CAL_SAMPLE_N;
      display.setCursor(0, 12); display.print("2/2 LINE DEN");
      display.setCursor(0, 24); display.print("Giu yen ca 8 mat");
      display.setCursor(0, 36); display.print("Sample: ");
      display.print(calSamples);
      display.print("/");
      display.print(CAL_SAMPLE_N);
      display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 51, pct2 * 126 / 100, 7, SSD1306_WHITE);
      break;
    }

    default:
      break;
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
  static unsigned long pageT = 0;
  if (millis() - lastT < 80) return;
  lastT = millis();

  // Vì chỉ còn 1 nút, OLED tự chuyển trang khi không ở chế độ calib.
  if (calPhase == CAL_IDLE && millis() - pageT > 1500) {
    pageT = millis();
    oledPage = (oledPage + 1) % 4;
  }

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
  int ev = pollBtn(bOne);

  // ── 1 nút duy nhất trên IO4 ─────────────────────
  // Giữ lâu  : bắt đầu lại calib từ bước WHITE x500
  // Nhấn ngắn khi đang chờ BLACK: lấy BLACK x500
  // Nhấn ngắn sau khi calib xong: RUN/STOP
  if (ev == 2) {
    startCalibration();
  } else if (ev == 1) {
    if (robotState == ST_CAL && calPhase == CAL_WAIT_BLACK) {
      startBlackSampling();
    } else if (!calibrated) {
      Serial.println("[WARN] Chua calibrate! Giu nut IO4 1.2s de lay WHITE truoc.");
    } else if (calPhase == CAL_IDLE) {
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
  // BLE vẫn giữ lại để debug/chỉnh PID. CAL trên dashboard vẫn dùng được dự phòng,
  // nhưng quy trình chính bây giờ là bấm nút vật lý IO4.
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

  if (bleCmdRun) {
    bleCmdRun = false;
    if (calibrated && calPhase == CAL_IDLE) {
      robotState = ST_RUN;
      lastErr = 0;
      Serial.println("[BLE] RUN");
    } else {
      Serial.println("[BLE] RUN ignored: chua calib xong.");
    }
  }

  if (bleCmdStop) {
    bleCmdStop = false;
    robotState = ST_STOP;
    stopMotors();
    Serial.println("[BLE] STOP");
  }

  if (bleCmdCal) {
    bleCmdCal = false;
    if (robotState == ST_CAL && calPhase == CAL_WAIT_BLACK) {
      startBlackSampling();
    } else {
      startCalibration();
    }
  }
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
  Serial.println("\n[BOOT] LineBot v2.2");

  // Nút bấm 1 nút
#if BUTTON_ACTIVE_LOW
  pinMode(BTN_ONE_PIN, INPUT_PULLUP);
#else
  pinMode(BTN_ONE_PIN, INPUT);
#endif

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
  analogSetPinAttenuation(MUX_SIG, ADC_11db);  // doc du dai 0-3.3V hon, hop voi cam bien line dung VCC 3V3

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
  display.setCursor(14,  2); display.print("=== LINE BOT v2.2 ===");
  display.setCursor(0,  16); display.print("BLE: "); display.print(BLE_DEVICE_NAME);
  display.setCursor(0,  28); display.print("1 nut IO4, active LOW");
  display.setCursor(0,  38); display.print("Hold: WHITE x500");
  display.setCursor(0,  48); display.print("Short: BLACK/RUN/STOP");
  display.setCursor(0,  58); display.print("Khong giu nut khi reset");
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
