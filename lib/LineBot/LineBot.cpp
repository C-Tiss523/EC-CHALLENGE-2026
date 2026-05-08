/*
 * LineBot library – ESP32 + TB6612FNG + HC4067 + OLED + BLE
 * (Phiên bản rút gọn: chỉ giữ PID dò line, bỏ Mission Sequencer và hardMoveMM)
 */

#include "LineBot.h"
#include "drive.h"
#include <Preferences.h>

namespace LineBot {

// NVS – lưu params PID qua reboot
Preferences prefs;

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
// Nut bam – 2 nut, ACTIVE HIGH, pull-down ngoai 10k
//   IO2 (BTN_ONE_PIN): calib WHITE/BLACK + RUN/STOP
//   IO4 (BTN_TWO_PIN): calib WHITE/BLACK + RUN/STOP (giong IO2)
// Luu y IO2/IO4 la chan boot-strap, KHONG giu nut khi reset/nap code!
// BLE dashboard cung dieu khien duoc song song.
// =====================================================
#define BTN_ONE_PIN 2
#define BTN_TWO_PIN 4
#define DEBOUNCE_MS 40UL

// ─── Nut 1 (GPIO2) ────────────────────────────────
bool btnStable    = LOW;
bool btnLastRaw   = LOW;
unsigned long btnLastChange = 0;

bool buttonPressedEvent() {
  bool raw = digitalRead(BTN_ONE_PIN);
  if (raw != btnLastRaw) { btnLastRaw = raw; btnLastChange = millis(); }
  if (millis() - btnLastChange > DEBOUNCE_MS) {
    if (raw != btnStable) {
      btnStable = raw;
      if (btnStable == HIGH) return true;
    }
  }
  return false;
}

// ─── Nut 2 (GPIO4) ────────────────────────────────
bool btn2Stable   = LOW;
bool btn2LastRaw  = LOW;
unsigned long btn2LastChange = 0;

bool button2PressedEvent() {
  bool raw = digitalRead(BTN_TWO_PIN);
  if (raw != btn2LastRaw) { btn2LastRaw = raw; btn2LastChange = millis(); }
  if (millis() - btn2LastChange > DEBOUNCE_MS) {
    if (raw != btn2Stable) {
      btn2Stable = raw;
      if (btn2Stable == HIGH) return true;
    }
  }
  return false;
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
uint16_t sensorFilt[NUM_SENSORS];
bool     filtReady = false;
uint16_t sensorMin [NUM_SENSORS];
uint16_t sensorMax [NUM_SENSORS];
uint16_t sensorTh  [NUM_SENSORS];
uint16_t sensorNorm[NUM_SENSORS];   // độ đậm đen 0–1000
bool     sensorBW  [NUM_SENSORS];   // true=đen, false=trắng

const bool SENSOR_ORDER_REVERSED = false;
bool blackIsHigh = false;
bool calibrated  = false;

// Cảm biến 0 bỏ qua vì bị hỏng
const bool SENSOR_ENABLED[NUM_SENSORS] = { true, true, true, true, true, true, true, true };

// =====================================================
// Motor – TB6612FNG
// =====================================================
#define AIN1 25
#define AIN2 33
#define PWMA 32
#define BIN1 27
#define BIN2 26
#define PWMB 14

#define ENAL 36
#define ENBL 39
#define ENAR 34
#define ENBR 35

#define CH_A 8
#define CH_B 9

Drive motorL;
Drive motorR;

// =====================================================
// Tham số PID – tuỳ chỉnh qua BLE
// =====================================================
float Kp          = 0.050f;
float Ki          = 0.000f;
float Kd          = 0.220f;
int   baseSpeed   = 150;
int   maxSpeed    = 255;
int   searchSpeed = 110;
int   lineDetTh   = 900;

#define DEFAULT_CENTER_POS 3500
int centerPos = DEFAULT_CENTER_POS;

// =====================================================
// Trạng thái robot
// =====================================================
enum RobotState { ST_STOP, ST_CAL, ST_RUN };
RobotState robotState = ST_STOP;

int oledPage = 1;  // 0–3

// =====================================================
// Biến debug runtime
// =====================================================
int   lastPos      = DEFAULT_CENTER_POS;
int   lastErr      = 0;
float integralErr  = 0.0f;
int   lastLPWM     = 0;
int   lastRPWM     = 0;
bool  lastFound    = false;

// Biến PID line-following
int   lostLineDir   = 0;
float filtDErr      = 0.0f;
unsigned long lostLineTime = 0;

// =====================================================
// Calibration – 2 bước, không chặn loop
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
// BLE
// =====================================================
#define BLE_DEVICE_NAME "LineBot"

#define SVC_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEL_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CFG_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CMD_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"

BLECharacteristic *pTelChar = nullptr;
BLECharacteristic *pCfgChar = nullptr;
bool bleConnected = false;

volatile bool bleCmdRun    = false;
volatile bool bleCmdStop   = false;
volatile bool bleCmdCal    = false;
volatile bool bleNewParams = false;

volatile float vKp, vKi, vKd;
volatile int   vBase, vMax, vSrch, vLineTh;

// ─── BLE Server callbacks ──────────────────────────
class BleServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    bleConnected = false;
    delay(500);
    pSrv->startAdvertising();
  }
};

// ─── Phân tích chuỗi config ───────────────────────
void parseConfigString(const String &s,
                       float &kp, float &ki, float &kd,
                       int &base, int &mx, int &srch, int &lineTh) {
  kp=Kp; ki=Ki; kd=Kd; base=baseSpeed; mx=maxSpeed;
  srch=searchSpeed; lineTh=lineDetTh;

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
      else if (key == "KI")     ki     = val.toFloat();
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
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue().c_str();
    float kpTmp, kiTmp, kdTmp;
    int baseTmp, maxTmp, srchTmp, lineThTmp;
    parseConfigString(v, kpTmp, kiTmp, kdTmp, baseTmp, maxTmp, srchTmp, lineThTmp);
    vKp = kpTmp; vKi = kiTmp; vKd = kdTmp;
    vBase = baseTmp; vMax = maxTmp; vSrch = srchTmp; vLineTh = lineThTmp;
    bleNewParams = true;
  }
  void onRead(BLECharacteristic *c) override {
    char buf[90];
    snprintf(buf, sizeof(buf),
      "KP:%.3f|KI:%.4f|KD:%.3f|BASE:%d|MAX:%d|SRCH:%d|LINETH:%d",
      Kp, Ki, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);
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
  BLEDevice::setMTU(185);

  BLEServer  *pSrv = BLEDevice::createServer();
  pSrv->setCallbacks(new BleServerCB());

  BLEService *pSvc = pSrv->createService(SVC_UUID);

  pTelChar = pSvc->createCharacteristic(TEL_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTelChar->addDescriptor(new BLE2902());

  pCfgChar = pSvc->createCharacteristic(CFG_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCfgChar->setCallbacks(new CfgCB());

  BLECharacteristic *pCmdChar = pSvc->createCharacteristic(CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCmdChar->setCallbacks(new CmdCB());

  pSvc->start();

  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SVC_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
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
#define ADC_SETTLE_US   80
#define ADC_SAMPLES      7
#define RAW_IIR_NUM      2
#define RAW_IIR_DEN      3
#define BW_HYST         45

void selectMux(uint8_t ch) {
  digitalWrite(MUX_S0,  ch       & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
}

static void sortSmall(uint16_t *a, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
}

uint16_t readMuxChRawStable(uint8_t ch) {
  selectMux(ch);
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

  uint32_t sum = 0;
  for (int i = 2; i <= 4; i++) sum += a[i];
  return (uint16_t)(sum / 3);
}

void readAllRaw() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) {
      sensorRaw[i]  = 0;
      sensorFilt[i] = 0;
      continue;
    }
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
#define MIN_PWM 55

void setMotors(int l, int r) {
  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);
  if (l != 0 && abs(l) < MIN_PWM) l = (l > 0) ? MIN_PWM : -MIN_PWM;
  if (r != 0 && abs(r) < MIN_PWM) r = (r > 0) ? MIN_PWM : -MIN_PWM;
  motorL.runSigned(l);
  motorR.runSigned(r);
  lastLPWM = l;
  lastRPWM = r;
}

void stopMotors() {
  motorL.runSigned(0);
  motorR.runSigned(0);
  lastLPWM = 0;
  lastRPWM = 0;
}

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

void computeCenterPos() {
  long sum = 0;
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;
    int idx = SENSOR_ORDER_REVERSED ? (NUM_SENSORS - 1 - i) : i;
    sum += idx * 1000;
    count++;
  }
  centerPos = (count > 0) ? (int)(sum / count) : DEFAULT_CENTER_POS;
}

void processSensors() {
  readAllRaw();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) {
      sensorNorm[i] = 0;
      sensorBW[i]   = false;
      continue;
    }
    int lo = sensorMin[i], hi = sensorMax[i];
    if (hi < lo + 10) hi = lo + 10;

    long m = map(sensorRaw[i], lo, hi, 0, 1000);
    m = constrain(m, 0, 1000);

    bool wasBlack = sensorBW[i];

    if (blackIsHigh) {
      sensorNorm[i] = (uint16_t)m;
      if (wasBlack) sensorBW[i] = (sensorRaw[i] > sensorTh[i] - BW_HYST);
      else          sensorBW[i] = (sensorRaw[i] > sensorTh[i] + BW_HYST);
    } else {
      sensorNorm[i] = (uint16_t)(1000 - m);
      if (wasBlack) sensorBW[i] = (sensorRaw[i] < sensorTh[i] + BW_HYST);
      else          sensorBW[i] = (sensorRaw[i] < sensorTh[i] - BW_HYST);
    }
  }
}

bool getLinePos(int &posOut) {
  processSensors();
  long ws = 0, total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!SENSOR_ENABLED[i]) continue;
    int idx = SENSOR_ORDER_REVERSED ? (NUM_SENSORS - 1 - i) : i;
    ws    += (long)sensorNorm[i] * idx * 1000;
    total += sensorNorm[i];
  }
  if (total < lineDetTh) return false;
  posOut = (int)(ws / total);
  return true;
}

// =====================================================
// Calibration 2 bước – KHÔNG chặn loop()
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
  Serial.println("[CAL] Step 2/2: dat CA 8 MAT tren LINE DEN, giu yen. Dang lay 500 mau BLACK...");
}

void finishTwoPointCalibration() {
  uint32_t sumWhiteAll = 0;
  uint32_t sumBlackAll = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorWhite[i] = (uint16_t)(calWhiteSum[i] / CAL_SAMPLE_N);
    sensorBlack[i] = (uint16_t)(calBlackSum[i] / CAL_SAMPLE_N);
    sensorMin[i]   = min(sensorWhite[i], sensorBlack[i]);
    sensorMax[i]   = max(sensorWhite[i], sensorBlack[i]);
    if (sensorMax[i] < sensorMin[i] + 20) sensorMax[i] = sensorMin[i] + 20;
    sensorTh[i]    = (sensorWhite[i] + sensorBlack[i]) / 2;
    sumWhiteAll += sensorWhite[i];
    sumBlackAll += sensorBlack[i];
  }

  blackIsHigh = (sumBlackAll > sumWhiteAll);
  calibrated  = true;
  calPhase    = CAL_DONE;

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
      for (int i = 0; i < NUM_SENSORS; i++) calWhiteSum[i] += sensorRaw[i];
      calSamples++;
      if (calSamples >= CAL_SAMPLE_N) {
        for (int i = 0; i < NUM_SENSORS; i++)
          sensorWhite[i] = (uint16_t)(calWhiteSum[i] / CAL_SAMPLE_N);
        calSamples = 0;
        calPhase   = CAL_WAIT_BLACK;
        calTimer   = millis();
        Serial.println("[CAL] WHITE sampled x500 OK.");
        Serial.println("[CAL] Dat CA 8 MAT len LINE DEN roi BAM CAL lan nua.");
      }
      break;

    case CAL_WAIT_BLACK:
      readAllRaw();
      break;

    case CAL_BLACK_SAMPLING:
      readAllRaw();
      for (int i = 0; i < NUM_SENSORS; i++) calBlackSum[i] += sensorRaw[i];
      calSamples++;
      if (calSamples >= CAL_SAMPLE_N) finishTwoPointCalibration();
      break;

    case CAL_DONE:
      robotState = ST_STOP;
      stopMotors();
      calPhase = CAL_IDLE;
      lastErr  = 0;
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
      display.print(calSamples); display.print("/"); display.print(CAL_SAMPLE_N);
      display.drawRect(0, 50, 128, 9, SSD1306_WHITE);
      display.fillRect(1, 51, pct * 126 / 100, 7, SSD1306_WHITE);
      break;
    }
    case CAL_WAIT_BLACK:
      display.setCursor(0, 12); display.print("WHITE OK x500");
      display.setCursor(0, 24); display.print("2/2 Dat len DEN");
      display.setCursor(0, 36); display.print("roi BAM CAL nua");
      display.setCursor(0, 50); display.print("Can DEN rong ca 8 mat");
      break;

    case CAL_BLACK_SAMPLING: {
      int pct2 = calSamples * 100 / CAL_SAMPLE_N;
      display.setCursor(0, 12); display.print("2/2 LINE DEN");
      display.setCursor(0, 24); display.print("Giu yen ca 8 mat");
      display.setCursor(0, 36); display.print("Sample: ");
      display.print(calSamples); display.print("/"); display.print(CAL_SAMPLE_N);
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
  display.setCursor(0, 10); display.print("Kp:"); display.print(Kp, 3);
  display.setCursor(64, 10); display.print("Ki:"); display.print(Ki, 4);
  display.setCursor(0, 20); display.print("Kd:"); display.print(Kd, 3);
  display.setCursor(64, 20); display.print("I:"); display.print((int)integralErr);
  display.setCursor(0, 30); display.print("base:"); display.print(baseSpeed);
  display.print(" max:"); display.print(maxSpeed);
  display.setCursor(0, 40); display.print("srch:"); display.print(searchSpeed);
  display.print(" th:"); display.print(lineDetTh);
  display.setCursor(0, 50); display.print("L:"); display.print(lastLPWM);
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
  bool pressed = buttonPressedEvent() || button2PressedEvent();

  if (pressed) {
    if (robotState == ST_CAL && calPhase == CAL_WAIT_BLACK) {
      startBlackSampling();
    } else if (!calibrated && calPhase == CAL_IDLE) {
      startCalibration();
    } else if (calibrated && calPhase == CAL_IDLE) {
      if (robotState == ST_RUN) {
        robotState = ST_STOP;
        stopMotors();
        Serial.println("[BTN] STOP");
      } else {
        robotState  = ST_RUN;
        lastErr     = 0;
        integralErr = 0.0f;
        Serial.println("[BTN] RUN");
      }
    }
  }

  if (bleNewParams) {
    bleNewParams = false;
    Kp          = constrain(vKp,     0.000f,  50.000f);
    Ki          = constrain(vKi,     0.000f,  20.000f);
    Kd          = constrain(vKd,     0.000f, 100.000f);
    baseSpeed   = constrain(vBase,   0, 255);
    maxSpeed    = constrain(vMax,    0, 255);
    searchSpeed = constrain(vSrch,   0, 255);
    lineDetTh   = constrain(vLineTh, 50, 8000);
    integralErr = 0.0f;
    Serial.printf("[BLE] Params: Kp=%.3f Ki=%.4f Kd=%.3f base=%d max=%d srch=%d lineTh=%d\n",
      Kp, Ki, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);

    prefs.begin("linebot", false);
    prefs.putFloat("kp",     Kp);
    prefs.putFloat("ki",     Ki);
    prefs.putFloat("kd",     Kd);
    prefs.putInt  ("base",   baseSpeed);
    prefs.putInt  ("max",    maxSpeed);
    prefs.putInt  ("srch",   searchSpeed);
    prefs.putInt  ("lineth", lineDetTh);
    prefs.end();
    Serial.println("[NVS] Params saved.");
  }

  if (bleCmdRun) {
    bleCmdRun = false;
    if (calibrated && calPhase == CAL_IDLE) {
      robotState  = ST_RUN;
      lastErr     = 0;
      integralErr = 0.0f;
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
// Line following – PID
// =====================================================
#define COAST_MS 20UL

int coastL = 0;
int coastR = 0;

void runLineFollower() {

  // ── 1. Đọc vị trí line ──────────────────────────
  int  pos   = centerPos;
  bool found = getLinePos(pos);

  // ── 2. Ghi nhận thời điểm mất line ──────────────
  if (lastFound && !found) {
    lostLineDir  = (lastErr >= 0) ? 1 : -1;
    lostLineTime = millis();
    coastL       = lastLPWM;
    coastR       = lastRPWM;
  }
  lastFound = found;
  lastPos   = pos;

  // ── 3. Mất line: COAST → SEARCH → STOP ──────────
  if (!found) {
    unsigned long lost = millis() - lostLineTime;

    if (lost > 3000) {
      stopMotors();
      robotState = ST_STOP;
      Serial.println("[ERR] Mat line >3s, tu dong dung.");
      return;
    }

    if (lost < COAST_MS) {
      int cl = coastL > 0 ? max(coastL - 20, 40) : min(coastL + 20, -40);
      int cr = coastR > 0 ? max(coastR - 20, 40) : min(coastR + 20, -40);
      setMotors(cl, cr);
      return;
    }

    int spinPWM = (int)map((long)lost, (long)COAST_MS, 1500,
                           searchSpeed, min(searchSpeed + 60, maxSpeed));
    spinPWM = constrain(spinPWM, searchSpeed, maxSpeed);
    if (lostLineDir > 0) setMotors( spinPWM, -spinPWM);
    else                 setMotors(-spinPWM,  spinPWM);
    return;
  }

  // ── 4. Thấy line: PID ────────────────────────────
  int err  = pos - centerPos;
  int dErr = err - lastErr;
  lastErr  = err;

  filtDErr = filtDErr * 0.75f + (float)dErr * 0.25f;

  integralErr += (float)err;
  integralErr  = constrain(integralErr, -2000.0f, 2000.0f);

  float corr = Kp * (float)err
             + Ki * integralErr
             + Kd * filtDErr;

  float errRatio   = constrain((float)abs(err) / 3500.0f, 0.0f, 1.0f);
  float speedScale = 1.0f - errRatio * 0.70f;
  int   dynBase    = max((int)((float)baseSpeed * speedScale), 0);

  int lPWM = constrain(dynBase + (int)corr, -maxSpeed, maxSpeed);
  int rPWM = constrain(dynBase - (int)corr, -maxSpeed, maxSpeed);

  setMotors(lPWM, rPWM);
}

// =====================================================
// Setup
// =====================================================
void begin() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] LineBot PID-only build");

  pinMode(BTN_ONE_PIN, INPUT);
  pinMode(BTN_TWO_PIN, INPUT);

  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
  digitalWrite(MUX_S0, LOW); digitalWrite(MUX_S1, LOW); digitalWrite(MUX_S2, LOW);

  motorL.motor_init(AIN1, AIN2, PWMA, ENAL, ENBL, CH_A);
  motorR.motor_init(BIN1, BIN2, PWMB, ENAR, ENBR, CH_B);

  analogReadResolution(12);
  analogSetPinAttenuation(MUX_SIG, ADC_11db);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERR] OLED init failed!");
    while (1) delay(1000);
  }

  clearCalibration();
  computeCenterPos();
  Serial.printf("[SENSOR] centerPos=%d\n", centerPos);
  stopMotors();
  readAllRaw();

  prefs.begin("linebot", true);
  Kp          = prefs.getFloat("kp",     Kp);
  Ki          = prefs.getFloat("ki",     Ki);
  Kd          = prefs.getFloat("kd",     Kd);
  baseSpeed   = prefs.getInt  ("base",   baseSpeed);
  maxSpeed    = prefs.getInt  ("max",    maxSpeed);
  searchSpeed = prefs.getInt  ("srch",   searchSpeed);
  lineDetTh   = prefs.getInt  ("lineth", lineDetTh);
  prefs.end();
  Serial.printf("[NVS] Loaded: Kp=%.3f Ki=%.4f Kd=%.3f base=%d max=%d srch=%d lineTh=%d\n",
    Kp, Ki, Kd, baseSpeed, maxSpeed, searchSpeed, lineDetTh);

  setupBLE();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10,  0); display.print("== LINE BOT PID ==");
  display.setCursor(0,  10); display.print("BLE: "); display.print(BLE_DEVICE_NAME);
  display.setCursor(0,  20); display.print("Nut IO2 & IO4 ACT-HI");
  display.setCursor(0,  30); display.print("[1] WHITE x500");
  display.setCursor(0,  39); display.print("[2] BLACK x500");
  display.setCursor(0,  48); display.print("[3+] RUN / STOP");
  display.setCursor(0,  57); display.print("BLE: CAL/RUN/STOP ok");
  display.display();
  delay(2500);
}

// =====================================================
// Loop
// =====================================================
void update() {
  handleInputs();
  updateCalibration();

  if (robotState == ST_RUN && calibrated && calPhase == CAL_IDLE) {
    runLineFollower();
  } else {
    if (calPhase == CAL_IDLE) processSensors();
  }

  renderOLED();
  renderSerial();
  sendTelemetry();
}

} // namespace LineBot