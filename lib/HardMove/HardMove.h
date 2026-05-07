/**
 * @file HardMove.h
 * @brief Thu vien di cung theo encoder cho xe 2 banh dung Motor + Encoder.
 *
 * Thong so mac dinh theo xe cua ban:
 *   - Encoder trai: 194 xung / 1 vong banh
 *   - Encoder phai: 194 xung / 1 vong banh
 *   - Ban kinh banh xe: 32.5 mm
 *   - Chu vi banh xe: 2 * PI * 32.5 = 204.204 mm
 *   - countsPerMM = 194 / 204.204 = 0.950033 count/mm
 *
 * Thu vien nay KHONG dung delay de doan quang duong.
 * Xe dung encoder de dem xung va tu bu lech 2 banh.
 */

#ifndef HARD_MOVE_H
#define HARD_MOVE_H

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"

struct HardMoveConfig {
    // ===== Thong so banh + encoder =====
    float wheelRadiusMM = 32.500f;
    float countsPerWheelRev = 194.000f;
    float countsPerMM = 0.950033f;

    // ===== Thong so dieu khien =====
    int minPWM = 65;              // PWM toi thieu de banh bat dau lan
    int slowPWM = 75;             // PWM khi gan toi dich
    int correctionMax = 35;       // Gioi han bu lech trai/phai
    float balanceKp = 0.90f;      // He so bu lech encoder
    float rampDistanceMM = 90.0f; // Vung tang/giam toc, don vi mm

    // ===== An toan =====
    uint32_t timeoutMs = 7000;    // Timeout chong ket xe
    uint16_t loopDelayMs = 2;     // Delay nho trong vong lap
    uint16_t brakeMs = 35;        // Thoi gian phanh ngan khi toi dich

    // In debug trong khi chay. De false neu muon Serial gon.
    bool debug = true;
};

class HardMove {
public:
    HardMove(Motor &leftMotor,
             Motor &rightMotor,
             Encoder &leftEncoder,
             Encoder &rightEncoder);

    void begin();
    void begin(const HardMoveConfig &config);

    void setConfig(const HardMoveConfig &config);
    HardMoveConfig getConfig() const;

    // Tinh lai countsPerMM tu ban kinh banh va count/vong.
    void setWheel(float radiusMM, float countsPerWheelRev);

    // Di cung theo mm.
    // mm > 0: tien
    // mm < 0: lui
    // maxPWM: toc do PWM toi da.
    // Tra ve true neu toi dich, false neu timeout/loi.
    bool moveMM(float mm, int maxPWM);

    // Tien/lui cho de goi.
    bool forwardMM(float mm, int maxPWM);
    bool backwardMM(float mm, int maxPWM);

    // Reset encoder hai banh.
    void resetEncoders();

    // Dung motor kieu coast.
    void stop();

    // Phanh ngan roi dung.
    void brakeStop();

    // Doc count debug.
    long leftCount() const;
    long rightCount() const;
    long absAverageCount() const;

    // Quy doi ho tro test.
    float countsToMM(long counts) const;
    long mmToCounts(float mm) const;

private:
    Motor &_leftMotor;
    Motor &_rightMotor;
    Encoder &_leftEncoder;
    Encoder &_rightEncoder;

    HardMoveConfig _cfg;
    bool _configured;

    int _rampPWM(long remainCount, long targetCount, int maxPWM);
};

#endif
