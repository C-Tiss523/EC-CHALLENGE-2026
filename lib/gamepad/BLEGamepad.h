#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ── UUIDs ──────────────────────────────────────────────────────────────────────
#define GP_SERVICE_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define GP_CMD_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ── Button bitmask constants ───────────────────────────────────────────────────
#define BTN_UP      0x01
#define BTN_DOWN    0x02
#define BTN_LEFT    0x04
#define BTN_RIGHT   0x08
#define BTN_A       0x10
#define BTN_B       0x20
#define BTN_X       0x40
#define BTN_Y       0x80

// Byte 2 (flags): bit 0 = START, bit 1 = SELECT
#define BTN_START   0x01
#define BTN_SELECT  0x02

// ── Packet layout (8 bytes): ───────────────────────────────────────────────────
//  [0] 0xAA  header
//  [1] buttons bitmask  (UP|DOWN|LEFT|RIGHT|A|B|X|Y)
//  [2] flags            (bit0=START, bit1=SELECT)
//  [3] reserved (0)
//  [4] reserved (0)
//  [5] reserved (0)
//  [6] reserved (0)
//  [7] XOR checksum (XOR of bytes 0–6)

struct GamepadState {
  uint8_t buttons;   // bitmask: UP DOWN LEFT RIGHT A B X Y
  uint8_t flags;     // bit0=START, bit1=SELECT
  bool    connected;
};

typedef void (*GamepadCallback)(const GamepadState&);

// ── Library ────────────────────────────────────────────────────────────────────
class BLEGamepad {
public:
  void begin(const char* name = "ESP32-Gamepad") {
    _inst = this;
    BLEDevice::init(name);
    auto* srv = BLEDevice::createServer();
    srv->setCallbacks(new _SrvCB());
    auto* svc = srv->createService(GP_SERVICE_UUID);
    _ch = svc->createCharacteristic(GP_CMD_UUID,
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);
    _ch->setCallbacks(new _CmdCB());
    svc->start();
    auto* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(GP_SERVICE_UUID);
    adv->start();
    Serial.printf("[BLE] Advertising: %s\n", name);
  }

  // Call in loop()
  void update() {
    if (_fresh && _cb) { _fresh = false; _cb(_state); }
  }

  void onData(GamepadCallback cb) { _cb = cb; }

  // ── D-Pad / ABXY helpers ─────────────────────────────────────────────────────
  bool isPressed(uint8_t btn) const { return (_state.buttons & btn) != 0; }
  bool isUp()     const { return isPressed(BTN_UP);    }
  bool isDown()   const { return isPressed(BTN_DOWN);  }
  bool isLeft()   const { return isPressed(BTN_LEFT);  }
  bool isRight()  const { return isPressed(BTN_RIGHT); }
  bool isA()      const { return isPressed(BTN_A);     }
  bool isB()      const { return isPressed(BTN_B);     }
  bool isX()      const { return isPressed(BTN_X);     }
  bool isY()      const { return isPressed(BTN_Y);     }

  // ── System button helpers ────────────────────────────────────────────────────
  bool isStart()  const { return (_state.flags & BTN_START)  != 0; }
  bool isSelect() const { return (_state.flags & BTN_SELECT) != 0; }

  const GamepadState& state() const { return _state; }
  bool connected() const { return _state.connected; }

private:
  static BLEGamepad*   _inst;
  BLECharacteristic*   _ch    = nullptr;
  GamepadState         _state = {};
  GamepadCallback      _cb    = nullptr;
  bool                 _fresh = false;

  void _parse(const uint8_t* d, size_t n) {
    if (n < 8 || d[0] != 0xAA) return;
    uint8_t cs = 0; for (int i = 0; i < 7; i++) cs ^= d[i];
    if (cs != d[7]) return;
    _state.buttons = d[1];
    _state.flags   = d[2];
    _fresh = true;
  }

  struct _SrvCB : BLEServerCallbacks {
    void onConnect(BLEServer*)      override { if (_inst) _inst->_state.connected = true;  Serial.println("[BLE] Connected");    }
    void onDisconnect(BLEServer* s) override { if (_inst) _inst->_state.connected = false; Serial.println("[BLE] Disconnected"); s->getAdvertising()->start(); }
  };
  struct _CmdCB : BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
      if (!_inst) return;
      auto v = c->getValue();
      _inst->_parse((const uint8_t*)v.data(), v.size());
    }
  };
};

BLEGamepad* BLEGamepad::_inst = nullptr;