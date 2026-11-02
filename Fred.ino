#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// Fred mapping:
// - Pin 14: continuous/360-style servo from analog triggers (L2/R2).
// - Pin 12: magnet digital output; controller LED blue=inactive red=active.
// - Pin 27: 180 servo from digital trigger buttons (L1/R1 default masks).
// - Pins 26,25,33,32: joystick-driven 180 servos from left vertical axis only.

static const int SERVO_360_PIN = 12;
static const int MAGNET_PIN = 14;
static const int BUTTON_SERVO_PIN = 27;
static const int JOY_SERVO_PINS[4] = {26, 25, 33, 32};
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2400;
static const float SERVO_POS_MIN = 0.0f;
static const float SERVO_POS_MAX = 180.0f;

static const int DEADZONE = 12;
static const uint32_t UPDATE_MS = 20;
static const int POSITION_AXIS_MIN = -500;
static const int POSITION_AXIS_MAX = 500;
static const uint32_t PRINT_MS = 200;

// Magnet output level when logical state is active/inactive.
static const uint8_t MAGNET_ACTIVE_LEVEL = HIGH;
static const uint8_t MAGNET_INACTIVE_LEVEL = LOW;

// L1/R1 masks for Bluepad32 on standard PS5 profile.
static const uint16_t BTN_TRIGGER_LEFT_MASK = 0x0010u;
static const uint16_t BTN_TRIGGER_RIGHT_MASK = 0x0020u;
static const uint16_t BTN_JOY_TOGGLE_MASK = 0x0008u;  // change if your controller maps this differently
static const uint16_t BTN_MAGNET_TOGGLE_MASK = 0x0001u;  // button 0

// Trigger range and deadzone
static const int TRIGGER_MAX = 1023;
static const int TRIGGER_DEADZONE = 20;
static const float MAX_DEG_PER_SEC_JOY = 45.0f;
static const float MAX_DEG_PER_SEC_360 = 90.0f;
static const float MAX_DEG_PER_SEC_BTN = 120.0f;

static const uint8_t TARGET_MAC[6] = {0xA4, 0x53, 0x85, 0xE7, 0xBF, 0x71};

static ControllerPtr controllers[BP32_MAX_GAMEPADS];
static Servo servo360;
static Servo servoButton;
static Servo joyServos[4];
static bool servo360Ready = false;
static bool servoButtonReady = false;
static bool joyReady[4] = {false, false, false, false};
static bool joyEnabled = true;
static float joyPos[4] = {90.0f, 90.0f, 90.0f, 90.0f};
static int joyOffsetDeg[4] = {-30, -30, -30, -30};
static int servo360Pos = 90;
static int buttonServoPos = 90;
static uint32_t lastUpdateMs = 0;
static uint32_t lastPrintMs = 0;
static bool magnetActiveState = false;
static bool lastLedMagnetState = false;
static bool lastLedTargetValid = false;
static uint16_t lastButtons = 0;
static uint8_t lastMisc = 0;
static uint8_t lastDpad = 0;
static float joyBasePos[4] = {90.0f, 90.0f, 90.0f, 90.0f};

// Automatically connects to the first controller matching the TARGET_MAC address.
static bool macMatchesTarget(const uint8_t mac[6]) {
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != TARGET_MAC[i]) return false;
  }
  return true;
}

static void printMac(const uint8_t mac[6]) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool isTargetController(ControllerPtr ctl) {
  if (!ctl) return false;
  const ControllerProperties p = ctl->getProperties();
  return macMatchesTarget(p.btaddr);
}

static void onConnectedController(ControllerPtr ctl) {
  const ControllerProperties p = ctl->getProperties();

  Serial.print("[BT] Connected controller MAC=");
  printMac(p.btaddr);

  if (!macMatchesTarget(p.btaddr)) {
    Serial.println(" (ignored)");
    return;
  }
  Serial.println(" (accepted)");

  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (!controllers[i]) {
      controllers[i] = ctl;
      Serial.printf("[BT] Stored target controller at idx=%d\n", i);
      return;
    }
  }
  Serial.println("[BT] No free controller slot for target controller");
}

static void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllers[i] == ctl) {
      controllers[i] = nullptr;
      Serial.printf("[BT] Target controller disconnected idx=%d\n", i);
      return;
    }
  }
}

static ControllerPtr firstTargetController() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllers[i] && controllers[i]->isConnected() && isTargetController(controllers[i])) {
      return controllers[i];
    }
  }
  return nullptr;
}

static float axisToVelocityDegPerSec(int axisRaw, float maxDegPerSec) {
  int v = constrain(axisRaw, POSITION_AXIS_MIN, POSITION_AXIS_MAX);
  if (abs(v) <= DEADZONE) return 0.0f;
  if (v > 0) v -= DEADZONE;
  else v += DEADZONE;
  const float norm = (float)v / (float)(POSITION_AXIS_MAX - DEADZONE);
  return norm * maxDegPerSec;
}

static bool magnetIsActive() {
  return magnetActiveState;
}

static void applyMagnetOutput() {
  const uint8_t level = magnetActiveState ? MAGNET_ACTIVE_LEVEL : MAGNET_INACTIVE_LEVEL;
  digitalWrite(MAGNET_PIN, level);
}

static void debugMagnetOutput() {
  const int level = digitalRead(MAGNET_PIN);
  Serial.printf("[MAG] pin=%d state=%s level=%d\n",
                MAGNET_PIN,
                magnetActiveState ? "ACTIVE" : "INACTIVE",
                level);
}

static void updateControllerLedForMagnet(ControllerPtr ctl, bool magnetActive) {
  if (!ctl) return;
  if (lastLedTargetValid && magnetActive == lastLedMagnetState) return;

  if (magnetActive) ctl->setColorLED(255, 0, 0);     // red
  else ctl->setColorLED(0, 255, 0);                   // green
  lastLedTargetValid = true;
  lastLedMagnetState = magnetActive;
  Serial.printf("[MAG] state=%s -> LED %s\n",
                magnetActive ? "ACTIVE" : "INACTIVE",
                magnetActive ? "RED" : "BLUE");
}

static float triggerToVelocityDegPerSec(int lt, int rt, float maxDegPerSec) {
  const int clt = constrain(lt, 0, TRIGGER_MAX);
  const int crt = constrain(rt, 0, TRIGGER_MAX);
  int diff = crt - clt;
  if (abs(diff) <= TRIGGER_DEADZONE) diff = 0;
  return ((float)diff / (float)TRIGGER_MAX) * maxDegPerSec;
}

static float buttonToVelocityDegPerSec(uint16_t buttons, float maxDegPerSec) {
  const bool left = (buttons & BTN_TRIGGER_LEFT_MASK) != 0;
  const bool right = (buttons & BTN_TRIGGER_RIGHT_MASK) != 0;
  if (left && !right) return -maxDegPerSec;
  if (right && !left) return maxDegPerSec;
  return 0.0f;
}

static void setJoyServosEnabled(bool enabled) {
  joyEnabled = enabled;
  for (int i = 0; i < 4; ++i) {
    if (enabled) {
      if (!joyReady[i]) {
        joyServos[i].setPeriodHertz(50);
        joyReady[i] = joyServos[i].attach((uint8_t)JOY_SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
      }
      if (joyReady[i]) joyServos[i].write((int)joyPos[i]);
    } else {
      if (joyReady[i]) {
        joyServos[i].detach();
        joyReady[i] = false;
      }
    }
  }
  Serial.printf("[JOY] servos %s\n", enabled ? "ENABLED" : "DISABLED");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Fred Remapped Control ===");
  Serial.print("[BT] Target MAC: ");
  printMac(TARGET_MAC);
  Serial.println();
  Serial.printf("[PINS] 360=%d  MAG=%d  BTN180=%d  JOY180={%d,%d,%d,%d}\n",
                SERVO_360_PIN, MAGNET_PIN, BUTTON_SERVO_PIN,
                JOY_SERVO_PINS[0], JOY_SERVO_PINS[1], JOY_SERVO_PINS[2], JOY_SERVO_PINS[3]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  BP32.enableVirtualDevice(false);

  pinMode(MAGNET_PIN, OUTPUT);

  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESP32PWM::allocateTimer(4);

  servo360.setPeriodHertz(50);
  servo360Ready = servo360.attach((uint8_t)SERVO_360_PIN, SERVO_MIN_US, SERVO_MAX_US);
  if (servo360Ready) servo360.write(servo360Pos);
  Serial.printf("[SERVO] 360 pin=%d %s\n", SERVO_360_PIN, servo360Ready ? "OK" : "FAIL");

  servoButton.setPeriodHertz(50);
  servoButtonReady = servoButton.attach((uint8_t)BUTTON_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  if (servoButtonReady) servoButton.write(buttonServoPos);
  Serial.printf("[SERVO] BTN pin=%d %s\n", BUTTON_SERVO_PIN, servoButtonReady ? "OK" : "FAIL");

  for (int i = 0; i < 4; ++i) {
    joyServos[i].setPeriodHertz(50);
    joyReady[i] = joyServos[i].attach((uint8_t)JOY_SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    if (joyReady[i]) joyServos[i].write((int)joyPos[i]);
    Serial.printf("[SERVO] JOY#%d pin=%d %s\n", i + 1, JOY_SERVO_PINS[i], joyReady[i] ? "OK" : "FAIL");
  }

  magnetActiveState = false;
  applyMagnetOutput();
  debugMagnetOutput();
  lastUpdateMs = millis();
  lastPrintMs = millis();
  lastButtons = 0;
  lastMisc = 0;
  lastDpad = 0;
}

void loop() {
  BP32.update();

  const uint32_t now = millis();
  if (now - lastUpdateMs < UPDATE_MS) return;

  const float dt = (float)(now - lastUpdateMs) / 1000.0f;
  lastUpdateMs = now;

  ControllerPtr ctl = firstTargetController();
  bool magnetActive = magnetIsActive();
  applyMagnetOutput();  // Keep output asserted continuously.

  if (!ctl) {
    if (lastLedTargetValid) {
      lastLedTargetValid = false;
      Serial.println("[BT] Target controller missing; LED updates paused");
    }
    return;
  }

  updateControllerLedForMagnet(ctl, magnetActive);

  const uint16_t buttons = (uint16_t)ctl->buttons();
  const uint8_t misc = (uint8_t)ctl->miscButtons();
  const uint8_t dpad = (uint8_t)ctl->dpad();
  const uint16_t changedButtons = (uint16_t)(buttons ^ lastButtons);
  const uint8_t changedMisc = (uint8_t)(misc ^ lastMisc);
  const uint8_t changedDpad = (uint8_t)(dpad ^ lastDpad);
  if (changedButtons || changedMisc) {
    Serial.printf("[BTN] buttons=0x%04X (chg=0x%04X) misc=0x%02X (chg=0x%02X) Lmask=%u Rmask=%u\n",
                  (unsigned)buttons, (unsigned)changedButtons,
                  (unsigned)misc, (unsigned)changedMisc,
                  (buttons & BTN_TRIGGER_LEFT_MASK) ? 1 : 0,
                  (buttons & BTN_TRIGGER_RIGHT_MASK) ? 1 : 0);
  }

  // Toggle logical magnet state with button 0.
  if ((changedButtons & BTN_MAGNET_TOGGLE_MASK) && (buttons & BTN_MAGNET_TOGGLE_MASK)) {
    magnetActiveState = !magnetActiveState;
    magnetActive = magnetActiveState;
    applyMagnetOutput();
    Serial.printf("[MAG] toggled by button0 -> %s\n", magnetActive ? "ACTIVE" : "INACTIVE");
    debugMagnetOutput();
  }

  // Toggle all joystick servos with one button edge.
  if ((changedButtons & BTN_JOY_TOGGLE_MASK) && (buttons & BTN_JOY_TOGGLE_MASK)) {
    setJoyServosEnabled(!joyEnabled);
  }

  // D-pad up/right/down/left each increments one servo offset by +1 degree.
  if (changedDpad) {
    if ((changedDpad & 0x01u) && (dpad & 0x01u)) {
      joyOffsetDeg[0] += 1;
      Serial.printf("[JOY] offset s0=%d\n", joyOffsetDeg[0]);
    }
    if ((changedDpad & 0x02u) && (dpad & 0x02u)) {
      joyOffsetDeg[1] += 1;
      Serial.printf("[JOY] offset s1=%d\n", joyOffsetDeg[1]);
    }
    if ((changedDpad & 0x04u) && (dpad & 0x04u)) {
      joyOffsetDeg[2] += 1;
      Serial.printf("[JOY] offset s2=%d\n", joyOffsetDeg[2]);
    }
    if ((changedDpad & 0x08u) && (dpad & 0x08u)) {
      joyOffsetDeg[3] += 1;
      Serial.printf("[JOY] offset s3=%d\n", joyOffsetDeg[3]);
    }
  }
  lastButtons = buttons;
  lastMisc = misc;
  lastDpad = dpad;

  const int lt = ctl->brake();
  const int rt = ctl->throttle();
  const float v360 = triggerToVelocityDegPerSec(lt, rt, MAX_DEG_PER_SEC_360);
  servo360Pos = (int)constrain((float)servo360Pos + (v360 * dt), SERVO_POS_MIN, SERVO_POS_MAX);
  if (servo360Ready) servo360.write(servo360Pos);

  const float vBtn = buttonToVelocityDegPerSec(buttons, MAX_DEG_PER_SEC_BTN);
  buttonServoPos = (int)constrain((float)buttonServoPos + (vBtn * dt), SERVO_POS_MIN, SERVO_POS_MAX);
  if (servoButtonReady) servoButton.write(buttonServoPos);

  // Per-servo joystick velocity mode.
  // Bluepad32 Y axes are negative when pushing up.
  const int joyAxis[4] = {
    ctl->axisX(),
    -ctl->axisY(),
    ctl->axisRX(),
    -ctl->axisRY()
  };
  for (int i = 0; i < 4; ++i) {
    const float vJoy = axisToVelocityDegPerSec(joyAxis[i], MAX_DEG_PER_SEC_JOY);
    joyBasePos[i] = constrain(joyBasePos[i] + (vJoy * dt), SERVO_POS_MIN, SERVO_POS_MAX);
    joyPos[i] = constrain(joyBasePos[i] + (float)joyOffsetDeg[i], SERVO_POS_MIN, SERVO_POS_MAX);
    if (joyEnabled && joyReady[i]) joyServos[i].write((int)joyPos[i]);
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.printf("[CTRL] LX=%d LY=%d RX=%d RY=%d LT=%d RT=%d DPAD=0x%02X MAG=%u JOY_EN=%u | 360=%d BTN=%d JOY=%.1f %.1f %.1f %.1f OFF=%d %d %d %d\n",
                  joyAxis[0], joyAxis[1], joyAxis[2], joyAxis[3],
                  lt, rt, (unsigned)dpad, magnetActive ? 1 : 0, joyEnabled ? 1 : 0,
                  servo360Pos, buttonServoPos,
                  joyPos[0], joyPos[1], joyPos[2], joyPos[3],
                  joyOffsetDeg[0], joyOffsetDeg[1], joyOffsetDeg[2], joyOffsetDeg[3]);
  }
}
