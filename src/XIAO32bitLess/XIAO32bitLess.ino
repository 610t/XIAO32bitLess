#include <Arduino.h>

#define BTN_PIN D1
#define SPEAKER_PIN A3

#if !defined(ARDUINO_WIO_TERMINAL)
#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/7, /* data=*/6, /* reset=*/U8X8_PIN_NONE);  // OLEDs without Reset of the Display

#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
#endif

#if defined(ARDUINO_XIAO_ESP32C3)
#include "I2C_MPU6886.h"
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);
#endif

#if defined(ARDUINO_WIO_TERMINAL)
// Display
#include "TFT_eSPI.h"
TFT_eSPI tft;
// For IMU
#include "LIS3DHTR.h"
#include <SPI.h>
LIS3DHTR<TwoWire> lis;
// GPIO
// for PortB
#define PIN0_INPUT GPIO_NUM_36  // analog input
#define PIN1_INPUT GPIO_NUM_26
#endif

#if defined(ARDUINO_XIAO_ESP32C3)
// For multitask for play tone
TaskHandle_t playToneTaskHandle = NULL;
#endif
bool isPlayTone = false;

// Play tone parameters
uint32_t duration;
uint8_t volume;

#if defined(ARDUINO_XIAO_ESP32C3)
#include <BLEDevice.h>
#include <BLEUtils.h>
#else
// Wio Terminal & nRF52840 Sense
#undef min
#undef max
#include <rpcBLEDevice.h>
#endif
#include <BLEServer.h>
#include <BLE2902.h>

#define MBIT_MORE_SERVICE "0b50f3e4-607f-4151-9091-7d008d6ffc5c"
#define MBIT_MORE_CH_COMMAND "0b500100-607f-4151-9091-7d008d6ffc5c"       // R&W(20byte)
#define MBIT_MORE_CH_STATE "0b500101-607f-4151-9091-7d008d6ffc5c"         // R(7byte)
#define MBIT_MORE_CH_MOTION "0b500102-607f-4151-9091-7d008d6ffc5c"        // R(18byte)    :pitch,roll,accel,and gyro
#define MBIT_MORE_CH_PIN_EVENT "0b500110-607f-4151-9091-7d008d6ffc5c"     // R&N
#define MBIT_MORE_CH_ACTION_EVENT "0b500111-607f-4151-9091-7d008d6ffc5c"  // R&N(20byte)  :Buttons with timestamp
#define MBIT_MORE_CH_ANALOG_IN_P0 "0b500120-607f-4151-9091-7d008d6ffc5c"  // R
#define MBIT_MORE_CH_ANALOG_IN_P1 "0b500121-607f-4151-9091-7d008d6ffc5c"  // R
#define MBIT_MORE_CH_ANALOG_IN_P2 "0b500122-607f-4151-9091-7d008d6ffc5c"  // R
#define MBIT_MORE_CH_MESSAGE "0b500130-607f-4151-9091-7d008d6ffc5c"       // R : only for v2
#define ADVERTISING_STRING "BBC micro:bit [m5scr]"

// COMMAND CH 20byte
uint8_t cmd[] = { 0x02,  // microbit version (v1:0x01, v2:0x02)
                  0x02,  // protocol 0x02 only
                  0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00 };

// STATE CH 7byte
uint8_t state[] = {
  0x00, 0x00, 0x00, 0x00,  // GPIO 0-3
  0x00,                    // lightlevel
  0x00,                    // temperature(+128)
  0x00                     // soundlevel
};

// MOTION CH 18 byte
uint8_t motion[] = {
  0x00, 0x00,  // pitch
  0x00, 0x00,  // roll
  0xff, 0xff,  // ax
  0xff, 0x00,  // ay
  0x00, 0xff,  // az
  0x00, 0x00,  // gx
  0x00, 0x00,  // gy
  0x00, 0x00,  // gz
  0x00, 0x00   // ??
};

// ACTION CH 20 byte
uint8_t action[] = {
  0x01,                    // BUTTON cmd; BUTTON:0x01, GESTURE: 0x02
  0x01, 0x00,              // Button Name;1:A,2:B,100:P0,101:P1,102:P2,121:LOGO
  0x00,                    // Event Name;1:DOWN, 2:UP, 3:CLICK, 4:LONG_CLICK, 5:HOLD, 6:DOUBLE_CLICK
  0x00, 0x00, 0x00, 0x00,  // Timestamp
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,
  0x12  // ACTION Event
};

// ANALOG PIN 2 byte
uint8_t analog[] = { 0x00, 0x00 };

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic[9] = { 0 };
bool deviceConnected = false;

// for pixel pattern
#if defined(ARDUINO_WIO_TERMINAL)
#define TEXT_SPACE 30
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#else
#define TEXT_SPACE 10
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#endif
#define WHITE 1
#define BLACK 0
uint16_t pixel[5][5] = { 0 };

void drawPixel(int x, int y, int c) {
  int w = DISPLAY_WIDTH;
  int h = DISPLAY_HEIGHT;

  int ps = (w < (h - TEXT_SPACE)) ? w / 5 : (h - TEXT_SPACE) / 5;  // Pixel size

#if defined(ARDUINO_WIO_TERMINAL)
  tft.fillRect(x * ps, y * ps + TEXT_SPACE, ps, ps, c);
#else
  if (c == WHITE) {
    u8g2.drawBox(x * ps, y * ps + TEXT_SPACE, ps, ps);
    u8g2.sendBuffer();
  }
#endif
};

void displayShowPixel() {
#if defined(ARDUINO_XIAO_ESP32C3)
  u8g2.clearDisplay();
#endif
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      log_i("%1d", pixel[y][x] & 0b1);
      if (pixel[y][x] & 0b1) {
#if defined(ARDUINO_WIO_TERMINAL)
        drawPixel(x, y, TFT_RED);
#else
        drawPixel(x, y, WHITE);
#endif
      } else {
#if defined(ARDUINO_WIO_TERMINAL)
        drawPixel(x, y, TFT_BLACK);
#endif
        // You must clear pixel with black.
        // drawPixel(x, y, 0);
      }
    }
  }
};

void fillScreen(int c) {
#if defined(ARDUINO_WIO_TERMINAL)
  tft.fillScreen(c);
#else
  if (c == BLACK) {
    u8g2.clearDisplay();
  } else {
    u8g2.drawBox(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    u8g2.sendBuffer();
  }
#endif
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    log_i("connect\n");
    deviceConnected = true;

#if defined(ARDUINO_XIAO_ESP32C3)
    u8g2.clearDisplay();
    u8g2.sendBuffer();
    fillScreen(BLACK);
#else
    fillScreen(TFT_BLACK);
#endif
  };

  void onDisconnect(BLEServer *pServer) {
    log_i("disconnect\n");
    deviceConnected = false;
#if defined(ARDUINO_XIAO_ESP32C3)
    ESP.restart();
#else
    setup();
#endif
  }
};

// dummy callback
class DummyCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    log_i("DUMMY Read\n");
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    log_i("DUMMY Write\n");
  }
};

// for cmd

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    log_i("CMD read\n");
    pCharacteristic->setValue(cmd, 20);
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    log_i("CMD write\n");
    ////// MUST implement!!
    //// CMD_CONFIG 0x00
    // MIC    0x01
    // TOUCH  0x02
    //// CMD_PIN  0x01
    // SET_OUTPUT 0x01
    // SET_PWM    0x02
    // SET_SERVO  0x03
    // SET_PULL   0x04
    // SET_EVENT  0x05

    std::string value = pCharacteristic->getValue();
    log_i("CMD len:%d\n", value.length());
    log_i("%s\n", value.c_str());
    const char *cmd_str = value.c_str();
    log_i("%s\n", cmd_str);
    char cmd = (cmd_str[0] >> 5);
    if (cmd == 0x02) {
      //// CMD_DISPLAY  0x02
      log_i("CMD display\n");
      char cmd_display = cmd_str[0] & 0b11111;
      if (cmd_display == 0x00) {
        // CLEAR    0x00
        log_i(">> clear\n");
        fillScreen(BLACK);
      } else if (cmd_display == 0x01) {
        // TEXT     0x01
        log_i(">> text\n");
        log_i("%s\n", &(cmd_str[1]));
#if defined(ARDUINO_WIO_TERMINAL)
        tft.fillRect(0, 0, 320, TEXT_SPACE - 1, BLACK);
        tft.drawString(String(&(cmd_str[1])), 0, 0);
#else
        u8x8.setCursor(0, 0);
        u8x8.printf("                 ");  // Clear text space
        u8x8.setCursor(0, 0);
        u8x8.printf("%s", &(cmd_str[1]));
#endif
      } else if (cmd_display == 0x02) {
        // PIXELS_0 0x02
        log_i(">> pixel0\n");
        for (int y = 0; y < 3; y++) {
          for (int x = 0; x < 5; x++) {
            pixel[y][x] = (cmd_str[y * 5 + (x + 1)] & 0xb);
          }
        }
      } else if (cmd_display == 0x03) {
        // PIXELS_1 0x03
        log_i(">> pixel1\n");
        for (int y = 3; y < 5; y++) {
          for (int x = 0; x < 5; x++) {
            pixel[y][x] = (cmd_str[(y - 3) * 5 + (x + 1)] & 0xb);
          }
        }
        displayShowPixel();
      }
    } else if (cmd == 0x03) {
      //// CMD_AUDIO  0x03
      log_i("CMD audio\n");
      char cmd_audio = cmd_str[0] & 0b11111;
      if (cmd_audio == 0x00) {
        // STOP_TONE  0x00
        log_i(">> Stop tone\n");
        isPlayTone = false;
      } else if (cmd_audio == 0x01) {
        // PLAY_TONE  0x01
        const uint8_t max_volume = 5;
        log_i(">> Play tone\n");
        duration = (cmd_str[4] & 0xff) << 24
                   | (cmd_str[3] & 0xff) << 16
                   | (cmd_str[2] & 0xff) << 8
                   | (cmd_str[1] & 0xff);
        volume = map(cmd_str[5], 0, 255, 0, max_volume);
        isPlayTone = true;
      }
    } else if (cmd == 0x04) {
      //// CMD_DATA (only v2) 0x04
      log_i("CMD DATA\n");

      // Show input data.
      log_i(">>> Data input:");
      for (int i = 0; i <= 20; i++) {
        log_i("(%d)%02x%c:", i, cmd_str[i], cmd_str[i]);
      }
      log_i("\n");

      // Convert from input data to label & data.
      char label[9] = { 0 };
      strncpy(label, &cmd_str[1], sizeof(label) - 1);
      String label_str = String(label);

      char data[12] = { 0 };
      strncpy(data, &cmd_str[9], sizeof(data) - 1);
      String data_str = String(data);

      // Convert from 8bit uint8_t x 4 to 32bit float with little endian.
      static union {
        uint32_t i;
        uint8_t b[sizeof(float)];
        float f;
      } conv_data;
      conv_data.b[0] = cmd_str[9];
      conv_data.b[1] = cmd_str[10];
      conv_data.b[2] = cmd_str[11];
      conv_data.b[3] = cmd_str[12];
      float data_val = conv_data.f;

      log_i("Label str:%s, Data str:%s, Data value:%f.\n", label_str, data_str, data_val);

      // Can't get correct command for number=0x13 and text=0x14. Why?
      char cmd_data = cmd_str[20];
      if (cmd_data == 0x13) {
        log_i("Data is Number.\n");
      } else if (cmd_data == 0x14) {
        log_i("Data is Text.\n");
      } else {
        log_i("Data is Unknown:%02x.\n", cmd_data);
      }
    }
  }
};

// for temperature
float temp = 0;

// for state
class StateCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {

#if defined(ARDUINO_WIO_TERMINAL)
    temp = lis.getTemperature();
    int light = (int)map(analogRead(WIO_LIGHT), 0, 511, 0, 255);
    log_i(">> Light Level " + String(light));
    state[4] = (light & 0xff);  // lightlevel
    int mic = (int)map(analogRead(WIO_MIC), 0, 511, 0, 255);
    state[6] = (mic & 0xff);  // soundlevel
    log_i(">> sound Level " + String(mic));
#else
    state[6] = (random(256) & 0xff);  // Random sensor value for soundlevel
#endif
    state[5] = ((int)(temp + 128) & 0xff);  // temperature(+128)

    log_i("STATE read %s", (char *)state);
    pCharacteristic->setValue(state, 7);
  }
};

// for accelerometer related values
#define ACC_MULT 512
#if !defined(RAD_TO_DEG)
#define RAD_TO_DEG 57.324
#endif
float ax = 0, ay = 0, az = 0;
int16_t iax, iay, iaz;
float gx, gy, gz;
float pitch, roll, yaw;

void updateIMU() {
#if defined(ARDUINO_WIO_TERMINAL)
  lis.getAcceleration(&ay, &ax, &az);
  pitch = atan(-ax / sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  roll = atan(ay / az) * RAD_TO_DEG;
#else
  imu.getAccel(&ax, &ay, &az);
  imu.getGyro(&gx, &gy, &gz);
  imu.getTemp(&temp);
#endif

  iax = (int16_t)(ax * ACC_MULT);
  iay = (int16_t)(ay * ACC_MULT);
  iaz = (int16_t)(az * ACC_MULT);
}

class MotionCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    updateIMU();

    motion[0] = ((int)(pitch * ACC_MULT) & 0xff);
    motion[1] = (((int)(pitch * ACC_MULT) >> 8) & 0xff);
    motion[2] = ((int)(roll * ACC_MULT) & 0xff);
    motion[3] = (((int)(roll * ACC_MULT) >> 8) & 0xff);
    motion[4] = (iax & 0xff);
    motion[5] = ((iax >> 8) & 0xff);
    motion[6] = (iay & 0xff);
    motion[7] = ((iay >> 8) & 0xff);
    motion[8] = (-iaz & 0xff);
    motion[9] = ((-iaz >> 8) & 0xff);
    pCharacteristic->setValue(motion, 20);

    // debug print
    char msg[256] = { 0 };
    for (int i = 0; i < sizeof(motion); i++) {
      sprintf(&msg[i * 3], "%02x,", motion[i], sizeof(motion) * 3 - 3 * i);
    }
    log_i("MOTION read: %s\n", msg);
  }
};

// for button
class ActionCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    log_i("BTN read\n");
    pCharacteristic->setValue("Read me!!");  // dummy data
  }
};

// for Analog pin
class AnalogPinCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    int r = 280;

    log_i("Analog Pin0 Read:%d\n", r);

    analog[0] = (r & 0xff);
    analog[1] = ((r >> 8) & 0xff);

    pCharacteristic->setValue(analog, 2);
  }
};

void setup() {
  Serial.begin(115200);

#if defined(ARDUINO_WIO_TERMINAL)
  // Display
  tft.begin();
  tft.setRotation(3);

  // IMU
  lis.begin(Wire1);
  delay(100);
  lis.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  lis.setFullScaleRange(LIS3DHTR_RANGE_2G);
  lis.openTemp();
  //// Button
  // 5 way switch
  pinMode(WIO_5S_UP, INPUT_PULLUP);
  pinMode(WIO_5S_DOWN, INPUT_PULLUP);
  pinMode(WIO_5S_LEFT, INPUT_PULLUP);
  pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
  pinMode(WIO_5S_PRESS, INPUT_PULLUP);
  // 3 Configurable Button
  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_B, INPUT_PULLUP);
  pinMode(WIO_KEY_C, INPUT_PULLUP);
  // Light sensor
  pinMode(WIO_LIGHT, INPUT);
  // microphone
  pinMode(WIO_MIC, INPUT);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);
#else
  // button
  pinMode(BTN_PIN, INPUT_PULLUP);

  // for OLED Display
  //// for text
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  //// for 5x5 LED
  u8g2.begin();
  u8g2.setFlipMode(1);
  u8g2.clearDisplay();
  u8g2.sendBuffer();
#endif

#if defined(ARDUINO_XIAO_ESP32C3)
  // Setup IMU MPU6886
  imu.begin();
#endif

  // for buzzer
  pinMode(SPEAKER_PIN, OUTPUT);

  // Create MAC address base fixed ID
  uint8_t mac0[6] = { 0 };

#if defined(ARDUINO_WIO_TERMINAL)
  // Create random mac address for avoid conflict ID.
  randomSeed(analogRead(A0));
  for (int i = 0; i < sizeof(mac0); i++) {
    mac0[i] = random(256);
  }
#else
  esp_efuse_mac_get_default(mac0);
#endif

  String ID;
  for (int i = 0; i < 6; i++) {
    char ID_char = (((mac0[i] - 0x61) & 0b0011110) >> 1) + 0x61;
    ID += ID_char;
  }
  log_i("ID char:%s\n", ID.c_str());
  // Advertisement string for Microbit More.
  char adv_str[32] = { 0 };
  String("BBC micro:bit [" + ID + "]").toCharArray(adv_str, sizeof(adv_str));
  // ID string to show at splash screen.
  char id_str[32] = { 0 };
  String("ID:" + ID).toCharArray(id_str, sizeof(id_str));

  // Start up screen
#if defined(ARDUINO_WIO_TERMINAL)
  fillScreen(TFT_BLUE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.print("Welcome to\nM5bit Less!!\nPlease connect to\n");
  tft.println(adv_str);
#else
  fillScreen(WHITE);
  u8x8.setCursor(0, 0);
  u8x8.printf("%s", id_str);
#endif

  log_i("BLE start.\n");
  log_i("%s\n", adv_str);

  BLEDevice::init(adv_str);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(BLEUUID(MBIT_MORE_SERVICE), 27);

  // CMD
  pCharacteristic[0] = pService->createCharacteristic(
    MBIT_MORE_CH_COMMAND,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristic[0]->setCallbacks(new CmdCallbacks());
  pCharacteristic[0]->addDescriptor(new BLE2902());

  // STATE
  pCharacteristic[1] = pService->createCharacteristic(
    MBIT_MORE_CH_STATE,
    BLECharacteristic::PROPERTY_READ);
  pCharacteristic[1]->setCallbacks(new StateCallbacks());
  pCharacteristic[1]->addDescriptor(new BLE2902());

  // MOTION
  pCharacteristic[2] = pService->createCharacteristic(
    MBIT_MORE_CH_MOTION,
    BLECharacteristic::PROPERTY_READ);
  pCharacteristic[2]->setCallbacks(new MotionCallbacks());
  pCharacteristic[2]->addDescriptor(new BLE2902());

  pCharacteristic[3] = pService->createCharacteristic(
    MBIT_MORE_CH_PIN_EVENT,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic[3]->setCallbacks(new DummyCallbacks());
  pCharacteristic[3]->addDescriptor(new BLE2902());

  // ACTION
  pCharacteristic[4] = pService->createCharacteristic(
    MBIT_MORE_CH_ACTION_EVENT,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic[4]->setCallbacks(new ActionCallbacks());
  pCharacteristic[4]->addDescriptor(new BLE2902());

  // PINS
  pCharacteristic[5] = pService->createCharacteristic(
    MBIT_MORE_CH_ANALOG_IN_P0,
    BLECharacteristic::PROPERTY_READ);
  pCharacteristic[5]->setCallbacks(new AnalogPinCallbacks());
  pCharacteristic[5]->addDescriptor(new BLE2902());

  pCharacteristic[6] = pService->createCharacteristic(
    MBIT_MORE_CH_ANALOG_IN_P1,
    BLECharacteristic::PROPERTY_READ);
  pCharacteristic[6]->setCallbacks(new AnalogPinCallbacks());
  pCharacteristic[6]->addDescriptor(new BLE2902());

  pCharacteristic[7] = pService->createCharacteristic(
    MBIT_MORE_CH_ANALOG_IN_P2,
    BLECharacteristic::PROPERTY_READ);
  pCharacteristic[7]->setCallbacks(new DummyCallbacks());
  pCharacteristic[7]->addDescriptor(new BLE2902());


  // MESSAGE (only for v2)
  pCharacteristic[8] = pService->createCharacteristic(
    MBIT_MORE_CH_MESSAGE,
    BLECharacteristic::PROPERTY_READ);
  pCharacteristic[8]->setCallbacks(new DummyCallbacks());
  pCharacteristic[8]->addDescriptor(new BLE2902());


  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

#if defined(ARDUINO_XIAO_ESP32C3)
  // Multitask for play tone.
  xTaskCreatePinnedToCore(playToneTask, "playToneTask", 4096, NULL, 1, &playToneTaskHandle, 1);
#endif
}

void sendBtn(uint8_t btnID, uint8_t btn, uint8_t btn_status, uint8_t prev) {
  memset((char *)(action), 0, 20);  // clear action buffer

  action[0] = 0x01;   // for Button event
  action[19] = 0x12;  // ACTION_EVENT

  action[1] = btnID;  // btnID 0x01:BtnA, 0x02:BtnB, 121:BtnC(LOGO)

  // Set TimeStamp (Little Endian)
  uint32_t time = (uint32_t)millis();
  action[4] = (time & 0xff);
  action[5] = (time >> 8) & 0xff;
  action[6] = (time >> 16) & 0xff;
  action[7] = (time >> 24) & 0xff;

  if (btn) {
    // Button CLICK
    log_i(" button clicked!\n");
    action[3] = 0x03;
    pCharacteristic[4]->setValue(action, 20);
    pCharacteristic[4]->notify();
  }
  if (btn_status == 0 && prev == 1) {
    // Button Up
    log_i(" button up!\n");
    action[3] = 0x02;
    pCharacteristic[4]->setValue(action, 20);
    pCharacteristic[4]->notify();
  } else if (btn_status == 1 && prev == 0) {
    // Button Down
    log_i(" button down!\n");
    action[3] = 0x01;
    pCharacteristic[4]->setValue(action, 20);
    pCharacteristic[4]->notify();
  }
}

void playToneTask(void *args) {
  while (1) {
    if (isPlayTone) {
      uint16_t freq = 1000000 / duration;

      log_i("Volume:%d\n", volume);
      log_i("Duration:%d\n", duration);
      log_i("Freq:%d\n", freq);
      // Play tone with frequency freq and duration.
      for (long i = 0; i < duration * 1000L; i += freq * 2) {
        digitalWrite(SPEAKER_PIN, HIGH);
        delayMicroseconds(freq);
        digitalWrite(SPEAKER_PIN, LOW);
        delayMicroseconds(freq);
      }
      isPlayTone = false;
    }
    delay(10);
  }
}

// Previous button state
uint8_t prevA = 0, prevB = 0, prevC = 0;
uint32_t old_label_time = 0;

void loop() {
  if (deviceConnected) {
    // Send notify data for button A, B and C(LOGO).
    uint8_t btnA = 0, btnB = 0, btnC = 0,
            btn_statusA = 0, btn_statusB = 0, btn_statusC = 0;

    // Get all button status
#if defined(ARDUINO_WIO_TERMINAL)
    if (digitalRead(WIO_KEY_A) == LOW) {
      btnA = 1;
      btn_statusA = 1;
    }
    if (digitalRead(WIO_KEY_B) == LOW) {
      btnB = 1;
      btn_statusB = 1;
    }
    if (digitalRead(WIO_KEY_C) == LOW) {
      btnC = 1;
      btn_statusC = 1;
    }
#else
    btnA = (digitalRead(BTN_PIN) == 0);
    btn_statusA = btnA;
#endif

#define BUTTON_DELAY 50

    //// Button A
    action[1] = 0x01;
    sendBtn(0x01, btnA, btn_statusA, prevA);
    prevA = btn_statusA;
    delay(BUTTON_DELAY);

    //// Button B
    action[1] = 0x02;
    sendBtn(0x02, btnB, btn_statusB, prevB);
    prevB = btn_statusB;
    delay(BUTTON_DELAY);

    //// Button C (LOGO)
    action[1] = 121;  // LOGO 121
    sendBtn(121, btnC, btn_statusC, prevC);
    prevC = btn_statusC;
    delay(BUTTON_DELAY);

    // updateGesture();

    // Send dummy data label='a' data=random('a'-'z') every 50ms
    uint32_t label_time = (uint32_t)millis();
    if (label_time - old_label_time > 50) {
      memset((char *)(action), 0, 20);  // clear action buffer
      action[19] = 0x14;                // DATA_TEXT
      action[0] = 0x61;                 // 'a'
      action[1] = 0;
      action[8] = 0x61 + random(26);  // 'a-z'
      action[9] = 0;
      pCharacteristic[4]->setValue(action, 20);
      pCharacteristic[4]->notify();
      old_label_time = label_time;
    }
  }
}