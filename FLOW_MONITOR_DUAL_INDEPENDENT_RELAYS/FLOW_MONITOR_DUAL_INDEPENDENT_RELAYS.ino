/* ***************************************************************
 * PROJECT: FLOW MONITOR & DUAL INDEPENDENT RELAYS
 * VERSION: v6.4 (EEPROM Retention & Save Confirmation)
 * *************************************************************** */

#include <EEPROM.h>
#include <TM1637Display.h>

// ПИНЫ
#define PIN_UP_BTN      8   
#define PIN_OK_BTN      9   
#define PIN_DN_BTN      10  
#define PIN_LED_ALARM   13  
#define PIN_LED_FLOW    A2  
#define PIN_ALARM_OUT   A3  
#define PIN_IN1         0   
#define PIN_IN2         1   
#define PIN_FLOW_SENSOR 2   
#define PIN_GREEN_IN    3   
#define CLK             A5  
#define DIO             A4  
#define PIN_RELAY_1     5   
#define PIN_RELAY_2     6   

TM1637Display display(CLK, DIO);

// Структура настроек
struct Config {
  byte checkByte; 
  uint16_t P1_Calib; 
  uint16_t P2_MinFlow;     
  byte P3_Delay;       
  uint16_t P4_R1Mode;     
  uint16_t P5_R1TimeA;  
  uint16_t P6_R1TimeB;  
  uint16_t P7_R2Mode;     
  uint16_t P8_R2TimeA;  
  uint16_t P9_R2TimeB;  
  byte P10_Bright;    
};
Config settings;

// Сегменты для надписи "SAVE"
const uint8_t SEG_SAVE[] = {
  0x6d, // S
  0x77, // A
  0x1e, // V
  0x79  // E
};

enum Mode { MODE_WORKING, MODE_MENU, MODE_EDIT };
Mode currentMode = MODE_WORKING;
enum Preset { P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, PRESET_COUNT };
Preset currentPreset = P1;

// Переменные потока и таймеров (как в v6.3)
volatile unsigned long flowPulseCount = 0; 
unsigned long lastFlowMillis = 0, flowPulseSnapshot = 0, currentFlow_x100 = 0, stabilizationStart = 0;
bool stabilizationComplete = false;
unsigned long rTimer[3] = {0, 0, 0}; 
int rStep[3] = {0, 0, 0}; 
bool rState[3] = {false, false, false};
bool lastIn[3] = {HIGH, HIGH, HIGH};
unsigned long lastDeb[3] = {0, 0, 0};

void flowISR() { flowPulseCount++; }

void setup() {
  pinMode(PIN_OK_BTN, INPUT_PULLUP); pinMode(PIN_UP_BTN, INPUT_PULLUP); pinMode(PIN_DN_BTN, INPUT_PULLUP);
  pinMode(PIN_IN1, INPUT_PULLUP); pinMode(PIN_IN2, INPUT_PULLUP); pinMode(PIN_GREEN_IN, INPUT_PULLUP); 
  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  pinMode(PIN_LED_ALARM, OUTPUT); pinMode(PIN_LED_FLOW, OUTPUT); pinMode(PIN_ALARM_OUT, OUTPUT);
  pinMode(PIN_RELAY_1, OUTPUT); pinMode(PIN_RELAY_2, OUTPUT);

  EEPROM.get(0, settings);
  // Если checkByte совпадает (например 0x63), блок IF пропускается и настройки остаются старыми
  if (settings.checkByte != 0x63) { 
    settings.checkByte = 0x63;
    settings.P1_Calib = 435; settings.P2_MinFlow = 50; settings.P3_Delay = 5;
    settings.P4_R1Mode = 0; settings.P5_R1TimeA = 10; settings.P6_R1TimeB = 5;
    settings.P7_R2Mode = 0; settings.P8_R2TimeA = 10; settings.P9_R2TimeB = 5;
    settings.P10_Bright = 3;
    EEPROM.put(0, settings);
  }
  
  display.setBrightness(settings.P10_Bright);
  uint8_t all[] = {0xff, 0xff, 0xff, 0xff}; display.setSegments(all);
  delay(1000); display.clear();
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), flowISR, FALLING);
}

void loop() {
  handleButtons();
  processRelayLogic(1); 
  processRelayLogic(2); 

  if (currentMode == MODE_WORKING) {
    if (millis() - lastFlowMillis >= 1000) {
      noInterrupts(); flowPulseSnapshot = flowPulseCount; flowPulseCount = 0; interrupts();
      lastFlowMillis = millis();
      if (settings.P1_Calib == 0) currentFlow_x100 = flowPulseSnapshot;
      else currentFlow_x100 = (flowPulseSnapshot * 6000UL) / (unsigned long)settings.P1_Calib;
    }

    bool alarmActive = !digitalRead(PIN_GREEN_IN);
    if (alarmActive) {
      if (stabilizationStart == 0) stabilizationStart = millis();
      if (!stabilizationComplete && (millis() - stabilizationStart >= (uint32_t)settings.P3_Delay * 1000)) stabilizationComplete = true;
      if (stabilizationComplete) {
        bool flowOk = (settings.P1_Calib == 0) ? (flowPulseSnapshot >= settings.P2_MinFlow) : (currentFlow_x100 >= settings.P2_MinFlow);
        digitalWrite(PIN_LED_FLOW, flowOk); digitalWrite(PIN_LED_ALARM, !flowOk); digitalWrite(PIN_ALARM_OUT, !flowOk);
      }
    } else {
      stabilizationStart = 0; stabilizationComplete = false;
      digitalWrite(PIN_LED_FLOW, LOW); digitalWrite(PIN_LED_ALARM, LOW); digitalWrite(PIN_ALARM_OUT, LOW);
    }
    display.showNumberDecEx(min(currentFlow_x100, 9999), (settings.P1_Calib > 0 ? 0x40 : 0), true);
  } else drawMenu();
}

void processRelayLogic(int id) {
  bool inVal = (id == 1) ? digitalRead(PIN_IN1) : digitalRead(PIN_IN2);
  uint16_t mode = (id == 1) ? settings.P4_R1Mode : settings.P7_R2Mode;
  uint32_t tA = (id == 1) ? (uint32_t)settings.P5_R1TimeA * 1000 : (uint32_t)settings.P8_R2TimeA * 1000;
  uint32_t tB = (id == 1) ? (uint32_t)settings.P6_R1TimeB * 1000 : (uint32_t)settings.P9_R2TimeB * 1000;
  int pin = (id == 1) ? PIN_RELAY_1 : PIN_RELAY_2;

  switch (mode) {
    case 0: digitalWrite(pin, !inVal); break;
    case 1:
      if (inVal == LOW && lastIn[id] == HIGH && (millis() - lastDeb[id] > 200)) {
        rState[id] = !rState[id]; digitalWrite(pin, rState[id]); lastDeb[id] = millis();
      }
      lastIn[id] = inVal; break;
    case 2:
      if (inVal == LOW && rStep[id] == 0 && (millis() - lastDeb[id] > 200)) { digitalWrite(pin, HIGH); rTimer[id] = millis(); rStep[id] = 1; }
      if (rStep[id] == 1 && (millis() - rTimer[id] >= (tA > 50 ? tA - 50 : tA))) { digitalWrite(pin, LOW); rStep[id] = 2; }
      if (rStep[id] == 2 && inVal == HIGH) rStep[id] = 0; break;
    case 3: // ЦИКЛ (Привязан к нажатию IN)
      if (inVal == LOW) {
        if (tA == 0) { digitalWrite(pin, LOW); rState[id] = false; return; }
        if (millis() - rTimer[id] >= (rState[id] ? (tA > 50 ? tA - 50 : tA) : (tB > 50 ? tB - 50 : tB))) { 
          rTimer[id] = millis(); rState[id] = !rState[id]; digitalWrite(pin, rState[id]); 
        }
      } else {
        digitalWrite(pin, LOW); rState[id] = false; rTimer[id] = millis();
      }
      break;
    case 4:
      if (inVal == LOW && rStep[id] == 0 && (millis() - lastDeb[id] > 200)) { rTimer[id] = millis(); rStep[id] = 1; }
      if (rStep[id] == 1 && (millis() - rTimer[id] >= (tB > 50 ? tB - 50 : tB))) { digitalWrite(pin, HIGH); rTimer[id] = millis(); rStep[id] = 2; }
      if (rStep[id] == 2 && (millis() - rTimer[id] >= (tA > 50 ? tA - 50 : tA))) { digitalWrite(pin, LOW); rStep[id] = 3; }
      if (rStep[id] == 3 && inVal == HIGH) rStep[id] = 0; break;
  }
}

void handleButtons() {
  static uint32_t btnT = 0; static bool okH = false;
  bool ok = !digitalRead(PIN_OK_BTN);
  
  if (ok) {
    if (!okH) { okH = true; btnT = millis(); }
    if (okH && (millis() - btnT >= 2000)) { 
      okH = false; 
      currentMode = (currentMode == MODE_WORKING ? MODE_MENU : MODE_WORKING); 
      display.clear(); delay(300); 
    }
  } else {
    if (okH) {
      if (currentMode == MODE_MENU) {
        currentMode = MODE_EDIT; 
      } else if (currentMode == MODE_EDIT) { 
        // ЗАПИСЬ И ПОДТВЕРЖДЕНИЕ
        EEPROM.put(0, settings); 
        display.setSegments(SEG_SAVE); 
        delay(500); // Показываем надпись SAVE
        display.setBrightness(settings.P10_Bright); 
        currentMode = MODE_MENU; 
      }
      okH = false; delay(200);
    }
  }
  if (currentMode != MODE_WORKING) {
    if (!digitalRead(PIN_UP_BTN)) { if (currentMode == MODE_MENU) currentPreset = (Preset)((currentPreset + 1) % PRESET_COUNT); else modifyValue(1); delay(150); }
    if (!digitalRead(PIN_DN_BTN)) { if (currentMode == MODE_MENU) currentPreset = (Preset)((currentPreset == 0) ? PRESET_COUNT - 1 : currentPreset - 1); else modifyValue(-1); delay(150); }
  }
}

void modifyValue(int step) {
  uint16_t* v = NULL;
  switch(currentPreset) {
    case P1: v = &settings.P1_Calib; break; case P2: v = &settings.P2_MinFlow; break;
    case P3: { int r = (int)settings.P3_Delay + step; settings.P3_Delay = constrain(r, 1, 255); return; }
    case P4: { int r = (int)settings.P4_R1Mode + step; settings.P4_R1Mode = constrain(r, 0, 4); return; }
    case P5: v = &settings.P5_R1TimeA; break; case P6: v = &settings.P6_R1TimeB; break;
    case P7: { int r = (int)settings.P7_R2Mode + step; settings.P7_R2Mode = constrain(r, 0, 4); return; }
    case P8: v = &settings.P8_R2TimeA; break; case P9: v = &settings.P9_R2TimeB; break;
    case P10: { int r = (int)settings.P10_Bright + step; settings.P10_Bright = constrain(r, 1, 7); return; }
  }
  if (v) { int32_t r = (int32_t)*v + step; *v = (uint16_t)constrain(r, 0, 9999); }
}

void drawMenu() {
  int n = currentPreset + 1;
  if (currentMode == MODE_MENU) {
    uint8_t s[4] = { 0x73, 0, 0, 0 };
    if (n < 10) { s[1] = display.encodeDigit(n); }
    else { s[1] = display.encodeDigit(1); s[2] = display.encodeDigit(0); }
    display.setSegments(s);
  } else {
    uint16_t val = 0;
    switch(currentPreset) {
      case P1: val = settings.P1_Calib; break; case P2: val = settings.P2_MinFlow; break;
      case P3: val = settings.P3_Delay; break; case P4: val = settings.P4_R1Mode; break;
      case P5: val = settings.P5_R1TimeA; break; case P6: val = settings.P6_R1TimeB; break;
      case P7: val = settings.P7_R2Mode; break; case P8: val = settings.P8_R2TimeA; break;
      case P9: val = settings.P9_R2TimeB; break; case P10: val = settings.P10_Bright; break;
    }
    display.showNumberDec(val);
  }
}