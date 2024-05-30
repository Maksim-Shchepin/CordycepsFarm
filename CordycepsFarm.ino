/* 
**************************************************************************

The sketch is written to monitor params at my Cordyceps Farm

Author: Shchepin Maksim @shchepin_ms
Current version: 1.01 as of 30.05.2024

**************************************************************************

*/
// check whether git finds the changes
// it had
//--------------------------------------------------Libraries

#include <AHTxx.h>
#include <Preferences.h>
#include "Timer.h"
#include <WiFiManager.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>

#define GH_INCLUDE_PORTAL
#include <GyverHub.h>

//--------------------------------------------------DEVICE

#define DEVICE_NAME "CORDYFARM" // unique name of device
#define DEVICE_NET "MyDevices" // unique prefix to prevent finding other devices via free mqtt
#define DEVICE_ID 0x000000 // unique id

//--------------------------------------------------WIFI

#define PORTAL_SSID "CORDYFARM_CONFIG"
#define AP_PWD "mycordyfarm"
#define CONNECT_TO_WIFI_TIMEOUT 10 //sec
#define CONFIG_PORTAL_TIMEOUT 120  //sec
#define WIFI_CONNECTION_TIMER_INTERVAL 5000 //ms
#define WIFI_CHANNEL 4
#define WIFI_SSID_HIDDEN 0
#define WIFI_MAX_CONNECTION 1

//--------------------------------------------------TIME

#define AUTO_LIGHT 0
#define MANUAL_LIGHT 1
#define LIGHT_ON 75600
#define LIGHT_OFF 32400
#define SEC_IN_DAY 86400
#define TIME_ZONE_CODE 17

//--------------------------------------------------DELAYS

#define EEPROM_SAVE_DELAY 5000 //ms
#define RESTART_DELAY 1000 //ms

//--------------------------------------------------SENSOR

#define SHT4X_NO_ERROR 0
#define SENSOR_RESTART_TIME 5000 //ms
#define EACH_X_TIME_SENSOR_HEATS 20
#define MEASURING_CYCLE_TIME 10000 //ms
#define SENSOR_DIDNT_START_ERROR 199
#define SENSOR_DIDNT_READ_DATA 249
#define EACH_MEASURING_BUFFER_SIZE 360

//--------------------------------------------------CANVAS

#define CANVAS_NAME "canvas"
#define NATIVE_FONT_INTERVAL 6
#define X_RES 535
#define Y_RES 430
#define X_BORDER_LABEL_SHIFT 4
#define Y_BORDER_LABEL_SHIFT 4
#define XL_SHIFT 30
#define XR_SHIFT 30
#define YT_SHIFT 15
#define YB_SHIFT 15

//-----------------------------------------------------------------------------------Wiring
#define LAMP_PIN 27

//-----------------------------------------------------------------------------------Emulation

boolean emulation = true;

//-----------------------------------------------------------------------------------Variables

String displayTime;

String mqttServer = F("m6.wqtt.ru");
String mqttPort = F("16808");
String mqttUser = F("u_QFLEV8");
String mqttPass = F("aaRYTT20");

//-----------------------------------------------------------------------------------Canvas

uint8_t labelTextAlpha = 128;

uint8_t hWeight = 1;
uint8_t hAlpha = 255;
uint32_t hColor = 0x00ff00;

uint8_t tWeight = 1;
uint8_t tAlpha = 255;
uint32_t tColor = 0x00ffff;

uint8_t gridAlpha = 255;
uint32_t gridColor = 0x3F3F3F;
uint8_t gridWeight = 1;

uint8_t numYLines = 3;
uint8_t numXLines = 5;

uint8_t hMax = 100;
uint8_t hMin = 20;
uint8_t tMax = 50;
uint8_t tMin = 10;

uint8_t humidityBuffer[EACH_MEASURING_BUFFER_SIZE];
uint8_t temperatureBuffer[EACH_MEASURING_BUFFER_SIZE];

int lightOnTime;
int lightOffTime;

unsigned long globalTimeOffset;
int timeZone;
int selectTimeZone;

uint8_t bigFS = 36;
uint8_t titleFS = 24;
uint8_t labelFS = 20;

static char errorMessage[64];
static int16_t error;

gh::Flags backOrRestart;

boolean manualLightControlFlag;
boolean autoLightControlFlag;
boolean sensorReady;
boolean showRestart;
boolean workMode;

//------------------------------------------------------------------------------------Objects
Timer wifiConnectionTimer(WIFI_CONNECTION_TIMER_INTERVAL);
Timer tenSecTimer(MEASURING_CYCLE_TIME);
Timer aMinTimer(60000);
Timer sensorRunTimer;
Timer restartTimer;
Timer canvasUpdateTimer;


WiFiManager wifiManager;
WiFiManagerParameter mqttServerParam;
WiFiManagerParameter mqttPortParam;
WiFiManagerParameter mqttUserParam;
WiFiManagerParameter mqttPassParam;

Preferences preferences;
GyverHub hub;
SensirionI2cSht4x sensor;


void setup() { //---------------------------------------------------------------------Setup
  Serial.begin(115200);
  initializeGPIO();
  getStateData();//---------------------------------------------------------"вспоминание из EEPROM"
  connectToWiFi();
  if (workMode) continueSetup();
}

void continueSetup() {
  runClimaticSensor();//---------------------------------------запуск датчика климата
  getClimaticData();
  initializeHubMQTT(); //------------------------------------------запуск Гувер хаба
  workMode = true;
}

void continueSetupWithoutWifi() {
  Serial.println("Continue without wifi -> start broadcast own AP");
  wifiConnectionTimer.disable();
  runClimaticSensor();//---------------------------------------запуск датчика климата
  getClimaticData();
  broadcastWifi();
  _initializeHub(); //------------------------------------------запуск Гувер хаба
  workMode = true;
}

void loop() { //----------------------------------------------------------------------------------Loop
  if (workMode) {
    workLoop();
  } else {
    setupLoop();
  }
}

//----------------------------------------------------------------------------------------------Continue Loop
void setupLoop() {
  wifiManager.process();

  if (wifiConnectionTimer.triggered()) {
    Serial.println("Waiting for configurating WIFI at 192.168.4.1");
  }
}

void workLoop() { //----------------------------------------------------------------------------------Work Loop

  hub.tick();

  if (restartTimer.triggered()) {
    restartTimer.disable();
    ESP.restart();
  }

  if (canvasUpdateTimer.triggered()) {
    canvasUpdateTimer.disable();
    showBufferedSensorData();
  }

  if (sensorRunTimer.triggered()) {
    sensorRunTimer.disable();
    if (emulation) return;
    runClimaticSensor();
  }

  if (tenSecTimer.triggered()) {
    getClimaticData();
    storeSensorDataToBuffer();
    showBufferedSensorData();
    updateClimateWidgets();
    lightControl();
  }

  if (aMinTimer.triggered()) {
    cTimeUpdate();
  }
}


//--------------------------------------------------------------------------------------------------Main Builder

void build(gh::Builder& b) {

  b.show(showRestart);
  b.Space();
  b.Label(F("GREENHOUSE")).noLabel().fontSize(bigFS).align(gh::Align::Center).noTab();
  b.Space();b.Space();b.Space();b.Space();b.Space();
  b.Label(F("Перезагрузка...")).noLabel().fontSize(titleFS).align(gh::Align::Center).noTab();
  
  b.show(!showRestart);
  b.Menu(F("Панель управления;Настройки бокса;Настройка графиков"));


  boolean showDash = (b.menu() == 0) && !showRestart;
  boolean showSetup = (b.menu() == 1) && !showRestart;
  boolean showChartSettings = (b.menu() == 2) && !showRestart;

  //--------------------------------------------------------------------------DashBoard-Builder---------------------------

  b.show(showDash);
  b.Canvas_(F(CANVAS_NAME), X_RES, Y_RES).noLabel();
  
  b.beginRow();
    b.Label(F("ВЛАЖНОСТЬ:")).noLabel().size(9).fontSize(titleFS).align(gh::Align::Left).noTab();
    b.Label_(F("cHumidity"), humidityBuffer[0]).noLabel().size(3).fontSize(titleFS).noTab().align(gh::Align::Right);
    b.Label(F("%")).noLabel().size(1).fontSize(titleFS).noTab();
    b.Label().noLabel().size(2).fontSize(titleFS).icon("f043").noTab();
  b.endRow();

  b.beginRow();
    b.Label(F("ТЕМПЕРАТУРА:")).noLabel().size(9).fontSize(titleFS).align(gh::Align::Left).noTab();
    b.Label_(F("cTemperature"), temperatureBuffer[0]).noLabel().size(3).fontSize(titleFS).noTab().align(gh::Align::Right);
    b.Label(F("C")).noLabel().size(1).fontSize(titleFS).noTab();
    b.Label().noLabel().size(2).fontSize(titleFS).icon("f2c9").noTab();
  b.endRow();

  b.Select(&autoLightControlFlag).label(F("УПРАВЛЕНИЕ СВЕТОМ")).text(F("ВРУЧНУЮ;АВТО")).attach(autoLightControlFlagChanged).hint(F("Две стратегии управления светом:\n\n1. Вручную - с помощью тумблера.\n\n2. Автоматически - по заданному времени включения и выключения."));

  //----light-auto
  b.show(autoLightControlFlag && showDash);
    b.beginRow();
      b.Time(&lightOnTime).label(F("ВКЛЮЧЕНИЕ СВЕТА")).attach(lightOnTimeChanged).hint(F("Каждый день в это время будет включаться свет."));
      b.Time(&lightOffTime).label(F("ВЫКЛЮЧЕНИЕ СВЕТА")).attach(lightOffTimeChanged).hint(F("Каждый день в это время будет выключаться свет."));
    b.endRow();
    //----humidity-manual
  b.show(!autoLightControlFlag && showDash);
    b.Switch(&manualLightControlFlag).label(F("ТУМБЛЕР СВЕТА")).attach(manualLightControlFlagChanged).hint(F("Ручное управление светом с тумблера - может быть ВКЛ или ВЫКЛ"));

//-----------------------------------------------------------------------------------------------Setup-Builder---------------------------------------------------------------------------------------------

  b.show(showSetup); 

  b.Label(F("СИНХРОНИЗАЦИЯ ВРЕМЕНИ")).noLabel().fontSize(titleFS).align(gh::Align::Left).noTab();

  b.beginRow();
    b.Label_(F("cTime"), displayTime).label(F("ТЕКУЩЕЕ ВРЕМЯ")).fontSize(labelFS).color(gh::Colors::Green).align(gh::Align::Left);
    b.Select(&selectTimeZone).label(F("ЧАСОВОЙ ПОЯС")).attach(timeZoneChanged).text(F("-12 UTC;-11 UTC;-10 UTC;-9:30 UTC;-9 UTC;-8 UTC;-7 UTC;-6 UTC;-5 UTC;-4 UTC;-3:30 UTC;-3 UTC;-2 UTC;-1 UTC;+0 UTC;+1 UTC;+2 UTC;+3 UTC;+3:30 UTC;+4 UTC;+4:30 UTC;+5 UTC;+5:30 UTC;+5:45 UTC;+6 UTC;+6:30 UTC;+7 UTC;+8 UTC;+8:45 UTC;+9 UTC;+9:30 UTC;+10 UTC;+10:30 UTC;+11 UTC;+12 UTC;+12:45 UTC;+13 UTC;+14 UTC"));
  b.endRow();
  b.Space();

  b.Label(F("НАСТРОЙКА MQTT")).noLabel().fontSize(titleFS).align(gh::Align::Left).noTab();
  b.Input(&mqttServer).label(F("АДРЕС СЕРВЕРА"));
  b.Input(&mqttPort).label(F("ПОРТ СЕРВЕРА"));
  b.Input(&mqttUser).label(F("АККАУНТ"));
  b.Pass(&mqttPass).label(F("ПАРОЛЬ"));
  b.Space();

  b.Flags(&backOrRestart).text(F("НАЗАД;СОХРАНИТЬ и ПЕРЕЗАПУСТИТЬ")).noLabel().attach(outOfSettings);

//---------------------------------------------------------------------------------------------Chart-Settings
// uint32_t hColor = 0x00ff00;
// uint32_t tColor = 0x00ffff;
// uint32_t gridColor = 0x3F3F3F;

  b.show(showChartSettings);
  b.Label(F("ЛИНИЯ ВЛАЖНОСТИ")).noLabel().fontSize(titleFS).align(gh::Align::Center).noTab();
  b.Slider(&hWeight).label(F("ТОЛЩИНА ЛИНИИ")).range(1, 5, 1);
  b.Slider(&hAlpha).label(F("ПРОЗРАЧНОСТЬ ЛИНИИ")).range(1, 255, 1);
  b.Slider(&hMax).label(F("МАКСИМАЛЬНАЯ ГРАНИЦА")).range(1, 100, 1);
  b.Slider(&hMin).label(F("МИНИМАЛЬНАЯ ГРАНИЦА")).range(1, 100, 1);

  b.Label(F("ЛИНИЯ ТЕМПЕРАТУРЫ")).noLabel().fontSize(titleFS).align(gh::Align::Center).noTab();
  b.Slider(&tWeight).label(F("ТОЛЩИНА ЛИНИИ")).range(1, 5, 1);
  b.Slider(&tAlpha).label(F("ПРОЗРАЧНОСТЬ ЛИНИИ")).range(1, 255, 1);
  b.Slider(&tMax).label(F("МАКСИМАЛЬНАЯ ГРАНИЦА")).range(1, 100, 1);
  b.Slider(&tMin).label(F("МИНИМАЛЬНАЯ ГРАНИЦА")).range(1, 100, 1);
  
  b.Label(F("ЛИНИИ СЕТКИ")).noLabel().fontSize(titleFS).align(gh::Align::Center).noTab();
  b.Slider(&gridWeight).label(F("ТОЛЩИНА ЛИНИЙ")).range(1, 5, 1);
  b.Slider(&gridAlpha).label(F("ПРОЗРАЧНОСТЬ ЛИНИЙ")).range(1, 255, 1);
  b.Slider(&numYLines).label(F("КОЛ-ВО ЛИНИЙ ПО ОСИ Y")).range(1, 10, 1);
  b.Slider(&numXLines).label(F("КОЛ-ВО ЛИНИЙ ПО ОСИ X")).range(1, 10, 1);
  b.Slider(&labelTextAlpha).label(F("ПРОЗРАЧНОСТЬ ТЕКСТА")).range(1, 255, 1);
  b.Space();


canvasUpdateTimer.enable(1000);
}

void broadcastWifi() {
  WiFi.hostname(DEVICE_NAME);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DEVICE_NAME, AP_PWD, WIFI_CHANNEL, WIFI_SSID_HIDDEN, WIFI_MAX_CONNECTION);
}

void outOfSettings() {
  if (backOrRestart.get(0) == 1) { // back to dashboard
    backOrRestart.write(0, 0);
    hub.menu = 0;
  }
  
  if (backOrRestart.get(1) == 1) { // restart
    backOrRestart.write(1, 0);
    saveMqttConfig();
    restartTimer.enable(RESTART_DELAY);
    showRestart = true;
  }
  hub.sendRefresh();
}

int yPoint(int value, int minValue, int maxValue) {
  int kY = (Y_RES - YB_SHIFT - YT_SHIFT) / (maxValue - minValue);
  int result = Y_RES - YT_SHIFT - kY * (value - minValue);
  int min = Y_RES - YT_SHIFT;
  int max = Y_RES - YT_SHIFT - kY * (maxValue - minValue);
  if (result < max) return max;
  if (result <= min) return result;
  return min;
}



void initializeHubMQTT() {
  int port = mqttPort.toInt();
  Serial.print(F("Connecting to MQTT AT: ")); Serial.print(mqttServer + " "); Serial.print(String(port) + F(" as user = ")); Serial.print(mqttUser + F(" and pass = ")); Serial.println(mqttPass);
  hub.mqtt.config(mqttServer, port, mqttUser, mqttPass);
  _initializeHub();
}

void _initializeHub() {
  hub.config(DEVICE_NET, DEVICE_NAME, "", DEVICE_ID); // net, name, icon, id
  hub.onBuild(build);
  hub.begin();
  hub.onUnix(onUnix);
}

void cTimeUpdate() {
  int hour = getCurrentHour();
  int minute = getCurrentMinute();
  String time;
  if (hour < 10) time = "0";
  time += String(hour) + ":";
  if (minute < 10) time += "0";
  time += String(minute);
  displayTime = time;

  if (!hub.canSend()) return; // не обновляем если не можем
  hub.sendUpdate(F("cTime"), displayTime);
}

void lightControl() {
  if (autoLightControlFlag) {
    autoLightControl();
  } else {
    manualLightControl();
  }
}

void manualLightControl() {
  if (manualLightControlFlag) {
    turnLightOn();
  } else {
    turnLightOff();
  }
}

void autoLightControl() {

  int currentTimeMin = getCurrentHour() * 60 + getCurrentMinute();
  int on = lightOnTime / 60;
  int off = lightOffTime / 60;

  if (on < off) {
    if ((currentTimeMin >= on)&&(currentTimeMin < off)) {
      turnLightOn();
    } else {
      turnLightOff();
    }
  }

  if (on > off) {
    if ((currentTimeMin >= on)||(currentTimeMin < off)) {
      turnLightOn();
    } else {
      turnLightOff();
    }
  }
}
// -------------------------------------------------------------------------------------Attached handlers


void manualLightControlFlagChanged() {
  if (manualLightControlFlag) {
    turnLightOn();
  } else {
    turnLightOff();
  }
  saveBoolState("manualLightCF", manualLightControlFlag);
}

void autoLightControlFlagChanged() {
  saveBoolState("autoLightCF", autoLightControlFlag);
  hub.sendRefresh();
}

void lightOnTimeChanged() {
  saveLongState("lightOnTime", lightOnTime);
}

void lightOffTimeChanged() {
  saveLongState("lightOffTime", lightOffTime);
}

//--------------------------------------------------------------------------------------Utilities

void connectToWiFi() {
  WiFi.hostname(DEVICE_NAME);
  WiFi.mode(WIFI_STA);

  // wifiManager.resetSettings();

  new (&mqttServerParam) WiFiManagerParameter("mqttServer", "mqtt server", mqttServer.c_str(), 40); // id/name, placeholder/prompt, default, length
  new (&mqttPortParam) WiFiManagerParameter("mqttPort", "mqtt port", mqttPort.c_str(), 5); // id/name, placeholder/prompt, default, length
  new (&mqttUserParam) WiFiManagerParameter("mqttUser", "mqtt username", mqttUser.c_str(), 20); // id/name, placeholder/prompt, default, length
  new (&mqttPassParam) WiFiManagerParameter("mqttPass", "mqtt password", mqttPass.c_str(), 20); // id/name, placeholder/prompt, default, length

  wifiManager.addParameter(&mqttServerParam);
  wifiManager.addParameter(&mqttPortParam);
  wifiManager.addParameter(&mqttUserParam);
  wifiManager.addParameter(&mqttPassParam);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
  wifiManager.setConfigPortalTimeoutCallback(continueSetupWithoutWifi);
  wifiManager.setSaveConfigCallback(continueSetupWifi);
  wifiManager.setConnectTimeout(CONNECT_TO_WIFI_TIMEOUT);

  if (wifiManager.autoConnect(PORTAL_SSID, AP_PWD)) {
    workMode = true;
    Serial.print("AUTO_CONNECTED, ip: "); Serial.println(WiFi.localIP());
  } else {
    Serial.print("AUTO_CONNECT FAILED, run portal ");
  }
}

void continueSetupWifi() {
  getPortalParams();
  saveMqttConfig();
  wifiConnectionTimer.disable();
  continueSetup();
}

void getPortalParams() {
  mqttServer = getParam("mqttServer");
  mqttPort = getParam("mqttPort");
  mqttUser = getParam("mqttUser");
  mqttPass = getParam("mqttPass");
}

String getParam(String name) {
  String value;
  if(wifiManager.server->hasArg(name)) {
    value = wifiManager.server->arg(name);
  }
  return value;
}

void saveMqttConfig() {
  saveStrState("mqttServer", mqttServer);
  saveStrState("mqttPort", mqttPort);
  saveStrState("mqttUser", mqttUser);
  saveStrState("mqttPass", mqttPass);
}


void runClimaticSensor() {
  Wire.begin();
  runSHT4x();
}

void runSHT4x() {
  sensor.begin(Wire, SHT40_I2C_ADDR_44);
  sensor.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  error = sensor.serialNumber(serialNumber);
  if (error != SHT4X_NO_ERROR) {
      Serial.print("Error trying to execute serialNumber(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      sensorReady = false;
      sensorRunTimer.enable(SENSOR_RESTART_TIME);
      return;
  }
  Serial.print("serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();
  sensorReady = true;
}

void turnLightOn() {
    digitalWrite(LAMP_PIN, HIGH);
}

void turnLightOff() {
    digitalWrite(LAMP_PIN, LOW);
}

void storeSensorDataToBuffer() {
  for (int i = EACH_MEASURING_BUFFER_SIZE - 1; i > 0; i--) { 
    humidityBuffer[i] = humidityBuffer[i - 1];
    temperatureBuffer[i] = temperatureBuffer[i - 1];
  }
}

void showBufferedSensorData() {

  if (!hub.canSend()) return;
  gh::CanvasUpdate canvas(CANVAS_NAME, &hub);
    canvas.clear();
  canvas.send();

  drawGrid();
  drawGridLabels();
  drawHumidity();
  drawTemperature();
  
}

void drawGrid() {

  gh::CanvasUpdate canvas(CANVAS_NAME, &hub);
    canvas.stroke(gridColor, gridAlpha);
    canvas.strokeWeight(gridWeight);
  
    for (int i = 0; i <= numYLines + 1; i++) {
      int X0 = XL_SHIFT;
      int X1 = X_RES - XR_SHIFT;
      int Y0 = YT_SHIFT + (i * (Y_RES - YT_SHIFT - YB_SHIFT) / (1 + numYLines));
      int Y1 = Y0;
      int hAxisValue = hMax - i * hMax / (numYLines + 2);
      int tAxisValue = tMax - i * tMax / (numYLines + 2);
      canvas.line(X0, Y0, X1, Y1);
    }
    for (int i = 0; i <= numXLines + 1; i++) {
      int X0 = XL_SHIFT + (i * (X_RES - XL_SHIFT -XR_SHIFT) / (1 + numXLines));
      int X1 = X0;
      int Y0 = YT_SHIFT;
      int Y1 = Y_RES - YB_SHIFT;
      canvas.line(X0, Y0, X1, Y1);
    }
  canvas.send();
}

void drawGridLabels() {
  String rH = F("RH %");
  String t = F("T *C");
      drawText(rH, hColor, labelTextAlpha, XL_SHIFT, 0);
      drawText(t, tColor, labelTextAlpha, X_RES - X_BORDER_LABEL_SHIFT - XR_SHIFT - t.length() * NATIVE_FONT_INTERVAL, 0);
    for (int i = 0; i <= numYLines + 1; i++) {
      int Y0 = YT_SHIFT + (i * (Y_RES - YT_SHIFT - YB_SHIFT) / (1 + numYLines));
      int hAxisValue = hMax - i * (hMax - hMin) / (numYLines + 1);
      int tAxisValue = tMax - i * (tMax - tMin) / (numYLines + 1);
      String h = String(hAxisValue) + "%";
      String t = String(tAxisValue) + "*";
      drawText(h, hColor, labelTextAlpha, XL_SHIFT - X_BORDER_LABEL_SHIFT - h.length() * NATIVE_FONT_INTERVAL, Y0 - Y_BORDER_LABEL_SHIFT);
      drawText(t, tColor, labelTextAlpha, X_RES - XR_SHIFT + X_BORDER_LABEL_SHIFT, Y0 - Y_BORDER_LABEL_SHIFT);
    }
}

void drawHumidity() {

  int step = (X_RES - XL_SHIFT - XR_SHIFT) / EACH_MEASURING_BUFFER_SIZE;
  gh::CanvasUpdate canvas(CANVAS_NAME, &hub);

    canvas.strokeWeight(hWeight);
    canvas.stroke(hColor, hAlpha);

    for (int i = 1; i < EACH_MEASURING_BUFFER_SIZE - 2; i++) {
      if ((humidityBuffer[i] != 0) && (humidityBuffer[i + 1] != 0)) {
        
        int X0 = XL_SHIFT + EACH_MEASURING_BUFFER_SIZE - i - step;
        int X1 = XL_SHIFT + EACH_MEASURING_BUFFER_SIZE - i;
        int Y0 = yPoint(humidityBuffer[i + 1], hMin, hMax);
        int Y1 = yPoint(humidityBuffer[i], hMin, hMax);

        canvas.line(X0, Y0, X1, Y1);
      }
    }
  canvas.send();
}

void drawTemperature() {

  int step = (X_RES - XL_SHIFT - XR_SHIFT) / EACH_MEASURING_BUFFER_SIZE;
  gh::CanvasUpdate canvas(CANVAS_NAME, &hub);

    canvas.strokeWeight(tWeight);
    canvas.stroke(tColor, tAlpha);

    for (int i = 1; i < EACH_MEASURING_BUFFER_SIZE - 2; i++) {
      if ((temperatureBuffer[i] != 0) && (temperatureBuffer[i + 1] != 0)) {
        
        int X0 = XL_SHIFT + EACH_MEASURING_BUFFER_SIZE - i - step;
        int X1 = XL_SHIFT + EACH_MEASURING_BUFFER_SIZE - i;
        int Y0 = yPoint(temperatureBuffer[i + 1], tMin, tMax);
        int Y1 = yPoint(temperatureBuffer[i], tMin, tMax);

        canvas.line(X0, Y0, X1, Y1);
      }
    }
  canvas.send();
}

void getClimaticData() {
  if (!sensorReady) {
    if (emulation) {
      temperatureBuffer[0] = random(10, 30);
      humidityBuffer[0] = random(60, 99);
    } else {
      temperatureBuffer[0] = SENSOR_DIDNT_START_ERROR;
      humidityBuffer[0] = SENSOR_DIDNT_START_ERROR;
    return;
    }
  }

  if (!emulation) {
    float aTemperature = 0.0;
    float aHumidity = 0.0;
    static uint8_t counter = 0;
    if (counter != EACH_X_TIME_SENSOR_HEATS) {
      counter++;
      error = sensor.measureHighPrecision(aTemperature, aHumidity);
      Serial.println(F("Regular measurment"));
    } else {
      counter = 0;
      error = sensor.activateHighestHeaterPowerLong(aTemperature, aHumidity);
      Serial.println(F("Long high heated measurment"));
    }
    
    if (error != SHT4X_NO_ERROR) {
      temperatureBuffer[0] = SENSOR_DIDNT_READ_DATA;
      humidityBuffer[0] = SENSOR_DIDNT_READ_DATA;
      Serial.print(F("Error trying to execute measureLowestPrecision(): "));
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
    temperatureBuffer[0] = int(aTemperature);
    humidityBuffer[0] = int(aHumidity);

    Serial.print("Temperature: ");
    Serial.println(aTemperature);
    Serial.print("Humidity: ");
    Serial.println(aHumidity);
  }
}

void updateClimateWidgets() {
  hub.sendUpdate("cTemperature", temperatureBuffer[0]);
  hub.sendUpdate("cHumidity", humidityBuffer[0]);
}

void getStateData() {

  preferences.begin("stateData");

  manualLightControlFlag = preferences.getBool("manualLightCF", MANUAL_LIGHT);
  autoLightControlFlag = preferences.getBool("autoLightCF", AUTO_LIGHT);
  lightOnTime = preferences.getLong("lightOnTime", LIGHT_ON);
  lightOffTime = preferences.getLong("lightOffTime", LIGHT_OFF);

  selectTimeZone = preferences.getInt("selectTimeZone", TIME_ZONE_CODE);

  mqttServer = preferences.getString("mqttServer");
  mqttPort = preferences.getString("mqttPort");
  mqttUser = preferences.getString("mqttUser");
  mqttPass = preferences.getString("mqttPass");

  preferences.end();
}

void initializeGPIO() {
  pinMode(LAMP_PIN, OUTPUT);
  digitalWrite(LAMP_PIN, LOW);
}

struct Point {
    int x;
    int y;
};

void drawOne(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {2, 2}, {2, 7}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5}, {3, 6}, {3, 7}, {4, 7}};
  for (int i = 0; i < 10; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawTwo(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 7}, {1, 2}, {2, 7}, {2, 6}, {2, 1}, {3, 7}, {3, 5}, {3, 1}, {4, 7}, {4, 4}, {4, 1}, {5, 7}, {5, 3}, {5, 2}};
  for (int i = 0; i < 14; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawThree(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 1}, {2, 7}, {2, 1}, {3, 7}, {3, 3}, {3, 1}, {4, 7}, {4, 4}, {4, 2}, {4, 1}, {5, 6}, {5, 5}, {5, 1}};
  for (int i = 0; i < 14; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawFour(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 5}, {1, 4}, {2, 5}, {2, 3}, {3, 5}, {3, 2}, {4, 7}, {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {5, 5}};
  for (int i = 0; i < 14; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawFive(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 3}, {1, 2}, {1, 1}, {2, 7}, {2, 3}, {2, 1}, {3, 7}, {3, 3}, {3, 1}, {4, 7}, {4, 3}, {4, 1}, {5, 6}, {5, 5}, {5, 4}, {5, 1}};
  for (int i = 0; i < 17; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawSix(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 5}, {1, 4}, {1, 3}, {2, 7}, {2, 4}, {2, 2}, {3, 7}, {3, 4}, {3, 1}, {4, 7}, {4, 4}, {4, 1}, {5, 6}, {5, 5}};
  for (int i = 0; i < 15; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawSeven(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 1}, {2, 7}, {2, 6}, {2, 5}, {2, 1}, {3, 4}, {3, 1}, {4, 3}, {4, 1}, {5, 2}, {5, 1}};
  for (int i = 0; i < 11; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawEight(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 5}, {1, 3}, {1, 2}, {2, 7}, {2, 4}, {2, 1}, {3, 7}, {3, 4}, {3, 1}, {4, 7}, {4, 4}, {4, 1}, {5, 6}, {5, 5}, {5, 3}, {5, 2}};
  for (int i = 0; i < 17; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawNine(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 3}, {1, 2}, {2, 7}, {2, 4}, {2, 1}, {3, 7}, {3, 4}, {3, 1}, {4, 6}, {4, 4}, {4, 1}, {5, 5}, {5, 4}, {5, 3}, {5, 2}};
  for (int i = 0; i < 15; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawZero(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {2, 7}, {2, 1}, {3, 7}, {3, 1}, {4, 7}, {4, 1}, {5, 6}, {5, 5}, {5, 4}, {5, 3}, {5, 2}};
  for (int i = 0; i < 16; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawPercent(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 2}, {1, 1}, {2, 5}, {2, 2}, {2, 1}, {3, 4}, {4, 7}, {4, 6}, {4, 3}, {5, 7}, {5, 6}, {5, 2}};
  for (int i = 0; i < 13; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawDegree(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {2, 3}, {2, 2}, {2, 1}, {3, 3}, {3, 1}, {4, 3}, {4, 2}, {4, 1}};
  for (int i = 0; i < 8; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawC(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {2, 7}, {2, 1}, {3, 7}, {3, 1}, {4, 7}, {4, 1}, {5, 6}, {5, 2}};
  for (int i = 0; i < 13; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawT(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 1}, {2, 1}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5}, {3, 6}, {3, 7}, {4, 1}, {5, 1}};
  for (int i = 0; i < 11; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawR(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {2, 4}, {2, 1}, {3, 4}, {3, 1}, {4, 5}, {4, 4}, {4, 1}, {5, 7}, {5, 6}, {5, 3}, {5, 2}};
  for (int i = 0; i < 18; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawH(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {5, 1}, {5, 2}, {5, 3}, {5, 4}, {5, 5}, {5, 6}, {5, 7}, {2, 4}, {3, 4}, {4, 4}};
  for (int i = 0; i < 17; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawEqual(gh::Canvas* cv0, int xShift, int yShift) {
  struct Point p[] = { {1, 5}, {1, 3}, {2, 5}, {2, 3}, {3, 5}, {3, 3}, {4, 5}, {4, 3}, {5, 5}, {5, 3}};
  for (int i = 0; i < 10; i++) cv0->point(xShift + p[i].x, yShift + p[i].y);
}

void drawSymbol(gh::Canvas* cv0, char symbol, int xShift, int yShift) {
  switch(symbol) {
    case '1': drawOne(cv0, xShift, yShift);
      break;
    case '2': drawTwo(cv0, xShift, yShift);
      break;
    case '3': drawThree(cv0, xShift, yShift);
      break;
    case '4': drawFour(cv0, xShift, yShift);
      break;
    case '5': drawFive(cv0, xShift, yShift);
      break;
    case '6': drawSix(cv0, xShift, yShift);
      break;
    case '7': drawSeven(cv0, xShift, yShift);
      break;
    case '8': drawEight(cv0, xShift, yShift);
      break;
    case '9': drawNine(cv0, xShift, yShift);
      break;
    case '0': drawZero(cv0, xShift, yShift);
      break;
    case '%': drawPercent(cv0, xShift, yShift);
      break;
    case '*': drawDegree(cv0, xShift, yShift);
      break;
    case 'C': drawC(cv0, xShift, yShift);
      break;
    case 'T': drawT(cv0, xShift, yShift);
      break;
    case 'R': drawR(cv0, xShift, yShift);
      break;
    case 'H': drawH(cv0, xShift, yShift);
      break;
    case '=': drawEqual(cv0, xShift, yShift);
      break;
    case ' ': break;
  }
}

void drawText (String text, uint32_t color, uint8_t alpha,  int xShift, int yShift) {
  int i = 0;
  char t[text.length() + 1];
  text.toCharArray(t, text.length()  + 1);

  gh::CanvasUpdate canvas(CANVAS_NAME, &hub);
    canvas.fill(color, alpha);
    while(text[i] != '\0') {
      drawSymbol(&canvas, t[i], xShift + i * NATIVE_FONT_INTERVAL, yShift);
      i++;
    }
  canvas.send();
}

void saveIntState(char* key, int value) {
  preferences.begin("stateData", false);
  preferences.putInt(key, value);
  preferences.end();
}

void saveLongState(char* key, long value) {
  preferences.begin("stateData", false);
  preferences.putLong(key, value);
  preferences.end();
}

void saveStrState(char* key, String value) {
  preferences.begin("stateData", false);
  preferences.putString(key, value);
  preferences.end();
}

void saveBoolState(char* key, boolean value) {
  preferences.begin("stateData", false);
  preferences.putBool(key, value);
  preferences.end();
}



void onUnix(uint32_t stamp) {
  if (globalTimeOffset != 0) return;
  globalTimeOffset = stamp - millis() / 1000;
  timeZoneChanged();
}

unsigned long getCurrentGlobalTime() {
  return globalTimeOffset + timeZone + millis() / 1000;
}

unsigned long getCurrentTime() {
  return getCurrentGlobalTime() % SEC_IN_DAY;
}

int getCurrentHour() {
  return getCurrentGlobalTime() / 3600 % 24; 
}

int getCurrentMinute() {
  return getCurrentGlobalTime() / 60 % 60;
}

int getCurrentSecond() {
  return getCurrentGlobalTime() % 60;
}


void timeZoneChanged() {
  saveIntState("selectTimeZone", selectTimeZone);
  switch (selectTimeZone) {
    case 0: timeZone = -720;
    break;
    case 1: timeZone = -660;
    break;
    case 2: timeZone = -600;
    break;
    case 3: timeZone = -570;
    break;
    case 4: timeZone = -540;
    break;
    case 5: timeZone = -480;
    break;
    case 6: timeZone = -420;
    break;
    case 7: timeZone = -360;
    break;
    case 8: timeZone = -300;
    break;
    case 9: timeZone = -240;
    break;
    case 10: timeZone = -210;
    break;
    case 11: timeZone = -180;
    break;
    case 12: timeZone = -120;
    break;
    case 13: timeZone = -60;
    break;
    case 14: timeZone = 0;
    break;
    case 15: timeZone = 60;
    break;
    case 16: timeZone = 120;
    break;
    case 17: timeZone = 180;
    break;
    case 18: timeZone = 210;
    break;
    case 19: timeZone = 240;
    break;
    case 20: timeZone = 270;
    break;
    case 21: timeZone = 300;
    break;
    case 22: timeZone = 330;
    break;
    case 23: timeZone = 345;
    break;
    case 24: timeZone = 360;
    break;
    case 25: timeZone = 390;
    break;
    case 26: timeZone = 420;
    break;
    case 27: timeZone = 480;
    break;
    case 28: timeZone = 525;
    break;
    case 29: timeZone = 540;
    break;
    case 30: timeZone = 570;
    break;
    case 31: timeZone = 600;
    break;
    case 32: timeZone = 630;
    break;
    case 33: timeZone = 660;
    break;
    case 34: timeZone = 720;
    break;
    case 35: timeZone = 765;
    break;
    case 36: timeZone = 780;
    break;
    case 37: timeZone = 840;
    break;
  }
  timeZone = timeZone * 60;
  cTimeUpdate();
}