#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWiFiManager.h>
#include <DNSServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>


/*--CDS--*/
#define RANGE_UPDATE_TIME   60000  
#define CDS_MARGIN          10
#define CDS_BUFF_MARGIN    30
#define CDS_PIN             A0
#define LPF_ALPHA           0.8
int cdsValue = 0;
uint16_t cdsMin = 500, cdsMax = 900;
uint16_t cdsMinBuff = 1024, cdsMaxBuff = 0;

/*--74HC595D--*/
#define SER_PIN     12
#define RCK_PIN     13
#define SCK_PIN     14
#define PWM_PIN     4
uint64_t bitRegister = 0;


/*--LED--*/
#define DEFAULT_BRIGHTNESS  100
byte numberToBit[] = {
  0xDE,   //0, 1101 1110
  0xC0,   //1, 1100 0000
  0xBC,   //2, 1011 1100
  0xF8,   //3, 1111 1000
  0xE2,   //4, 1110 0010
  0x7A,   //5, 0111 1010
  0x7E,   //6, 0111 1110
  0xD0,   //7, 1101 0000
  0xFE,   //8, 1111 1110
  0xFA,   //9, 1111 1010
};
int brightness = DEFAULT_BRIGHTNESS;
int autoBrightness = 0;
boolean colonState = LOW;

/*--TIME--*/
int rawTime = 0;
int currTime = 0;
unsigned int timeOffset = 32400;


/*--WIFI--*/
const char* ssidAutoConnect = "L-Clock-Auto-connect";
const char* ssidAP = "L-Clock";
const char* passwordAP = "1234567890";
boolean wifiConnectOK = false;

WiFiUDP udp;
NTPClient timeClient(udp, "kr.pool.ntp.org", timeOffset, 3600000);  //60m*60s*1000ms

AsyncWebServer server(80);
DNSServer dns;


/*--setting--*/
enum {PASSIVITY=0, BY_TIME, BY_CDS};
boolean hour12 = true;
boolean showAMPM = true;
int brightMode = PASSIVITY;
boolean isChanged = false;
#define BRIGHTNESS_MAX  100
#define BRIGHTNESS_MIN  10


/*--EEPROM--*/
#define ADDR_HOUR12       0x10
#define ADDR_BRIGHT_MODE  0x11
#define ADDR_BRIGHTNESS   0x12
#define ADDR_COLON        0x13


unsigned long diff(unsigned long now, unsigned long prev, unsigned long d)
{
  if (now>prev) return now-prev;
  else return 0xFFFFFFFF-prev+now+1;
}


void eepromInit(){
  EEPROM.begin(4096);
  hour12 = EEPROM.read(ADDR_HOUR12);
  brightMode = EEPROM.read(ADDR_BRIGHT_MODE);
  if((brightMode != PASSIVITY) && (brightMode != BY_TIME) && (brightMode != BY_CDS)){
    brightMode = PASSIVITY;
  }

  //brightness = EEPROM.read(ADDR_BRIGHTNESS);
  //if(brightness < BRIGHTNESS_MIN) brightness = BRIGHTNESS_MIN;
  //else if(brightness > BRIGHTNESS_MAX) brightness = BRIGHTNESS_MAX;

  showAMPM = EEPROM.read(ADDR_COLON);
}


void ntpConnect(){
  setSegment(0xF7, 0x4C, 0x2E, 0x6C);
  showSegment();
  
  AsyncWiFiManager wifiManager(&server,&dns);
  //wifiManager.resetSettings();
  wifiManager.autoConnect(ssidAutoConnect);
  timeClient.begin();
  
  wifiConnectOK = true;
}


void serverInit(){
  const char* PARAM_MESSAGE = "message";
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssidAP, passwordAP);
  Serial.print(F("\n\nIP : "));
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Hello, world");
  });

  // Send a GET request to <IP>/get?message=<message>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String message;
    if (request->hasParam("sync_request")) {
      message = message = String(rawTime)+","+String(timeOffset)+","+String(hour12)+","+String(showAMPM)+","+String(brightMode)+","+String(brightness);
    }
    else if(request->hasParam("time_request")) {
      message = String(rawTime);
    }
    else if(request->hasParam("set_hour_mode")){
      int buff = request->getParam("set_hour_mode")->value().toInt();
      isHour24 = buff;
      Serial.print("set hour mode : "); Serial.println(buff);
      isChanged = true;
      message = "success";
    }
    else if(request->hasParam("set_colon_mode")){
      int buff = request->getParam("set_hour_mode")->value().toInt();
      showAMPM = buff;
      Serial.print("set colon mode : "); Serial.println(buff);
      isChanged = true;
      message = "success";
    }
    else if(request->hasParam("set_bright_mode")){
      brightMode = request->getParam("set_bright_mode")->value().toInt();
      //brightMode = data.toInt();
      message = "success";
    }
    else if(request->hasParam("set_brightness")){
      String data = request->getParam("set_brightness")->value();
      brightness = data.toInt();
      Serial.println("set time : " + data);
      isChanged = true;
      message = "success";
    }
    else if(request->hasParam("set_time")){
      String data = request->getParam("set_time")->value();
      timeOffset = data.toInt();
      Serial.println("set time : " + data);
      timeClient.setTimeOffset(timeOffset);
      isChanged = true;
      message = "offset:"+String(timeOffset);
    }
    else {
      message = "No message sent";
    }
    
    request->send(200, "text/plain", "Hello, GET: " + message);
  });
  
  /*
  // Send a POST request to <IP>/post with a form field message set to <message>
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam("sync_request", true)) {
      message = String(rawTime)+","+String(timeOffset)+","+String(hour12)+","+String(showAMPM)+","+String(brightMode)+","+String(brightness);
    }
    else if(request->hasParam("time_request", true)) {
      message = String(rawTime);
    }
    else if(request->hasParam("set_hour_mode", true)){
      int buff = request->getParam("set_hour_mode", true)->value().toInt();
      hour12 = buff;
      Serial.print("set hour mode : "); Serial.println(buff);
      isChanged = true;
      message = "success";
    }
    else if(request->hasParam("set_colon_mode", true)){
      int buff = request->getParam("set_hour_mode", true)->value().toInt();
      showAMPM = buff;
      Serial.print("set colon mode : "); Serial.println(buff);
      isChanged = true;
      message = "success";
    }
    else if(request->hasParam("set_bright_mode", true)){
      //String data = request->getParam("set_bright_mode", true)->value();
      brightMode = request->getParam("set_bright_mode", true)->value().toInt();
      //brightMode = data.toInt();
      message = "success";
    }
    else if(request->hasParam("set_brightness", true)){
      String data = request->getParam("set_brightness", true)->value();
      brightness = data.toInt();
      Serial.println("set time : " + data);
      isChanged = true;
      message = "success";
    }
    else if(request->hasParam("set_time", true)){
      String data = request->getParam("set_time", true)->value();
      timeOffset = data.toInt();
      Serial.println("set time : " + data);
      timeClient.setTimeOffset(timeOffset);
      isChanged = true;
      message = "offset:"+String(timeOffset);
    }
    else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });*/
  
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Not found");
  });
  server.begin();
}

boolean updateTime(){
  static int prevMin = 0;
  timeClient.update();

  int currMin = timeClient.getMinutes();
  if((prevMin != currMin) || isChanged){
    prevMin = currMin;
    isChanged = false;
    rawTime = currTime = timeClient.getHours()*100 + currMin;
    if(hour12 && currTime > 1259) currTime -= 1200;

    return true;
  }
  return false;
}

boolean tick_tock(){
  static int prevSec = 0;
  int currSec = timeClient.getSeconds();

  if(prevSec != currSec){
    prevSec = currSec;
    return true;
  }
  return false;
}

void setColon(boolean state){
  if(state){
    if(showAMPM){
      if(rawTime%100 >= 12) bitRegister |= (1 << 8);
      else bitRegister |= (1 << 16);
    }
    else bitRegister |= (1 << 8) | (1 << 16);
  }
  else{
    bitRegister &= ~((1 << 8) | (1 << 16));
  }
}

void setSegment(uint8_t first, uint8_t second, uint8_t third, uint8_t forth){
  bitRegister &= (1 << 8) | (1 << 16);
  bitRegister |= first << 24;
  bitRegister |= second << 16;
  bitRegister |= third << 8;
  bitRegister |= forth;
}

void showSegment(){
  digitalWrite(RCK_PIN, LOW);
  uint8_t register1 = (uint8_t)(bitRegister >> 24);
  uint8_t register2 = (uint8_t)(bitRegister >> 16);
  uint8_t register3 = (uint8_t)(bitRegister >> 8);
  uint8_t register4 = (uint8_t)bitRegister;
  
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, register4);
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, register3);
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, register2);
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, register1);
  
  digitalWrite(RCK_PIN, HIGH);
}

void getCDS(){
  static unsigned long prevSampleTime = 0, minTime = 0, maxTime = 0, minBuffTime = 0, maxBuffTime = 0;
  
  unsigned long currSampleTime = millis();
  if(diff(currSampleTime, prevSampleTime) > 100){
    prevSampleTime = currSampleTime;
    
    cdsValue = (1.0-LPF_ALPHA)*cdsValue + LPF_ALPHA*analogRead(CDS_PIN);

    if((cdsValue < (cdsMin-CDS_MARGIN)) && minTime==0) minTime = currSampleTime;
    else minTime = 0;
    if((cdsValue > (cdsMax+CDS_MARGIN)) && maxTime==0) maxTime = currSampleTime;
    else maxTime = 0;

    if(currSampleTime - minTime > RANGE_UPDATE_TIME){
      cdsMin = cdsValue;
    }
    if(currSampleTime - maxTime > RANGE_UPDATE_TIME){
      cdsMax = cdsValue;
    }

    
    if((cdsValue < (cdsMinBuff-CDS_BUFF_MARGIN)) && minBuffTime==0) minBuffTime = currSampleTime;
    else minBuffTime = 0;
    if((cdsValue > (cdsMaxBuff+CDS_BUFF_MARGIN)) && maxBuffTime==0) maxBuffTime = currSampleTime;
    else maxBuffTime = 0;

    if(diff(currSampleTime, minBuffTime) > 86400000UL){   //86400000 = 1Day (1000ms * 60s * 60m * 24h)
      cdsMin = cdsValue;
    }
    if(diff(currSampleTime, maxBuffTime) > 86400000UL){   //86400000 = 1Day (1000ms * 60s * 60m * 24h)
      cdsMax = cdsValue;
    }
  }
}

void cdsBrightness(){
  int buff = cdsValue;
  if(buff < cdsMin) buff = cdsMin;
  else if(buff > cdsMax) buff = cdsMax;
  
  setBrightness(map(cdsValue, cdsMin, cdsMax, 10, 100));
}

void setBrightness(int value){
  analogWrite(PWM_PIN, 1024.0-(1024.0*value/100.0));
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCK_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(CDS_PIN, INPUT);

  analogWriteFreq(21000);
  
//  eepromInit();
  setBrightness(brightness);
  
  showSegment();
  ntpConnect();
  serverInit();
}

void loop() {
  //time updated
  if(updateTime()){
    if(hour12){
      if(currTime > 1259) currTime -= 1200;
      else if(currTime<60) currTime += 1200;
    }
    int n[4]={}, buff = currTime;
    for(int i=0; i<4; i++){
      n[i] = buff/pow(10.0, 3.0-i);
      buff = buff%int(pow(10.0, 3.0-i));
    }

//    bitRegister = 0;
    setSegment(n[0]==0? 0 : numberToBit[n[0]], numberToBit[n[1]], numberToBit[n[2]], numberToBit[n[3]]);

    if(WiFi.status() != WL_CONNECTED){
      wifiConnectOK = false;
    }
  }

  //every second
  if(tick_tock()){
    if(wifiConnectOK == true){
      setColon(HIGH);
    }
    else{
      colonState = !colonState;
      setColon(colonState);
    }
    showSegment();
  }


  //always
  getCDS();
  switch(brightMode){
    case PASSIVITY:
      setBrightness(brightness);
      break;
    case BY_TIME:
      break;
    case BY_CDS:
      cdsBrightness();
      break;
  }
}
