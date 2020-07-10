#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWiFiManager.h>
#include <DNSServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>


/*--CDS--*/
#define CDS_PIN     A0
#define LPF_ALPHA 0.8
int cdsValue = 0;

/*--74HC595D--*/
#define SER_PIN     12
#define RCK_PIN     13
#define SCK_PIN     14
#define PWM_PIN     4
uint64_t bitRegister = 0;


/*--LED--*/
#define DEFAULT_BRIGHTNESS  70
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
#define BRIGHTNESS_MAX  100
#define BRIGHTNESS_MIN  10


/*--EEPROM--*/
#define ADDR_HOUR12       0x10
#define ADDR_BRIGHT_MODE  0x11
#define ADDR_BRIGHTNESS   0x12
#define ADDR_COLON        0x13


void eepromInit(){
  EEPROM.begin(4096);
  hour12 = EEPROM.read(ADDR_HOUR12);
  brightMode = EEPROM.read(ADDR_BRIGHT_MODE);
  if((brightMode != PASSIVITY) && (brightMode != BY_TIME) && (brightMode != BY_CDS)){
    brightMode = PASSIVITY;
  }

  brightness = EEPROM.read(ADDR_BRIGHTNESS);
  if(brightness < BRIGHTNESS_MIN) brightness = BRIGHTNESS_MIN;
  else if(brightness > BRIGHTNESS_MAX) brightness = BRIGHTNESS_MAX;

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
  /*server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String message;
    if (request->hasParam(PARAM_MESSAGE)) {
      message = request->getParam(PARAM_MESSAGE)->value();
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", "Hello, GET: " + message);
  });*/

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
      hour12 = request->getParam("set_hour_mode", true)->value().toInt();
      message = "success";
    }
    else if(request->hasParam("set_colon_mode", true)){
      showAMPM = request->getParam("set_colon_mode", true)->value().toInt();
      message = "success";
    }
    else if(request->hasParam("set_bright_mode", true)){
      //String data = request->getParam("set_bright_mode", true)->value();
      brightMode = request->getParam("set_bright_mode", true)->value().toInt();
      //brightMode = data.toInt();
      message = "success";
    }
    else if(request->hasParam("set_brightness", true)){
      String data = request->getParam("set_brightnesstness", true)->value();
      brightness = data.toInt();
      message = "success";
    }
    else if(request->hasParam("set_time", true)){
      String data = request->getParam("set_time", true)->value();
      timeOffset = data.toInt();
      message = "success";
    }
    else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });
  
  //server.onNotFound(notFound);
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Not found");
  });
  server.begin();
}


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}



boolean updateTime(){
  static int prevMin = 0;
  timeClient.update();

  int currMin = timeClient.getMinutes();
  if(prevMin != currMin){
    prevMin = currMin;
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
    bitRegister |= (1 << 15) | (1 << 23);
  }
  else{
    bitRegister &= ~((1 << 15) | (1 << 23));
  }
}
void setSegment(uint8_t first, uint8_t second, uint8_t third, uint8_t forth){
  bitRegister &= ((1 << 15) | (1 << 23));
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
  static unsigned long prevSampleTime = 0;
  
  unsigned long currSampleTime = millis();
  if(currSampleTime - prevSampleTime > 100){
    prevSampleTime = currSampleTime;
    
    cdsValue = LPF_ALPHA*cdsValue + (1.0-LPF_ALPHA)*analogRead(CDS_PIN);
  }
}

void setBrightness(byte value){
  analogWrite(PWM_PIN, 1024-(1024*value/100));
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCK_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(CDS_PIN, INPUT);
  
  eepromInit();
  setBrightness(brightness);
  
  showSegment();
  ntpConnect();
  serverInit();
}

void loop() {
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

    setSegment(n[0]==0? 0 : numberToBit[n[0]], numberToBit[n[1]], numberToBit[n[2]], numberToBit[n[3]]);
    Serial.println(currTime);

    if(WiFi.status() != WL_CONNECTED){
      wifiConnectOK = false;
    }
  }

  if(tick_tock){
    if(wifiConnectOK == true){
      setColon(HIGH);
    }
    else{
      colonState = !colonState;
      setColon(colonState);
    }
    showSegment();
  }

  switch(brightMode){
    case PASSIVITY:
      setBrightness(brightness);
      break;
    case BY_TIME:
      break;
    case BY_CDS:
      getCDS();
      setBrightness(map(cdsValue, 0, 1023, 0, 100));
      break;
  }
}
