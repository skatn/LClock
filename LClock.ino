#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWiFiManager.h>
#include <DNSServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

/*--74HC595D--*/
#define SER_PIN     12
#define RCK_PIN     13
#define SCK_PIN     14
#define PWM_PIN     4
uint64_t bitRegister = 0;


/*--LED--*/
#define DEFAULT_BRIGHTNESS  716   //70% brightness
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
uint8_t brightness = DEFAULT_BRIGHTNESS;
int autoBrightness = 0;
boolean colonState = LOW;


/*--WIFI--*/
const char* ssidAutoConnect = "L-Clock-Auto-connect";
const char* ssidAP = "L-Clock";
const char* passwordAP = "1234567890";
boolean wifiConnectOK = false;

WiFiUDP udp;
NTPClient timeClient(udp, "kr.pool.ntp.org", 32400, 10000);  //60m*60s*1000ms

AsyncWebServer server(80);
DNSServer dns;


/*--TIME--*/
int currTime = 0;
boolean hour12 = true;




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
      if (request->hasParam("first_request", true)) {
          //message = 
      }
      else if(request->hasParam("brightness", true)){
          message = request->getParam("brightness", true)->value();
      }
      else {
          message = "No message sent";
      }
      request->send(200, "text/plain", message);
  });
  
  server.onNotFound(notFound);
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
    currTime = timeClient.getHours()*100 + currMin;
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
  #define alpha 0.9
  static int value = 0;
  static unsigned long prevSampleTime = 0;
  
  unsigned long currSampleTime = millis();
  if(currSampleTime - prevSampleTime > 10){
    prevSampleTime = currSampleTime;
    
    value = alpha*value + (1.0-alpha)*analogRead(A0);
    Serial.println(value);
  }
}



void setup() {
  delay(1000);
  Serial.begin(115200);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCK_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  pinMode(PWM_PIN, OUTPUT);
  
  showSegment();
  ntpConnect();
  serverInit();
  
  analogWrite(PWM_PIN, 1023-brightness);
}

void loop() {
  getCDS();
  
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
}
