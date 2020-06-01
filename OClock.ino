#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

/*--74HC595D--*/
#define SER_PIN     12
#define RCK_PIN     13
#define SCK_PIN     14
#define PWM_TOP     4
#define PWM_BOTTOM  5


/*--LED--*/
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
int num = 0;
int backLEDCnt = 32;
uint32_t backLED = 0;


/*--WIFI--*/
char ssid[100] = "Glocode_2G";
char password[100] = "78750825gl";

WiFiUDP udp;
NTPClient timeClient(udp, "kr.pool.ntp.org", 32400, 3600000);  //kr.pool.ntp.org    3600000


/*--TIME--*/
int currTime = 0;
boolean hour12 = true;



void wifiInit(){
  byte buffering[] = {0x10, 0x80, 0x40, 0x08, 0x04, 0x02};
  int cnt = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    show(0, buffering[cnt], buffering[cnt], buffering[cnt], buffering[cnt]);
    cnt++;
    if(cnt > 5) cnt=0;
    delay(500);
  }
  timeClient.begin();
}

void updateTime(){
  static int prevMin = 0;
  timeClient.update();

  int currMin = timeClient.getMinutes();
  if(prevMin != currMin){
    prevMin = currMin;
    currTime = timeClient.getHours()*100 + currMin;
    if(hour12 && currTime > 1259) currTime -= 1200;
  }
}


void show(uint32_t bottomLED, uint8_t first, uint8_t second, uint8_t third, uint8_t forth){
  uint8_t bottom1 = bottomLED >> 24;
  uint8_t bottom2 = bottomLED >> 16;
  uint8_t bottom3 = bottomLED >> 8;
  uint8_t bottom4 = bottomLED;
  
  digitalWrite(RCK_PIN, LOW);
  shiftOut(SER_PIN, SCK_PIN, LSBFIRST, bottom4);
  shiftOut(SER_PIN, SCK_PIN, LSBFIRST, bottom3);
  shiftOut(SER_PIN, SCK_PIN, LSBFIRST, bottom2);
  shiftOut(SER_PIN, SCK_PIN, LSBFIRST, bottom1);
  
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, forth);
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, third);
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, second);
  shiftOut(SER_PIN, SCK_PIN, MSBFIRST, first);
  digitalWrite(RCK_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(SER_PIN, OUTPUT);
  pinMode(RCK_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  pinMode(PWM_TOP, OUTPUT);
  pinMode(PWM_BOTTOM, OUTPUT);

  digitalWrite(PWM_TOP, LOW);
  digitalWrite(PWM_BOTTOM, LOW);

  wifiInit();
}

void loop() {
  static unsigned long pTime = 0;
  updateTime();

  
  unsigned long cTime = millis();
  if(cTime-pTime > 100){
    pTime = cTime;
    Serial.println(currTime);

    
    backLED = 1 << backLEDCnt;
    backLEDCnt--;
    if(backLEDCnt < 0)backLEDCnt = 32;

    int n[4]={}, buff = currTime;
    for(int i=0; i<4; i++){
      n[i] = buff/pow(10.0, 3.0-i);
      buff = buff%int(pow(10.0, 3.0-i));
    }

    show(backLED, n[0]==0? 0 : numberToBit[n[0]], numberToBit[n[1]], numberToBit[n[2]], numberToBit[n[3]]);
  }
}
