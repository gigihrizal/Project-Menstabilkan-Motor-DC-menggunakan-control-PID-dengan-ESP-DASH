#include <ESPRotary.h>
//Library Wifi ESP-DASH
#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#endif
#include <ESPDash.h>
//Mendefinisikan Pin Driver motor
#define PWM1 D6
#define PWM2 D7
//Mendifinisikan Pin Encoder
#define ENCA  D1
#define ENCB D2
//#define ppr 7
#define ENCODEROUTPUT 505 //dalam satu putaran
//Variabel kecepatan
long previousMillis = 0;
long currentMillis = 0;

//Variabel Encoder
volatile long encoderValue = 0;
int interval = 1000;  //1000ms

int rpm = 0;
int Sp= 0;      //kecepatan yang diatur ke driver motor 100
//Definisi ESP Rotary
ESPRotary r;

//PID 50RPM//
//const float Kp = 0.49;
//const float Ki = 0;
//const float Kd = 0;

//PID 100RPM//
//const float Kp = 0.49;
//const float Ki = 0;
//const float Kd = 0;

double Kp = 0.49;
double Ki = 0;
double Kd = 0;

float error;
float lastError;

float integral;
float derivative;

//Nama hotspot
const char* ssid = "Arduino"; // SSID
const char* password = "killometer"; // Password

//Memulai WebServer
AsyncWebServer server(80);

//Membuat widget Slider
ESPDash dashboard(&server);
//Card card1(&dashboard, BUTTON_CARD, "START-STOP");
Card slider1(&dashboard, SLIDER_CARD, "Kp", "", 0, 30);
Card slider2(&dashboard, SLIDER_CARD, "Ki", "", 0, 30);
Card slider3(&dashboard, SLIDER_CARD, "Kd", "", 0, 30);
Card slider4(&dashboard, SLIDER_CARD, "Sp", "", 0, 255);

void setup() {
  
  Serial.begin(115200);
  //Proses menyambungkan ESP ke Hotspot
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi Failed!\n");
      return;
  }
  //mengetahui IP Address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  
  delay(50);
  Serial.println("Start Reading = ");
  
  r.begin(ENCA, ENCB);
  r.setChangedHandler(rotate);
  //r.setLeftRotationHandler(showDirection);
  //r.setRightRotationHandler(showDirection);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  //pinMode(tombol, INPUT_PULLUP);
}
void rotate(ESPRotary& r) {
  encoderValue = r.getPosition();
}

void RPM () {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);
    //Serial.print(encoderValue);
    //Serial.print(" pulse / ");
    //Serial.print(ENCODEROUTPUT);
    //Serial.print(" pulse per rotation x 60 seconds = ");
    //Serial.println(rpm);
    //Serial.print(" RPM ");
    encoderValue = 0;
    r.resetPosition();
  }
}

void loop() {
{
  RPM ();

  r.loop();
  slider1.attachCallback([&](int value) {
      Kp = value;
      slider1.update(value);
      dashboard.sendUpdates();
    });

    slider2.attachCallback([&](int value) {
      Ki = value;
      slider2.update(value);
      Serial.print(Ki);
      dashboard.sendUpdates();
    });

    slider3.attachCallback([&](int value) {
      Kd = value;
      slider3.update(value);
      dashboard.sendUpdates();
    });

    slider4.attachCallback([&](int value) {
      Sp = value;
      slider4.update(Sp);
      dashboard.sendUpdates();
    });
  
  float target = Sp;
  float actual = rpm;
  error = target - actual;

  integral += error;
  derivative = error - lastError;
  lastError = error;
  
  float hasil = Kp * error + Ki * integral + Kd * derivative;
  //integral = constrain(hasil, 0, 255);
  hasil = constrain(hasil, 0 , 255);
  analogWrite(PWM1, hasil);
  //Serial.print(target);
  //Serial.print(" / ");
  //Serial.print(actual);
  //Serial.print(" / ");
  //Serial.println(hasil);

  Serial.print(Kp);         //nilai Kp
  Serial.print(",");
  Serial.print(Sp);         //Target atau Set point
  Serial.print(",");
  Serial.print(actual);     //Pembacana rpm pada sensor
  Serial.print(",");
  Serial.println(hasil);    //Hasil perhitungan PID
  //Serial.println("//");
  //card1.attachCallback([&](int Output){    
  //Serial.println("Tombol On-Off: "+String((Output)?"true":"false"));  
  //analogWrite(PWM2, Output);
  //card1.update(Output);
  //dashboard.sendUpdates();
//});
  
}
}
//Baca Encoder//
// on change
//void rotate(ESPRotary& r) {
   //Serial.println(r.getPosition());
//}
// on left or right rotattion
//void showDirection(ESPRotary& r) {
  //Serial.println(r.directionToString(r.getDirection()));
//}

//Baca RPM//
