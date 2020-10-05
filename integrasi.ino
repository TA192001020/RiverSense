#include <SoftwareSerial.h>
#include "esp32-hal-ledc.h"
#include <math.h>
#define sensor_in 34
#define COUNT_LOW 5200
#define COUNT_HIGH 8300
#define TIMER_WIDTH 16

//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(17, 16); //SIM800L Tx & Rx is connected to ESP32 DOIT #17 & #16
boolean val;
boolean STRT=LOW;
boolean Calc=HIGH;
boolean Hold=LOW; //Holds motor in either 0 or 90 degree
boolean Ret=HIGH; //Enables motor to return to 0 degree

String id = "#069";
String v_string;
long int mytime = 0;
float v;
int relay = 5;
int rpm;
int lap;
int lap_ctr=1;
int Counter=0;
int i;
int proxi_ctr=0;

void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);
  pinMode(sensor_in, INPUT);

  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width 
  ledcAttachPin(2, 1);   // GPIO 2 assigned to channel 1
  ledcWrite(1, 8300); //Motor on 0 degree Position
  
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  Serial.println("Initializing...");
  
  // pengiriman SMS tanda permulaan
  delay(5000);
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+6285725790869\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  mySerial.print("Riversense dengan id "+id+" mulai beroperasi."); //text content
  updateSerial();
  mySerial.write(26);
  updateSerial();
  mySerial.flush();
  for (int i = 0; i < 20; i++) {
    updateSerial();
    delay(1000);
  }
  mySerial.end();
}

void loop()
{
  //relay menyala
  digitalWrite(relay, LOW);
  Serial.println("Command completed relay turned ON");
  
  // pengukuran laju air
  while (Hold == LOW){
    for (int i=COUNT_HIGH ; i >= COUNT_LOW ; i=i-50)
     {
        ledcWrite(1, i);     
        delay(50);
        if(i <= COUNT_LOW){
          Hold = HIGH;
          Ret = LOW;
        }
     }
     
  }

  while (Hold == HIGH){
     if(Ret == LOW){
       val = (digitalRead (sensor_in));
       proximity();
     }

  
   
  }

  // konversi v dari float menjadi String
   v_string = String(v);

  // pengiriman via SMS data laju air
  delay(5000);
  mySerial.begin(9600);
  updateSerial();
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+6285725790869\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  mySerial.print(id+" "+v_string); //text content
  updateSerial();
  mySerial.write(26);
  mySerial.flush();
  for (int i = 0; i < 20; i++) {
    updateSerial();
    delay(1000);
  }
  mySerial.end();
  Ret = HIGH;

       for (int i=COUNT_LOW ; i <= COUNT_HIGH ; i=i+50)
       {
          ledcWrite(1, i);     
          delay(50);
          if(i >= COUNT_HIGH){
            Ret = LOW;
          }
       }
     
  // relay mati
  digitalWrite(relay, HIGH);
  Serial.println("Going to sleep now");
  Serial.println("Command completed relay turned OFF");
  Serial.flush();
  delay(60000);
}

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

void proximity() {
  
  mytime = millis();

  if(mytime==0){
    lap_ctr = 1;
  }

  if((val==HIGH && Calc==HIGH)){
    Counter++;
    Calc = LOW;
  }
  else if((val==LOW && Calc==LOW)){
    Calc = HIGH;
  }

  lap = mytime / 60000;
  
  if(lap>=lap_ctr){
      rpm = Counter / 2;
      v=(0.0022*rpm)+0.4776;
      Serial.print("RPM = ");
      Serial.println(rpm); 
      Serial.print("Speed = ");
      Serial.println(v);
      Serial.println("------------------------------");
      lap_ctr++;
      Counter = 0;
      proxi_ctr++;
      if(proxi_ctr == 2){
        Hold = LOW;
        proxi_ctr = 0;
      }
  }
  
}
