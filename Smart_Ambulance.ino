/*
   Arduino Based Smart Ambulance
   This Project Uses ECG, Temp Sensor, Gsm, Gps And Blood Pressure Sensor To Monitor The Data And Send It To Adafruit.io

*/

/* Most Common Variables To Change */
#define PHNUM1          ""        //Sms to Be Sent To Police
#define PHNUM2          ""
#define SMS
#define PYT


// we define The GSM Modem Used Here
#define TINY_GSM_MODEM_SIM800

/* Include All The necessary Libraries */

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>


/* Variables To Store The Internet Credentials */

const char apn[]  = "portalnmms";
const char user[] = "";
const char pass[] = "";

/* Variables To Store The Mqtt Credentials */

#ifdef ANISH
const char* broker = "";
const char* User = "";
const char* Pass = "";
const char* topicInit = "ambulance/state";
#define Port  18376
#endif


#ifdef PYT
const char* broker = "";
const char* User = "";
const char* Pass = "";
const char* topicInit = "ambulance/state";
#define Port 18786

#endif



/* Start Defining The Lcd and Gsm and Gps To Get The data And Some pins */

LiquidCrystal_I2C lcd(0x27, 20, 4);

//For GSM Board
#define SerialAT  Serial1

//For GPS Board
#define GpsSerial Serial2
#define GPSBaud 9600


// The TinyGPS++ object
TinyGPSPlus gps;



TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

/* Declaring The Pinout's For Sensors */
#define TempPin A0      //  Analog 0 For Temperature
#define ECGPin A1       //  Analog 1 For Ecg
#define BpPin A2      // Analog 2 For Blood Pressure
#define ButtonPin 2     // Distress Button on Pin 2
#define Ecgl1 5     // Checking if The Ecg Is Connected
#define Ecgl2 6     // Checking If The Ecg Is Connected


/* Variables To Store The Timings, Gps Co-ordinates, Sensor Data, Phone Number etc*/

/* Extras */
boolean Once = false;


/* Map Link For Sms */
const char *MapLink       = " http://maps.google.com/maps?q=";

/* Storing Button Booloeans and Data */
boolean process = false;
boolean lastButton = LOW;
boolean currentButton = LOW;

/* Sensors Data */
uint16_t TempData;
uint16_t EcgData;
uint16_t BpVariable;

char Replay[5];

/* Gps Co-ordinates */
float Gpslat = 0.0;     //  Store The Gps Latitude
float Gpslng = 0.0;     //  Store The Gps Longitude
float Gpsalt = 0.0;     //  Store The Gps Altitude

/* Mqtt Publishing Topics */
const char* MQTT_GPS = "feeds/gps/csv";
const char* MQTT_TEMP = "feeds/temperature";
const char* MQTT_ECG = "feeds/ecg";


/* Vairables To Store Timings */

#define Interval_Temp 2000      // Send Temperatre after 2 seconds
#define Interval_Ecg  1200      // Send ECG Data After 1.2 Second
#define Interval_Gps  10000     // Send Gps Data Every 10 Second

long lastReconnectAttempt = 0;
long lastTempSent = 0;
long lastEcgSent = 0;
long lastGpsSent = 0;

#define INIT_NETWORK  1
#define INIT_GPRS     2

/* ReDeclaring All the Functions Created */
void mqttCallback(char* topic, byte* payload, unsigned int len);
boolean mqttConnect();
void KeepMqttAlive(void);
void InitMqtt(void);
void InitGsm(uint8_t type);
void InitGps(void);
static void smartDelay(unsigned long ms, uint8_t type);
boolean GetGpsData(void);
void SendGps(void);
void SendSms(const char *PhNum, uint8_t g);
void SendEcg(void);
void SendTemp(void);
boolean debounce(boolean last);
void CheckButton(void);
void DisplayEmergency(uint8_t test);
void ftoa(double n, char *res, int afterpoint);
int intToStr(int x, char str[], int d);
void rever(char *str, int len);

void setup() {
  //Start With Lcd
  lcd.init();
  lcd.backlight();
  pinMode(ButtonPin, INPUT);   //Set the Button to inputPullup
  attachInterrupt(digitalPinToInterrupt(ButtonPin), CheckButton, CHANGE);
  pinMode(Ecgl1, INPUT);
  pinMode(Ecgl2, INPUT);
  delay(10);

  //Start The Initialization For Gps Modem
  InitGps();

  // Init the gsm Modem to check for network
  InitGsm(INIT_NETWORK);
  //Connect To The Mqtt
  InitMqtt();

}

void loop() {

  // Wait For Distress Button
  if (process == true) {


    if (!Once) {
      DisplayEmergency(1);
      // Store Gps Data
      if (GetGpsData()) {
        //send the Sms with Coordinates
#ifdef SMS
        SendSms(PHNUM1, 1);
        delay(5000);
        SendSms(PHNUM2, 0);
        delay(4000);
#endif
        InitGsm(INIT_GPRS);
        Once = true;
      }
    }
    KeepMqttAlive();

    //Check If Mqtt Is Active
    if (mqtt.connected()) {
      unsigned long currentTime = millis();

      if (currentTime - lastEcgSent > Interval_Ecg) {
        lastEcgSent = currentTime;
        //Check Ecg
        SendEcg();
      }

      if (currentTime - lastTempSent > Interval_Temp) {
        lastTempSent = currentTime;
        //Check Temp
        SendTemp();
      }

      if (currentTime - lastGpsSent > Interval_Gps) {
        lastGpsSent = currentTime;
        //Check Gps
        SendGps();
      }

    }
  }
  else {
    DisplayEmergency(0);
  }


}


/* Below Are The Custom Functions We make */

// For Keeping Mqtt Alive

boolean mqttConnect() {
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to ");
  lcd.setCursor(0, 1);
  lcd.print(broker);
  if (!mqtt.connect("GsmClientTest", User, Pass)) {
    lcd.setCursor(0, 2);
    lcd.print("fail");
    return false;
  }
  lcd.setCursor(0, 2);
  lcd.print("Ok");
  delay(50);
  if (mqtt.publish(topicInit, "Smart Ambulance Active")) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mqtt Connected");
    lcd.setCursor(0, 1);
    lcd.print("Message Delivered");
    delay(2000);
    lcd.clear();
  }
  return mqtt.connected();
}

//To Keep Mqtt Alive

void KeepMqttAlive() {

  if (mqtt.connected()) {
    mqtt.loop();
  } else {
    // Reconnect every 10 seconds
    if (millis() - lastReconnectAttempt > 100) {
      lastReconnectAttempt = millis();
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }

  return true;

}

// For Initializing Mqtt In Setup

void InitMqtt() {
  // MQTT Broker setup
  mqtt.setServer(broker, Port);
  mqtt.setCallback(mqttCallback);
}

// For Initializing GSM in Setup

void InitGsm(uint8_t type) {
  SerialAT.begin(9600);
  delay(2000);

  if (type == INIT_NETWORK) {
    bool Inita = false;

    while (!Inita) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Initializing");
      lcd.setCursor(0, 1);
      lcd.print("GSM Modem!");

      modem.restart();

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Waiting For Network");

      if (!modem.waitForNetwork()) {
        lcd.setCursor(0, 1);
        lcd.print("Failed To Connect");
      }
      else {
        Inita = true;

        lcd.setCursor(0, 1);
        lcd.print("Network Connected!");
        delay(1000);
      }

    }
  }
  else if (type == INIT_GPRS) {
    bool Inita = false;

    while (!Inita) {

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connecting To ");
      lcd.setCursor(0, 1);
      lcd.print(apn);

      if (!modem.gprsConnect(apn, user, pass)) {
        lcd.setCursor(0, 1);
        lcd.print("Internet Connection ");
        lcd.setCursor(0, 2);
        lcd.print("Failed, Retrying!");
      }
      else {
        Inita = true;

        lcd.setCursor(0, 1);
        lcd.print("Internet Connection ");
        lcd.setCursor(0, 2);
        lcd.print("Established!");
        delay(1000);
      }

    }
  }
}


// For Initializing The GPS Module

void InitGps() {

  bool Inita = false;

  while (!Inita) {
    GpsSerial.begin(GPSBaud);
    delay(200);


    lcd.setCursor(0, 0);
    lcd.print("Initializing");
    lcd.setCursor(0, 1);
    lcd.print("Gps Modem");
    delay(2000);

    do {
      while (GpsSerial.available() > 0) {
        if (gps.encode(GpsSerial.read())) {
          if (gps.location.isValid()) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Gps Locked!");
            delay(1000);
            lcd.clear();
            delay(100);
            Inita = true;
          }
          else {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Retrying To Lock Sat");
            delay(3000);
            lcd.clear();
            delay(100);
          }
        }
      }
    }
    while (!Inita);

  }
}

// Smart Delay To Feed Gps Data
static void smartDelay(unsigned long ms, uint8_t type)
{
  if (type == 1) {
    unsigned long start = millis();
    do
    {
      while (GpsSerial.available())
        gps.encode(GpsSerial.read());
    } while (millis() - start < ms);
  }

  else {

  }
}


// To Get GPS Data And Store In Variable

boolean GetGpsData() {

  bool gotData = false;
  while (!gotData) {
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Getting Location");
    smartDelay(1000, 1);

    if (gps.location.isValid()) {
      Gpslat = (gps.location.lat());
      Gpslng = (gps.location.lng());
      Gpsalt = (gps.altitude.feet());

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lat: ");
      lcd.print(Gpslat, 6);
      lcd.setCursor(0, 1);
      lcd.print("Lng: ");
      lcd.print(Gpslng, 6);
      delay(200);
      gotData = true;
    }

    else {
      gotData = false;
    }
  }
  return true;
}

//Sending GPS Data

void SendGps() {

  smartDelay(1000, 1);

  if (gps.location.isValid()) {
    Gpslat = (gps.location.lat());
    Gpslng = (gps.location.lng());
    Gpsalt = (gps.altitude.feet());

    lcd.setCursor(0, 3);
    lcd.print("Lat:");
    lcd.print(Gpslat);
    lcd.setCursor(9, 3);
    lcd.print(" ");
    lcd.setCursor(10, 3);
    lcd.print("Lng:");
    lcd.print(Gpslng);
    delay(300);

    char gpsbuffer[30];
    char *p = gpsbuffer;

    dtostrf(10, 3, 4, p);
    p += strlen(p);
    p[0] = ','; p++;

    dtostrf(Gpslat, 3, 6, p);
    p += strlen(p);
    p[0] = ','; p++;

    dtostrf(Gpslng, 3, 6, p);
    p += strlen(p);
    p[0] = ','; p++;

    dtostrf(Gpsalt, 2, 1, p);
    p += strlen(p);

    p[0] = 0;

    if ((Gpslat != 0) && (Gpslng != 0)) {
      mqtt.publish(MQTT_GPS, gpsbuffer);
    }
  }
}

// For Sending The SMS
void SendSms(const char *PhNum, uint8_t g) {
  if (g == 1) {
    char Lat[10], Lon[10], gsmmsg[100];
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending Sms");

    SerialAT.println("AT\r");
    delay(250);
    SerialAT.println("AT+CMGF=1\r");
    delay(250);
    SerialAT.println("AT+CNMI=2,2,2,0,0\r");
    delay(250);



    gsmmsg[0] = '\0';
    strcat(gsmmsg, "Smart Ambulance Going To Hospital From Location ");
    strcat(gsmmsg, MapLink);
    ftoa(Gpslat, Lat, 4);
    ftoa(Gpslng, Lon, 4);
    strcat(gsmmsg, Lat);
    strcat(gsmmsg, ",");
    strcat(gsmmsg, Lon);

    SerialAT.print("AT+CMGS=\"0");
    SerialAT.print(PhNum);
    SerialAT.println("\"\r");
    delay(1000);
    SerialAT.println(gsmmsg);
    delay(500);
    SerialAT.println((char)26);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Message Sent!");
  }
  else {

    SerialAT.println("AT\r");
    delay(250);
    SerialAT.println("AT+CMGF=1\r");
    delay(250);
    SerialAT.println("AT+CNMI=2,2,2,0,0\r");
    delay(250);

    SerialAT.print("AT+CMGS=\"0");
    SerialAT.print(PhNum);
    SerialAT.println("\"\r");
    delay(1000);
    SerialAT.println("*Driver*");
    delay(500);
    SerialAT.println((char)26);
  }
}

void SendEcg() {
  lcd.setCursor(0, 0);
  lcd.print("Ecg: ");
  if ((digitalRead(Ecgl1) == 1) || (digitalRead(Ecgl2) == 1)) {

    lcd.setCursor(4, 0);
    lcd.print("Not Connected!");

  }
  else {
    char ECGD[10];
    EcgData = analogRead(ECGPin);
    lcd.setCursor(4, 0);
    lcd.print(EcgData);
    lcd.setCursor(7, 0);
    lcd.print("             ");

    intToStr(EcgData, ECGD, 3);
    //Send Mqtt message Here
    mqtt.publish(MQTT_ECG, ECGD, 3);
  }

}

void SendTemp() {
  char  Temperature[10];
  lcd.setCursor(0, 1);
  lcd.print("Temp:");

  TempData = analogRead(TempPin);
  float millivolts = (TempData / 1024.0) * 5000;
  float degrees = millivolts / 10;

  lcd.setCursor(5, 1);
  lcd.print(degrees);
  lcd.setCursor(10, 1);
  lcd.print("C   ");
  ftoa(degrees, Temperature, 2);
  //send Data
  mqtt.publish(MQTT_TEMP, Temperature);

}

boolean debounce(boolean last) {
  boolean current = digitalRead(ButtonPin);
  if (last != current) {
    delay(5);
    current = digitalRead(ButtonPin);
  }
  return current;
}

void CheckButton() {

  currentButton = debounce(lastButton);
  if (lastButton == LOW && currentButton == HIGH) {
    process = !process;
  }

  lastButton = currentButton;


}

void DisplayEmergency(uint8_t test) {
  if (test == 1) {
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY");
    delay(3000);
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SYSTEM DE-ACTIVATED");
    delay(3000);
    lcd.clear();
    lcd.noBacklight();

  }
}


// Converts a floating point number to string.
void ftoa(double n, char *res, int afterpoint) {
  // Extract integer part
  int ipart = (int)n;
  // Extract floating part
  double fpart = n - (float)ipart;
  // convert integer part to string
  int i = intToStr(ipart, res, 0);
  // check for display option after point
  if (afterpoint != 0)  {
    res[i] = '.';  // add dot
    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter is needed
    // to handle cases like 233.007
    fpart = fpart * pow(10, afterpoint);
    intToStr((int)fpart, res + i + 1, afterpoint);
  }
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d) {
  int i = 0;
  while (x) {
    str[i++] = (x % 10) + '0';
    x = x / 10;
  }
  // If number of digits required is more, then
  // add 0s at the beginning
  while (i < d)
    str[i++] = '0';

  rever(str, i);
  str[i] = '\0';
  return i;
}

void rever(char *str, int len) {
  int i = 0, j = len - 1, temp;
  while (i < j) {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++; j--;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {}
