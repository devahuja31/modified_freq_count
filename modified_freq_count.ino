//all//frequency measurement system
const byte        interruptPin = 34;              // Assign the interrupt pin
volatile uint64_t StartValue;                     // First interrupt value
volatile uint64_t PeriodCount;                    // period in counts of 0.000001 of a second
float             Freg;                           // frequency     
char              str[21];                        // for printing uint64_t values
 
hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?

/////////////////////////////////json
//#include <ArduinoJson.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
//////////////////////////////////oled
// Auxiliar variables to store the current output state
String header;
long WiFi_Read1,WiFi_Read2,WiFi_Read3,WiFi_Read4,WiFi_Read5,WiFi_Read6,PWM_O,Speed_Point_O,Angle_Tol_O,Angle_Set_O,Pos_Feed_O,Dead_Band_O,Dec_Mul_O,Acc_Mul_O,Pos_To_Targ_O,Max_Dist_O,Max_Range_O;
long loopii;
//char Saved_PWD[16],Saved_MQTT_User[16],Saved_MQTT_PWD[16],Saved_MQTT_Topic[16],DummyStr[50];
String Saved_SSID="";
String Saved_MQTT_User="";
String Saved_PWD="";
String Saved_MQTT_PWD="";
String Saved_MQTT_Topic="";


String output26State = "off";
String output27State = "off";  
int stringSize;
int ik;
void SetValue();
// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 100
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebServer.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     12 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
void SetValue();
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
////////////////////////////////

#include <WiFi.h>
#include <PubSubClient.h>
const char* ssid1     = "mqtt_json";
const char* password1 = "123456789";
// Set web server port number to 80
WebServer server(80);
String WiFiSSID,MQTT_Topic,MQTT_PWD,MQTT_Name,WiFiPWD;
String Argument_Name, Clients_Response1, Clients_Response2;
// Please input the SSID and password of WiFi

//const char* ssid     ="PLogic2";// "I-Tech";//"umeed_nouman";
//const char* password = "13022278";//"#keeptrying";
const char* mqtt_server = "78.156.102.68";

const char* ssid = "AB03-Wireless";
const char* password =  "pro#pro#pro";

const char* ssid11 = "Birkelund";//here we need to put the ssid name and passowrd of the available wifi right now to make it work qucik without config it .
const char* password11 =  "NuVaRing67890";

//String ssid = "AB03-Wireless";
//String password =  "pro#pro#pro";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
float temperature = 0;
float humidity = 0;

// LED Pin
const int ledPin = 4;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


int led4=15;
int led3=2;
int led2=4; 
int led1=5;
int led5=19;
int led6=18;
/////////////////////////////////////////////////////

// Digital Event Interrupt
// Enters on falling edge in this example
//=======================================

long st=0;
long et=0;
void IRAM_ATTR handleInterrupt() 
{
  st=millis();
  if(st-et<15){return;}
  else{et=st;}
 
  portENTER_CRITICAL_ISR(&mux);
      uint64_t TempVal= timerRead(timer);         // value of timer at interrupt
      PeriodCount= TempVal - StartValue;          // period count between rising edges in 0.000001 of a second
      StartValue = TempVal;                       // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux);
}


// Converts unit64_t to char for printing
// Serial.println(uintToStr( num, str ));
//================================================
char * uintToStr( const uint64_t num, char *str )
{
  uint8_t i = 0;
  uint64_t n = num;
  do
    i++;
  while ( n /= 10 );
  
  str[i] = '\0';
  n = num;
 
  do
    str[--i] = ( n % 10 ) + '0';
  while ( n /= 10 );

  return str;
}
////////////////////////////////////////////////////////////////////////











void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
 EEPROM.begin(512);
  /*
  for(int k=0;k<15;k++)
      Saved_SSID[k]=char(EEPROM.read(k));
  */
  
  Saved_SSID=get_ssid();
  Saved_PWD=get_ssidpwrd();
Saved_MQTT_User=get_mqttun();
Saved_MQTT_PWD=get_mqttpw();
Saved_MQTT_Topic=get_mqtttp();
  //Serial.print("SSID SET ");Serial.println(Saved_SSID);
 /* for(int k=15;k<30;k++)
     Saved_PWD[k-15]=char(EEPROM.read(k));
  for(int k=30;k<45;k++)
      Saved_MQTT_User[k-30]=char(EEPROM.read(k));
  for(int k=45;k<60;k++)
      Saved_MQTT_PWD[k-45]=char(EEPROM.read(k));
  for(int k=60;k<75;k++)
      Saved_MQTT_Topic[k-60]=char(EEPROM.read(k));
   */
      Serial.println(Saved_SSID);
      Serial.println(Saved_PWD);
      Serial.println(Saved_MQTT_PWD);
      Serial.println(Saved_MQTT_Topic);
  inputString.reserve(200);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(led3,OUTPUT);
  pinMode(led4,OUTPUT);
  pinMode(led5,OUTPUT);
  pinMode(led6,OUTPUT);
  digitalWrite(led1,0);
  digitalWrite(led2,0);
  digitalWrite(led3,0);
  digitalWrite(led4,0);
  digitalWrite(led5,0);
  digitalWrite(led6,0);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT,3); // Animate bitmaps
//delay(5000);
 updatetext();
  
  setup_wifi();
//  client.setServer(mqtt_server, 1883);
//  client.setCallback(callback);
  pinMode(ledPin, OUTPUT);
  //
   pinMode(interruptPin, INPUT_PULLUP);                                            // sets pin high

/*
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING); // attaches pin to interrupt on Falling Edge
  timer = timerBegin(0, 80, true);  //more precision                                              // this returns a pointer to the hw_timer_t global variable
  //timer = timerBegin(0, 40, true);                                                                                // 0 = first timer
                                                                                  // 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second                                                                                // true - counts up
  timerStart(timer);                                                              // starts the timer
*/
//
}

void setup_wifi() {
  Serial.println("Going to check For AP");
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("PRESS KEY FOR CONFIG"));
  display.display();      // Show init

  delay(3000);
  int reset_button_count=0,APconfiguremode=0;
  pinMode(13, INPUT);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  int buttonState = digitalRead(13);
  Serial.print("Pin Read is ");Serial.println(buttonState);
 while(!buttonState)
{

  Serial.println("reset button");
  reset_button_count++;
  delay(500);
  if(reset_button_count>=3)
  {
    APconfiguremode=1;
    break;
  }  
}
APconfiguremode=0;//needs to comment out
if(APconfiguremode)
{
 
  Serial.println("ap configure done"); 
   // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("IN AP Mod"));
  display.display();      // Show init
  WiFi.softAP(ssid1, password1);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
    display.println(IP);
  display.display();
  server.begin();
  
    // Next define what the server should do when a client connects
  server.on("/", SetValue); // The client connected with no arguments e.g. http:192.160.0.40/
  while(1){
    
  server.handleClient();
}
}
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Con Wifi"));
 // display.setCursor(0, 0);
 //////////
 char ssid_1[50];
 Saved_SSID.toCharArray(ssid_1, 50);
 char pwrd_1[50];
 Saved_PWD.toCharArray(pwrd_1, 50);
  ssid=ssid_1;
  password=pwrd_1;
  display.println(ssid);
  display.display();  
  delay(1000);
  Serial.println(ssid);
   
  WiFi.begin(ssid11, password11);


  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Con Ok"));
  display.display();  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "fc/dk1") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("fc/dk1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
long lcc=0;
void loop() {
  
 /*
  starts:
  portENTER_CRITICAL(&mux);
  portEXIT_CRITICAL(&mux);
  if(Freg<40||Freg>1000){goto starts;}
 */
 error_loop_again:
  Freg =1000000.00/PeriodCount;                       // PeriodCount in 0.000001 of a second
 // Freg=Freg/2;
  Freg=get_freqq(20);
  if(Freg>51){goto error_loop_again;}
  //Serial.print("Frequency   ");Serial.println(Freg,3);
  unsigned int volt_read=get_volt();
  update_data(Freg,volt_read);
  json_data_out(Freg,volt_read);
  Serial2.print("$");Serial2.print(Freg,3);Serial2.write(",");Serial2.print(volt_read);Serial2.println("*");
  if(++lcc>20)
  {
    update_web(Freg,volt_read);
    lcc=0;
    char freqString[8];
    dtostrf(Freg, 1, 3, freqString);
    Serial.println("Data Uploading to Server");

  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("MQTT Req"));
  display.display(); 

    update_web(Freg,volt_read);/////////////////////////////////////////////dynalogic
    //Serial.print("Frequency To Server : ");
    //Serial.println(freqString);
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  if (!client.connected()) {
      //display.setCursor(0, 0);
      display.setTextSize(1); 
  display.println(F("Try to reconnect"));
  display.display();
    reconnect();
  }
  client.loop();
  client.publish("fc/dk1",freqString );
     display.setTextSize(1); 
  display.println(F("Data Posted!!"));
  display.display();
 delay(3000);
  }
  
  
  long t_t_t=millis();
  while(millis()-t_t_t<500)
  {
  serialEvent() ;  
  }
  if (stringComplete) {
    Serial.println(inputString);
    String ns=inputString;
    if(ns.startsWith("$led1 on"))
    {
      digitalWrite(led1,1);
    } else     if(ns.startsWith("$led1 off"))
    {
      digitalWrite(led1,0);
    }
       if(ns.startsWith("$led2 on"))
    {
      digitalWrite(led2,1);
    } else     if(ns.startsWith("$led2 off"))
    {
      digitalWrite(led2,0);
    }
       if(ns.startsWith("$led3 on"))
    {
      digitalWrite(led3,1);
    } else     if(ns.startsWith("$led3 off"))
    {
      digitalWrite(led3,0);
    }
       if(ns.startsWith("$led4 on"))
    {
      digitalWrite(led4,1);
    } else     if(ns.startsWith("$led4 off"))
    {
      digitalWrite(led4,0);
    }
       if(ns.startsWith("$led5 on"))
    {
      digitalWrite(led5,1);
    } else     if(ns.startsWith("$led5 off"))
    {
      digitalWrite(led5,0);
    }
       if(ns.startsWith("$led6 on"))
    {
      digitalWrite(led6,1);
    } else     if(ns.startsWith("$led6 off"))
    {
      digitalWrite(led6,0);
    }
    inputString = "";
    stringComplete = false;
  }
  //delay(500);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar=='$')
    {
      inputString="";
    }
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

String get_ssid()
{
  int ik=0;
   String nss="";
          for(ik=0;ik<15;ik++)
          {
            nss=nss+char(EEPROM.read(ik));
            Serial.write(char(EEPROM.read(ik)));
          }
          Serial.print("String read ");
        Serial.println(nss);
return(nss);
}

String get_ssidpwrd()
{
  int ik=0;
   String nss="";
          for(ik=15;ik<30;ik++)
          {
            nss=nss+char(EEPROM.read(ik));
            Serial.write(char(EEPROM.read(ik)));
          }
          Serial.print("String read ");
        Serial.println(nss);
return(nss);
}

String get_mqttun()
{
  int ik=0;
   String nss="";
          for(ik=30;ik<45;ik++)
          {
            nss=nss+char(EEPROM.read(ik));
            Serial.write(char(EEPROM.read(ik)));
          }
          Serial.print("String read ");
        Serial.println(nss);
return(nss);
}
String get_mqttpw()
{
  int ik=0;
   String nss="";
          for(ik=45;ik<60;ik++)
          {
            nss=nss+char(EEPROM.read(ik));
            Serial.write(char(EEPROM.read(ik)));
          }
          Serial.print("String read ");
        Serial.println(nss);
return(nss);
}
String get_mqtttp()
{
  int ik=0;
   String nss="";
          for(ik=60;ik<75;ik++)
          {
            nss=nss+char(EEPROM.read(ik));
            Serial.write(char(EEPROM.read(ik)));
          }
          Serial.print("String read ");
        Serial.println(nss);
return(nss);
}
//

float get_freqq(unsigned int number_sample)
{
  unsigned int ty=0;
  float freq_a=0;
  for(ty=0;ty<number_sample;ty++)
  {
  long highp=pulseIn(34, HIGH);
  long highl=pulseIn(34, LOW);
  long ttt=highp+highl;
  float ff=(float)ttt/1000000;
  ff=(float)1/ff;  
  freq_a=freq_a+ff;
  }
  return(freq_a/number_sample);
}
