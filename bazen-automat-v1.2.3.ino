// Needed LIBs ...
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <AnalogSmooth.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DS3231.h>
#include "DHT.h"
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

// Declare and set LCD object

// set the LCD address to 0x27 for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Init the DS3231 RTC using the hardware interface
DS3231  rtc(SDA, SCL);
  // Init a Time-data structure
  Time  t;

// Declare KNOBS (rotary encoders) objects
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   Avoid using pins with LEDs attached
//   PinMode for (DT, CLK) are set to INPUT automaticly

Encoder knob1(2, 24);  // Running time & Volume Encoder (PINs DT[INT 4], CLK)
Encoder knob2(3, 27);  // PH & ORP Control&Calibration Encoder (PINs DT[INT 5], CLK)
Encoder knob3(19, 30); // SUN Control Encoder (PINs DT[2], CLK)

static byte pin_KB1 = 22;        //KNOB1 - Pump runnunig time/day (short) and (long) pool watter reservoar capacity (m3).
static byte pin_KB2 = 25;        //KNOB2 - PH dosing set-point (short) and (long) PH PROBE calibration process control.
static byte pin_KB3 = 28;        //KNOB3 - Temperature of watter in solar panel set-point to control pump (short) and (long) ORP calibration process control.

//  Declare analog ports with probes attached on:
static int pin_PHprobe = A0;    //PH PROBE input port.
static int pin_ORPprobe = A1;   //ORP PROBE input port.
//  Declare PRESSURE Probe attached on analog pin
static int pin_BARprobe = A2;   
//  Declare LDR Probe attached on analog pin
static int pin_LDRprobe = A3;   

    // Analog smooth readings/smoothings:
    
    // Window size for LDR&BAR Probe. Takes 10 samples in one process.
    AnalogSmooth as10 = AnalogSmooth(10);
    
    // Window size for ORP Probe. Takes 50 samples in one process.
    AnalogSmooth as50 = AnalogSmooth(50);

    // Window size for PH Probe. Takes 100 samples in one process.
    AnalogSmooth as100 = AnalogSmooth(100);

// Digital pins and attached devices on them:
  // Lights wall-buttons
static byte pin_WB1 = 9;
static byte pin_WB2 = 10;
static byte pin_WB3 = 11;
static byte pin_WB4 = 12;
static byte pin_WB5 = 13;
  // relays ...
static byte relay1_UVLAMP = 40;
static byte relay2_PUMP = 41;
static byte relay3_LIGHT = 42;
static byte relay4_LIGHT = 43;
static byte relay5_LIGHT = 44;
static byte relay6_LIGHT = 45;
static byte relay7_LIGHT = 46;
static byte relay8_LIGHT = 47;
  // PIR sensor
static byte pin_PIR1 = 4;
  // MOSFETs
static byte fet1_PH_pls = 5;
static byte fet2_PH_mns = 6;
  //  433MHz control INPUT
static byte pin_433_A = 48;
static byte pin_433_B = 49;
static byte pin_433_C = 50;
static byte pin_433_D = 51;
// end of named digital pins.


// BLYNK object and settings needed for INTERNET CONNECTION:
#define BLYNK_PRINT Serial    // Comment this out to disable BLYNK prints to serial and save space there.
#define EspSerial Serial2     // Hardware Serial (Tx2/Rx2) on Mega where ESP (UART) is connected.

  ESP8266 wifi(&EspSerial);

  // You should get Auth Token in the Blynk App.
  // Go to the Project Settings (nut icon).
  //char auth[] = "d7c937670ee1484baef8dcef0970b5d8";       //Blynk cloud key
  char auth[] = "1ea5d158d90d44bd9fc8ac86c176aab7";         //Private server key
  
  // Your WiFi credentials.
  // Set password to "" for open networks.
  char ssid[] = "WLAN2-vlan10-privat";
  char pass[] = "kecup-home";
//

///// TEMPERATURE SENSORS:
// DHT22 outside temperature&humidity sensor
#define DHTPIN1 7
#define DHTTYPE DHT22

// DS18B20 addresses & OneWireBus
  // Data wire is plugged onto pin 8 on the Arduino
#define ONE_WIRE_BUS 8

  // Setup a oneWire instance to communicate with any OneWire devices 
OneWire oneWire(ONE_WIRE_BUS);
 
  // Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

  //Dallas sensor address and declaration (from 1wire scan)
byte DS18B20_1[8] = {0x28, 0xB8, 0x73, 0xCC, 0x06, 0x00, 0x00, 0x61};   //Sensor at wall
byte DS18B20_2[8] = {0x28, 0xFF, 0x29, 0xEF, 0x43, 0x16, 0x03, 0xE1};   //Sensor in a pipe
byte DS18B20_3[8] = {0x28, 0xFF, 0x5B, 0x17, 0x44, 0x16, 0x03, 0x18};   //Sensor in top of absorber

// END OF LINB, OBJECTS, CONSTANTS


void setup() {

// start of pins modes ------------------------------------------------------------------
  // knobs...
pinMode(pin_KB1, INPUT);
pinMode(pin_KB2, INPUT);
pinMode(pin_KB3, INPUT);
pinMode(pin_PHprobe, INPUT);
pinMode(pin_ORPprobe, INPUT);
pinMode(pin_BARprobe, INPUT);
  // relays ...
pinMode(relay1_UVLAMP, OUTPUT);
pinMode(relay2_PUMP, OUTPUT);
pinMode(relay3_LIGHT, OUTPUT);
pinMode(relay4_LIGHT, OUTPUT);
pinMode(relay5_LIGHT, OUTPUT);
pinMode(relay6_LIGHT, OUTPUT);
pinMode(relay7_LIGHT, OUTPUT);
pinMode(relay8_LIGHT, OUTPUT);
  // PIR movement sensor
pinMode(pin_PIR1, INPUT);
  // MOSFETs
pinMode(fet1_PH_pls, OUTPUT);
pinMode(fet2_PH_mns, OUTPUT);
// light control, buttons and remote ...
  // 433 RemoteControl
pinMode(pin_433_A, INPUT);
pinMode(pin_433_B, INPUT);
pinMode(pin_433_C, INPUT);
pinMode(pin_433_D, INPUT);
  // WallButtons
pinMode(pin_WB1, INPUT);
pinMode(pin_WB2, INPUT);
pinMode(pin_WB3, INPUT);
pinMode(pin_WB4, INPUT);
pinMode(pin_WB5, INPUT);
// endo of pins modes ------------------------------------------------------------------
  
  Serial.begin(9600);
  Serial.println("POOL AUTOMAT v1.2.3");
  Serial.println();

 // Let set col.& rows of your LCD type[1602 or 2004]and initialize.
 lcd.begin(20,4);   // initialize the lcd for 16 chars 2 lines, turn on backlight
 // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(100);
    lcd.noBacklight();
    delay(100);
  }
  lcd.backlight(); // finish with backlight on 
  // Display. (Set Serial Monitor option to "No Line Ending")
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("Pool automat v1.2.3");
  delay(1000);
  lcd.clear();

delay(100);

// Initialize the RTC object:
  rtc.begin();
  
  // The following lines can be uncommented to set the date and time
    //rtc.setDOW(WEDNESDAY);      // Set Day-of-Week to SUNDAY
    //rtc.setTime(13, 40, 45);     // Set the time to 12:00:00 (24hr format)
    //rtc.setDate(22, 2, 2017);   // Set the date to January 1st, 2014

  // Send date--time to Serial0
    Serial.print(rtc.getDateStr());
    Serial.print(" -- ");
    Serial.println(rtc.getTimeStr());

// Start BLYNK object and connect the Internet
  // Set ESP8266 baud rate

    //EspSerial.begin(9600);
    EspSerial.begin(19200);
    delay(10);
    Serial2.println("AT+CWMODE=1");       //Set ESP in STATION mode only
    delay(10);
    Serial2.println("AT+RFPOWER=50");     //It increas TX power, needed for stable wlink 
    delay(10);
    Blynk.begin(auth, wifi, ssid, pass, "iot.vazac.eu", 8442);
    delay(10);
    
// Start up the OneWire and DS18B20 device, library 
  sensors.begin();

    //lowering resolution (9bit - 0,5C, 10bit - 0,25C ... 12bit) 
    //No resolution means 12 bit precision.
    sensors.setResolution(DS18B20_1, 10);
    sensors.setResolution(DS18B20_2, 10);
    sensors.setResolution(DS18B20_3, 10);
//

// Start with pump switched off, and check the temps for now, just for sure.
   PUMP_mode_off(); //stop pump anyway

      digitalWrite(fet1_PH_pls, LOW); //stop dozing anyway
      digitalWrite(fet2_PH_mns, LOW); //stop dozing anyway
   
   TEMP_readings(); //temperature reeads before loop()
//

delay(1000);

//END OF SETUP PART
}

// KNOBS integers:
boolean buttonK1 = 1; //FALSE means pressed! 
boolean buttonK2 = 1; //FALSE means pressed! 
boolean buttonK3 = 1; //FALSE means pressed! 

boolean shortK1 = 0; 
boolean shortK2 = 0; 
boolean shortK3 = 0; 

boolean longK1 = 0;
boolean longK2_PH4 = 0;     //for dividing PH probe calibration processto two while section
boolean longK2_PH7 = 0;     //for dividing PH probe calibration processto two while section
boolean longK3_ORP475 = 0;  //for dividing ORP probe calibration process to two while section
boolean longK3_ORP650 = 0;  //for dividing ORP probe calibration processto two while section

int KB1_Time = 1;
int KB2_Time = 1;
int KB3_Time = 1;

float newK1, newK2, newK3;
//

// TEMP integers for readings from DS18B20 probes:
  // Temperature integers (all in Celsius):
float T_pool = 99.99;      //avarage pool temperature calc. form sensors pipe&wall
float T_pipe = 99.99;      //sesor in pipe inside
float T_solar = 99.99;     //sensor top of the absorber
float T_wall = 99.99;      //sensor at wall of the pool 
float T_air = 99.99;       //DHT22 sensor outside (should pointed to North)
float H_air = 99.99;       //DHT22 sensor outside (should pointed to North)

float TP_max = 32.00;          //MAX pool temperature, reached will only trigger up notification!

float TS_diffON = 3.00;     //Hystereses to control pump (T_setpoint + TS_diffON will start the pump) - constant in Celsius.
float TS_diffOFF = 1.00;    //Hystereses to control pump (T_setpoint + TS_diffOFF will stop the pump) - constant in Celsius.
float TS_setpoint = 0.00;   //SOLAR HEATING SETPOINT
float TS_eprom;                    //TEMPERATURE SETPOINT POINT IN FLASH MEMORY
static byte TS_addr = 10;           //CONSTANT EEPROM ADDRESS

float tempC;                //Temporary variable for Dallas sensors sensors.getTempC().

boolean TR_fail = false; 
//char TR_error[] = "Temp reading, error!";
//

// POOL capacity and pumping time counters and setpoints:
int RT_setpoint = 0;        //RUNNING DURATION per day

int RT_counter_start = 0;    //Pump Running Time counters to determine if RT_eprom value is reached or not
int RT_timediff_prvday = 0;  //Pased over-filtering time above requested time from previous day. Will be cated current day filtering cycle.
int RT_counter = 0;          //Reselected on daily basis

int RT_eprom;               //DURATON INT IN EEPROM MEMORY
static byte RT_addr = 30;    //CONSTANT EEPROM ADDRESS

int PV_setpoint = 0;        //POOL VOLUME!!!
int PV_eprom;               //POOL VOLUME INT IN EEPROM MEMORY
static byte PV_addr = 35;    //CONSTANT EEPROM ADDRESS
//

// PH & ORP vaule. Int. for readings from both probes:
  //PH variables:
float PH_cal;               //FINAL calibrated readings.

float PH_setpoint = 0.00;   //Point which is moved to eprom -> PHSP_eprom set point. 
float PHSP_eprom;             //PH SETPOINT INT IN EEPROM MEMORY (DOSING control value (to start DC motor) PH+/-)
static byte PHSP_addr = 20;    //CONSTANT EEPROM ADDRESS

float PH_constM;            //Part of PH formula
float PH_constB;            //Part of PH formula

float PH4_refV;               //PH4 reference readings during calibration
float PH4RV_eprom;            //PH4 reference calibration voltage in EPROM.
static byte PH4RV_addr = 40;   //CONSTANT EEPROM ADDRESS

float PH7_refV;               //PH7 reference readings during calibration
float PH7RV_eprom;            //PH7 reference calibration voltage in EPROM.
static byte PH7RV_addr = 45;   //CONSTANT EEPROM ADDRESS

float RawPHvalue;             //PH analog readings for smoothing.
  
  //ORP variables:
float ORP_constM;             //Part of ORP formula
float ORP_constB;             //Part of ORP formula

float ORP475_refmV;               //Redox 475mV reference readings during calibration
float ORP475RV_eprom;             //Redox 475mV reference calibration voltage in EPROM.
static int ORP475RV_addr = 50;    //CONSTANT EEPROM ADDRESS

float ORP650_refmV;               //Redox 650mV reference readings during calibration
float ORP650RV_eprom;             //Redox 650mV reference calibration voltage in EPROM.
static byte ORP650RV_addr = 55;    //CONSTANT EEPROM ADDRESS

float RawORPvalue;                //RAW ORP readings.

float ORP_float;                  //Calibrated readings not rounded upwards.
int ORP_cal;                      //FINAL rounded calibrated readings.
//

// In pipe watter preassure probe (at A2)
float RawBARvalue;
float BAR_value;
//

// LDR sun-light level sensor (at A3)
byte LDR_percent;
//

// Timmers and clock variables
int currenthour;    //RTC hour extraction to variable.
int currentminute;  //RTC minutes extration to variable
int absoluttime;    //RTC minutes from midnight

boolean main_time;  // Identify main or minor working time window depand on current hour, it is used to control pumping functions windows.
boolean last_time;  //To recognize main-time (working windows) changed. Used for notification only.

unsigned long LCD_Time;                 //LCD display refresh period
unsigned long LCD_BackLight;            //LCD display BackLight period
unsigned long PIPE_Measurement_Time;    //Measurement period in PUMP Control part.
unsigned long Counting_Time;            //Counting period of "RT_counter" period in PUMP Control part.
unsigned long PH_Dozing_period = 0;     //Time betwenn dozing PH (both)
unsigned long GETSensors_short;         //Period timer for reading values from sensors (some of them, short period);
unsigned long GETSensors_long;          //Period timer for reading values from sensors (rest of them, long period);
//

// PUMP_motor variables:
boolean PUMP_running = false;
char PUMP_state[] = "notR";             //To display current state of filtration pump
int HD_percent;                         //Variable to calc and display percentige value of requested filtration running per day.
//

// DOZING_pump-motor variables:
static float DOZE_delivery = 500.0;     //Dozing motor capacity in ml/min.
char DOZE_running[] = "not";            //to display current state of dozing pumps
//

// LIGHTS control variables:
boolean L_GP_currentstate;        //Light garden reflector pointed to pool.
boolean L_GH_currentstate;        //Light garden reflector pointed to pool.
boolean L_POOL_currentstate;      //Light garden reflector pointed to pool.
boolean L_HOUSE_currentstate;     //Light garden reflector pointed to pool.
boolean L_GREEN_currentstate;     //Light garden reflector pointed to pool.
boolean PIR1_state;               //PIR sensor pointed to house.    
  
int PIR1_timer;               //PIR switch-off timer variable.
 
unsigned long L_GH_lasttime;    //Debounce timer for buttons reads.
unsigned long L_GP_lasttime;    //Debounce timer for buttons reads.
unsigned long L_POOL_lasttime;    //Debounce timer for buttons reads.
unsigned long L_HOUSE_lasttime;    //Debounce timer for buttons reads.
unsigned long L_GREEN_lasttime;    //Debounce timer for buttons reads.
//

// Others
boolean blynker = false;
//

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

//This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to address + 3.
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
//

// This function will return temperature from DS18B20 sensors.
  //Dallas SensorValues function - getTempC. 
float sensorValue (byte deviceAddress[]) {
  
  tempC = sensors.getTempC (deviceAddress);
  return tempC;
}

// BLYNK WIDGEDS:
// -------------------------------------------------------------------------------------------------------------------------

// LED as a filtration pump running app-indicator
WidgetLED ledpin1(V0);    

//LED app-indicator. Garden-reflector1, pointed to house    
WidgetLED ledpin13(V13);  
//LED app-indicator. Garden-reflector1, pointed to pool   
WidgetLED ledpin14(V14);  
//LED app-indicator. GardenHouse in-light
WidgetLED ledpin15(V15);  
//LED app-indicator. GreenHouse in-light  
WidgetLED ledpin16(V16);
//LED app-indicator. pool-under-water reflectors    
WidgetLED ledpin17(V17);  
      
// Blynker - system checker, visialization
WidgetLED ledpin18(V18);  

//SOLAR temp. slider
BLYNK_WRITE(V11) 
{
  BLYNK_LOG("Got a new SOLAR req.temp slider value: %s", param.asInt());
  
  float BLYNK_SOLAR_setpoint = param.asInt();

  TS_setpoint = (BLYNK_SOLAR_setpoint / 10.0);
    Serial.print("Got new SOLAR requested value: ");
    Serial.println(TS_setpoint);
  
    if (TS_eprom != TS_setpoint) {
    
    EEPROMWritelong(TS_addr, (TS_setpoint * 10.0));     //Multiply by 10 to get complex number from float
    Serial.print("SOALR REQ.TEMP SetPoint changed using Blynk! Saving new value and refreshing display!");
    Serial.println(" ");
    }
}

//PH ballance slider
BLYNK_WRITE(V12) 
{
  BLYNK_LOG("Got a new PH slider value: %s", param.asInt());
  
  float BLYNK_PH_setpoint = param.asInt();

  PH_setpoint = (BLYNK_PH_setpoint / 10.0);
    Serial.print("Got new PH requested value: ");
    Serial.println(PH_setpoint);
  
    if (PHSP_eprom != PH_setpoint) {
    EEPROM.write(PHSP_addr, (PH_setpoint * 10.0) );      //Multiply by 10 to get complex number from float      
    Serial.print("PH REQ.BALLANCE SetPoint changed using Blynk! Saving new value and refreshing display!");
    Serial.println(" ");
    }
}


void loop() {
////////////// LOOP  //////////////

// == Clock, timinig and scheduling part
// -------------------------------------------------------------------------------------------------------------------------
t = rtc.getTime();

currenthour = (t.hour);
currentminute = (t.min);
absoluttime = ( (currenthour * 60) + currentminute);

  // Send date to Serial0
    //Serial.print(rtc.getDateStr());
    //Serial.print(" -- ");
  // Send time tp serial
    //Serial.println(rtc.getTimeStr());

  if (currenthour >= 8 && currenthour <= 17) {                  //working hours are from 8 to 17 oclock. In these hours we expected sun shine / solar heating.
          main_time = true;
          
          if (main_time == true && last_time == false ) {
            Serial.println("Working window is recognized as MAIN TIME - morning-afternoon! Just FYI.");
          }
          last_time = true;
          
      } else {
          main_time = false;
          
          if (main_time == false && last_time == true ) {
            Serial.println("Working window is recognized as MINOR TIME - evening&night! Just FYI.");
          }
          last_time = false;
      }

// nightly reset of pumping counters/timers 
if (currenthour == 0 && currentminute == 1) {
  
  if ( RT_counter > (RT_eprom * 60) ) { 
    RT_timediff_prvday = ( RT_counter - (RT_eprom * 60) );
  }
    else { 
      RT_timediff_prvday = 0;  //reset previous day over-filtering timer if over-trigger has not reached.
  }
  
  RT_counter = 0;     //Running time summarized pump motor counter, reseted on daily basis, at 00:01.
}


// == Checking temperature sensors values (99.99 or 130.00 == error, no data) at the start.
// -------------------------------------------------------------------------------------------------------------------------
if ( T_wall == 99.99 || T_pipe == 99.99 || T_solar == 99.99 || T_air == 99.99) {
  TR_fail = true;
  }
  else {
  TR_fail = false;
  }
  // ERROR ON DISPLAY, if exists
  while ( TR_fail == true) {
    Serial.println();
    Serial.print("Temp reading, error!");
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Temp reading, error!");
    delay(5000);
    lcd.clear();
    break;
    }

// == EEPROM VALUE READINGs and checks:
// -------------------------------------------------------------------------------------------------------------------------
  //Some should be converted from complex values back into floats
RT_eprom = EEPROM.read(RT_addr);
PV_eprom = EEPROM.read(PV_addr);
TS_eprom = (float)(EEPROMReadlong(TS_addr) /10.0);
PHSP_eprom = (float)(EEPROM.read(PHSP_addr) /10.0);
PH4RV_eprom = (float)(EEPROMReadlong(PH4RV_addr) /1000.0);
PH7RV_eprom = (float)(EEPROMReadlong(PH7RV_addr) /1000.0);
ORP475RV_eprom = (float)(EEPROMReadlong(ORP475RV_addr) /100.0);
ORP650RV_eprom = (float)(EEPROMReadlong(ORP650RV_addr) /100.0);


// At the bagining is important to set/check basic values.
  //Check Flash for requested filtering time (running time) per day value, if not - lets set it now.
  if (RT_eprom <= 0 || RT_eprom >= 25) {
  lcd.clear();
  shortK1 = true;
  }
  //Check Flash for right pool capacity/volume value (m3), if not - lets set it now.
  if (PV_eprom <= 0 || PV_eprom >= 101) {
  lcd.clear();
  longK1 = true;
  }
  //Check Flash for right solar panel requested temperature, if not - lets set it now.
  if (TS_eprom <= 19 || TS_eprom >= 51) {
  lcd.clear();
  shortK3 = true;
  }
  //Check Flash for right PH setpoint value, if not - lets set it now.
  if (PHSP_eprom <= 6.3 || PHSP_eprom >= 7.9) {
  lcd.clear();
  shortK2 = true;
  }/* //TMP
  //Check Flash if right PH values are exist there, if not - lets jump to calibration process of both
  if (PH4RV_eprom < 249.0 || PH4RV_eprom > 351.0) {
  lcd.clear();
  longK2_PH4 = true;
  }
  if (PH7RV_eprom < 249.0 || PH7RV_eprom > 441.0) {
  lcd.clear();
  longK2_PH7 = true;
  }
  //Check Flash if right ORP values are exist there, if not - lets jump to calibration process of both. Should be around 1610/2100.
  if (ORP475RV_eprom < 1550.0 || ORP475RV_eprom > 1700.0) {
  lcd.clear();
  longK3_ORP475 = true;
  }
  if (ORP650RV_eprom < 1950.0 || ORP650RV_eprom > 2250.0) {
  lcd.clear();
  longK3_ORP650 = true;
  }*/


// == NORMAL LCD Display state printing and settings:
// -------------------------------------------------------------------------------------------------------------------------
 if ( (millis() - LCD_Time) >= 5000 ) {
  
  info_display(); 
  LCD_Time = millis();
}

if ( (millis() - LCD_BackLight) >= 1800000 || currenthour == 0) {
  
  lcd.noBacklight();                    //switch off LCD bacground light after 30mins incativity  
  LCD_BackLight = millis();
}
if ((newK1 = knob1.read() / 4) != 0) {

  lcd.backlight();                      //switch LCD background light back on
  
  newK1 = 0;
  knob1.write(0);
}


// == READS knobs button state
// -------------------------------------------------------------------------------------------------------------------------
 //KNOB Button 1
buttonK1 = digitalRead(pin_KB1);
while (buttonK1 == false){
 KB1_Time++;
 delay(100);
 buttonK1 = digitalRead(pin_KB1);
 if (KB1_Time >= 20){                    //3s meens long press, no need to wait for user to release
         KB1_Time = 1;
         longK1 = true;
         //Serial.print("Knob K1 LONG press! ShortK1 selected - set your pool watter volume (m3) now!");
         //Serial.println();
            // --------------
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("POOL WATTER VOLUME");
            lcd.setCursor(5,2);
            lcd.print("CALC. POINT!");
            delay(1000);   
            lcd.clear();
            break;
 }        
 }  if (KB1_Time >= 2 && KB1_Time <=10) {          //1s< means short press
          KB1_Time = 1;
          shortK1 = true;
          //Serial.print("Knob K1 SHORT pressed! LongK1 selected - set the daily pumping window (hours per day) now!");
          //Serial.println();
            // --------------
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("PUMPING HOURS/DAY");
            lcd.setCursor(5,2);
            lcd.print("SET POINT!");
            delay(1000);   
            lcd.clear();
}
// Settings under KNOB1, sub-menu 1 (SHORT PRESS) - requested pump running time per day. 
            while (shortK1 == true && longK1 == false) {
                  //Serial.print("  Set requested pumping hours for an day. ");
                  //Serial.print("Use Knob1 to adjust current value: ");
                  //Serial.println(RT_setpoint);
                  //Serial.println();
                  //-----------------
                  lcd.setCursor(0,0);
                  lcd.print("Set pump. hours/d:");
                  lcd.setCursor(15,2);
                  lcd.print(RT_setpoint);
                    // Knob #1, (POOL) Req. min. day pump running time (RT) setpoint:
                  delay(100);
                  newK1 = knob1.read()/4;
                  RT_setpoint = RT_eprom + (newK1);
                  if ( RT_setpoint <= 0 || RT_setpoint >= 25 ) {
                      //Serial.println("Invalid range! Shold be 1 to 24! Set it again ...");
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("Out of range (1-24)!");
                      delay(1500);
                      lcd.clear();
                  }
                  // Writing value and returning to home screan.
                  if (digitalRead(pin_KB1) == LOW) {
                      EEPROM.write(RT_addr, RT_setpoint);
                      //Serial.print("    Pressed! Saving new value and returning display to home!");
                      //Serial.println("");
                      //-----------------
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("SAVING new value! ");
                      lcd.setCursor(15,3);
                      lcd.print(RT_setpoint);
                      delay(1000);
                      lcd.clear();
                      newK1 = 0;
                      knob1.write(0);
                      shortK1 = false; 
                      }
}            
              // Settings KNOB1, sub-menu 2 (LONG PRESS) - pool watter capacity needed for calculation.
            while (shortK1 == false && longK1 == true) {
                  //Serial.print("  Set your pool watter capacity in m3. ");
                  //Serial.print("Use Knob1 to adjust right value: ");
                  //Serial.println(PV_setpoint);
                  //Serial.println("");
                  //-----------------
                  lcd.setCursor(0,0);
                  lcd.print("Set pool capacity.");
                  lcd.setCursor(5,1);
                  lcd.print("Put in m3:");
                  lcd.setCursor(15,2);
                  lcd.print(PV_setpoint);
                    // Knob #1, (POOL) Pool watter volume (PV) for later calculation:
                    delay(100);
                    newK1 = knob1.read()/4;
                    PV_setpoint = PV_eprom + (newK1);
                    if ( PV_setpoint <= 0 || PV_setpoint >= 101 ) {
                      //Serial.println("Invalid range! Shold be 1 to 100! Set it again ...");
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("Out of range(1-100)!");
                      delay(1500);
                      lcd.clear();
                    }
                    // Writing value and returning to home screan.
                    if (digitalRead(pin_KB1) == LOW) {
                        EEPROM.write(PV_addr, PV_setpoint);
                        //Serial.print("    Pressed! Saving new value and returning display to home!");
                        //Serial.println();
                        //-----------------
                        lcd.clear();
                        lcd.setCursor(0,1);
                        lcd.print("SAVING new value! ");
                        lcd.setCursor(15,3);
                        lcd.print(PV_setpoint);
                        delay(1000);
                        lcd.clear();
                        newK1 = 0;
                        knob1.write(0);
                        longK1 = false; 
                        }
}

 //KNOB Button 2
buttonK2 = digitalRead(pin_KB2);
while (buttonK2 == false){
 KB2_Time++;
 delay(100);
 buttonK2 = digitalRead(pin_KB2);
 if (KB2_Time >= 20){                    //3s meens long press, no need to wait for user to release
         KB2_Time = 1;
         longK2_PH4 = true;
         longK2_PH7 = true;
         //Serial.print("Knob2 LONG pressed! LongK2 selected - Starting PH probe calibration process!");
         //Serial.println();
            // --------------
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("PH CALIBRATION");
            lcd.setCursor(5,2);
            lcd.print("PROCESS!");
            delay(1000);   
            lcd.clear();
            break;
 }        
 }  if (KB2_Time >= 2 && KB2_Time <=10) {          //1s< means short press
          KB2_Time = 1;
          shortK2 = true;
          //Serial.print("Knob2 SHORT pressed! ShortK2 selected - set requested PH ballance value for in the pool now!");
          //Serial.println();
            // --------------
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("POOL PH VALUE");
            lcd.setCursor(5,2);
            lcd.print("SET POINT!");
            delay(1000);   
            lcd.clear();
}
// Settings under KNOB2, sub-menu 1 (SHORT PRESS) - requested PH to be balanced later by the dosing pumps(PH+/PH-)
            while (shortK2 == true && longK2_PH4 == false && longK2_PH7 == false) {
                  //Serial.print("  Set requested point/value to balance PH in the pool. ");
                  //Serial.print("Use Knob2 to adjust current value: ");
                  //Serial.println(PH_setpoint);
                  //Serial.println();
                  //-----------------
                  lcd.setCursor(0,0);
                  lcd.print("Set req. PH balance:");
                  lcd.setCursor(15,2);
                  lcd.print(PH_setpoint);
                    // Knob #1, (POOL) Required PH vallue to keep
                  delay(100);
                  newK2 = knob2.read()/4;
                  PH_setpoint = PHSP_eprom + (newK2/10);
                  if ( PH_setpoint <= 6.5 || PH_setpoint >= 7.5 ) {
                      Serial.println("Invalid range! Shold be clouse to 7! Set it again ...");
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("Out of range (cca7)!");
                      delay(1000);
                      lcd.setCursor(12,3);
                      lcd.print(PH_setpoint);
                      delay(500);
                      lcd.clear();
                  }            
                  // Writing value and returning to home screan.
                  if (digitalRead(pin_KB2) == LOW) {
                      EEPROM.write(PHSP_addr, (PH_setpoint * 10.0));      //Multiply by 10 to get complex number from float
                      //Serial.print("    Pressed! Saving new value and returning display to home!");
                      //Serial.println("");
                      //-----------------
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("SAVING new value! ");
                      lcd.setCursor(15,3);
                      lcd.print(PH_setpoint);
                      delay(1000);
                      lcd.clear();
                      newK2 = 0;
                      knob2.write(0);
                      shortK2 = false; 
                      }
}       
/////////////////////// PH PROBE CALIBRATION FUNCTIONS:
// Settings under KNOB2, sub-menu 2 (LONG PRESS) - PH calibration process - two reference points (PH4, PH7). Voids are declared bellow.

            //PH4 calibration part 1
            while (shortK2 == false && longK2_PH4 == true) {

                  delay(1000);
                  PH4_calibration();        // PH4 calibration function / process (void below)
}
            //PH7 calibration part 2
            while (shortK2 == false && longK2_PH7 == true) {

                  delay(1000);
                  PH7_calibration();        // PH4 calibration function / process (void below)
}

 
 //KNOB Button 3
buttonK3 = digitalRead(pin_KB3);
while (buttonK3 == false){
 KB3_Time++;
 delay(100);
 buttonK3 = digitalRead(pin_KB3);
 if (KB3_Time >= 20){                    //3s meens long press, no need to wait for user to release
         KB3_Time = 1;
         longK3_ORP475 = true;
         longK3_ORP650 = true;
         //Serial.print("Knob3 LONG press! LongK3 selected - Starting ORP probe calibration process!");
         //Serial.println();
            // --------------
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("ORP CALIBRATION");
            lcd.setCursor(5,2);
            lcd.print("PROCESS");
            delay(1000);   
            lcd.clear();
            break;
 }        
 }  if (KB3_Time >= 2 && KB3_Time <=10) {          //1s< means short press
          KB3_Time = 1;
          shortK3 = true;
          //Serial.print("Knob3 SHORT press! ShortK3 selected - set temperature point to start/stop pumping depand on hystereses values!");
          //Serial.println();
            // --------------
            lcd.clear();
            lcd.setCursor(2,1);
            lcd.print("REQ. SOLAR TEMP.");
            lcd.setCursor(5,2);
            lcd.print("PUMPING POINT!");
            delay(1000);   
            lcd.clear();
}
// Settings under KNOB3, sub-menu 1 (SHORT PRESS) - solar pannel temperature point to active pumping (depand on hystereses). 
            while (shortK3 == true && longK3_ORP475 == false && longK3_ORP650 == false) {
                  //Serial.print("  Set solar panel temperature point which start filtration pump. ");
                  //Serial.print("Use Knob3 to adjust current value: ");
                  //Serial.println(TS_setpoint);
                  //Serial.println();
                  //-----------------
                  lcd.setCursor(0,0);
                  lcd.print("Set solar temp.:");
                  lcd.setCursor(15,2);
                  lcd.print(TS_setpoint);
                    // Knob #3, (POOL) Req. starting point for watter pumping. (TS) setpoint:
                  delay(100);
                  newK3 = knob3.read()/2;
                  TS_setpoint = TS_eprom + (newK3 / 10);
                  if ( TS_setpoint <= 19 || TS_setpoint >= 51 ) {
                      //Serial.println("Invalid range! Shold be 20 to 50! Set it again ...");
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("Out of range(19-50)!");
                      delay(1000);
                      lcd.setCursor(12,3);
                      lcd.print(TS_setpoint);
                      delay(500);

                      lcd.clear();
                  }
                  // Writing value and returning to home screan.
                  if (digitalRead(pin_KB3) == LOW) {
                      EEPROMWritelong(TS_addr, (TS_setpoint * 10.0));     //Multiply by 10 to get complex number from float
                      //Serial.print("    Pressed! Saving new value and returning display to home!");
                      //Serial.println("");
                      //-----------------
                      lcd.clear();
                      lcd.setCursor(0,1);
                      lcd.print("SAVING new value! ");
                      lcd.setCursor(15,3);
                      lcd.print(TS_setpoint);
                      delay(1000);
                      lcd.clear();
                      newK3 = 0;
                      knob3.write(0);
                      shortK3 = false; 
                      }
}            
/////////////////////// REDOX PROBE CALIBRATION FUNCTIONS:

              // Settings KNOB3, sub-menu 2 (LONG PRESS) - Starting the ORP calibration process.
                //ORP 475mV calibration, part 1
            while (shortK3 == false && longK3_ORP475 == true) {

              ORP475_calibration();   //Function definition is locaited below in voids section.
}
                //ORP 650mV calibration, part 2
            while (shortK2 == false && longK3_ORP650 == true) {

              ORP650_calibration();   //Function definition is locaited below in voids section.
}


// == PUMP CONTROLER:
// -------------------------------------------------------------------------------------------------------------------------
if (PUMP_running == false) { 

  //Start motor when temp in sol. absorber reached set poit + diff.
  if ( main_time == true && T_solar != 99.99 && (T_solar - TS_diffON) >= TS_eprom ) {
  
    PUMP_mode_on();         // Start pump engine (and set PUMP_running to true)!
      ledpin1.on();           //set indicator of virt. led to ON
  
    // start counting time while pumping (running time)
    RT_counter_start = absoluttime;

    PUMP_state[0] = 'R'; //serial display current runing reason
    PUMP_state[1] = 'S';
    PUMP_state[2] = 'o';
    PUMP_state[3] = 'l';
    
  }
  //Start filtration and keep running untill to reach the filtration set-point.
  if ( main_time == false && currenthour > 7 && (RT_counter + RT_timediff_prvday) < (RT_eprom * 60) ) {
    
    PUMP_mode_on();         // Start pump engine (and set PUMP_running to true)!  
      ledpin1.on();           //set indicator of virt. led to ON

    // start counting time while pumping (running time).  
    //RT_timediff = absoluttime;    
    RT_counter_start = absoluttime;
    
    PUMP_state[0] = 'R'; //serial display current runing reason
    PUMP_state[1] = 'C';
    PUMP_state[2] = 'n';
    PUMP_state[3] = 't';
    
  } 
  // Check pool temperature, if it is above the max temp. set-point, start night filtration cycle to cool water in absorber.
  if (main_time == false && currenthour == 2 && T_pool != 99.99 && T_pool >= TP_max) {

    PUMP_mode_on();           // Start pump engine (and set PUMP_running to true)!
      ledpin1.on();           //set indicator of virt. led to ON
      
    // start counting time while pumping (running time)
    RT_counter_start = absoluttime;

    PUMP_state[0] = 'R';   //serial display current runing reason
    PUMP_state[1] = 'O';
    PUMP_state[2] = 'v';
    PUMP_state[3] = 'H';
     
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("POOL OVERHEATED!!!");
    lcd.setCursor(3,3);
    lcd.print("=> cooling =>");
    delay(5000);
  }
} 
    
if (PUMP_running == true) {

 DOZING_mode_on();          // PH+ or PH- chemistry dozing to the water. Chemistry volume is calculated depand on pool volume.

 //Counting up daily pump runing counter
if ( (millis() - Counting_Time) >= 60000) {

  RT_counter++; //+1 every minute of run filter pump. On this counter depand below stop-if part (secound one).

  Counting_Time = millis();       // set timer
}
 
 //Check water temp and presure inside the pipe during motor is running on.
if ( ( millis() - PIPE_Measurement_Time ) >= 10000 ) {
  
  BAR_readings();                    // In pipe preasure measurement.
  
  T_pipe = sensorValue(DS18B20_2);   // In pipe temperature of floating water.

  PIPE_Measurement_Time = millis();       // set timer
   
}

  //Stop motor when temp in sol. absorber drops under the set-point - diff.
  if ( main_time == true && ( absoluttime - RT_counter_start) >= 2 && T_solar != 99.99 && (T_solar - TS_diffOFF) <= TS_eprom ) {
   
   PUMP_mode_off(); // Stop pump engine (and set PUMP_running to false)!
    ledpin1.off();           //set indicator of virt. led to OFF

    //Summarized counter is set to 0 every midnigt, if it is biger then requested min. filtering daily cycle, then the increment will be used as minus part next day.
    //RT_counter += (absoluttime - RT_counter_start);    
    
    PUMP_state[0] = 'S';
    PUMP_state[1] = 'S';
    PUMP_state[2] = 'o';
    PUMP_state[3] = 'l';
    
   }
  //Stop motor when RT_eprom (daily filtration hours portion is reached or is above of this limit). Over-time from previos day is added, if exist. See daily reset funcition at the top.
  if (main_time == false && ( absoluttime - RT_counter_start) >= 2 && (RT_counter + RT_timediff_prvday) >= (RT_eprom * 60) ) {    //nevypne po prechodu do main_time-true, tim spis, je-li solar vyssi
                                                                                                                                    
   PUMP_mode_off(); // Stop pump engine (and set PUMP_running to false)!
    ledpin1.off();           //set indicator of virt. led to OFF
    
    //Summarized counter is set to 0 every midnigt, if it is biger then requested min. filtering daily cycle, then the increment will be used as minus part next day.
    //RT_counter += (absoluttime - RT_counter_start);    

    PUMP_state[0] = 'S';
    PUMP_state[1] = 'C';
    PUMP_state[2] = 'n';
    PUMP_state[3] = 't';

   }
  if ( currenthour == 5 || (T_pool != 99.99 && T_pool == (TP_max - 2.0) ) ) {

   PUMP_mode_off(); // Stop pump engine (and set PUMP_running to false)!
    ledpin1.off();           //set indicator of virt. led to OFF
    
    //Summarized counter is set to 0 every midnigt, if it is biger then requested min. filtering daily cycle, then the increment will be used as minus part next day.
    //RT_counter += (absoluttime - RT_counter_start);    
    
    PUMP_state[0] = 'S';
    PUMP_state[1] = 'O';
    PUMP_state[2] = 'v';
    PUMP_state[3] = 'H';

   }
}

// == LIGHT CONTROLER:
// -------------------------------------------------------------------------------------------------------------------------
  
  LIGHTS_in_garden();   // garden reflectors (from 17 till 23 o'clock)

  LIGHTS_in_pool();     // under water lights (from 19 till 23 o'clock)

  LIGHTS_inside();      //lights inside green-house and garden-house (from 5am till 24 o'clock )

  
// == Sensor matrics periodic readings and Blynk sync.
// -------------------------------------------------------------------------------------------------------------------------  
    
if ( millis() - GETSensors_short >= 9500) {       //every 10 seconds

  TEMP_readings();      //Get temps&humidity from DHT_ and Dallas sensors (air&humidity, temp-pool-pipe, temp-pool-wall, temp-solar-absorber).

  LDR_reading();        //Get visible light intesity value from LDR (AS10).

if (Blynk.connected() == true) {
    
    //T_air - virtual write V1
      Blynk.virtualWrite(1, T_air);
    //H_air - virtual write V2
      Blynk.virtualWrite(2, H_air);
    //T_pool - virtual write V3
      Blynk.virtualWrite(3, T_pool);
    //T_solar - virtual write V4
      Blynk.virtualWrite(4, T_solar);
    
    //Sliders:  
    //TS-setpoint value - virtual write V11
      Blynk.virtualWrite(11, (TS_eprom * 10.0) );
    //PH-setpoint value - virt. write V12:
      Blynk.virtualWrite(12, (PHSP_eprom * 10.0) ); 

  if (blynker == false) {
  
    ledpin18.on();  //Blynk.check.indicator ON
    blynker = true;
  
  } else {
    
      ledpin18.off();  //Blynk.check.indicator OFF
      blynker = false;
  }
}  
  //restart-short-timer
  GETSensors_short = millis();
}

if ( millis() - GETSensors_long >= 29900) {       //every 30 seconds

  PH_readings();      //get fres value from PH_probe (AS100)
  
  ORP_readings();     //get fresh value from ORP_probe (AS50)

if (Blynk.connected() == true) {
    
    //PH_cal - virtual write V5
      Blynk.virtualWrite(5, PH_cal);
    //ORP_cal - virtual write V6
      Blynk.virtualWrite(6, ORP_cal);
    //BAR_value - virtual write V7
      Blynk.virtualWrite(7, BAR_value);
    //HD_percent - virtual write V8
      Blynk.virtualWrite(8, HD_percent);
    //LDR_percent - virtual write V9
      Blynk.virtualWrite(9, LDR_percent);
    //T_max - virtual write V10
      Blynk.virtualWrite(10, TP_max);
}
  //restart-long-timer
  GETSensors_long = millis();
}

// == Blynk data sync
// -------------------------------------------------------------------------------------------------------------------------  
if (Blynk.connected() == false) {

  //if disconnected
  Serial.print("BLYNK WAS DISCONNECTED at ");
  Serial.println(rtc.getTimeStr());  
     
  //Blynk.connect();  //reconnect BLYNK
  delay(10);
  
  Blynk.run();  //RUN BLYNK APP and transmit all date
  delay(10);
  
  Blynk.syncAll();    //sync data
  //Blynk.email("Disconnected", "PoolController is reconnected again."); 
}
    
    Blynk.run();  //RUN BLYNK APP and transmit all date

} // END OF LOOP -------------------------------------------------------------------------------------------------------------


///////////   VOIDS  ////////////////////////////

//------------------------------------------------------------------------------------------------ 
void LIGHTS_in_garden() {
  
// Garden reflectors can be switched on only at evening and in darkness.
if (LDR_percent < 25 && currenthour > 16 && currenthour <= 23) {

  // ON, IF some button has been pressed.
  if (L_GP_currentstate != true && (digitalRead(pin_433_B) == true || digitalRead(pin_WB3) == true) && ( millis() - L_GP_lasttime > 500) ) {
    
    digitalWrite(relay7_LIGHT, HIGH);
    ledpin14.on();  //blynk indicator
    
    L_GP_lasttime = millis();
    L_GP_currentstate = true; 
    //Serial.println("Garden light reflector pointed to pool goes ON - by switch or remote.");
      lcd.setCursor(9,2);
      lcd.print("*");
  }
  
  // ON, IF some button has been pressed.
  if (L_GH_currentstate != true && (digitalRead(pin_433_A) == true || digitalRead(pin_WB4) == true) && ( millis() - L_GH_lasttime > 500) ) {
    
    digitalWrite(relay8_LIGHT, HIGH);
    ledpin13.on();  //blynk indicator
    
    L_GH_lasttime = millis();
    L_GH_currentstate = true; 
    PIR1_state = false;
    //Serial.println("Garden light reflector pointed to house goes ON - by switch or remote.");
      lcd.setCursor(9,3);
      lcd.print("*");

    //ON, OR IF PIR sensore detected movement.
  } else if (L_GH_currentstate != true && digitalRead(pin_PIR1) == true && ( millis() - L_GH_lasttime > 1000) ) {
    
      digitalWrite(relay8_LIGHT, HIGH);
      ledpin13.on();  //blynk indicator
      
      L_GH_lasttime = millis();
      L_GH_currentstate = true; 
      PIR1_state = true;
      PIR1_timer = absoluttime;
      //Serial.println("Garden light reflector pointed to house goes ON - by PIR, starting 10 min. counter.");
        lcd.setCursor(9,3);
        lcd.print("!");
  } 
  //OFF, IF some button has been pressed.
  if (L_GP_currentstate == true && (digitalRead(pin_433_B) == true || digitalRead(pin_WB3) == true) && ( millis() - L_GP_lasttime > 500) ) {
    
    digitalWrite(relay7_LIGHT, LOW);
    ledpin14.off();  //blynk indicator
    
    L_GP_currentstate = false; 
    L_GP_lasttime = millis();
    //Serial.println("Garden light reflector pointed to pool goes OFF - by switch or remote.");
      lcd.setCursor(9,2);
      lcd.print(" ");
  }
  //OFF, IF some button has been ressed.
  if (L_GH_currentstate == true && PIR1_state == false && (digitalRead(pin_433_A) == true || digitalRead(pin_WB4) == true) && ( millis() - L_GH_lasttime > 500) ) {
    
    digitalWrite(relay8_LIGHT, LOW);
    ledpin13.off();  //blynk indicator
    
    L_GH_currentstate = false; 
    L_GH_lasttime = millis();
    //Serial.println("Garden light reflector pointed to house goes OFF - by switch or remote.");
      lcd.setCursor(9,3);
      lcd.print(" ");

    //OFF, by the PIR timmer.
  } else if (L_GH_currentstate == true && PIR1_state == true && ( absoluttime - PIR1_timer > 5) ) {
      
      digitalWrite(relay8_LIGHT, LOW);
      ledpin13.off();  //blynk indicator
      
      L_GH_currentstate = false;
      PIR1_state = false; 
      L_GH_lasttime = millis();
      //Serial.println("Garden light reflector pointed to house goes OFF - by PIR timer!");
        lcd.setCursor(9,3);
        lcd.print(" ");
  }
  } else {
    // otherwise will be switched off and will stay off. Usable if someone will forged to switch light off.

    digitalWrite(relay7_LIGHT, LOW);  //Garden POOL reflector
      lcd.setCursor(9,2);
      lcd.print(" ");
      ledpin14.off();  //blynk indicator
      
    digitalWrite(relay8_LIGHT, LOW);  //Garden HOUSE reflector
      lcd.setCursor(9,3);
      lcd.print(" ");
      ledpin13.off();  //blynk indicator
  }
}

//------------------------------------------------------------------------------------------------
void LIGHTS_in_pool() {
// Garden reflector dorected to pool can be witched on only at evening and in darkness.
if (LDR_percent < 25 && currenthour > 18 && currenthour <= 23) {

  // ON, IF some button has been pressed.
  if (L_POOL_currentstate != true && (digitalRead(pin_433_C) == true || digitalRead(pin_WB2) == true) && ( millis() - L_POOL_lasttime > 500) ) {
    
    digitalWrite(relay6_LIGHT, HIGH);
    ledpin17.on();  //blynk indicator
    
    L_POOL_lasttime = millis();
    L_POOL_currentstate = true; 
    //Serial.println("Pool under-water light reflectors goes ON - by switch or remote.");
      lcd.setCursor(9,0);
      lcd.print("*");

  } 
  //OFF, IF some button has been pressed.
  if (L_POOL_currentstate == true && (digitalRead(pin_433_C) == true || digitalRead(pin_WB2) == true) && ( millis() - L_POOL_lasttime > 500) ) {
    
    digitalWrite(relay6_LIGHT, LOW);
    ledpin17.off();  //blynk indicator
        
    L_POOL_currentstate = false; 
    L_POOL_lasttime = millis();
    //Serial.println("Pool under-water light reflectors goes OFF - by switch or remote.");
      lcd.setCursor(9,0);
      lcd.print(" ");
  }
  } else {
    // otherwise will be switched off and stay off. Usable if someone will forged to switch light off.
    
    digitalWrite(relay6_LIGHT, LOW);
      lcd.setCursor(9,0);
      lcd.print(" ");
      ledpin17.off();  //blynk indicator
  }
}

//------------------------------------------------------------------------------------------------
void LIGHTS_inside() {
// Lights inside the garden house can be witched on only at practicly anytime.
if (currenthour > 5 && absoluttime <= 1439) {

  // ON, IF some button has been pressed.
  if (L_HOUSE_currentstate != true && (digitalRead(pin_WB1) == true) && ( millis() - L_HOUSE_lasttime > 500) ) {
    
    digitalWrite(relay5_LIGHT, HIGH);
    ledpin15.on();  //blynk indicator
        
    L_HOUSE_lasttime = millis();
    L_HOUSE_currentstate = true; 
    //Serial.println("Light inside the garden house goes ON - by the wall switch.");

  } else if (L_GREEN_currentstate != true && (digitalRead(pin_WB5) == true) && ( millis() - L_GREEN_lasttime > 500) ) {
    
    digitalWrite(relay4_LIGHT, HIGH);
    ledpin16.on();  //blynk indicator
        
    L_GREEN_lasttime = millis();
    L_GREEN_currentstate = true; 
    //Serial.println("Light inside the green house goes ON - by the wall switch.");
      lcd.setCursor(9,1);
      lcd.print("*");    
  }
  //OFF, IF some button has been pressed.
  if (L_HOUSE_currentstate == true && (digitalRead(pin_WB1) == true) && ( millis() - L_HOUSE_lasttime > 500) ) {
    
    digitalWrite(relay5_LIGHT, LOW);
    ledpin15.off();  //blynk indicator
        
    L_HOUSE_currentstate = false; 
    L_HOUSE_lasttime = millis();
    //Serial.println("Ligth inside the garden house goes OFF - by the wall switch..");
    
  } else if (L_GREEN_currentstate == true && (digitalRead(pin_WB5) == true) && ( millis() - L_GREEN_lasttime > 500) ) {
    
    digitalWrite(relay4_LIGHT, LOW);
    ledpin16.off();  //blynk indicator
        
    L_GREEN_currentstate = false; 
    L_GREEN_lasttime = millis();
    //Serial.println("Ligth inside the green house goes OFF - by the wall switch..");
      lcd.setCursor(9,1);
      lcd.print(" ");
  }
  } else {
    // otherwise will be switched off and stay off. Usable if someone will forged to switch light off.

    digitalWrite(relay4_LIGHT, LOW);
      lcd.setCursor(9,1);
      lcd.print(" ");
      ledpin16.off();  //blynk indicator
      
    //not idicated on LCD  
    digitalWrite(relay5_LIGHT, LOW);
    ledpin15.off();  //blynk indicator
  }
}

//------------------------------------------------------------------------------------------------
void PH_readings() {
//PH formula: "y = mx + b"
  //
  //m = (y2-y1) / (x2-x1)
  //m = ((7-4) / (PH7_reference_sample - PH4_reference_sample))
  //
  //b = (y1) - m*(x1)
  //b = 4 - PH_constM*PH4_reference
  //
  //pH (calibrated) = m(analogReadings) + b

RawPHvalue = (1023.0 - as100.analogReadSmooth(pin_PHprobe) );

  Serial.print("RAW PH value (negativ sample): ");
  Serial.print(RawPHvalue);
  Serial.println(" ");
  //delay(500);

PH_constM = ( (7.0 - 4.0)/(PH7RV_eprom - PH4RV_eprom) );

  //Serial.print("PH Constant M: ");
  //Serial.print(PH_constM);
  //Serial.println(" ");
  //delay(500);

PH_constB = (4.0 - (PH_constM * PH4RV_eprom) );

  //Serial.print("PH Constant B: ");
  //Serial.print(PH_constB);
  //Serial.println(" ");
  //delay(500);
  
PH_cal = ( (PH_constM * RawPHvalue) + PH_constB);
  
  //Serial.println("============================= ");
  //Serial.println("FINAL PH VALUE IS: ");
  //Serial.print(PH_cal);
  //Serial.println("============================= ");
  //Serial.println(" ");
  //delay(1000);
}

//------------------------------------------------------------------------------------------------
void ORP_readings() {
//ORP formula: "y = mx + b"
  //
  //m = (y2-y1) / (x2-x1)
  //m = ((7-4) / (ORP650_reference_mV - ORP475_reference_mV))
  //
  //b = (y1) - m*(x1)
  //b = 475 - ORP_constM*ORP475_refmV
  //
  //ORP (calibrated) = m(analogReadings) + b

RawORPvalue = ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);     //Smooth analog readings ORP raw value (in mV)

  //Serial.print("RAW ORP value (mV): ");
  //Serial.print(RawORPvalue);
  //Serial.println(" ");
  //delay(500);

ORP_constM = ( (650.0 - 475.0) / (ORP650RV_eprom - ORP475RV_eprom) );

  //Serial.print("ORP Constant M: ");
  //Serial.print(ORP_constM);
  //Serial.println(" ");
  //delay(500);

ORP_constB = (475.0 - (ORP_constM * ORP475RV_eprom) );

  //Serial.print("ORP Constant B: ");
  //Serial.print(ORP_constB);
  //Serial.println(" ");
  //delay(500);
  
ORP_float = ( ( (ORP_constM * RawORPvalue) + ORP_constB) + 0.55);   //0.55 for rounded upwards

ORP_cal = (int)ORP_float;
  
  //Serial.println("============================= ");
  //Serial.println("FINAL ORP VALUE IS: ");
  //Serial.print(ORP_cal);
  //Serial.println("============================= ");
  //Serial.println(" ");
  //delay(1000);
}

//------------------------------------------------------------------------------------------------
void BAR_readings() {
//Pressure in pipe from analog sensor
  
  // Sensor Working voltage: DC 5V;
  // Sensor Output voltage: 0.5~4.5V DC;
  // Sensor Working current: <=10mA;
  // Sensor Working pressure: 0~0.5MPa;
  
  RawBARvalue = as10.analogReadSmooth(pin_BARprobe);

  //Serial.print("Preassure RAW value (ADC): ");
  //Serial.print(RawBARvalue);
  //Serial.println(" ");
  //delay(500);


  // 0 to 5V at 10-bit resolution
  // 0.5 = 102  ==> 0.0 MPa
  // 4.5 = 922  ==> 0.5 MPa
  // (so the 0,5MPa range is 820 range in analog reading)

  //Search analog value selection in range (min, max)

  BAR_value = constrain(RawBARvalue, 120, (120 + 830));

  // Preassure formula 

      //analog_minReading = 102 (reads 120 at 0 bars);
      //analog_maxRange = 830;
      //sensor maxMPa = 0,5;
      //psiPerMPa = 145.0377
      //barPerMPa = 0,1
      
  BAR_value = ( ( (RawBARvalue - 120) / ( 830 * 0.5) ) / 0.1 );    //bar pressure. For Mpa cut off the formula tail: /0.1.

  //shorting to only 2 dec. places
  BAR_value = ( (int)(BAR_value * 100.0));
  BAR_value = ( (float)BAR_value / 100);
  
  //Serial.print("Preassure value (BAR): ");
  //Serial.print(BAR_value);
  //Serial.println(" ");
  //delay(500);
}

//------------------------------------------------------------------------------------------------
void TEMP_readings() {
 // DHT senor object and readings

  // DHT temperture sensor object
    DHT dht1(DHTPIN1, DHTTYPE);

  dht1.begin();
  T_air = dht1.readTemperature();
  
  H_air = dht1.readHumidity();
  
  //Serial.print("AIR Temperature is: ");
  //Serial.println(T_air);

  // Call DS18B20 sensors.requestTemperatures() to issue a global temperature - request to all devices on the bus:
  //For 10-bits resoution it will take around 200ms to get data/values, see the tab here: http://www.homautomation.org/2015/11/17/ds18b20-how-to-change-resolution-9101112-bits/
  
  sensors.requestTemperatures(); // Send the command to get temperatures from all 1Wire Dallass (DS18B20) sensors.
  //delay(200);
    
  //Serial.print(" Requesting temperatures...");
  //Serial.println("DONE");

  T_wall = sensorValue(DS18B20_1);            //  Index 0, first sensor (POOL WALL)

  //Serial.print("Temperature at WALL (POOL WALL 0,5m DEEP: ");
  //Serial.println(T_pool);
  
  T_pipe = sensorValue(DS18B20_2);            //  Index 1, secounf sensor (POOL PIPE), reading during PUMP_mode_on()
  
  //Serial.print("Temperature in PIPE (WATER INCOMMING FROM POOL) is: ");
  //Serial.println(T_pipe);

    T_pool = ( ( (T_pipe * 2.0) + (T_wall * 3.0) ) / 5.0);

    //Serial.print("Calculated Temperature in POOL is: ");
    //Serial.println(T_pool);

  T_solar = sensorValue(DS18B20_3);           //  Index 2, thrid sensor (SOLAR ABSORBER)

  //Serial.print("Temperature in SOLAR ABSORBER is: ");
  //Serial.println(T_solar);
}

//------------------------------------------------------------------------------------------------
void LDR_reading() {

// LDR sun light level sensor (at A3)

LDR_percent = map(as10.analogReadSmooth(pin_LDRprobe), 0, 1023, 0, 100);

  //Serial.print("Light intensity: ");
  //Serial.print(LDR_percent);
  //Serial.println("%");
  
}

//------------------------------------------------------------------------------------------------
void PUMP_mode_on() {

   // start UV lamp first (needs time to fully light up)
  digitalWrite(relay1_UVLAMP, HIGH);
  delay(500);
  
  // start PUMP engine, switch it ON
  digitalWrite(relay2_PUMP, HIGH);

  PUMP_running = true;    //used in IF functions to get current pumping state
  
  unsigned long temp_in_pipe_timer = millis()+1000;
  
  if (millis() >= temp_in_pipe_timer) {

    T_pipe = sensorValue(DS18B20_2);            //  Index 1, secounf sensor (POOL PIPE)
  
    //Serial.print("Temperature in PIPE (WATER INCOMMING FROM POOL) is: ");
    //Serial.println(T_pipe);
  }
  
}

//------------------------------------------------------------------------------------------------
void PUMP_mode_off() {

  // stop PUMP engine, switch it OFF
  digitalWrite(relay2_PUMP, LOW);
  delay(500);

  PUMP_running = false;   //used in IF functions to get current pumping state

  // stop UV lamp first switch it OFF
  digitalWrite(relay1_UVLAMP, LOW);
  
}

//------------------------------------------------------------------------------------------------
void DOZING_mode_on() {

  // Protection for bad doze calcultaion depand on unstable reading of PH val.
  if (PH_Dozing_period == 0) {
  
  PH_Dozing_period = absoluttime; //timer will be ready for dozing after 3h later. 
} 

// == DOZING CONTROL, two parts for full doze and hlaf doze if it is close to set point value. 0.5 - 13.5 avoid start dozing while measuring/calculation of PH is out of range (typicaly INF or 0.0).
if ( (PH_cal >= 0.5 && PH_cal <= 13.5) && ( (PHSP_eprom - PH_cal) >= 0.05 || (PH_cal - PHSP_eprom) >= 0.05) ) {

  //space for furter online monitoring statemants.

  //PH PLUS, small differences - 3h period and half volume of chemistry
  if ( (absoluttime - PH_Dozing_period) >= 180 && (PHSP_eprom - PH_cal) > 0.1 && (PHSP_eprom - PH_cal) <= 0.4 ) {
  
    // PH+ liquid chemical exact dosing: 20 ml / 1 m3 water will increase PH point on 0,1 up.
    // DC12V Peristaltic dosing Pump Tygon LFL 6mm(ID) tube and can deliver 500ml/min - static global variable "DOZE_delivery".
    // see: http://www.ebay.com/itm/272432554270?_trksid=p2060353.m1438.l2649&ssPageName=STRK%3AMEBIDX%3AIT

    //Formula for one dose (ml) - will shift PH in a pool +0.1: (PH_SetPoint - PH_measured / chemical-increase-point-for-m3 * (one-chemical-doze-for-m3 * pool-volume-m3 ) )

    float PH_diff_pls = (PHSP_eprom - PH_cal);                                            //differential between requested PH point and meassurement.
    float PH_dozing_ml = ( ( (PH_diff_pls / 0.1) * (20.0 * (float)PV_eprom) ) / 2.0 );    //calc dose for whole pool volume and then cut it to half (is to clouse to SP).
    float PH_dozing_time = (PH_dozing_ml / DOZE_delivery);                                // (doze volume / peristaltic pump speed) = time in minutes.
    
    RT_counter = (RT_counter + ( (int)PH_dozing_time + 0.5) );  //becouse of delay below, we need to add dozing-time duration to filter pump running time counter.

     Blynk.email("PH+, half", "Now dozing ...");
    
      DOZE_running[0] = 'y';
      DOZE_running[1] = '+';
      DOZE_running[2] = 'h';

      digitalWrite(fet1_PH_pls, HIGH);              //PH+ dozing motor on
        Serial.print("Now dozing PH+ (half doze), please wait for: ");
        Serial.print(PH_dozing_time);
        Serial.print(" min + (delay 0.5+) / (");
        Serial.print(PH_dozing_ml);
        Serial.print(" ml )");
        //----------------------
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("PH+ half dozing ON!");
        lcd.setCursor(2,1);
        lcd.print("Duration: ");
        lcd.print(PH_dozing_time);
        lcd.setCursor(2,2);
        lcd.print("Doze vol: ");
        lcd.print(PH_dozing_ml);
        lcd.setCursor(7,3);
        lcd.print("=> dozing =>");
      
      delay(PH_dozing_time * 60000.0);              // ... delivery calculated chemistry volume depand on dozing pump capacity / time.
    
      digitalWrite(fet1_PH_pls, LOW);               //PH+ dozing motor off
        lcd.setCursor(7,3);
        lcd.print("=> Done! 1/2");
        
      DOZE_running[0] = 'n';
      DOZE_running[1] = 'o';
      DOZE_running[2] = 't';
      
      delay(30000);                                 //keep water curculate for 1/2 minute more 
        Serial.print("PH+ dozing (half doze),done! Next dozing time +3h later.");
        lcd.clear();
        
    PH_Dozing_period = absoluttime;
    
  }
  //PH PLUS, big differences - 6h period and exac volume of chemistry
  else if ( (absoluttime - PH_Dozing_period) >= 360 && (PHSP_eprom - PH_cal) > 0.4 ) {
  
    // PH- liquid chemical exact dosing: 20 ml / 1 m3 water will increase PH point on 0,1 up.
    // DC12V Peristaltic dosing Pump Tygon LFL 6mm(ID) tube and can deliver 500ml/min - static global variable "DOZE_delivery".

    //Formula for one dose (ml) - will shift PH in a pool +0.1: (PH_SetPoint - PH_measured / chemical-increase-point-for-m3 * (one-chemical-doze-for-m3 * pool-volume-m3 ) )

    float PH_diff_pls = (PHSP_eprom - PH_cal);                                          //differential between requested PH point and meassurement.
    float PH_dozing_ml = ( (PH_diff_pls / 0.1) * (20.0 * (float)PV_eprom) );            //calc dose for whole pool volume and then cut it to half (is to clouse to SP).
    float PH_dozing_time = (PH_dozing_ml / DOZE_delivery);                              // (doze volume / pumping speed) = time in minutes, then mutiplyed to millis.

    RT_counter = (RT_counter + ( (int)PH_dozing_time + 0.75) );  //becouse of delay below, we need to add dozing-time duration to filter pump running time counter.

      Blynk.email("PH+, full", "Now dozing ...");
    
      DOZE_running[0] = 'y';
      DOZE_running[1] = '+';
      DOZE_running[2] = 'f';
      
      digitalWrite(fet1_PH_pls, HIGH);              //PH+ dozing motor on
        Serial.print("Now dozing PH+ (full doze), please wait for: ");
        Serial.print(PH_dozing_time);
        Serial.print(" min + (delay 1+) / (");
        Serial.print(PH_dozing_ml);
        Serial.print(" ml )");
        //----------------------
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("PH+ full dozing ON!");
        lcd.setCursor(2,1);
        lcd.print("Duration: ");
        lcd.print(PH_dozing_time);
        lcd.setCursor(2,2);
        lcd.print("Doze vol: ");
        lcd.print(PH_dozing_ml);
        lcd.setCursor(7,3);
        lcd.print("=> dozing =>");

      delay(PH_dozing_time * 60000.0);              // ... delivery calculated chemistry volume depand on dozing pump capacity / time.
    
      digitalWrite(fet1_PH_pls, LOW);               //PH+ dozing motor off
        lcd.setCursor(7,3);
        lcd.print("=> Done! 1/1");

      DOZE_running[0] = 'n';
      DOZE_running[1] = 'o';
      DOZE_running[2] = 't';
      
      delay(60000);                                 //keep water curculate 1 minute more
        Serial.println("PH+ dozing (full doze),done! Next dozing time min. +6h.");
        lcd.clear();

    PH_Dozing_period = absoluttime;
    
  }

  //PH MINUS, small differences - 3h period and half volume of chemistry
  if ( ( absoluttime - PH_Dozing_period) >= 180 && (PH_cal - PHSP_eprom) > 0.1 && (PH_cal - PHSP_eprom) <= 0.4 ) {
  
    // PH- liquid chemical exact dosing: 30 ml / 1 m3 water will reduce PH point on 0,1 down.
    // DC12V Peristaltic dosing Pump Tygon LFL 6mm(ID) tube and can deliver 500ml/min - static global variable "DOZE_delivery".

    float PH_diff_mns = (PH_cal - PHSP_eprom);                                            //differential between requested PH point and meassurement.
    float PH_dozing_ml = ( ( (PH_diff_mns / 0.1) * (30.0 * (float)PV_eprom) ) / 2.0 );    //calc dose for whole pool volume and then cut it to half (is to clouse to SP).
    float PH_dozing_time = (PH_dozing_ml / DOZE_delivery);                                // (doze volume / pumping speed) = time in minutes, then mutiplyed to millis.

    RT_counter = (RT_counter + ( (int)PH_dozing_time + 0.5) );  //becouse of delay below, we need to add dozing-time duration to filter pump running time counter.

     Blynk.email("PH-, half", "Now dozing ...");
     
      DOZE_running[0] = 'y';
      DOZE_running[1] = '-';
      DOZE_running[2] = 'h';

      digitalWrite(fet2_PH_mns, HIGH);              //PH- dozing motor on
        Serial.print("Now dozing PH- (half doze), please wait for: ");
        Serial.print(PH_dozing_time);
        Serial.print(" min + (delay 0.5+) / (");
        Serial.print(PH_dozing_ml);
        Serial.print(" ml )");
        //----------------------
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("PH- half dozing ON!");
        lcd.setCursor(2,1);
        lcd.print("Duration: ");
        lcd.print(PH_dozing_time);
        lcd.setCursor(2,2);
        lcd.print("Doze vol: ");
        lcd.print(PH_dozing_ml);
        lcd.setCursor(7,3);
        lcd.print("=> dozing =>");
        
      delay(PH_dozing_time * 60000.0);              // ... delivery calculated chemistry volume depand on dozing pump capacity / time.
    
      digitalWrite(fet2_PH_mns, LOW);               //PH- dozing motor off
        lcd.setCursor(7,3);
        lcd.print("=> Done! 1/2");
      
      DOZE_running[0] = 'n';
      DOZE_running[1] = 'o';
      DOZE_running[2] = 't';
      
      delay(30000);                                 //keep water curculate 1 minute more
        Serial.println("PH- dozing (half doze),done! Next dozing time min. +3h.");
        lcd.clear();
                      
    PH_Dozing_period = absoluttime;
    
  }
  //PH MINUS, big differences - 6h period and exac volume of chemistry
  else if ( ( absoluttime - PH_Dozing_period) >= 360 && (PH_cal - PHSP_eprom) > 0.4 ) {
  
    // PH- liquid chemical exact dosing: 30 ml / 1 m3 water will reduce PH point on 0,1 down.
    // DC12V Peristaltic dosing Pump Tygon LFL 6mm(ID) tube and can deliver 500ml/min - static global variable "DOZE_delivery".

    float PH_diff_mns = (PH_cal - PHSP_eprom);                                            //differential between requested PH point and meassurement.
    float PH_dozing_ml = ( (PH_diff_mns / 0.1) * (30.0 * (float)PV_eprom) );              //calc dose for whole pool volume and then cut it to half (is to clouse to SP).
    float PH_dozing_time = (PH_dozing_ml / DOZE_delivery);                                // (doze volume / pumping speed) = time in minutes, mutiplyed to millis.

    RT_counter = (RT_counter + ( (int)PH_dozing_time + 0.75) );  //becouse of delay below, we need to add dozing-time duration to filter pump running time counter.

      Blynk.email("PH-, full", "Now dozing ...");
    
      DOZE_running[0] = 'y';
      DOZE_running[1] = '-';
      DOZE_running[2] = 'f';

      digitalWrite(fet2_PH_mns, HIGH);              //PH- dozing motor on
        Serial.print("Now dozing PH- (full doze), please wait for: ");
        Serial.print(PH_dozing_time);
        Serial.print(" min + (delay 1+) / (");
        Serial.print(PH_dozing_ml);
        Serial.print(" ml )");
        //----------------------
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("PH- full dozing ON!");
        lcd.setCursor(2,1);
        lcd.print("Duration: ");
        lcd.print(PH_dozing_time);
        lcd.setCursor(2,2);
        lcd.print("Doze vol: ");
        lcd.print(PH_dozing_ml);
        lcd.setCursor(7,3);
        lcd.print("=> dozing =>");
              
      delay(PH_dozing_time * 60000.0);              // ... delivery calculated chemistry volume depand on dozing pump capacity / time.
    
      digitalWrite(fet2_PH_mns, LOW);               //PH- dozing motor on
        lcd.setCursor(7,3);
        lcd.print("=> Done! 1/1");
      
      DOZE_running[0] = 'n';
      DOZE_running[1] = 'o';
      DOZE_running[2] = 't';
      
      delay(60000);                                 //keep water curculate 1 minute more
        Serial.println("PH- dozing (full doze),done! Next dozing time min. +6h.");
        lcd.clear();
        
    PH_Dozing_period = absoluttime;
    
  }
}

}


//------------------------------------------------------------------------------------------------
void PH4_calibration() {
                   
// Settings KNOB2, sub-menu 2 (LONG PRESS) - Starting the PH calibration process. 
              
  //PH4 calibration point 1
            
                  //Serial.println("");
                  //Serial.println("Calibration process of PH probe. PART 1 - PH4!");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("PH PROBE CALIB. PH4:");
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Dive PH Probe into the PH4 buffer solution and wait. ");
                  //Serial.println("");
                  lcd.setCursor(0,2);
                  lcd.print("!PROBE to PH4 buff.!");
                  lcd.setCursor(10,3);
                  lcd.print("-300+");   //analog rav volts = 3550mV
                  delay(300000);
                  Serial.println("Measuring in progress --------> ---------------->>");
                  lcd.setCursor(0,2);
                  lcd.print(">>....measuring...>>");
                  delay(5000);
                      
                  float PH4_F = 0.0;
                  float PH4_S = 0.0;

                  //Serial.println(" .... taking first 10000 samples of PH4 buffer.");
                  
                  for (int f = 0; f < 100; f++) {
                    PH4_F += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                    delay(500);
                  }
                  PH4_F = (PH4_F / 100.0);
                  Serial.println(PH4_F);
                  delay(500);

                  //Serial.println(" .... taking secound 10000 samples of PH4 buffer for their comparsion.");
                  
                  for (int s = 0; s < 100; s++) {
                    PH4_S += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                    delay(500);
                  }
                  PH4_S = (PH4_S / 100.0);
                  Serial.println(PH4_S);
                  delay(500);
                    
                  if ( ( (PH4_F - PH4_S) >= 5.0 && (PH4_S - PH4_F) < 0.0 ) || ( (PH4_S - PH4_F) >= 5.0 && (PH4_F - PH4_S) < 0.0 ) ); {
                    
                    //reset both variables to zero point and start again.
                    PH4_F = 0.0;
                    PH4_S = 0.0;
                    
                  while ( ( (PH4_F - PH4_S) >= 5.0 && (PH4_S - PH4_F) < 0.0 ) || ( (PH4_S - PH4_F) >= 5.0 && (PH4_F - PH4_S) < 0.0 ) ); {
                      
                      //Serial.println(" Value spread is to high (more then 5 points). Repeating whole messurement ...");
                      //Serial.println(" ");
                      //Serial.println(" .... taking another 10000 samples of PH4 buffer.");    
                      
                        for (int f = 0; f < 100; f++) {
                          PH4_F += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                          delay(500);
                  }
                    PH4_F = (PH4_F / 100.0);
                    Serial.println(PH4_F);
                    
                      //Serial.println(" .... taking another 10000 samples of PH4 buffer for their comparsion.");

                        for (int s = 0; s < 100; s++) {
                          PH4_S += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                          delay(500);
                  }
                    PH4_S = (PH4_S / 100.0);
                    Serial.println(PH4_S);
                      
                  }
                  }
                  //Serial.println(" Checking mesured value. Done!");
                  lcd.setCursor(0,3);
                  lcd.print("!Calibration...DONE!");
                      
                  PH4_refV = ( (PH4_F + PH4_S) / 2.0);
    
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Put PH probe OUT from PH4 buffer solution. ");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("PH PROBE CALIB. PH4:");
                  lcd.setCursor(3,1);
                  lcd.print("FINISHED!");
                  delay(500);
                  Serial.println("PH4 reference value: ");
                  Serial.print(PH4_refV);
                  lcd.setCursor(3,2);
                  lcd.print("Reference: ");
                  lcd.setCursor(14,2);
                  lcd.print(PH4_refV);
                  delay(1000);

                    // Writing PH4 reference coltage value and then continue to PH7
                    while (digitalRead(pin_KB2) == HIGH) {    //HIGH = not-pressed                    
                        delay(50);
                        //Serial.println("");
                        //Serial.println("    Press KB2 (PH button) to save this value!");
                        //Serial.println("");
                        lcd.setCursor(0,3);
                        lcd.print("Press to write ...");
                        delay(50);
                        } 
                    
                    //Serial.print("    Pressed! Saving new PH4 value and switching to PH7 part!");
                    //Serial.println();
                    //-----------------
                    lcd.clear();
                    lcd.setCursor(0,1);
                    lcd.print("SAVING new value! ");
                    lcd.setCursor(10,3);
                    lcd.print(PH4_refV);
                    delay(1000);
                    lcd.clear();
                    //-----------------
                    
                    //PH4 referance.Forcing to complex value and casting from float to long.
                    long PH4_refVf = (long)(PH4_refV * 1000.0);
                    
                    EEPROMWritelong(PH4RV_addr, PH4_refVf);
                        
                    delay(500);
                    longK2_PH4 = false;                    
}

//------------------------------------------------------------------------------------------------
void PH7_calibration() {

// Settings KNOB2, sub-menu 2 (LONG PRESS) - Starting the PH calibration process. 
              
  //PH7, calibration point 2

                  //Serial.println("");
                  //Serial.println("Calibration process of PH probe. PART 2 - PH7!");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("PH PROBE CALIB. PH7:");
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Dive PH Probe into the PH7 buffer solution and wait. ");
                  //Serial.println("");
                  lcd.setCursor(0,2);
                  lcd.print("!PROBE to PH7 buff.!");
                  lcd.setCursor(10,3);
                  lcd.print("-430+");
                  delay(300000);
                  Serial.println("Measuring in progress --------> ---------------->>");
                  lcd.setCursor(0,2);
                  lcd.print(">>....measuring...>>");
                  delay(5000);
                      
                  float PH7_F = 0.0;
                  float PH7_S = 0.0;

                  //Serial.println(" .... taking first 10000 samples of PH7 buffer.");
                  
                  for (int f = 0; f < 100; f++) {
                    PH7_F += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                    delay(500);
                  }
                  PH7_F = (PH7_F / 100.0);
                  Serial.println(PH7_F);
                  delay(500);

                  //Serial.println(" .... taking secound 10000 samples of PH7 buffer for their comparsion.");
                  
                  for (int s = 0; s < 100; s++) {
                    PH7_S += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                    delay(500);
                  }
                  PH7_S = (PH7_S / 100.0);
                  Serial.println(PH7_S);
                  delay(500);
                  
                  if ( ( (PH7_F - PH7_S) >= 5.0 && (PH7_S - PH7_F) < 0.0 ) || ( (PH7_S - PH7_F) >= 5.0 && (PH7_F - PH7_S) < 0.0 ) ); {

                    //reset both variables to zero point
                    PH7_F = 0.0;
                    PH7_S = 0.0;
                  
                  while ( ( (PH7_F - PH7_S) >= 5.0 && (PH7_S - PH7_F) < 0.0 ) || ( (PH7_S - PH7_F) >= 5.0 && (PH7_F - PH7_S) < 0.0 ) ); {
                      
                      //Serial.println(" Value spread is to high (more then 5 points). Repeating whole messurement ...");
                      //Serial.println(" ");
                      //Serial.println(" .... taking another 10000 samples of PH7.");
                        
                        for (int f = 0; f < 100; f++) {
                          PH7_F += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                          delay(500);
                  }
                    PH7_F = (PH7_F / 100.0);
                    Serial.println(PH7_F);

                      //Serial.println(" .... taking another 10000 samples of PH7 for coparsion.");
                      
                        for (int s = 0; s < 100; s++) {
                          PH7_S += (1023.0 - as100.analogReadSmooth(pin_PHprobe) );  
                          delay(500);
                  }
                    PH7_S = (PH7_S / 100.0);
                    Serial.println(PH7_S);
                      
                  }
                  }
                  //Serial.println(" Checking mesured value. Done!");
                  lcd.setCursor(0,3);
                  lcd.print("!Calibration...DONE!");
                  
                  PH7_refV = ( (PH7_F + PH7_S) / 2.0);
    
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Put PH probe OUT from PH7 buffer solution. ");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("PH PROBE CALIB. PH7:");
                  lcd.setCursor(3,1);
                  lcd.print("FINISHED!");
                  delay(500);
                  Serial.println("PH7 reference value: ");
                  Serial.print(PH7_refV);
                  lcd.setCursor(3,2);
                  lcd.print("Reference: ");
                  lcd.setCursor(14,2);
                  lcd.print(PH7_refV);
                  delay(1000);

                    // Writing PH7 reference coltage value and finishing calibration process.
                    while (digitalRead(pin_KB2) == HIGH) {    //HIGH = not-pressed
                        delay(50);
                        //Serial.println("");
                        //Serial.println("    Press KB2 (PH button) to save this value!");
                        //Serial.println("");
                        lcd.setCursor(0,3);
                        lcd.print("Press to write ...");
                        delay(50);
                        } 
                    
                    //Serial.print("    Pressed! Saving new PH7 value and finishing calibration process!");
                    //Serial.println();
                    //-----------------
                    lcd.clear();
                    lcd.setCursor(0,1);
                    lcd.print("SAVING new value! ");
                    lcd.setCursor(10,3);
                    lcd.print(PH7_refV);
                    delay(1000);
                    lcd.clear();
                    //-----------------
                    
                    //PH7 referance.Forcing to complex value and casting from float to long.
                    long PH7_refVf = (long)(PH7_refV * 1000.0);
                    
                    EEPROMWritelong(PH7RV_addr, PH7_refVf);
                    
                    delay(500);
                    longK2_PH7 = false;
}


//------------------------------------------------------------------------------------------------
void ORP475_calibration() {

    //Redox Probe calibration process. ORP 475mV buffer solution needed.

                  //Serial.println("");
                  //Serial.println("Calibration process of ORP probe. PART 1 - ORP 475mV");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("ORP PR. CALI. 475mV:");
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Dive ORP Probe into the 475mV buffer solution and wait. Expected value (+/- 1620mV).");
                  //Serial.println("");
                  lcd.setCursor(0,2);
                  lcd.print("!PROBE to 475 buff.!");
                  lcd.setCursor(10,3);
                  lcd.print("-1620+");
                  delay(120000);
                  Serial.println("Measuring in progress --------> ---------------->>");
                  lcd.setCursor(0,2);
                  lcd.print(">>....measuring...>>");
                  delay(5000);
                      
                  float ORP475_F = 0.0;
                  float ORP475_S = 0.0;

                  //Serial.println(" .... taking first 1000 samples of ORP 475mV buffer.");
                  
                  for (int f = 0; f < 10; f++) {
                    ORP475_F += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                    delay(500);
                  }
                  ORP475_F = (ORP475_F / 10.0);
                  Serial.println(ORP475_F);
                  delay(500);

                  //Serial.println(" .... taking secound 1000 samples of ORP 475mV buffer for their comparsion.");
                  
                  for (int s = 0; s < 10; s++) {
                    ORP475_S += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                    delay(500);
                  }
                  ORP475_S = (ORP475_S / 10.0);
                  Serial.println(ORP475_S);
                  delay(500);

                  if ( ( (ORP475_F - ORP475_S) >= 10.0 && (ORP475_S - ORP475_F) < 0.0 ) || ( (ORP475_S - ORP475_F) >= 10.0 && (ORP475_F - ORP475_S) < 0.0) ); {

                    //reset both variables to zero point and start again.
                    ORP475_F = 0.0;
                    ORP475_S = 0.0;
                  
                  while ( ( (ORP475_F - ORP475_S) >= 10.0 && (ORP475_S - ORP475_F) < 0.0 ) || ( (ORP475_S - ORP475_F) >= 10.0 && (ORP475_F - ORP475_S) < 0.0) ); {

                      //Serial.println(" Value spread is to high (more then 10 mV). Repeating whole messurement ...");
                      //Serial.println(" ");
                      //Serial.println(" .... taking another 1000 samples of ORP 475mV buffer.");    
                      
                        for (int f = 0; f < 10; f++) {
                          ORP475_F += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                          delay(500);
                  }
                    ORP475_F = (ORP475_F / 10.0);
                    Serial.println(ORP475_F);
                    
                      //Serial.println(" .... taking another 1000 samples of ORP 475mV buffer for their comparsion.");

                        for (int s = 0; s < 10; s++) {
                          ORP475_S += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                          delay(500);
                  }
                    ORP475_S = (ORP475_S / 10.0);
                    Serial.println(ORP475_S);
                                                           
                  }
                  }
                  //Serial.println(" Checking mesured value. Done!");
                  lcd.setCursor(0,3);
                  lcd.print("!Calibration...DONE!");

                  ORP475_refmV = ( (ORP475_F + ORP475_S) / 2.0);
    
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Put ORP probe OUT from 475mV buffer solution.");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("ORP PR. CALI. 475mV:");
                  lcd.setCursor(3,1);
                  lcd.print("FINISHED!");
                  delay(500);
                  Serial.println("ORP 475mV reference value: ");
                  Serial.print(ORP475_refmV);
                  lcd.setCursor(3,2);
                  lcd.print("Reference: ");
                  lcd.setCursor(14,2);
                  lcd.print(ORP475_refmV);
                  delay(1000);

                    // Writing ORP 475mV reference voltage value and then continue to ORP 650mV
                    while (digitalRead(pin_KB3) == HIGH) {    //HIGH = not-pressed                    
                        delay(50);
                        //Serial.println("");
                        //Serial.println("    Press KB3 (SOLAR/ORP button) to save this value!");
                        //Serial.println("");
                        lcd.setCursor(0,3);
                        lcd.print("Press to write ...");
                        delay(50);
                        } 
                    
                    //Serial.print("    Pressed! Saving new ORP 475 value and switching to ORP 650 part!");
                    //Serial.println();
                    //-----------------
                    lcd.clear();
                    lcd.setCursor(0,1);
                    lcd.print("SAVING new value! ");
                    lcd.setCursor(10,3);
                    lcd.print(ORP475_refmV);
                    delay(1000);
                    lcd.clear();
                    //-----------------
                    
                    //ORP 475 referance. Forcing to complex value and casting from float to long.
                    long ORP475_refmVf = (long)(ORP475_refmV * 100.0);
                    
                    EEPROMWritelong(ORP475RV_addr, ORP475_refmVf);
                        
                    delay(500);
                    longK3_ORP475 = false;

}


//------------------------------------------------------------------------------------------------
void ORP650_calibration() {

    //Redox Probe calibration process. ORP 650mV buffer solution needed.

                  delay(2000);
                  //Serial.println("");
                  //Serial.println("Calibration process of ORP probe. PART 2 - ORP 650mV!");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("ORP PR. CALI. 650mV:");
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Dive ORP Probe into the 650mV buffer solution and wait. Expected value (+/- 2100mV).");
                  //Serial.println("");
                  lcd.setCursor(0,2);
                  lcd.print("!PROBE to 650 buff.!");
                  lcd.setCursor(10,3);
                  lcd.print("-2100+");
                  delay(120000);
                  Serial.println("Measuring in progress --------> ---------------->>");
                  lcd.setCursor(0,2);
                  lcd.print(">>....measuring...>>");
                  delay(5000);
                      
                  float ORP650_F = 0.0;
                  float ORP650_S = 0.0;
                  
                  //Serial.println(" .... taking first 1000 samples of ORP 650mV buffer.");
                  
                  for (int f = 0; f < 10; f++) {
                    ORP650_F += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                    delay(500);
                  }
                  ORP650_F = (ORP650_F / 10.0);
                  Serial.println(ORP650_F);
                  delay(500);

                  Serial.println(" .... taking secound 1000 samples of ORP 650mV buffer for their comparsion.");
                  
                  for (int s = 0; s < 10; s++) {
                    ORP650_S += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                    delay(500);
                  }
                  ORP650_S = (ORP650_S / 10.0);
                  Serial.println(ORP650_S);
                  delay(500);

                  if ( ( (ORP650_F - ORP650_S) >= 10.0 && (ORP650_S - ORP650_F) < 0.0 ) || ( (ORP650_S - ORP650_F) >= 10.0 && (ORP650_F - ORP650_S) < 0.0) ); {

                    //reset both variables to zero point and start again.
                    ORP650_F = 0.0;
                    ORP650_S = 0.0;
                  
                  while ( ( (ORP650_F - ORP650_S) >= 10.0 && (ORP650_S - ORP650_F) < 0.0 ) || ( (ORP650_S - ORP650_F) >= 10.0 && (ORP650_F - ORP650_S) < 0.0) ); {

                      //Serial.println(" Value spread is to high (more then 10 mV). Repeating whole messurement ...");
                      //Serial.println(" ");
                      //Serial.println(" .... taking another 1000 samples of ORP 650mV buffer.");    
                      
                        for (int f = 0; f < 10; f++) {
                          ORP650_F += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                          delay(500);
                  }
                    ORP650_F = (ORP650_F / 10.0);
                    Serial.println(ORP650_F);
                    
                      //Serial.println(" .... taking another 1000 samples of ORP 650mV buffer for their comparsion.");

                        for (int s = 0; s < 10; s++) {
                          ORP650_S += ( (as50.analogReadSmooth(pin_ORPprobe) * (5.0/1024.0)) * 1000.0);
                          delay(500);
                  }
                    ORP650_S = (ORP650_S / 10.0);
                    Serial.println(ORP650_S);
                                                           
                  }
                  }
                  //Serial.println(" Checking mesured value. Done!");
                  lcd.setCursor(0,3);
                  lcd.print("!Calibration...DONE!");

                  ORP650_refmV = ( (ORP650_F + ORP650_S) / 2.0);
    
                  delay(1000);
                  //Serial.println("");
                  //Serial.println("Put PH probe out from 650mV buffer solution. ");
                  lcd.clear();
                  lcd.setCursor(0,0);
                  lcd.print("ORP PR. CALI. 475mV:");
                  lcd.setCursor(3,1);
                  lcd.print("FINISHED!");
                  delay(500);
                  Serial.println("ORP 650mV reference value: ");
                  Serial.print(ORP650_refmV);
                  lcd.setCursor(3,2);
                  lcd.print("Reference: ");
                  lcd.setCursor(14,2);
                  lcd.print(ORP650_refmV);
                  delay(1000);

                    // Writing ORP 650mV reference voltage value and finishing calibration process.
                    while (digitalRead(pin_KB3) == HIGH) {    //HIGH = not-pressed
                        delay(50);
                        //Serial.println("");
                        //Serial.println("    Press KB3 (SOLAR/ORP button) to save this value!");
                        //Serial.println("");
                        lcd.setCursor(0,3);
                        lcd.print("Press to write ...");
                        delay(50);
                        } 
                    
                    //Serial.print("    Pressed! Saving new ORP 650 value and finishing calibration process!");
                    //Serial.println();
                    //-----------------
                    lcd.clear();
                    lcd.setCursor(0,1);
                    lcd.print("SAVING new value! ");
                    lcd.setCursor(10,3);
                    lcd.print(ORP650_refmV);
                    delay(1000);
                    lcd.clear();
                    //-----------------
                    
                    //ORP 650 referance.Forcing to complex value and casting from float to long.
                    long ORP650_refmVf = (long)(ORP650_refmV * 100.0);
                    
                    EEPROMWritelong(ORP650RV_addr, ORP650_refmVf);
                    
                    delay(500);
                    longK3_ORP650 = false;
                    
}


//------------------------------------------------------------------------------------------------
void info_display() {

    //percentage calculation (filtering hours in current day - 100% if it will reach RTSP_eprom trigger / set-point)
   HD_percent = ( (RT_counter / 60.0) / (RT_eprom / 100.0) );
  
// ACTUAL INFORMATION ONTO I2C LCD
  //  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ta ");
  lcd.print(T_air);  
  lcd.setCursor(0,1);
  lcd.print("Tp ");
  lcd.print(T_pool);
  lcd.setCursor(0,2);
  lcd.print("Ts ");
  lcd.print(T_solar);  
  lcd.setCursor(0,3);
  lcd.print("Tr ");
  lcd.print(TS_eprom);
  lcd.setCursor(11,0);
  lcd.print("PH  ");
  lcd.print(PH_cal);  
  lcd.setCursor(11,1);
  lcd.print("ORP ");
  lcd.print(ORP_cal);  
  //lcd.print("PHr ");
  //lcd.print(PHSP_eprom);  
  lcd.setCursor(11,2);
  lcd.print("BAR ");
  lcd.print(BAR_value);  
  lcd.setCursor(11,3);
  lcd.print("H%D ");
  lcd.print(HD_percent );
  //lcd.print(RT_counter / 60);
  // INTO HW SERIAL0
  Serial.print("T-P: ");
  Serial.print(T_pool); 
  Serial.print(" | ");
  Serial.print("T-S: ");
  Serial.print(T_solar);
  Serial.print(" | ");
  Serial.print("T-A: ");
  Serial.print(T_air);
  Serial.print(" + ");
  Serial.print("H-A: ");
  Serial.print(H_air);
  Serial.print(" | ");
  Serial.print("TS-SP: ");
  Serial.print(TS_eprom);
  Serial.print(" | ");
  Serial.print("TP-MAX: ");
  Serial.print(TP_max);
  Serial.print(" | ");
  Serial.print("PH-SP: ");
  Serial.print(PHSP_eprom);
  Serial.print(" | ");
  Serial.print("PH: ");
  Serial.print(PH_cal);
  Serial.print(" | ");
  Serial.print("PH-Doz: ");
  Serial.print(DOZE_running);
  Serial.print(" | ");
  Serial.print("ORP: ");
  Serial.print(ORP_cal);
  Serial.print(" | ");
  Serial.print("HD-SP: ");
  Serial.print(RT_eprom);
  Serial.print(" | ");
  Serial.print("H%D: ");
  Serial.print(HD_percent);
  Serial.print(" | ");
  Serial.print("PUMP: ");
  Serial.print(PUMP_state);
  Serial.print(" | ");
  Serial.print("BAR: ");
  Serial.print(BAR_value);
  Serial.print(" | ");
  Serial.print("SUN: ");
  Serial.print(LDR_percent);
  Serial.print(" | ");
  Serial.println();  
}

