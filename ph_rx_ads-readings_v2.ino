#include <Wire.h>
#include <Adafruit_ADS1015.h>

// *** OBJECTS ***
Adafruit_ADS1015 ads1015RX(0x48);   
Adafruit_ADS1015 ads1015PH(0x49);   

// *** PH reference
float PH4_mV = 2887.4;      //PH reference voltage
float PH7_mV = 2424.5;
                                
// ** PH **
float PH_constM;
float PH_constB;
float RawPHvalue_smooth;
float RawPHvalue_actual;
float PH_cal;
//***

// *** ORP reference
float RX475_mV = 0.0;     //ORP reference voltage
float RX650_mV = 0.0;

// ** ORP **
float RX_constM;
float RX_constB;
float RawRXvalue_smooth;
float RawRXvalue_actual;
float RX_cal;
//***

float MyTemp = 21.5;

String _SerialRead = "";    //SerialReadings-cache

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  delay(50);
  
  ads1015RX.begin();
  delay(50);
  ads1015PH.begin();

Serial.println("Taking Readings from PH Sensor");

}

void loop() {
  // put your main code here, to run repeatedly:

  // receive data from serial console:
  if (Serial.available() > 0) {
      
      Serial.println("");
      Serial.print("Reading Seria1 input, looking for command........");
         
    while (Serial.available() > 0) {          //Only run when there is data available
  
    _SerialRead += char(Serial.read());       //Here every received char will be
                                        
    if (_SerialRead == "PH4") {               //Checks string and parse received command!

      PH4_mV = PHX_calibration("ph");
      
      Serial.println("got-ph4!");
      Serial.println("");
      
      _SerialRead = "";                 //Do something then clear the string
    }
       else if (_SerialRead == "PH7") {               //Checks string and parse received command!

      PH7_mV = PHX_calibration("ph");
      
      Serial.print("got-ph7!");
      Serial.println(" ");
      
      _SerialRead = "";                 //Do something then clear the string
    }
        else if (_SerialRead == "RX475") {               //Checks string and parse received command!

        RX475_mV = PHX_calibration("rx");
        
        Serial.print("got-rx475!");
        Serial.println(" ");
             
        _SerialRead = "";                 //Do something then clear the string
    }
        else if (_SerialRead == "RX650") {               //Checks string and parse received command!

        RX650_mV = PHX_calibration("rx");
        
        Serial.print("got-rx650!");
        Serial.println(" ");
        
        _SerialRead = "";                 //Do something then clear the string
    }     
   }
  } 

// ORP/PH formula: "y = mx + b"
//
//m = (y2-y1) / (x2-x1)
//m = ((7-4) / (PH7_rawV - PH4_rawV))
//
//b = (y1) - m*(x1)
//b = 4 - PH_constM*PH4_rawV
//
//pH (calibrated) = m(analogReadings) + b


  RawPHvalue_actual = 0.0;
  RawPHvalue_smooth = 0.0;
  
  //-------------------------------------------------------------------------------------------------- 
  RawPHvalue_smooth = PHX_readings("ph", 10, 1);                        //already converted
  RawPHvalue_actual = (ads1015PH.readADC_SingleEnded(0) * 3.001466);    //12bits;                         
  //--------------------------------------------------------------------------------------------------  

  RawRXvalue_smooth = 0.0;
  RawRXvalue_actual = 0.0;
  
int RawRXval_buf[20];

  //--------------------------------------------------------------------------------------------------  
  RawRXvalue_smooth = PHX_readings("rx", 10, 1);                             //already converted
  RawRXvalue_actual = (ads1015RX.readADC_SingleEnded(0) * 3.001466);  //12bits;  
  //-------------------------------------------------------------------------------------------------- 

  Serial.println(" ");
  Serial.println("============================================================ ");
  Serial.print("RAW hodnota PH-smoothed / RX-smoothed (mV): ");
  Serial.print(RawPHvalue_smooth);
  Serial.print(" / ");
  Serial.print(RawRXvalue_smooth);
  Serial.println("");
  //-----------------------------------------------------------
  Serial.print("RAW hodnota PH-once / RX-once (mV):         ");
  Serial.print(RawPHvalue_actual);
  Serial.print(" / ");
  Serial.print(RawRXvalue_actual);
  Serial.println(" ");
  
PH_constM = ( (7.0 - 4.0)/(PH7_mV - PH4_mV) );
RX_constM = ( (650.0 - 450.0)/(RX650_mV - RX475_mV));

  Serial.println(" ");
  Serial.print("Konstanty M, PH / RX:                       ");
  Serial.print(PH_constM);
  Serial.print("  /  ");
  Serial.print(RX_constM);
  Serial.println(" ");
  delay(10);

PH_constB = (4.0-(PH_constM * PH4_mV));
RX_constB = (475.0-(RX_constM * RX475_mV));

Serial.print("Konstanta B, PH / RX:                       ");
  Serial.print(PH_constB);
  Serial.print(" / ");
  Serial.print(RX_constB);
  Serial.println(" ");
  delay(10);
  
PH_cal = ((PH_constM * RawPHvalue_smooth) + PH_constB);
RX_cal = ((RX_constM * RawRXvalue_smooth) + RX_constB);

  Serial.println("============================================================ ");
  Serial.print("FINALNI HODNOTY, PH / RX:                   ");
  Serial.print(PH_cal);
  Serial.print(" / ");
  Serial.print(RX_cal);

  // Final temperature compensation part, values mapped based on this article - table in the midle of this page:
  // https://chem.libretexts.org/Core/Physical_and_Theoretical_Chemistry/Acids_and_Bases/Acids_and_Bases_in_Aqueous_Solutions/The_pH_Scale/Temperature_Dependence_of_the_pH_of_pure_Water
  
  if (MyTemp >= 26.0 && MyTemp <= 40.0) {

    int tempcomp = map( ((int)MyTemp * 100), 2500, 4000, 0, 23);    //offset map: 0 at 25C and -0,23 at 40C
    PH_cal -= ( (float)tempcomp / 100.0);

      Serial.println("Compensant(-): ");
      Serial.print(PH_cal);
    }
    else if (MyTemp >= 0.0 && MyTemp <= 24.0) {                     //offset map: 0.47 at 0C and 0 at 25C

      int tempcomp = map( ((int)MyTemp * 100), 0, 2500, 47, 0); 
      PH_cal += ( (float)tempcomp / 100.0);
      
      Serial.println("Compensant(+): ");
      Serial.print(PH_cal);
      }
      else {
        Serial.println("Compensant(=): ");
        }
  
  Serial.println("");
  
  delay(880);

} //loop




//------------------------------------------------------------------------------------------------ ADS Calibration Function

float PHX_calibration(char cal[2]) {                  //expecting "ph" od "rx" string to be passed to this and PHX_reading() func.
              
  float PHX_refV = 0.0; //calculated smooth value, it will be returned
  
  float PHX_F;      //first sample portion to be compared with secound one
  float PHX_S;      //secound sample to be compared with first one

  Serial.println("Put probe into buffer, wait few mins");
  delay(120000);

 do {
  
   Serial.println("");
   Serial.println("Calculation --> --> ");

    delay(5000);

   Serial.println("...first portion of (100smpls/10ms).");
    PHX_F = PHX_readings(cal, 100, 10);
  
   Serial.println(PHX_F, 4);
    delay(500);

   Serial.println("...secound portion (100smpls/10ms).");
    PHX_S = PHX_readings(cal, 100, 10);

    Serial.println(PHX_S, 4);
    delay(500);
 
    } while ( ( ((PHX_F + PHX_S) / 2.0) - PHX_F >= 0.5) || ( ((PHX_F + PHX_S) / 2.0) - PHX_S >= 0.5) ); 
                        
  PHX_refV = ( (PHX_F + PHX_S) / 2.0);    //getting average value from both blocks
  delay(1000);

 Serial.println("");
 Serial.println("Put out probe from buffer. ");
 Serial.println("");
 Serial.println("Final obtained value: ");
 Serial.println(PHX_refV);
 
 delay(1000);

 Serial.print("Calibrated value received ................");
 
 //returning smooth value in mV
 return PHX_refV;

} //end-od calibration

//------------------------------------------------------------------------------------------------ ADS Reading Function

float PHX_readings(char sel[2], int sum, int dly) {

 float Raw_smooth = 0.0;  //smooth value, it will be returned from this function

 long Raw_buf[sum];
 long Raw_sum = 0;

 // re-reding of analog values from I2C [ ADS modules under PH-analog modules ]
 
 for (int i = 0; i < sum; i++) {        //Geting requested portion of samples from the sensor.

  if (sel == "ph") {                  //PH probe selection for readings

    Raw_buf[i] = ads1015PH.readADC_SingleEnded(0);
    delay(dly);                         //Set the delay (period) based on value passed to into

  }
  else if (sel == "rx") {             //ORP-Redox-Rx probe selection for readings
  
    Raw_buf[i] = ads1015RX.readADC_SingleEnded(0);
    delay(dly);                       //Set the delay (period) based on value passed to into      
  }
  else {
    Serial.println("");
    Serial.print("Error: ");
    Serial.println("In PH/RX probe selection!");
  }
                    
 } //smoothing process
  for (int i = 0; i < (sum - 1); i++) {     //sorting all analog data from small to large

    for (int j = i+1; j < sum; j++) {

      if (Raw_buf[i] > Raw_buf[j]) {
            
          int temp = Raw_buf[i];
              Raw_buf[i] = Raw_buf[j];
              Raw_buf[j] = temp;
      } //for
    } //if
  } //for
  
    for (int i = 2; i < (sum - 2); i++) {   //taking the average value from center of sorted sample
    
      Raw_sum += Raw_buf[i];
      }
  
  //Calculation of miliVolts smooth reading (volts must be positive 0-5V)
  Raw_smooth = (((float)Raw_sum / ((float)sum - 4.0) ) * 3.001466);       //convertig 12bits to mV
      
  //returning smooth value in mV
  return Raw_smooth;
  
} // ph-or-orp_reading function



