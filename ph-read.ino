#include <AnalogSmooth.h>

// Window size for PH Probe sampling, 10
AnalogSmooth as10 = AnalogSmooth(10);

// Window size for PH Probe calibration, range from 1 - 500
AnalogSmooth as100 = AnalogSmooth(100);

static int pin_PHprobe = A0;
static int pin_KB1 = 22;
static int pin_KB2 = 25;

float PH4_rawV = 290.0;
float PH7_rawV = 390.0;
//10 = 1150
float PH_constM;
float PH_constB;
float RawPHvalue;
float PH_cal;

void setup() {
  // put your setup code here, to run once:

pinMode(pin_KB1, INPUT);
pinMode(pin_KB2, INPUT);

pinMode(pin_PHprobe, INPUT);

Serial.begin(9600);

}

float 

void loop() {
  // put your main code here, to run repeatedly:

//PH formula: "y = mx + b"
//
//m = (y2-y1) / (x2-x1)
//m = ((7-4) / (PH7_rawV - PH4_rawV))
//
//b = (y1) - m*(x1)
//b = 4 - PH_constM*PH4_rawV
//
//pH (calibrated) = m(analogReadings) + b

//for (int i = 0; i <= 10; i++) {
//    RawPHvalue = (1023.0 - analogRead(pin_PHprobe) );
//      delay(200);
//      }
//RawPHvalue = RawPHvalue / 10;

  RawPHvalue = 0.0;
  RawPHvalue = as10.analogReadSmooth(pin_PHprobe);
  RawPHvalue = ((RawPHvalue * (5.0/1024.0)*1000.0));  

  Serial.print("RAW hodnota PH (mV): ");
  Serial.print(RawPHvalue);
  Serial.println(" ");
  delay(500);
  //RawPHvalue = 0.0;

PH_constM = ( (7.0 - 4.0)/(PH7_rawV-PH4_rawV) );

  Serial.print("Konstanta M: ");
  Serial.print(PH_constM);
  Serial.println(" ");
  delay(500);

PH_constB = (4.0-(PH_constM*PH4_rawV));

Serial.print("Konstanta B: ");
  Serial.print(PH_constB);
  Serial.println(" ");
  delay(500);
  
PH_cal = ((PH_constM*RawPHvalue)+PH_constB);
  
  Serial.println("============================= ");
  Serial.print("FINALNI HODNOTA PH: ");
  Serial.print(PH_cal);
  Serial.println("");
  Serial.print("============================= ");
  Serial.println(" ");
  delay(1000);

if ( digitalRead(pin_KB1) == LOW ) {
  Serial.print("Calibracni proces PH sondy. Pro pokracovani potvrd.");
  delay(50);
  if ( digitalRead(pin_KB1) == LOW ) {
    Serial.print("Dive PH Probe into the PH4 buffer solution: ");
    delay(2000);
    Serial.print("Calibration in progress ------------------> ----------------------->>");
    delay(500);
    
    float PH4_10 = 0.0;
    float PH4_100 = 0.0;

    PH4_10 = (1023.0 - as10.analogReadSmooth(pin_PHprobe) );
    delay(1000);
    PH4_100 = (1023.0 - as100.analogReadSmooth(pin_PHprobe) );
    
    while ( (PH4_10 - PH4_100) >= 5.0 ); {
          delay(1000);
          PH4_10 = (1023.0 - as10.analogReadSmooth(pin_PHprobe) );
          delay(1000);
          PH4_100 = (1023.0 - as100.analogReadSmooth(pin_PHprobe) );
          
            delay(100);
            Serial.println(" Checking mesured value. Done!");          
    }
    //Smoothing average = ( average * 9 + a ) / 10
    PH4_rawV = ( ((PH4_100*9.0) + PH4_10) / 10.0);
    
  delay(1000);
  Serial.println("Now put PH probe out from PH4 buffer solution. ");
  delay(100);
  Serial.println("Raw PH4 Calibrated value: ");
  Serial.print(PH4_rawV);
  Serial.println("_______________________ ");
  delay(2000);
  }
}

if ( digitalRead(pin_KB2) == LOW ) {
  delay(50);
  if ( digitalRead(pin_KB2) == LOW ) {
    Serial.print("Dive PH Probe into the PH7 buffer solution: ");
    delay(2000);
    Serial.print("Calibration in progress ------------------> ----------------------->>");
    delay(500);
    
    float PH7_10 = 0.0;
    float PH7_100 = 0.0;

    PH7_10 = (1023.0 - as10.analogReadSmooth(pin_PHprobe) );
    delay(1000);
    PH7_100 = (1023.0 - as100.analogReadSmooth(pin_PHprobe) );
    
    while ( (PH7_10 - PH7_100) >= 5.0 ); {
          delay(1000);
          PH7_10 = (1023.0 - as10.analogReadSmooth(pin_PHprobe) );
          delay(1000);
          PH7_100 = (1023.0 - as100.analogReadSmooth(pin_PHprobe) );
          
            delay(100);
            Serial.println(" Checking mesured value. Done!");          
    }
    //Smoothing average = ( average * 9 + a ) / 10
    PH7_rawV = ( ((PH7_100*9.0) + PH7_10) / 10.0);
    
  delay(1000);
  Serial.println("Now put PH probe out from buffer solution. ");
  delay(100);
  Serial.println("Raw PH7 calibrated value: ");
  Serial.print(PH7_rawV);
  Serial.println("_______________________ ");
  delay(2000);
  }  
}


}
