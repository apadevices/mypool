#include <AnalogSmooth.h>

// Window size for PH Probe sampling, 10
AnalogSmooth as10 = AnalogSmooth(10);

static int pin_ORPprobe = A0;

float ORPA_rawmV = 1850.0;
float ORPB_rawmV = 2420.0;

float ORP_constM;
float ORP_constB;
float RawORPvalue;

float ORP_cal;
long ORP_cal2X;
float ORP_cal2;

void setup() {
  // put your setup code here, to run once:

pinMode(pin_ORPprobe, INPUT);

Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  RawORPvalue = 0.0;
  RawORPvalue = as10.analogReadSmooth(pin_ORPprobe);
  RawORPvalue = ((RawORPvalue * (5.0/1024.0)*1000.0));  

  Serial.print("RAW hodnota ORP (mV): ");
  Serial.print(RawORPvalue);
  Serial.println(" ");
  delay(500);
    
//---------------------------------------------

// -- by the formula
ORP_constM = ( (650.0 - 475.0) / (ORPB_rawmV - ORPA_rawmV) );

  Serial.print("Konstanta M: ");
  Serial.print(ORP_constM);
  Serial.println(" ");
  delay(500);

ORP_constB = (475.0-(ORP_constM*ORPA_rawmV));

Serial.print("Konstanta B: ");
  Serial.print(ORP_constB);
  Serial.println(" ");
  delay(500);
  
ORP_cal = ((ORP_constM*RawORPvalue)+ORP_constB);
  
  Serial.println("============================= ");
  Serial.print("FINALNI HODNOTA ORP: ");
  Serial.print(ORP_cal);
  Serial.println("");
  Serial.print("============================= ");
  Serial.println(" ");
  delay(1000);


/// -- mapping
long RawORP2value = (long)(RawORPvalue * 100.0);
long ORP2A_rawmV = (long)(ORPA_rawmV * 100.0);
long ORP2B_rawmV = (long)(ORPB_rawmV * 100.0);

ORP_cal2X = map(RawORP2value, ORP2A_rawmV, ORP2B_rawmV, 47500, 65000);
ORP_cal2 = (float)(ORP_cal2X / 100);

  Serial.println("============================= ");
  Serial.print("FINALNI HODNOTA ORP2: ");
  Serial.print(ORP_cal2);
  Serial.println("");
  Serial.print("============================= ");
  Serial.println(" ");
  delay(1000);

}
