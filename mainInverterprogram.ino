/* Arduino Fuzzy FB Inverter with IoT Monitoring System 

   Abraham Babtistio
   I0718001 - UNS EE Dept.
   Created    : 12 September 2022
   Processor  : ATmega2560
   Compiler   : Arduino IDE
*/

#include <Wire.h>
#include <stdio.h>
#include <math.h>
#include "Fuzzy.h"
#include<SoftwareSerial.h>
#include <TimerOne.h>
#include <PZEM004Tv30.h>
PZEM004Tv30 pzem(&Serial3);  // Use Serial3 for PZEM COMM
Fuzzy *fuzzy = new Fuzzy();

/*************************
 * Battery Reading
 *************************/
float batreading;
float batvoltage;
float realbatvoltage;

uint16_t feedBackVal;
float A;
volatile double percentMod;
float pulseWidth;

SoftwareSerial wifi(18,19);
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)

int i = 0;
float h = 0;

int sinewave[90] = {0, 36, 71, 107, 142, 178, 213, 247, 282, 316, 350, 383, 416, 448, 480, 511, 542, 572, 601, 630, 658,
                    685, 711, 736, 760, 784, 806, 828, 848, 868, 886, 903, 919, 935, 949, 961, 973, 983, 993, 1001, 1007, 1013, 1017, 1021,
                    1022, 1023, 1022, 1021, 1017, 1013, 1007, 1001, 993, 983, 973, 961, 949, 935, 919, 903, 886, 868, 848, 828, 806, 784, 760, 736,
                    711, 685, 658, 630, 601, 572, 542, 511, 480, 448, 416, 383, 350, 316, 282, 247, 213, 178, 142, 107, 71, 36
                   };

int x = 0; //untuk counting data sinewave
int halfp = 0; //inisial untuk setengah gelombang

  float e_volt = 0;
  float e_volt_pre =0;
  float rate = 0;
  float bacaADC1;
  unsigned long lastTime = 0;
  unsigned long dt = 1000; // dt in milliseconds
  volatile float outputgain = 1;
  float teg;

/****************************************
 * Define Constants for Ubidots
 ****************************************/
namespace {
  bool flow_control = true; // control the flow of the requests
  const char * USER_AGENT = "UbidotsESP8266"; // Assgin the user agent
  const char * VERSION =  "1.0"; // Assign the version 
  const char * METHOD = "POST"; // Set the method
  const char * TOKEN = "BBFF-3Sx9h51erTRYO9Jvw7mvqvbHRWLUYH"; // Assign your Ubidots TOKEN
  const char * DEVICE_NAME = "ESP8266"; // Assign the desire device name 
  const char * DEVICE_LABEL = "Inverter"; // Assign the device label 
  const char * VARIABLE_LABEL_1 = "Tegangan"; // Assign the variable label 
  const char * VARIABLE_LABEL_2 = "Arus"; // Assign the variable label
  const char * VARIABLE_LABEL_3 = "Daya"; // Assign the variable label
  const char * VARIABLE_LABEL_4 = "Energi"; // Assign the variable label 
  const char * VARIABLE_LABEL_5 = "Frekuensi"; // Assign the variable label
  const char * VARIABLE_LABEL_6 = "pf"; // Assign the variable label
  const char * VARIABLE_LABEL_7 = "VBat"; // Assign the variable label
}

char first_command[700]; // command
char telemetry_unit[100]; // response of the telemetry unit

/* Space to store values to send */
char str_sensor1[10];
char str_sensor2[10];
char str_sensor3[10];
char str_sensor4[10];
char str_sensor5[10];
char str_sensor6[10];
char str_sensor7[10];


void setup() {
  Timer1.initialize(111);         //inisialiasi frekuensi pwm
  
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  wifi.begin(115200);
  fuzzyrules();
}

void loop() {
  Timer1.attachInterrupt(pulsa);  //aktifkan interupt
  batreading = analogRead(A8);
  batvoltage = batreading * 14.0/1023;
  realbatvoltage = (0.9714*batvoltage)+0.0521;
  h = 0.1;
  float voltage = pzem.voltage();
     if(voltage != NAN){
         Serial.print("Voltage: "); 
         Serial.print(voltage);
         Serial.println("V");
     } else {
         Serial.println("Error reading voltage");
     }

     float current = pzem.current();
     if(current != NAN){
         Serial.print("Current: ");
         Serial.print(current);
         Serial.println("A");
     } else {
         Serial.println("Error reading current");
     }

     float power = pzem.power();
     if(current != NAN){
         Serial.print("Power: ");
         Serial.print(power);
         Serial.println("W");
     } else {
         Serial.println("Error reading power");
     }

     float energy = pzem.energy();
     if(current != NAN){
         Serial.print("Energy: ");
         Serial.print(energy,3);
         Serial.println("kWh");
     } else {
         Serial.println("Error reading energy");
     }

     float frequency = pzem.frequency();
     if(current != NAN){
         Serial.print("Frequency: ");
         Serial.print(frequency, 1);
         Serial.println("Hz");
     } else {
         Serial.println("Error reading frequency");
     }

     float pf = pzem.pf();
     if(current != NAN){
         Serial.print("PF: ");
         Serial.println(pf);
     } else {
         Serial.println("Error reading power factor");
     }
     Serial.println();
     Serial.print("Error = ");
     Serial.print(e_volt);
     Serial.println();
     Serial.print("CE = ");
     Serial.print(rate);
     Serial.println();     
     Serial.print("Gain = ");
     Serial.print(outputgain);
     Serial.println();

    if (millis() - lastTime  >= dt)   // wait for dt milliseconds
  {
    lastTime = millis();
    //bacaADC1 = random(100, 300); //Koding Testing
    //bacaADC1 = 0;
    bacaADC1 = pzem.voltage();
    if(isnan(bacaADC1)){
      teg = 220;
    }
    else {
      teg = bacaADC1;
    }
    e_volt = 220 - teg;
    if(e_volt > 40) e_volt = 40;
    if(e_volt < -40) e_volt = -40;
    rate = (e_volt-e_volt_pre);
    if(rate > 40) rate = 40;
    if(rate < -40) rate = -40;
    e_volt_pre = e_volt;

    if (current == 0) outputgain = 1;
    
    if (teg < 200 && teg > 225){ 
      goto way1;
    }
    if (teg >= 200 && teg <= 225) {
      goto way2;
    }

    way1:
    
  /*================Section of Fuzzification & Defuzzification====================*/
    fuzzy->setInput(1, e_volt);
    fuzzy->setInput(2, rate);
    fuzzy->fuzzify();

    outputgain = fuzzy->defuzzify(1);
    /*===============End of Section of Fuzzification & Defuzzification==============*/

    way2:
    ;
   }
   
    
     /*
      * Ubidots Variable Conversion
      */
      /* PZEM reading*/ 
    float sensor1 = voltage;
    float sensor2 = current;
    float sensor3 = power;
    float sensor4 = energy;
    float sensor5 = frequency;
    float sensor6 = pf;
    float sensor7 = realbatvoltage;

  /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  dtostrf(sensor1, 4, 2, str_sensor1);
  dtostrf(sensor2, 4, 2, str_sensor2);
  dtostrf(sensor3, 4, 2, str_sensor3);
  dtostrf(sensor4, 4, 2, str_sensor4);
  dtostrf(sensor5, 4, 2, str_sensor5);
  dtostrf(sensor6, 4, 2, str_sensor6);
  dtostrf(sensor7, 4, 2, str_sensor7);

  /* Building first the logger command */ 
  sprintf(first_command, "init#");
  sprintf(first_command, "%s%s/%s|%s|%s|", first_command, USER_AGENT, VERSION, METHOD, TOKEN);
  sprintf(first_command, "%s%s:%s=>", first_command, DEVICE_NAME, DEVICE_LABEL);
  sprintf(first_command, "%s%s:%s", first_command, VARIABLE_LABEL_1, str_sensor1);
  sprintf(first_command, "%s,%s:%s", first_command, VARIABLE_LABEL_2, str_sensor2); // uncomment this line to send sensor 2 values
  sprintf(first_command, "%s,%s:%s", first_command, VARIABLE_LABEL_3, str_sensor3); // uncomment this line to send sensor 3 values
  sprintf(first_command, "%s,%s:%s", first_command, VARIABLE_LABEL_4, str_sensor4); // uncomment this line to send sensor 4 values
  sprintf(first_command, "%s,%s:%s", first_command, VARIABLE_LABEL_5, str_sensor5); // uncomment this line to send sensor 5 values
  sprintf(first_command, "%s,%s:%s", first_command, VARIABLE_LABEL_6, str_sensor6); // uncomment this line to send sensor 6 values
  sprintf(first_command, "%s,%s:%s", first_command, VARIABLE_LABEL_7, str_sensor7); // uncomment this line to send sensor 7 values
  sprintf(first_command, "%s|end#final", first_command);

  //Sending Data to Ubidots
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  /* Prints the command sent */
  Serial.println(first_command);// uncomment this line to print the command
  
  /* Sends the command to the telemetry unit */
  wifi.print(first_command);
  //free(first_command);

  /* Reading the telemetry unit */
  int i = 0;
  while (wifi.available() > 0) {
    telemetry_unit[i++] = (char)wifi.read(); 
  }
  Serial.println(telemetry_unit);
  i = 0; 
  }
}

void fuzzyrules()
{
  /*=========Fuzzy Logic Controller Input & Output Variable Section===============*/

  // Naming Input 1 --> error
  FuzzyInput *error = new FuzzyInput(1);
  // Instantiating a FuzzySet object mfe1
  FuzzySet *mfe1 = new FuzzySet(-60, -40, -40, -20);
  error->addFuzzySet(mfe1);
  
  // Instantiating a FuzzySet object mfe2
  FuzzySet *mfe2 = new FuzzySet(-40, -20, -20, 0);
  error->addFuzzySet(mfe2);
  
  // Instantiating a FuzzySet object mfe3
  FuzzySet *mfe3 = new FuzzySet(-20, 0, 0, 20);
  // Including the FuzzySet into FuzzyInput
  error->addFuzzySet(mfe3);

  // Instantiating a FuzzySet object mfe4
  FuzzySet *mfe4 = new FuzzySet(0, 20, 20, 40);
  // Including the FuzzySet into FuzzyInput
  error->addFuzzySet(mfe4);
  
  // Instantiating a FuzzySet object mfe5
  FuzzySet *mfe5 = new FuzzySet(20, 40, 40, 60);
  // Including the FuzzySet into FuzzyInput
  error->addFuzzySet(mfe5);
  
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyInput(error);


  // Naming Input 2 --> changeoferror
  FuzzyInput *coe = new FuzzyInput(2);
  // Instantiating a FuzzySet object mfce1
  FuzzySet *mfce1 = new FuzzySet(-60, -40, -40, -20);
  coe->addFuzzySet(mfce1);
  
  // Instantiating a FuzzySet object mfe2
  FuzzySet *mfce2 = new FuzzySet(-40, -20, -20, 0);
  coe->addFuzzySet(mfce2);
  
  // Instantiating a FuzzySet object mfe3
  FuzzySet *mfce3 = new FuzzySet(-20, 0, 0, 20);
  // Including the FuzzySet into FuzzyInput
  coe->addFuzzySet(mfce3);

  // Instantiating a FuzzySet object mfe4
  FuzzySet *mfce4 = new FuzzySet(0, 20, 20, 40);
  // Including the FuzzySet into FuzzyInput
  coe->addFuzzySet(mfce4);
  
  // Instantiating a FuzzySet object mfe5
  FuzzySet *mfce5 = new FuzzySet(20, 40, 40, 60);
  // Including the FuzzySet into FuzzyInput
  coe->addFuzzySet(mfce5);
  
  // Including the FuzzyInput into Fuzzy
  fuzzy->addFuzzyInput(coe);

  // Naming Output --> outputg
  FuzzyOutput *outputg = new FuzzyOutput(1);
  
  // Instantiating a FuzzySet object NB
  FuzzySet *NB = new FuzzySet(0.8, 0.8, 0.8, 0.9);
  outputg->addFuzzySet(NB);

  // Instantiating a FuzzySet object
  FuzzySet *NS = new FuzzySet(0.8, 0.9, 0.9, 1);
  outputg->addFuzzySet(NS);
  
  // Instantiating a FuzzySet object
  FuzzySet *Z = new FuzzySet(0.9, 1, 1, 1.1);
  outputg->addFuzzySet(Z);

  // Instantiating a FuzzySet object
  FuzzySet *PS = new FuzzySet(1, 1.1, 1.1, 1.2);
  outputg->addFuzzySet(PS);

  // Instantiating a FuzzySet object
  FuzzySet *PB = new FuzzySet(1.1, 1.2, 1.2, 1.2);
  outputg->addFuzzySet(PB);
    
  // Including the FuzzyOutput into Fuzzy
  fuzzy->addFuzzyOutput(outputg);
    

/*=========End of Fuzzy Logic Controller Input & Output Variable Section===============*/

/*================Fuzzy Logic Controller Rules Section===================*/
//================Error = mfe1=============
  // Building FuzzyRule "IF error = mfe1 AND change of error = mfce1 THEN output = NB"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe1_And_coe_mfce1 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe1_And_coe_mfce1->joinWithAND(mfe1, mfce1);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput1 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput1->addOutput(NB);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, iferror_mfe1_And_coe_mfce1, thenOutput1);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule1);

  // Building FuzzyRule "IF error = mfe1 AND change of error = mfce2 THEN output = NB"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe1_And_coe_mfce2 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe1_And_coe_mfce2->joinWithAND(mfe1, mfce2);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput2 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput2->addOutput(NB);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, iferror_mfe1_And_coe_mfce2, thenOutput2);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule2);

  // Building FuzzyRule "IF error = mfe1 AND change of error = mfce3 THEN output = NS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe1_And_coe_mfce3 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe1_And_coe_mfce3->joinWithAND(mfe1, mfce3);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput3 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput3->addOutput(NS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, iferror_mfe1_And_coe_mfce3, thenOutput3);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule3);

  // Building FuzzyRule "IF error = mfe1 AND change of error = mfce4 THEN output = NS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe1_And_coe_mfce4 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe1_And_coe_mfce4->joinWithAND(mfe1, mfce4);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput4 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput4->addOutput(NS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, iferror_mfe1_And_coe_mfce4, thenOutput4);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule4);

  // Building FuzzyRule "IF error = mfe1 AND change of error = mfce5 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe1_And_coe_mfce5 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe1_And_coe_mfce5->joinWithAND(mfe1, mfce5);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput5 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput5->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, iferror_mfe1_And_coe_mfce5, thenOutput5);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule5);
  
//================Error = mfe2=============
  // Building FuzzyRule "IF error = mfe2 AND change of error = mfce1 THEN output = NB"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe2_And_coe_mfce1 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe2_And_coe_mfce1->joinWithAND(mfe2, mfce1);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput6 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput6->addOutput(NB);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, iferror_mfe2_And_coe_mfce1, thenOutput6);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule6);

  // Building FuzzyRule "IF error = mfe2 AND change of error = mfce2 THEN output = NS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe2_And_coe_mfce2 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe2_And_coe_mfce2->joinWithAND(mfe2, mfce2);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput7 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput7->addOutput(NS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, iferror_mfe2_And_coe_mfce2, thenOutput7);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule7);

  // Building FuzzyRule "IF error = mfe2 AND change of error = mfce3 THEN output = NS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe2_And_coe_mfce3 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe2_And_coe_mfce3->joinWithAND(mfe2, mfce3);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput8 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput8->addOutput(NS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, iferror_mfe2_And_coe_mfce3, thenOutput8);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule8);

  // Building FuzzyRule "IF error = mfe2 AND change of error = mfce4 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe2_And_coe_mfce4 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe2_And_coe_mfce4->joinWithAND(mfe2, mfce4);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput9 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput9->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, iferror_mfe2_And_coe_mfce4, thenOutput9);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule9);

  // Building FuzzyRule "IF error = mfe2 AND change of error = mfce5 THEN output = PS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe2_And_coe_mfce5 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe2_And_coe_mfce5->joinWithAND(mfe2, mfce5);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput10 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput10->addOutput(PS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, iferror_mfe2_And_coe_mfce5, thenOutput10);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule10);
  
//================Error = mfe3=============
  // Building FuzzyRule "IF error = mfe3 AND change of error = mfce1 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe3_And_coe_mfce1 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe3_And_coe_mfce1->joinWithAND(mfe3, mfce1);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput11 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput11->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, iferror_mfe3_And_coe_mfce1, thenOutput11);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule11);

  // Building FuzzyRule "IF error = mfe3 AND change of error = mfce2 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe3_And_coe_mfce2 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe3_And_coe_mfce2->joinWithAND(mfe3, mfce2);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput12 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput12->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, iferror_mfe3_And_coe_mfce2, thenOutput12);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule12);

  // Building FuzzyRule "IF error = mfe3 AND change of error = mfce3 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe3_And_coe_mfce3 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe3_And_coe_mfce3->joinWithAND(mfe3, mfce3);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput13 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput13->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, iferror_mfe3_And_coe_mfce3, thenOutput13);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule13);

  // Building FuzzyRule "IF error = mfe3 AND change of error = mfce4 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe3_And_coe_mfce4 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe3_And_coe_mfce4->joinWithAND(mfe3, mfce4);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput14 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput14->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, iferror_mfe3_And_coe_mfce4, thenOutput14);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule14);

  // Building FuzzyRule "IF error = mfe3 AND change of error = mfce5 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe3_And_coe_mfce5 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe3_And_coe_mfce5->joinWithAND(mfe3, mfce5);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput15 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput15->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, iferror_mfe3_And_coe_mfce5, thenOutput15);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule15);

//================Error = mfe4=============
  // Building FuzzyRule "IF error = mfe4 AND change of error = mfce1 THEN output = NS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe4_And_coe_mfce1 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe4_And_coe_mfce1->joinWithAND(mfe4, mfce1);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput16 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput16->addOutput(NS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, iferror_mfe4_And_coe_mfce1, thenOutput16);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule16);

  // Building FuzzyRule "IF error = mfe4 AND change of error = mfce2 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe4_And_coe_mfce2 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe4_And_coe_mfce2->joinWithAND(mfe4, mfce2);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput17 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput17->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, iferror_mfe4_And_coe_mfce2, thenOutput17);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule17);

  // Building FuzzyRule "IF error = mfe4 AND change of error = mfce3 THEN output = PS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe4_And_coe_mfce3 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe4_And_coe_mfce3->joinWithAND(mfe4, mfce3);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput18 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput18->addOutput(PS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, iferror_mfe4_And_coe_mfce3, thenOutput18);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule17);

  // Building FuzzyRule "IF error = mfe4 AND change of error = mfce4 THEN output = PS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe4_And_coe_mfce4 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe4_And_coe_mfce4->joinWithAND(mfe4, mfce4);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput19 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput19->addOutput(PS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, iferror_mfe4_And_coe_mfce4, thenOutput19);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule19);

  // Building FuzzyRule "IF error = mfe4 AND change of error = mfce5 THEN output = PB"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe4_And_coe_mfce5 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe4_And_coe_mfce5->joinWithAND(mfe4, mfce5);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput20 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput20->addOutput(PB);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, iferror_mfe4_And_coe_mfce5, thenOutput20);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule20);
  
//================Error = mfe5=============
  // Building FuzzyRule "IF error = mfe5 AND change of error = mfce1 THEN output = Z"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe5_And_coe_mfce1 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe5_And_coe_mfce1->joinWithAND(mfe5, mfce1);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput21 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput21->addOutput(Z);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, iferror_mfe5_And_coe_mfce1, thenOutput21);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule21);

  // Building FuzzyRule "IF error = mfe5 AND change of error = mfce2 THEN output = PS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe5_And_coe_mfce2 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe5_And_coe_mfce2->joinWithAND(mfe5, mfce2);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput22 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput22->addOutput(PS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, iferror_mfe5_And_coe_mfce2, thenOutput22);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule22);

  // Building FuzzyRule "IF error = mfe5 AND change of error = mfce3 THEN output = PS"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe5_And_coe_mfce3 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe5_And_coe_mfce3->joinWithAND(mfe5, mfce3);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput23 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput23->addOutput(PS);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, iferror_mfe5_And_coe_mfce3, thenOutput23);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule23);

  // Building FuzzyRule "IF error = mfe5 AND change of error = mfce4 THEN output = PB"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe5_And_coe_mfce4 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe5_And_coe_mfce4->joinWithAND(mfe5, mfce4);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput24 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput24->addOutput(PB);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, iferror_mfe5_And_coe_mfce4, thenOutput24);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule24);

  // Building FuzzyRule "IF error = mfe5 AND change of error = mfce5 THEN output = PB"
  // Instantiating a FuzzyRuleAntecedent objects
  FuzzyRuleAntecedent *iferror_mfe5_And_coe_mfce5 = new FuzzyRuleAntecedent();
  // Creating a FuzzyRuleAntecedent with just a single FuzzySet
  iferror_mfe5_And_coe_mfce5->joinWithAND(mfe5, mfce5);
  // Instantiating a FuzzyRuleConsequent objects
  FuzzyRuleConsequent *thenOutput25 = new FuzzyRuleConsequent();
  // Including a FuzzySet to this FuzzyRuleConsequent
  thenOutput25->addOutput(PB);
  // Instantiating a FuzzyRule objects
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, iferror_mfe5_And_coe_mfce5, thenOutput25);
  // Including the FuzzyRule into Fuzzy
  fuzzy->addFuzzyRule(fuzzyRule25);
  
/*================End of Fuzzy Logic Controller Rules Section===================*/
}

void pulsa() {
  if (x > 90 && halfp == 1) {
  halfp = 0;
  x = 0;
  }
  //setengah gelombang pertama
  if (x > 90 && halfp == 0) {
    halfp = 1;
    x = 0;
  }
  //setengah gelombang kedua
  x++;
  if (halfp == 0) {
    Timer1.pwm(11, sinewave[x] * 0.9 * outputgain); //pin 11 menghasilkan gelombang pulsa
    Timer1.pwm(12, 0);              // pin pwm 12 nol
  }
  if (halfp == 1) {
    Timer1.pwm(11, 0);               //pin 11 nol
    Timer1.pwm(12, sinewave[x] * 0.9 * outputgain); //pin 12 menghasilkan gelombang pulsa
  }
}
