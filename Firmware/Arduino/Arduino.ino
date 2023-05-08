#include <StreamUtils.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ArduinoJson.h>
#include "DFRobot_OxygenSensor.h"

#define LIGHT_METER A4
#define PH_METER A5
#define _MQ135 A2
#define MQ3 A3
#define CO2 A1
#define CONDUCTIVITY_METER A0
#define DHTPIN 4

#define DHTTYPE DHT11

#define MAIN_PUMP 8
#define CONDUCTIVITY_PUMP 9
#define PH_PUMP 10

#define SENSOR_DATA_PIN   (3)   // Sensor PWM interface
#define INTERRUPT_NUMBER   (0)   // interrupt number

#define FRAME_DELAY 500

#define UPDATE_SETPOINTS 0x01
#define SEND_READINGS 0x02

#define UPDATE_PH 0x00
#define UPDATE_CONDUCTIVITY 0x01
#define UPDATE_PUMP_PERIOD 0x02
#define UPDATE_DUTY_CYCLE 0x03

#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 10 // num of sample 

#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  5             // collect number, the collection range is 1-100.

const float offset = 22.203;      
const float slope = -0.028846;

const float cond_slope = 4.78983;

const uint32_t MILLIS_PER_SECOND = 1000;
const uint16_t SECONDS_PER_MINUTE = 60;



// Used in interrupt, calculate pulse width variable
volatile unsigned long pwm_high_start_ticks=0, pwm_high_end_ticks=0;
volatile unsigned long pwm_high_val=0, pwm_low_val=0;
// interrupt flag
volatile uint8_t flag=0;
float co2_concentration = 400;
float oxygen_conc = 21.0;

volatile unsigned long main_pump_on_timer = millis();
volatile unsigned long main_pump_off_timer = millis();
volatile bool main_pump_on = false;
bool ph_pump_on = false;
bool conductivity_pump_on = false;
bool takeActionPH = false;
bool takingActionPH = false;
bool takeActionCond = false;
bool takingActionCond = false;

uint32_t ph_pump_on_timer = millis();
uint32_t ph_pump_off_timer = millis();
uint32_t conductivity_pump_on_timer = millis();
uint32_t conductivity_pump_off_timer = millis();
uint32_t oxygenTimer = millis();

uint32_t ph_pump_on_interval = 60*MILLIS_PER_SECOND;
uint32_t ph_pump_off_interval = 180*MILLIS_PER_SECOND;
uint32_t conductivity_pump_on_interval = 60*MILLIS_PER_SECOND;
uint32_t conductivity_pump_off_interval = 180*MILLIS_PER_SECOND;
uint32_t oxygenInterval = 60*MILLIS_PER_SECOND;

unsigned long analogSampleTimepoint = millis();
unsigned long lastCommunication = millis();
byte command_code = 0;

volatile float pH_setpoint = 7.0;
volatile uint16_t conductivity_setpoint = 1000;
volatile uint32_t pump_period = 30ul*SECONDS_PER_MINUTE*MILLIS_PER_SECOND;
volatile uint8_t duty_cycle = 1;

uint32_t main_pump_on_interval = pump_period*duty_cycle/100;
uint32_t main_pump_off_interval = pump_period - main_pump_on_interval;

float humidity = 0;

int conductivityBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int conductivityBufferTemp[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

int lightBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int lightBufferTemp[SCOUNT];


int phBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int phBufferTemp[SCOUNT];

int carbonBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int carbonBufferTemp[SCOUNT];


DFRobot_OxygenSensor oxygen;
DHT dht(DHTPIN, DHTTYPE);


uint64_t timer = 0;
uint16_t interval = 1000;

void interrupt_change()
{
  if (digitalRead(SENSOR_DATA_PIN)) {
    pwm_high_start_ticks = micros();    // store the current micros() value
    if(2 == flag){
      flag = 4;
      if(pwm_high_start_ticks > pwm_high_end_ticks) {
        pwm_low_val = pwm_high_start_ticks - pwm_high_end_ticks;
      }
    }else{
      flag = 1;
    }
  } else {
    pwm_high_end_ticks = micros();    // store the current micros() value
    if(1 == flag){
      flag = 2;
      if(pwm_high_end_ticks > pwm_high_start_ticks){
        pwm_high_val = pwm_high_end_ticks - pwm_high_start_ticks;
      }
    }
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
  else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

int getTDS(){

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
  for(int copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  {
    conductivityBufferTemp[copyIndex]= conductivityBuffer[copyIndex];
  }
  averageVoltage = getMedianNum(conductivityBufferTemp,SCOUNT) * (float)VREF*1000.0/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
//  float compensationCoefficient=1.0+0.02*(temperature-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
//  float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
//  tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
  
  tdsValue = cond_slope*averageVoltage;

  takeActionCond = tdsValue < conductivity_setpoint;
  return (tdsValue);
}

float getPH(){
  for(int copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  {
    phBufferTemp[copyIndex]= phBuffer[copyIndex];
  }
  averageVoltage = getMedianNum(phBufferTemp, SCOUNT);

  float ph = slope*averageVoltage+offset;

  takeActionPH = ph<pH_setpoint;
  

  return (ph); 
} 


int getLightIntensity(){
  for(int copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  {
    lightBufferTemp[copyIndex]= lightBuffer[copyIndex];
  }
  averageVoltage = getMedianNum(lightBufferTemp, SCOUNT) * (float)VREF/ 1024.0;

  return (averageVoltage); // Needs fixing
} 

int get_co2(){
  for(int copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  {
    carbonBufferTemp[copyIndex]= carbonBuffer[copyIndex];
  }
  averageVoltage = getMedianNum(carbonBufferTemp, SCOUNT) * (float)VREF*1000/ 1024.0;

  return ((averageVoltage-400)*50.0/16.0);
} 

void handleCO2()
{
  if(flag == 4){
    flag = 1;
    float pwm_high_val_ms = (pwm_high_val * 1000.0) / (pwm_low_val + pwm_high_val);
  
    if (pwm_high_val_ms < 0.01){
      Serial.println("Fault");
    }
    else if (pwm_high_val_ms < 80.00){
      Serial.println("preheating");
    }
    else if (pwm_high_val_ms < 998.00){
      co2_concentration = (pwm_high_val_ms - 2) * 5;
    }
  }
}

void handleAnalogReadings()
{
  if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    conductivityBuffer[analogBufferIndex] = analogRead(CONDUCTIVITY_METER); //read the analog value and store into the buffer
    lightBuffer[analogBufferIndex] = analogRead(LIGHT_METER); //read the analog value and store into the buffer
    phBuffer[analogBufferIndex] = analogRead(PH_METER); //read the analog value and store into the buffer
    carbonBuffer[analogBufferIndex]= analogRead(CO2);
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT)
    analogBufferIndex = 0;
  }
}

void handlePumps()
{
  if (!main_pump_on && (millis() - main_pump_off_timer > main_pump_off_interval))
  {
    digitalWrite(MAIN_PUMP,HIGH);
    main_pump_on = true;
    main_pump_on_timer = millis();
    Serial.println("main pump on");

    Serial.println(main_pump_on_interval);
    Serial.println(main_pump_off_interval);
    Serial.println(pump_period);
  }

  if (main_pump_on && (millis() - main_pump_on_timer > main_pump_on_interval))
  {
    digitalWrite(MAIN_PUMP, LOW);
    main_pump_on = false;
    main_pump_off_timer = millis();
    Serial.println("main pump off");    
  }

  // Check if pH pump needs to be turned on
  if (!ph_pump_on && takeActionPH && !takingActionPH){
      ph_pump_on = true;
      ph_pump_on_timer = millis(); // Start timer to keep track of how long the pump has been on
      takingActionPH = true; // Set flag to indicate that the pump is currently running
      digitalWrite(PH_PUMP, HIGH); // Turn on pH pump
  }

  // Check if pH pump needs to be turned off
  if (ph_pump_on && (millis() - ph_pump_on_timer > ph_pump_on_interval)){
      ph_pump_on = false;
      ph_pump_off_timer = millis(); // Start timer to keep track of how long the pump has been off
      digitalWrite(PH_PUMP, LOW); // Turn off pH pump
  }

  // Check if it's time to stop taking action with the pH pump
  if (!ph_pump_on && (millis() - ph_pump_off_timer > ph_pump_off_interval)){
      takingActionPH = false; // Set flag to indicate that it's time to stop taking action with the pH pump
  }

  // Check if conductivity pump needs to be turned on
  if (!conductivity_pump_on && takeActionCond && !takingActionCond){
      conductivity_pump_on = true;
      conductivity_pump_on_timer = millis(); // Start timer to keep track of how long the pump has been on
      takingActionCond = true; // Set flag to indicate that the pump is currently running
      digitalWrite(CONDUCTIVITY_PUMP, HIGH); // Turn on conductivity pump
  }

  // Check if conductivity pump needs to be turned off
  if (conductivity_pump_on && (millis() - conductivity_pump_on_timer > conductivity_pump_on_interval)){
      conductivity_pump_on = false;
      conductivity_pump_off_timer = millis(); // Start timer to keep track of how long the pump has been off
      digitalWrite(CONDUCTIVITY_PUMP, LOW); // Turn off conductivity pump
  }

  // Check if it's time to stop taking action with the conductivity pump
  if (!conductivity_pump_on && (millis() - conductivity_pump_off_timer > conductivity_pump_off_interval)){
      takingActionCond = false; // Set flag to indicate that it's time to stop taking action with the conductivity pump
  }  
}

void handleOxygen()
{
  if (millis() - oxygenTimer > oxygenInterval)
  {
    oxygen_conc = oxygen.getOxygenData(COLLECT_NUMBER);
    oxygenTimer = millis();
  }
}

void handleCommunication()
{
  if (millis() - lastCommunication > FRAME_DELAY)
  {
    if(Serial1.available()>0){
      command_code = Serial1.read();
      Serial.println(command_code);
    } else{
      return;
      }

    switch (command_code)
    {
      case UPDATE_SETPOINTS:{
        Serial.println("Updating setpoints");
        String value;
        
        value = Serial1.readStringUntil('\n');
        if (value.length() == 0){
          Serial.println("Received empty parameter, aborting");
          break;
        }       
        update_setting(value, UPDATE_PH);
        
        value = Serial1.readStringUntil('\n');
        if (value.length() == 0){
          Serial.println("Received empty parameter, aborting");
          break;
        }  
        update_setting(value, UPDATE_CONDUCTIVITY);
        
        value = Serial1.readStringUntil('\n');
        if (value.length() == 0){
          Serial.println("Received empty parameter, aborting");
          break;
        }  
        update_setting(value, UPDATE_PUMP_PERIOD);
        
        value = Serial1.readStringUntil('\n');
        if (value.length() == 0){
          Serial.println("Received empty parameter, aborting");
          break;
        } 
        update_setting(value, UPDATE_DUTY_CYCLE);
       
        
        lastCommunication = millis();
        break;
      }

      case SEND_READINGS:{
        send_readings();
        lastCommunication = millis();
        break;
      }

      default: {
        while (Serial1.available()) {
          Serial1.read();
        }
        Serial.println("Invalid message received, incoming serial buffer flushed");
      }
    }
  }
}

void update_setting(String value, uint8_t setting_code)
{
  switch (setting_code)
  {
    case UPDATE_PH:{
      pH_setpoint = atof(value.c_str());
      Serial.println(String("pH setpoint is now ") + pH_setpoint);
      break;
    }

    case UPDATE_CONDUCTIVITY:{
      conductivity_setpoint = atoi(value.c_str());
      Serial.println(String("Conductivity setpoint is now ") + conductivity_setpoint);
      break;
    }

    case UPDATE_PUMP_PERIOD:{
      pump_period = (uint32_t)atoi(value.c_str())*SECONDS_PER_MINUTE*MILLIS_PER_SECOND;
      main_pump_on_interval = pump_period*duty_cycle/100;
      main_pump_off_interval = pump_period - main_pump_on_interval;
      Serial.println(String("Pump period is now ") + pump_period);
      break;
    }

    case UPDATE_DUTY_CYCLE:{
      duty_cycle = (uint8_t) atoi(value.c_str());
      main_pump_on_interval = pump_period*duty_cycle/100;
      main_pump_off_interval = pump_period - main_pump_on_interval;
      Serial.println(String("Duty cycle is now ") + duty_cycle);
      break;
    }

    default:{
      Serial.println("Update setting message longer than expected");
    }
  }
}

void send_readings()
{
  DynamicJsonDocument doc(64);
  doc["ph"]=(int)(100*getPH());
  doc["conductivity"]=(int)getTDS();
  doc["temperature"]= (int)temperature; // DHT API already called in getTDS()
  doc["humidity"] = (int)humidity;
  doc["lightIntensity"]=(int)getLightIntensity();
  doc["oxygen"]=(int)(10*oxygen_conc);
  doc["co2"]=(int)get_co2();
 
//  WriteLoggingStream loggedFile(Serial1,Serial);
  Serial.println("Sending to ESP32");
  serializeJson(doc,Serial1);
  Serial.println("Finished sending to ESP32");  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
      delay(100); // wait for serial port to connect. Needed for Leonardo only
     }
  Serial1.begin(9600);

  Serial.println(main_pump_on_interval);
  Serial.println(main_pump_off_interval);


  pinMode(SENSOR_DATA_PIN, INPUT);
  pinMode(MAIN_PUMP, OUTPUT);
  pinMode(PH_PUMP, OUTPUT);
  attachInterrupt(INTERRUPT_NUMBER, interrupt_change, CHANGE);
  
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");

  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  handlePumps();

  handleAnalogReadings();
  
  handleCO2();

  handleCommunication();

  handleOxygen();
  
}
