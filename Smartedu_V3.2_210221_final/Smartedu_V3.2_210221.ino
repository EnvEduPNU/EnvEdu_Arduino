//CO2
#include <SoftwareSerial.h>

//pH lib
#include "DFRobot_PH.h"
#include "EEPROM.h"

//DO lib
#include <avr/pgmspace.h>

//Temp lib
#include <OneWire.h>
#include <DallasTemperature.h>


//Dust
#include <pm2008_i2c.h>

//Pressure
#include "DFRobot_BME280.h"
#include "Wire.h"

//LUX
#include "Adafruit_VEML7700.h"
Adafruit_VEML7700 veml = Adafruit_VEML7700();

//humidity
#include "DHT.h"
#define DHTPIN 8
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
float turb_CALvalue = 0;
//pH pin define
#define PH_PIN A0

DFRobot_PH ph;
float voltage;
float phValue, temperature;
float neutralVoltage = 1500.0;
float acidVoltage = 2032.44;
#define PHVALUEADDR 0x00
int CAL_flag = 0;

//DO pin define
float doValue;
#define DoSensorPin  A2    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5000    //for arduino uno, the ADC reference is the AVCC, that is 5000mV(TYP)
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength + 1];  // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;

#define SaturationDoVoltageAddress 12          //the address of the Saturation Oxygen voltage stored in the EEPROM
#define SaturationDoTemperatureAddress 16      //the address of the Saturation Oxygen temperature stored in the EEPROM
float SaturationDoVoltage, SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
  14.46, 14.22, 13.82, 13.44, 13.09,
  12.74, 12.42, 12.11, 11.81, 11.53,
  11.26, 11.01, 10.77, 10.53, 10.30,
  10.08, 9.86,  9.66,  9.46,  9.27,
  9.08,  8.90,  8.73,  8.57,  8.41,
  8.25,  8.11,  7.96,  7.82,  7.69,
  7.56,  7.43,  7.30,  7.18,  7.07,
  6.95,  6.84,  6.73,  6.63,  6.53,
  6.41,
};

//temp pin define
#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

//Turbidity pin define
#define Turbidity_pin A4
int turbValue;
float turbVoltage;
float ntu;

//Soil Moisture pin define
#define Moisture A6

//CO2

SoftwareSerial mySerial(10, 11);
unsigned char hexdata[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};// gas density command (여긴 수정하면 CO2센서가 제대로 동작하지 않습니다)
float CO2;
float CO2_Bias =0.0;

//Dust
PM2008_I2C pm2008_i2c;
float dust;

//Pressure
typedef DFRobot_BME280_IIC    BME;    // ******** use abbreviations instead of full names ********
BME   bme(&Wire, 0x77);   // select TwoWire peripheral and set sensor address
#define SEA_LEVEL_PRESSURE    1015.0f

void setup() {
  voltage = 0;
  Serial.begin(115200);
  Serial1.begin(115200);  
  Serial.println("Start");
  pm2008_i2c.begin();
  pm2008_i2c.command();
  bme.reset();
  dht.begin();
  veml.begin();
  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_800MS);
  //Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }
  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }
  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
  Serial.println("bme read data test");
  while (bme.begin() != BME::eStatusOK) {
    Serial.println("bme begin faild");
    printLastOperateStatus(bme.lastOperateStatus);
    delay(500);
  }
  Serial.println("bme begin success");
  ph.begin();
  pinMode(DoSensorPin, INPUT);
  readDoCharacteristicValues();
  sensors.begin();
  //발견한 디바이스 갯수
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // parasite power 모드일 때에는  2핀(GND와 DQ 핀)만 연결하면 됨.
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  //버스에서 첫번째 장치의 주소를 가져온다.
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

  //버스에서 발견한 첫번째 장치의 주소 출력
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  //데이터시트에서 확인결과 9~12비트까지 설정 가능
  sensors.setResolution(insideThermometer, 12);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC);
  Serial.println();
  mySerial.begin(9600);
}

void loop() {

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 30U)  //every 30 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  
  static unsigned long tempSampleTimepoint = millis();
  if (millis() - tempSampleTimepoint > 500U) // every 500 milliseconds, read the temperature
  {
    tempSampleTimepoint = millis();
    mySerial.write(hexdata, 9);
    //temperature = readTemperature();  // add your temperature codes here to read the temperature, unit:^C
  }

  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) //time interval: 1s
  {
    timepoint = millis();

    //CO2 측정
    CO2;
    mySerial.write(hexdata, 9);
    for (int i = 0, j = 0; i < 9; i++)
    {
      if (mySerial.available() > 0)
      {
        long hi, lo;
        int ch = mySerial.read();
        if (i == 2) {
          hi = ch;    //High concentration
        }
        if (i == 3) {
          lo = ch;    //Low concentration
        }
        if (i == 8) {
          CO2 = hi * 256 + lo - CO2_Bias; //CO2 concentration
          Serial.print("CO2_Bias: ");
          Serial.println(CO2_Bias);
        }
      }
    }
    
    //미세먼지 측정
    uint8_t ret = pm2008_i2c.read();
    if (ret == 0) {
      dust = pm2008_i2c.pm2p5_tsi;
    }
    
    //토양습도 측정
    
    int Soil_Moisture = analogRead(Moisture);
    //Serial.print("Soil_Moisture:");
    //Serial.println(Soil_Moisture);

    //탁도 측정
    turbValue = analogRead(Turbidity_pin);
    turbVoltage = turbValue * (5.0 / 1024.0);
    Serial.print("turbVoltage: ");
    Serial.println(turbVoltage);
    turbVoltage += turb_CALvalue;
    Serial.println("  turb_CALvalue : "+String(turb_CALvalue));
    ntu = (-356.95 * turbVoltage * turbVoltage) + (1925.1 * turbVoltage) - 1633.1;
    if(ntu<=0)
    {
      if(turbVoltage <= 0) // 연결이 끊어진 상태
        ntu = -99999.00;
      else // 연결되었으나 보정이 필요한 상태
        ntu =0;
    }
    //Serial.print("NTU:");
    //Serial.println(ntu);

    //DO측정 및 온도측정
    sensors.requestTemperaturesByIndex(0);
    temperature = printTemperature(insideThermometer);
    Serial.print("printTemperature(insideThermometer) -> temperature : ");
    Serial.println(temperature);
    if(temperature <=-127)
      temperature = -99999;
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    if(analogRead(DoSensorPin) == 0)
      doValue = -99999.00;
    else
    doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature + 0.5) ) * averageVoltage / SaturationDoVoltage;
    Serial.print("DO :");
    Serial.println(doValue, 2);

    //PH농도 측정
    voltage = analogRead(PH_PIN) / 1024.0 * 5000;
    Serial.print("ph v: ");
    Serial.println(voltage);
    Serial.println(temperature);
    if(voltage == 0)
      phValue = -99999;
    else
    phValue = ph.readPH(voltage, temperature);
    Serial.println();
    //Serial.print("temperature:");
    //Serial.println(temperature, 1);
    //Serial.print("pH:");
    //Serial.println(phValue, 2);
    //Serial.println("-------------");
    
    //기압 측정
    float pressure = bme.getPressure() / 100;
    //조도 측정
    float brightness = veml.readLux();
    //습도 측정
    float humidity = dht.readHumidity();
    
    //Serial.print("Pressure:");
    //Serial.println(pressure);
    //Serial.print("LUX:");
    //Serial.println(brightness);
    //Serial.print("Humidity:");
    //Serial.println(humidity);
    //Serial.print("Dust:");
    //Serial.println(dust);
    sendData(humidity, ntu, phValue, dust, doValue, CO2, brightness, (float)Soil_Moisture, pressure,temperature);
    delay(3000);
  }
  //특정 센서에 대한 보정 요구가 있을 때 보정 절차
  if (serialDataAvailable() > 0)
  {
    byte modeIndex = uartParse();  //parse the uart command received
    Serial.println(modeIndex);
    doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
  }
}

//CHOISW START
void sendData(float a, float b, float c, float d, float e, float f, float g, float h,float i,float j) {
  String packet = String(a) + "," + String(b) + "," + String(c) + "," + String(d) + "," + String(e) + "," + String(f) + "," + String(g) + "," + String(h) + "," + String(i) +","+String(j) + ",*";
  Serial1.println( packet );       // tell the app the LED is now on
  //Serial.print("Sent - ");
  Serial.println( packet );
}
//CHOISW END

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial1.available() > 0 )
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer, 0, (ReceivedBufferLength + 1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial1.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
      receivedBufferIndex = 0;
      Serial.print("Signal : ");
      Serial.println(receivedBuffer);
      strupr(receivedBuffer);
      return true;
    } else {
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
  byte modeIndex = 0;
  Serial.println("Receive Text = " + String(receivedBuffer));
  if (strstr(receivedBuffer, "ENTERPH") != NULL)
    modeIndex = 1;
  else if (strstr(receivedBuffer, "EXITPH") != NULL)
    modeIndex = 3;
  else if (strstr(receivedBuffer, "CALPH")  != NULL)
    modeIndex = 2;

  else if (strstr(receivedBuffer, "ENTERDO") != NULL)
    modeIndex = 4;
  else if (strstr(receivedBuffer, "EXITDO") != NULL)
    modeIndex = 6;
  else if (strstr(receivedBuffer, "CALDO") != NULL)
    modeIndex = 5;
  else if (strstr(receivedBuffer, "ENTERTUR") != NULL)
    modeIndex = 7;
  else if (strstr(receivedBuffer, "CALTUR") != NULL)
    modeIndex = 8;
  else if (strstr(receivedBuffer, "EXITTUR") != NULL)
    modeIndex = 9;
  else if (strstr(receivedBuffer, "ENTERCO2") != NULL)
    modeIndex = 10;
  else if (strstr(receivedBuffer, "CALCO2") != NULL)
    modeIndex = 11;
  else if (strstr(receivedBuffer, "EXITCO2") != NULL)  
    modeIndex = 12;
   
/*
if (strstr(receivedBuffer, 1) != NULL)
    modeIndex = 1;
  else if (strstr(receivedBuffer, 3) != NULL)
    modeIndex = 3;
  else if (strstr(receivedBuffer, 2)  != NULL)
    modeIndex = 2;

  else if (strstr(receivedBuffer, "ENTERDO") != NULL)
    modeIndex = 4;
  else if (strstr(receivedBuffer, "EXITDO") != NULL)
    modeIndex = 6;
  else if (strstr(receivedBuffer, "CALDO") != NULL)
    modeIndex = 5;
    */
  return modeIndex;
}

void doCalibration(byte mode)
{
  char *receivedBufferPtr;
  static boolean doCalibrationFinishFlag = 0, enterCalibrationFlag = 0, phCalibrationFinish = 0;
  float voltageValueStore;
  switch (mode)
  {
    case 0:
      if (enterCalibrationFlag)
        Serial.println(F("Command Error"));
      break;

    case 1:    //선언 필요 변수 : voltage, neutralVoltage, acidVoltage, PHVALUEADDR
      enterCalibrationFlag = 1;
      phCalibrationFinish  = 0;
      Serial.println();
      Serial.println(F(">>>Enter PH Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the 4.0 or 7.0 standard buffer solution<<<"));
      CAL_flag = 1;
      Serial.println();
      break;

    case 2:
      if (enterCalibrationFlag) {   //
        if ((voltage > 1322) && (voltage < 1678)) { // buffer solution:7.0{
          Serial.println();
          Serial.print(F(">>>Buffer Solution:7.0"));
          Serial.print("; before neutralVoltage: ");
          Serial.println(neutralVoltage);
          neutralVoltage = voltage;
          Serial.print("; after neutralVoltage: ");
          Serial.println(neutralVoltage); 
          Serial.println(F(",Send EXITPH to Save and Exit<<<"));
          Serial.println();
          phCalibrationFinish = 1;
          CAL_flag = 2;
        } else if ((voltage > 1854) && (voltage < 2210)) { //buffer solution:4.0
          Serial.println();
          Serial.print(F(">>>Buffer Solution:4.0"));
          Serial.print("; before neutralVoltage: ");
          Serial.println(neutralVoltage);
          neutralVoltage = voltage;
          Serial.print("; after neutralVoltage: ");
          Serial.println(neutralVoltage); 
          Serial.println(F(",Send EXITPH to Save and Exit<<<"));
          Serial.println();
          phCalibrationFinish = 1;
          CAL_flag= 2;
        } else {
          Serial.println();
          Serial.print(F(">>>Buffer Solution Error Try Again<<<"));
          Serial.println();                                    // not buffer solution or faulty operation
          phCalibrationFinish = 0;
          CAL_flag=1;
        }
      }
      break;

    case 3:
      if (enterCalibrationFlag) {
        Serial.println();
        if (phCalibrationFinish) {
          if ((voltage > 1322) && (voltage < 1678)) {
            EEPROM_write(PHVALUEADDR, neutralVoltage);
          } else if ((voltage > 1854) && (voltage < 2210)) {
            EEPROM_write(PHVALUEADDR + 4, acidVoltage);
          }
          Serial.print(F(">>>Calibration Successful"));
          CAL_flag =0;          
        } else {
          Serial.print(F(">>>Calibration Failed"));
        }
        Serial.println(F(",Exit PH Calibration Mode<<<"));
        Serial.println();
        
        phCalibrationFinish  = 0;
        enterCalibrationFlag = 0;
      }
      break;
    case 4:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      CAL_flag = 1;
      break;

    case 5:
      if (enterCalibrationFlag)
      {
        Serial.println();
        Serial.println(F(">>>Saturation Calibration Finish!<<<"));
        Serial.println();
        EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
        EEPROM_write(SaturationDoTemperatureAddress, temperature);
        SaturationDoVoltage = averageVoltage;
        SaturationDoTemperature = temperature;
        doCalibrationFinishFlag = 1;
        Serial.println("SaturationDoVoltageAddress" + String(SaturationDoVoltageAddress));
        Serial.println("averageVoltage" + String(averageVoltage));
        CAL_flag = 2;
      }
      break;

    case 6:
      if (enterCalibrationFlag)
      {
        Serial.println();
        if (doCalibrationFinishFlag)
          Serial.print(F(">>>Calibration Successful"));
        else
          Serial.print(F(">>>Calibration Failed"));
        CAL_flag = 0;
        Serial.println(F(",Exit Calibration Mode<<<"));
        Serial.println();
        doCalibrationFinishFlag = 0;
        enterCalibrationFlag = 0;
      }
      break;
  
    case 7:
      Serial.println("Turbidity Calibration Start");
      CAL_flag = 1;
      break;
    
    case 8:
      Serial.println("Turbidity Calibration Success");
      CAL_flag = 2;
      turbValue = analogRead(Turbidity_pin);
      turbVoltage = turbValue * (5.0 / 1024.0);
      turb_CALvalue = 4.3386 - turbVoltage;
      Serial.println("CALvalue = "+String(turb_CALvalue));
      break;
    
    case 9:
      Serial.println("EXIT Turbidity calibration");
      CAL_flag =0;
      break;

//CHOISW_START 
      case 10:
      Serial.println("CO2 Calibration Start");
      CAL_flag =1;
      break;   
    
      case 11:
      Serial.println("CO2 calibration Success!");
      CAL_flag =2;
      //지금 찍은 농도값을 410.00으로 맞춰주고 차이(CO2_bias)값을 구한다
      float temp = CO2;//BIAS 계산을 위해 최근 CO2값을 저장 
      CO2 = 410.00;// 대기 중 이산화탄소 농도 기준값으로 보정
      CO2_Bias =temp-CO2;//보정치를 구해 추후 측정에 반영
      Serial.println("CALvalue = "+String(CO2_Bias));
      break;

      case 12:
      Serial.println("EXIT CO2 calibration");
      CAL_flag =0;
      break;
//CHOISW_END
    }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
  {
    bTab[i] = bArray[i];
  }
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

void readDoCharacteristicValues(void)
{
  EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);
  EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
  Serial.print("SaturationDoVoltageAddress: ");
  Serial.println(EEPROM.read(SaturationDoVoltageAddress));
  Serial.print("SaturationDoVoltage: ");
  Serial.println(SaturationDoVoltage);
  Serial.print("SaturationDoTemperatureAddress: ");
  Serial.println(EEPROM.read(SaturationDoTemperatureAddress));
  Serial.print("SaturationDoTemperature: ");
  Serial.println(SaturationDoTemperature);
  if(EEPROM.read(SaturationDoVoltageAddress) == 0xFF && EEPROM.read(SaturationDoVoltageAddress+1) == 0xFF && EEPROM.read(SaturationDoVoltageAddress+2) == 0xFF && EEPROM.read(SaturationDoVoltageAddress+3) == 0xFF)
  {
    SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
    EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
  }
  if(EEPROM.read(SaturationDoTemperatureAddress) == 0xFF && EEPROM.read(SaturationDoTemperatureAddress+1) == 0xFF && EEPROM.read(SaturationDoTemperatureAddress+2) == 0xFF && EEPROM.read(SaturationDoTemperatureAddress+3) == 0xFF)
  { 
    SaturationDoTemperature = 25.0;   //default temperature is 25^C
    EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
  }
}

// 온도를 출력하는 함수
float printTemperature(DeviceAddress deviceAddress)
{

  float  tempC = sensors.getTempC(deviceAddress);

  /*Serial.print("Temp C: ");
    Serial.print(tempC);
    Serial.print(" Temp F: ");
    Serial.println(DallasTemperature::toFahrenheit(tempC));   */
  return tempC;
}

//디바이스 주소를 출력하는 함수
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void printLastOperateStatus(BME::eStatus_t eStatus)
{
  switch (eStatus) {
    case BME::eStatusOK:    Serial.println("everything ok"); break;
    case BME::eStatusErr:   Serial.println("unknown error"); break;
    case BME::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
    case BME::eStatusErrParameter:    Serial.println("parameter error"); break;
    default: Serial.println("unknow status"); break;
  }
}
