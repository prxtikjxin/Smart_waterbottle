#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

BlynkTimer timer;

#define BLYNK_TEMPLATE_ID "TMPLb5Pa7_D3"
#define BLYNK_DEVICE_NAME "Smart Water Bottle"
#define BLYNK_AUTH_TOKEN "BzBb2CWJoZU2o61pKnhMfhaHtdDCSkT8"

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "poco";//Enter your WIFI name
char pass[] = "123456789";//Enter your WIFI password

//water level
int val = 0 ;


// temperature
const int oneWireBus = 4;    // GPIO where the DS18B20 is connected to  D2
OneWire oneWire(oneWireBus); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor 

//Time Counter
unsigned long nextUpdate;
int count=0 ;
int prev=0 ;
int diff=0;


//LED's 
 const int GREEN = 16; //D0
 const int BLUE = 5; //D1
 const int RED = 2; //D4
 int ending = 0;
 bool isDrinking = true;
 bool isEmpty = false;
 bool state = true;
 bool empty_state = false;
 
// gyro
const uint8_t MPU6050SlaveAddress = 0x68; // MPU6050 Slave Device Address
const uint8_t scl = D6;  // Select SDA and SCL pins for I2C communication 
const uint8_t sda = D7;
const uint16_t AccelScaleFactor = 16384; // sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t GyroScaleFactor = 131;


// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

//initial port and output pins 
void setup() {
  Serial.begin(9600);
  pinMode(GREEN,OUTPUT);
  pinMode(BLUE,OUTPUT);
  pinMode(RED,OUTPUT);
  digitalWrite(GREEN,HIGH);

  
  //gyro
  Wire.begin(sda, scl); 
  MPU6050_Init();  

  sensors.begin(); // Start the DS18B20 sensor temperature

  //Initialize the Blynk library
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  timer.setInterval(300L, getSendData);

  nextUpdate = millis() + 10000UL;  //timer
}

void loop() {

  //Run the Blynk library
  Blynk.run(); 
  timer.run(); // Initiates SimpleTimer
 
// gyro sensor

  double Ax, Ay, Az, T, Gx, Gy, Gz;
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);
  delay(50);


  if(  (-0.20<= Ax >= 0.09) && (-0.20<= Ay >= 0.09) && (0.90<= Az >= 0.99)){
    
     //water level code
      count=analogRead(A0);
      // Water Level Sensor output pin connected A0
      Serial.println(count);  // See the Value In Serial Monitor
      if(count>100){
        
       if (millis() >= nextUpdate)
      {
      diff = abs(prev-count);
      if(diff>90){
        Serial.print("Good");
        isDrinking = true;
        digitalWrite(BLUE,LOW);
        state = true;
      }
      else
      {
        Serial.println("Please Sir, Drink Water");
        isDrinking = false;
        digitalWrite(BLUE,HIGH);
        state = false;
      }
      nextUpdate = millis() + 10000UL;
      prev = count ;
      }
    }
    else
    {
      isEmpty = true;
      if(ending==0){
      Serial.println("Fill up your bottle");
      ending=1;
      }
      digitalWrite(RED,HIGH);
      delay(1000);
      digitalWrite(RED,LOW);
      delay(1000);
//      exit(0);
    }
  }
    delay(100);      // for timer
    
}


 void getSendData()
{ 
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  Blynk.virtualWrite(0, temperatureC); //virtual pin V0
  Blynk.virtualWrite(1, count); 
  if(isEmpty){
  Blynk.virtualWrite(4, "Please fill up your Bottle."); 
  isEmpty = false;
  }


  if(!isDrinking){
    Blynk.virtualWrite(3, "Time to drink some water.");   
    isDrinking= true;
  }
  
  
}


void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}
//configure MPU6050
void MPU6050_Init(){
  delay(100);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
