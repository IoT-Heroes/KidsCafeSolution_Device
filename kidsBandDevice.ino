#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#define USE_ARDUINO_INTERRUPTS true  
#include <PulseSensorPlayground.h> 
#include "IoTMakers.h"
#include "Shield_Wrapper.h"
#include "I2Cdev.h"
#include "MPU6050.h"


#define SDCARD_CS 4
#define PIN_LED 9
#define IO_LED 13
#define BUTTON 3

Shield_Wrapper  g_shield;
IoTMakers g_im;
PulseSensorPlayground pulseSensor;
MPU6050 accelgyro; // 3축센서  객체 

const char deviceID[]   = "band1";
const char authnRqtNo[] = "12345qwer";
const char extrSysID[]  = "OPEN_TCP_001PTL001_1000006334";
const float WORKING_THRESHOLD = 0.2;


// 변수
int16_t ax, ay, az;
int16_t gx, gy, gz;
float xavg, yavg, zavg;

int flag = 0; 
int call = 0; // 호출 상태 
int steps = 0; // 걸음수 


const int PulseWire = 0;  
const int LED13 = 13;  
int PULSE_THRESHOLD = 550;
boolean onoff = LOW;

/*
Arduino Shield
*/
void sdcard_deselect()
{
	pinMode(SDCARD_CS, OUTPUT);
	digitalWrite(SDCARD_CS, HIGH); //Deselect the SD card
}
void init_shield()
{
	sdcard_deselect();
	
	const char* WIFI_SSID = "honghot";
	const char* WIFI_PASS = "khk04252311";
	g_shield.begin(WIFI_SSID, WIFI_PASS);

	g_shield.print();
}


/*
IoTMakers
*/



void init_iotmakers()
{
	Client* client = g_shield.getClient();
	if ( client == NULL )	{
		Serial.println(F("No client from shield."));
		while(1);
	}

 
	g_im.init(deviceID, authnRqtNo, extrSysID, *client);
	g_im.set_numdata_handler(mycb_numdata_handler);
	g_im.set_strdata_handler(mycb_strdata_handler);
	g_im.set_dataresp_handler(mycb_resp_handler);

	// IoTMakers 서버 연결

	Serial.println(F("connect()..."));
	while ( g_im.connect() < 0 ){
		Serial.println(F("retrying."));
		delay(3000);
	}

	// Auth
   
	Serial.println(F("auth."));
	while ( g_im.auth_device() < 0 ) {
		Serial.println(F("fail"));
		while(1);
	}

	Serial.print("FreeRAM=");
	Serial.println(g_im.getFreeRAM());
}



void setup() 
{
	Serial.begin(9600);
  	while ( !Serial )  {
	  ;
	}

	pinMode(BUTTON, INPUT);
	pinMode(IO_LED, OUTPUT);

	init_shield();

 
	init_iotmakers();

 // i2c초기화 
  Wire.begin();

  // 시리얼 초기화 
  Serial.begin(9600);

  // 3축센서 초기화 
  stepSensorSetUp();

  // 심박센서 초기화 
  pulseSensorSetUp(); 

//	digitalWrite(PIN_LED, LOW);

}

void loop()
{
	static unsigned long tick = millis();

  
  send_calldata();
 
//	// 1초 주기로 센서 정보 송신
	if ( ( millis() - tick) > 1000 )
	{
	  send_stepdata();
    
    send_pulsedata();
		tick = millis();
 	}
   
  call = 0;
	// IoTMakers 서버 수신처리 및 keepalive 송신
	g_im.loop();
}
/* 심박센서 관련 함수 */ 

// 심박센서 초기화 함수 
void pulseSensorSetUp()
{
  // Configure the PulseSensor object, by assigning our variables to it. 
  pulseSensor.analogInput(PulseWire);   
  
  pulseSensor.setThreshold(PULSE_THRESHOLD);   

  // Double-check the "pulseSensor" object was created and "began" seeing a signal. 
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }
}
// 심박 데이터 전송 함수 
int send_pulsedata()
{
  int myBPM = pulseSensor.getBeatsPerMinute(); 
  if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened". 
    Serial.println("♥  A HeartBeat Happened ! ");
    Serial.print("BPM: ");Serial.println(myBPM);  
    if ( g_im.send_numdata("pulse", (double)myBPM) < 0 ){
      Serial.println(F("fail"));  
      return -1;
    }
  }
  return 0;   
}

/* 호출 */

int send_calldata()
{
  if(digitalRead(BUTTON) == LOW){
    delay(10);
    if(digitalRead(BUTTON) == HIGH){
      digitalWrite(IO_LED, onoff);
      Serial.println("Call! Call!");
      onoff = (!onoff);
      call = 1;
      if ( g_im.send_numdata("call", (double)call) < 0 ){
        Serial.println(F("fail"));  
        return -1;
      }
      delay(100);
      
    }
  }

  return 0;
}
/* 걸음센서 관련 함수 */

// 걸음센서 초기화 함수 
void stepSensorSetUp()
{
   // 가속도센서 초기화 
  accelgyro.initialize();
  
  // 자이로 250도/초, 가속도 : -2.0G ~ 2.0G
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // 센서 OFFSET 초기화 
   adjust_accelgyro();

  // 가속도센서의 중간값 구하기 
  // 가속도변화를 판단하기 위해서 필요 
  calibrate();
}


boolean is_working(double dist) {
  return dist > WORKING_THRESHOLD;
}


void adjust_accelgyro(){
  Serial.println("Upadting internal sensor offsets....");
  accelgyro.setXAccelOffset(-950);
  accelgyro.setYAccelOffset(-5200);
  accelgyro.setZAccelOffset(-920);
  accelgyro.setXGyroOffset(20);
  accelgyro.setYGyroOffset(56);
  accelgyro.setZGyroOffset(-85);
  
}

void calibrate(){
  float sumx = 0;
  float sumy = 0;
  float sumz = 0;

  for(int i = 0; i < 100; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumx += ax / 16384.0;
    sumy += ay / 16384.0;
    sumz += az / 16384.0;
    
  }

  xavg = sumx / 100.0;
  yavg = sumy / 100.0;
  zavg = sumz / 100.0;
}

void on_step(){
  steps = steps + 1;
  Serial.print("step = ");Serial.println(steps);
}

// 걸음값 전송 함수 
int send_stepdata()
{
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  double fax = ax / 16384.0;
  double fay = ay / 16384.0;
  double faz = az / 16384.0;

  double dist = sqrt(pow(fax - xavg,2) + pow(fay - yavg, 2) + pow(faz - zavg, 2));
  double major_factor = max(fax, max(fay, faz)); 
 

  if(is_working(dist) && flag == 0 && major_factor > 0) {
    flag = 1;
    steps = steps + 1;
    Serial.print("step = ");Serial.println(steps);
     delay(200);
     if ( g_im.send_numdata("step", (double)steps) < 0 ) {
      Serial.println(F("fail"));  
      return -1;
    } 
  } 
  if(!is_working(dist) && flag == 1)
  {
    flag = 0;
  } 
//  if ( g_im.send_numdata("step", (double)steps) < 0 ) {
//      Serial.println(F("fail"));  
//      return -1;
//  } 
  delay(200);

  
	return 0;   
}



/* Push 데이터 처리 함수 */

void mycb_numdata_handler(char *tagid, double numval)
{
	// !!! USER CODE HERE
	//Serial.print(tagid);Serial.print(F("="));Serial.println(numval);
}
void mycb_strdata_handler(char *tagid, char *strval)
{
	// !!! USER CODE HERE
	//Serial.print(tagid);Serial.print(F("="));Serial.println(strval);
  
	if ( strcmp(tagid, "led")==0 && strcmp(strval, "on")==0 )  	
		digitalWrite(PIN_LED, HIGH);
	else if ( strcmp(tagid, "led")==0 && strcmp(strval, "off")==0 )  	
		digitalWrite(PIN_LED, LOW);
}
void mycb_resp_handler(long long trxid, char *respCode)
{
	if ( strcmp(respCode, "100")==0 )
		Serial.println("resp:OK");
	else
		Serial.println("resp:Not OK");
}
