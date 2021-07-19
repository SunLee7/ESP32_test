#include <Arduino.h>
#include "myheadfile.h"
#include <TFT_eSPI.h>
#include "lvgl.h"
#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "好好学习，天天上网";
const char* password = "Freescale";

// const char* ssid = "Sun Lee";
// const char* password = "11112222";

float floatbuff[10];

TFT_eSPI tft;
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
int16_t encoder_moves = 0 ,encoder_pressed = 0;

void Taskmain(void *pvParameters);
void Tasktick(void *pvParameters);
void Taskwifi(void *pvParameters);

uint8_t OTAFlag = 0;
void OTAStart();

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(0, INPUT);
  Serial.begin(115200);
  tft.init();
  tft.initDMA();
  tft.fillScreen(TFT_BLACK);
  mylv_init();
  mpu.begin();
  mpu_temp = mpu.getTemperatureSensor();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
  }


  xTaskCreatePinnedToCore(Taskmain      //任务名称
                          , "Taskmain"
                          , 10000        //堆栈大小
                          , NULL
                          , 1           //优先级 3最大 1最小
                          , NULL
                          , 0);         //核心选择
  xTaskCreatePinnedToCore(Tasktick        //任务名称
                          , "Tasktick"
                          , 1024        //堆栈大小
                          , NULL
                          , 2           //优先级 3最大 1最小
                          , NULL
                          , 0);         //核心选择
  xTaskCreatePinnedToCore(Taskwifi        //任务名称
                          , "Taskwifi"
                          , 1024        //堆栈大小
                          , NULL
                          , 1           //优先级 3最大 1最小
                          , NULL
                          , 1);         //核心选择
}

//向上位机传送被拆分后的浮点数据
void SendData(uint8_t size, float* dat)
{
	int i, j;
	for(i = 0;i < size; i++)
	{
		for(j = 0;j  < 4; j ++)
		{
			Serial.write(*((uint8_t *)(dat+i)+j));
		}
	}
}

void loop() {
  // put your main code here, to run repeatedly:
  if(OTAFlag) ArduinoOTA.handle();
}

void Taskmain(void *pvParameters)
{
  (void) pvParameters;
  while(1)
  {
    //读取6050数据
    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    //读取串口数据
    char dat = Serial.read();
    if(dat == 0x01)
      digitalWrite(2, 0);
    else if(dat == 0x02)
      digitalWrite(2, 1);

    //向上位机发送数据
    // floatbuff[0] = accel.acceleration.x;
    // floatbuff[1] = accel.acceleration.y;
    // floatbuff[2] = accel.acceleration.z;
    // floatbuff[3] = gyro.gyro.x;
    // floatbuff[4] = gyro.gyro.y;
    // floatbuff[5] = gyro.gyro.z;
    // SendData(6, floatbuff);
    // Serial.write(0x00);
    // Serial.write(0x00);
    // Serial.write(0x80);
    // Serial.write(0x7f);
    //将数据显示在LGVL的Label上
    // lv_label_set_text_fmt(MPU_Lable, "x:%f\ny:%f\nz:%f\n%d\n%d", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, encoder_pressed, encoder_moves);
    // lv_obj_align(MPU_Lable, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);

    if(!digitalRead(0)) //GPIO0按键读取
    {
      vTaskDelay(50);
      if(!digitalRead(0))
      {
        OTAStart();
        // digitalWrite(2, !digitalRead(2));
        // // Serial.printf("%4.d",i++);
        // while(!digitalRead(0));
      }
    }
    lv_task_handler();
    vTaskDelay(5);
  }
}

void Tasktick(void *pvParameters)
{
  (void) pvParameters;
  portTickType xLastWakeTime;
  static uint16_t tms500 = 0;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    lv_tick_inc(1);
    tms500++;
    if(tms500 >= 250){
      tms500 = 0;
      if(accel.acceleration.x > 3) encoder_pressed = 1;
      else encoder_pressed = 0;
      if(accel.acceleration.y < -3) encoder_moves++;
      if(accel.acceleration.y > 3) encoder_moves--;
    }
    vTaskDelayUntil( &xLastWakeTime, 1);
  }
}

void Taskwifi(void *pvParameters)
{
  (void) pvParameters;
  static uint8_t i = 0;
  while(1)
  {
    // Serial.println("Scan start!");
    // int n = WiFi.scanNetworks();
    // Serial.println("Scan done!");
    // if(n == 0)
    // {
    //   Serial.println("No network found!");
    // }
    // else
    // {
    //   Serial.print(n);
    //   Serial.println(" Found");
    //   for(int i; i < n; i ++)
    //   {
    //     Serial.print(i + 1);
    //     Serial.print(":");
    //     Serial.print(WiFi.SSID(i));
    //     Serial.print("(");
    //     Serial.print(WiFi.RSSI(i));
    //     Serial.print(")");
    //     Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
    //   }
    // }
    // Serial.println(" ");
    // sigmaDeltaWrite(0, i);
    i++;
    if(i == 180) i = 0;
    dacWrite(26, 256 * sin(i*PI/180.0));
    digitalWrite(2, !digitalRead(2));
    vTaskDelay(1);
  }
}

void OTAStart()
{
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    OTAFlag = 1;
}