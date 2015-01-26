/*
   This file is part of SensorsMQTT Project source code.
   SensorsMQTT is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   SensorsMQTT is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with SensorsMQTT.  If not, see <http://www.gnu.org/licenses/>.

   Author: Diogo Soares <info@diogosoares.ca>
   Project Homepage: https://github.com/diogos88/SensorsMQTT
   */

#pragma region define

///////////////////
// Communication //
///////////////////
#define ENABLE_SERIAL
#define ENABLE_MQTT_CLIENT // MQTT protocol v3.1
//#define ENABLE_DISPLAY

///////////////////
//    Sensors    //
///////////////////
#define ENABLE_TEMPERATURE_HUMIDITY_SENSOR
//#define ENABLE_DISTANCE_SENSOR
//#define ENABLE_MOTION_SENSOR
//#define ENABLE_ROTARY_ENCODER
//#define ENABLE_GAS_SENSOR
//#define ENABLE_SOIL_MOISTURE_SENSOR

///////////////////
//      Pins     //
///////////////////

// Digital Pins 10 - 11 - 12 - 13 are reserved for ethernet shield

#define DIGITAL_PIN_ROTARY_ENCODER_DT 2 // Rotary Encoder - A
#define DIGITAL_PIN_ROTARY_ENCODER_CLK 3 // Rotary Encoder - B
#define DIGITAL_PIN_ROTARY_ENCODER_SW 4 // Rotary Encoder - Switch
#define DIGITAL_PIN_DISTANCE_SENSOR_TRIGGER 5 // HC-SR04
#define DIGITAL_PIN_DISTANCE_SENSOR_ECHO 6 // HC-SR04
#define DIGITAL_PIN_MOTION_SENSOR 7 // HC-SR501
#define DIGITAL_PIN_TEMPERATURE_HUMIDITY_SENSOR 8 //DHT-11
#define DIGITAL_PIN_SOIL_MOISTURE_SENSOR 9 // SOIL MOISTURE SENSOR UC-138

#define ANALOG_PIN_GAS_SENSOR A0 // MQ2

#pragma endregion

#pragma region include and declare

char* m_baseTopic = "/home/bedroom/";
char m_topicBuf[32], m_messageBuf[8];

#ifdef ENABLE_MQTT_CLIENT
#include <SPI.h>
#include <UIPEthernet.h>
#include <PubSubClient.h>

byte m_mac[] = { 0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
byte m_server[] = { 192, 168, 2, 19 };

EthernetClient m_ethClient;
PubSubClient m_mqttClient(m_server, 1883, mqttNotificationCallback, m_ethClient);
#endif

#ifdef ENABLE_TEMPERATURE_HUMIDITY_SENSOR
#include <SPI.h>
#include <DHT.h>
#define DELAY_TEMPERATURE_HUMIDITY_SENSOR 10000

DHT m_dht;
unsigned long m_timeTemperatureHumiditySensor;
float m_temperature, m_humidity;
#endif

#ifdef ENABLE_DISTANCE_SENSOR
#include <SPI.h>
#include <NewPing.h>
#define DELAY_DISTANCE_SENSOR 200

const long MAX_DISTANCE = 300;
unsigned long m_timeDistanceSensor;
NewPing m_Sonar(DIGITAL_PIN_DISTANCE_SENSOR_TRIGGER, DIGITAL_PIN_DISTANCE_SENSOR_ECHO, MAX_DISTANCE);
unsigned int m_distance;
#endif

#ifdef ENABLE_MOTION_SENSOR
#define DELAY_MOTION_SENSOR 500

bool m_motionTripped;
unsigned long m_timeMotionSensor;
#endif

#ifdef ENABLE_ROTARY_ENCODER
#define DELAY_ROTARY_ENCODER 10

volatile int m_encoderPos = 0;
int m_lastReportedPos = 1;
static boolean m_rotating = false;
unsigned long m_timeRotaryEncoder;

boolean m_InterruptASet = false;
boolean m_InterruptBSet = false;
#endif

#ifdef ENABLE_DISPLAY
#include "U8glib.h"
#define DELAY_DISPLAY 50
U8GLIB_SSD1306_128X64 m_display(U8G_I2C_OPT_NO_ACK);
unsigned long m_timeDisplay;
#endif

#ifdef ENABLE_GAS_SENSOR
#define RL_VALUE                     5     //define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR          9.83  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO, which is derived from the chart in datasheet
#define CALIBARAION_SAMPLE_TIMES     50    //define how many samples you are going to take in the calibration phase
#define CALIBRATION_SAMPLE_INTERVAL  500   //define the time interal(in milisecond) between each samples in the
#define READ_SAMPLE_INTERVAL         50    //define how many samples you are going to take in normal operation
#define READ_SAMPLE_TIMES            5     //define the time interal(in milisecond) between each samples in 
#define GAS_LPG                      0
#define GAS_CO                       1
#define GAS_SMOKE                    2
#define DELAY_GAS_SENSOR 50

float m_Ro = 10000.0;

float m_LPGCurve[3] = { 2.3, 0.21, -0.47 };
float m_COCurve[3] = { 2.3, 0.72, -0.34 };
float m_smokeCurve[3] = { 2.3, 0.53, -0.44 };

unsigned long m_timeGasSensor;
#endif

#ifdef ENABLE_SOIL_MOISTURE_SENSOR
#define DELAY_SOIL_MOISTURE_SENSOR 1000

bool m_soilMoistureTripped;
unsigned long m_timeSoilMoistureSensor;
#endif

#pragma endregion

#pragma region functions

void send(char* topic, char* msg) {
#ifdef ENABLE_SERIAL

   Serial.print(millis());
   Serial.print(" - ");
   Serial.print(topic);
   Serial.print(" ");
   Serial.println(msg);
   delay(10);

#endif

#ifdef ENABLE_MQTT_CLIENT

   if (!m_mqttClient.connected())
      reconnectMQTTClient();

   if (m_mqttClient.connected())
      m_mqttClient.publish(topic, msg);

#endif
}

void draw() {
   // https://code.google.com/p/u8glib/wiki/userreference#drawStr

#ifdef ENABLE_DISPLAY
   m_display.setFont(u8g_font_unifont);
   //m_display.setFont(u8g_font_osb21);

   int Ypos = 16;

#ifdef ENABLE_TEMPERATURE_HUMIDITY_SENSOR
   String temperature = String("T:") + String(int(m_temperature)) + String(".") + String(getDecimal(m_temperature, 1), DEC) + String("C");
   char temperatureBuf[15];
   temperature.toCharArray(temperatureBuf, 15);

   m_display.drawStr(0, Ypos, temperatureBuf);

   String humidity = String("H:") + String(int(m_humidity)) + String(".") + String(getDecimal(m_humidity, 1), DEC) + String("%");
   char humidityBuf[15];
   humidity.toCharArray(humidityBuf, 15);

   m_display.drawStr(64, Ypos, humidityBuf);
#else
   m_display.drawStr(0, Ypos, "T: N/A");
   m_display.drawStr(64, Ypos, "H: N/A");
#endif

   Ypos += 16;

#ifdef ENABLE_DISTANCE_SENSOR
   String distance = String("D: ") + String(m_distance, DEC) + String("cm");
   char distanceBuf[15];
   distance.toCharArray(distanceBuf, 15);

   m_display.drawStr(0, Ypos, distanceBuf);
#else
   m_display.drawStr(0, Ypos, "D: N/A");
#endif

   Ypos += 16;

#ifdef ENABLE_ROTARY_ENCODER
   String rotaryEncoder = String("RE: ") + String(m_encoderPos, DEC);
   char rotaryEncoderBuf[15];
   rotaryEncoder.toCharArray(rotaryEncoderBuf, 15);

   m_display.drawStr(0, Ypos, rotaryEncoderBuf);
#else
   m_display.drawStr(0, Ypos, "RE: N/A");
#endif

   Ypos += 16;

#ifdef ENABLE_MOTION_SENSOR
   String motion = String("Motion: ") + String((m_motionTripped) ? "true" : "false");
   char motionBuf[15];
   motion.toCharArray(motionBuf, 15);

   m_display.drawStr(0, Ypos, motionBuf);
#else
   m_display.drawStr(0, Ypos, "Motion: N/A");
#endif

#endif
}

long getDecimal(float val, int precision) {
   int intPart = int(val);
   long decPart = (10 * precision) * (val - intPart);
   if (decPart > 0)return(decPart);
   else if (decPart < 0)return((-1)*decPart);
   else if (decPart = 0)return(00);
}

#ifdef ENABLE_MQTT_CLIENT
void connectEthernet() {
   Serial.println("STATUS: connect ethernet");
   while (Ethernet.begin(m_mac) != 1)
   {
      Serial.println("ERROR: Cannot receive an IP address from DHCP server");
      delay(3000);
   }

   Serial.print("IP = ");
   Serial.println(Ethernet.localIP());
}

void reconnectMQTTClient() {
   Serial.println("ERROR: MQTT reconnect");
   m_mqttClient.disconnect();
   delay(3000);
   connectMQTTClient();
}

void connectMQTTClient() {
   Serial.println("STATUS: connect MQTT client");

   char ipBuf[24];
   sprintf(ipBuf, "arduino%d.%d.%d.%d", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);

   Serial.print("MQTT name = ");
   Serial.println(ipBuf);

   while (m_mqttClient.connect(ipBuf) != 1)
   {
      Serial.println("ERROR: Cannot connect to MQTT server");
      delay(3000);
   }

   Serial.println("MQTT: device ready");
   m_mqttClient.publish(m_baseTopic, "device ready");

   //String subscribeTopic = m_baseTopic + String("#");
   //cstr = new char[subscribeTopic.length() + 1];
   //strcpy(cstr, subscribeTopic.c_str());
   //m_mqttClient.subscribe(cstr);
   //delete cstr;
}
#endif

#ifdef ENABLE_GAS_SENSOR
// https://github.com/mysensors/Arduino/blob/master/libraries/MySensors/examples/AirQualitySensor/AirQualitySensor.ino

float MQResistanceCalculation(int raw_adc)
{
   return (((float)RL_VALUE*(1023 - raw_adc) / raw_adc));
}

float MQCalibration(int mq_pin)
{
   int i;
   float val = 0;

   for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {            //take multiple samples
      val += MQResistanceCalculation(analogRead(mq_pin));
      delay(CALIBRATION_SAMPLE_INTERVAL);
   }
   val = val / CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

   val = val / RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
   //according to the chart in the datasheet 

   return val;
}

float MQRead(int mq_pin)
{
   int i;
   float rs = 0;

   for (i = 0; i < READ_SAMPLE_TIMES; i++) {
      rs += MQResistanceCalculation(analogRead(mq_pin));
      delay(READ_SAMPLE_INTERVAL);
   }

   rs = rs / READ_SAMPLE_TIMES;

   return rs;
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
   if (gas_id == GAS_LPG) {
      return MQGetPercentage(rs_ro_ratio, m_LPGCurve);
   }
   else if (gas_id == GAS_CO) {
      return MQGetPercentage(rs_ro_ratio, m_COCurve);
   }
   else if (gas_id == GAS_SMOKE) {
      return MQGetPercentage(rs_ro_ratio, m_smokeCurve);
   }

   return 0;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
   return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
#endif

#pragma endregion

#pragma region callback

void mqttCallback() {
#ifdef ENABLE_MQTT_CLIENT
   Ethernet.maintain();
   m_mqttClient.loop();
#endif
}

void mqttNotificationCallback(char* topic, byte* payload, unsigned int length) {
#ifdef ENABLE_MQTT_CLIENT

#endif
}

void temperatureHumiditySensorCallback(){
#ifdef ENABLE_TEMPERATURE_HUMIDITY_SENSOR
   if (m_timeTemperatureHumiditySensor < millis()) {
   
      m_temperature = m_dht.getTemperature();
      sprintf(m_topicBuf, "%s%s", m_baseTopic, "temperature");
      dtostrf(m_temperature, 4, 2, m_messageBuf);
      send(m_topicBuf, m_messageBuf);

      m_humidity = m_dht.getHumidity();
      sprintf(m_topicBuf, "%s%s", m_baseTopic, "humidity");
      dtostrf(m_humidity, 4, 2, m_messageBuf);
      send(m_topicBuf, m_messageBuf);

      m_timeTemperatureHumiditySensor = millis() + DELAY_TEMPERATURE_HUMIDITY_SENSOR;
   }
#endif
}

void distanceSensorCallback() {
#ifdef ENABLE_DISTANCE_SENSOR
   if (m_timeDistanceSensor < millis()) {
      
      m_distance = m_Sonar.ping_cm();
      sprintf(m_topicBuf, "%s%s", m_baseTopic, "distance");
      sprintf(m_messageBuf, "%d", m_distance);
      send(m_topicBuf, m_messageBuf);

      m_timeDistanceSensor = millis() + DELAY_DISTANCE_SENSOR;
   }
#endif
}

void motionSensorCallback() {
#ifdef ENABLE_MOTION_SENSOR
   if (m_timeMotionSensor < millis()) {
      
      m_motionTripped = digitalRead(DIGITAL_PIN_MOTION_SENSOR);
      sprintf(m_topicBuf, "%s%s", m_baseTopic, "distance");
      sprintf(m_messageBuf, "%s", (m_motionTripped == HIGH) ? "true" : "false");
      send(m_topicBuf, m_messageBuf);
      
      m_timeMotionSensor = millis() + DELAY_MOTION_SENSOR;
   }
#endif
}

void rotaryEncoderCallback() {
#ifdef ENABLE_ROTARY_ENCODER
   if (m_timeRotaryEncoder < millis()) {

      m_rotating = true;

      if (m_lastReportedPos != m_encoderPos) {
         m_lastReportedPos = m_encoderPos;

         

         m_motionTripped = digitalRead(DIGITAL_PIN_MOTION_SENSOR);
         sprintf(m_topicBuf, "%s%s", m_baseTopic, "volume");
         sprintf(m_messageBuf, "%d", m_encoderPos);
         send(m_topicBuf, m_messageBuf);
      }
      if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_SW) == LOW)  {
         m_encoderPos = 0;
      }

      m_timeRotaryEncoder = millis() + DELAY_ROTARY_ENCODER;
   }
#endif
}

void displayCallback() {
#ifdef ENABLE_DISPLAY
   if (m_timeDisplay < millis()) {

      m_display.firstPage();
      do {
         draw();
      } while (m_display.nextPage());

      m_timeDisplay = millis() + DELAY_DISPLAY;
   }
#endif
}

void gasSensorCallback() {
#ifdef ENABLE_GAS_SENSOR
   if (m_timeGasSensor < millis()) {
            
      sprintf(m_topicBuf, "%s%s", m_baseTopic, "lpg");
      sprintf(m_messageBuf, "%d", MQGetGasPercentage(MQRead(ANALOG_PIN_GAS_SENSOR) / m_Ro, GAS_LPG));
      send(m_topicBuf, m_messageBuf);

      sprintf(m_topicBuf, "%s%s", m_baseTopic, "co");
      sprintf(m_messageBuf, "%d", MQGetGasPercentage(MQRead(ANALOG_PIN_GAS_SENSOR) / m_Ro, GAS_CO));
      send(m_topicBuf, m_messageBuf);

      sprintf(m_topicBuf, "%s%s", m_baseTopic, "smoke");
      sprintf(m_messageBuf, "%d", MQGetGasPercentage(MQRead(ANALOG_PIN_GAS_SENSOR) / m_Ro, GAS_SMOKE));
      send(m_topicBuf, m_messageBuf);

      m_timeGasSensor = millis() + DELAY_GAS_SENSOR;
   }
#endif
}

void soilMoistureSensorCallback() {
#ifdef ENABLE_SOIL_MOISTURE_SENSOR
   if (m_timeSoilMoistureSensor < millis()) {
            
      m_soilMoistureTripped = (digitalRead(DIGITAL_PIN_SOIL_MOISTURE_SENSOR) == HIGH);
      sprintf(m_topicBuf, "%s%s", m_baseTopic, "lpg");
      sprintf(m_messageBuf, "%s", (m_soilMoistureTripped) ? "false" : "true");
      send(m_topicBuf, m_messageBuf);

      m_timeSoilMoistureSensor = millis() + DELAY_SOIL_MOISTURE_SENSOR;
   }
#endif
}

#pragma endregion

#pragma region interrupt

void doEncoderA() {
#ifdef ENABLE_ROTARY_ENCODER
   //if (m_rotating) delay(1);

   //if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_DT) != m_InterruptASet) {
   //   m_InterruptASet = !m_InterruptASet;

   //   if (m_InterruptASet && !m_InterruptBSet)
   //      m_encoderPos += 1;

   //   m_rotating = false;
   //}

   // look for a low-to-high on channel A
   if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_DT) == HIGH) {
      // check channel B to see which way encoder is turning
      if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_CLK) == LOW) {
         m_encoderPos++;         // CW
      }
      else {
         m_encoderPos--;         // CCW
      }
   }
   else   // must be a high-to-low edge on channel A                                       
   {
      // check channel B to see which way encoder is turning  
      if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_CLK) == HIGH) {
         m_encoderPos++;          // CW
      }
      else {
         m_encoderPos--;          // CCW
      }
   }
#endif
}

void doEncoderB() {
#ifdef ENABLE_ROTARY_ENCODER
   //if (m_rotating) delay(1);

   //if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_CLK) != m_InterruptBSet) {
   //   m_InterruptBSet = !m_InterruptBSet;

   //   if (m_InterruptBSet && !m_InterruptASet)
   //      m_encoderPos -= 1;

   //   m_rotating = false;
   //}

   // look for a low-to-high on channel B
   if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_CLK) == HIGH) {
      // check channel A to see which way encoder is turning
      if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_DT) == HIGH) {
         m_encoderPos++;         // CW
      }
      else {
         m_encoderPos--;         // CCW
      }
   }
   // Look for a high-to-low on channel B
   else {
      // check channel B to see which way encoder is turning  
      if (digitalRead(DIGITAL_PIN_ROTARY_ENCODER_CLK) == LOW) {
         m_encoderPos++;          // CW
      }
      else {
         m_encoderPos--;          // CCW
      }
   }
#endif
}

#pragma endregion

#pragma region setup

void setupSerial() {
#ifdef ENABLE_SERIAL
   Serial.begin(115200);
   Serial.println("Serial enabled");
#endif
}

void setupMQTTClient() {
#ifdef ENABLE_MQTT_CLIENT
   connectEthernet();
   connectMQTTClient();
#endif
}

void setupDebugLED() {
#ifdef ENABLE_DEBUG_LED
   pinMode(DIGITAL_PIN_DEBUG_LED, OUTPUT);
#endif
}

void setupTemperatureHumiditySensor() {
#ifdef ENABLE_TEMPERATURE_HUMIDITY_SENSOR
   m_dht.setup(DIGITAL_PIN_TEMPERATURE_HUMIDITY_SENSOR);
   send("/home/global", "Temperature/Humidity sensor enabled");
#else
   send("/home/global", "Temperature/Humidity sensor disabled");
#endif
}

void setupMotionSensor() {
#ifdef ENABLE_MOTION_SENSOR
   pinMode(DIGITAL_PIN_MOTION_SENSOR, INPUT);
   send("/home/global", "Motion sensor enabled");
#else
   send("/home/global", "Motion sensor disabled");
#endif
}

void setupRotaryEncoder() {
#ifdef ENABLE_ROTARY_ENCODER
   pinMode(DIGITAL_PIN_ROTARY_ENCODER_CLK, INPUT_PULLUP);
   pinMode(DIGITAL_PIN_ROTARY_ENCODER_DT, INPUT_PULLUP);
   pinMode(DIGITAL_PIN_ROTARY_ENCODER_SW, INPUT_PULLUP);

   attachInterrupt(0, doEncoderA, CHANGE);
   attachInterrupt(1, doEncoderB, CHANGE);

   send("/home/global", "Rotary encoder enabled");
#else
   send("/home/global", "Rotary encoder disabled");
#endif
}

void setupDistanceSensor() {
#ifdef ENABLE_DISTANCE_SENSOR
   send("/home/global", "Distance sensor enabled");
#else
   send("/home/global", "Distance sensor disabled");
#endif
}

void setupDisplay() {
#ifdef ENABLE_DISPLAY
   // m_display.setRot180();

   if (m_display.getMode() == U8G_MODE_R3G3B2) {
      m_display.setColorIndex(255);     // white
   }
   else if (m_display.getMode() == U8G_MODE_GRAY2BIT) {
      m_display.setColorIndex(3);         // max intensity
   }
   else if (m_display.getMode() == U8G_MODE_BW) {
      m_display.setColorIndex(1);         // pixel on
   }
   else if (m_display.getMode() == U8G_MODE_HICOLOR) {
      m_display.setHiColorByRGB(255, 255, 255);
   }

   send("/home/global", "Display enabled");
#else
   send("/home/global", "Display disabled");
#endif
}

void setupGasSensor() {
#ifdef ENABLE_GAS_SENSOR
   m_Ro = MQCalibration(ANALOG_PIN_GAS_SENSOR);
   send("/home/global", "Gas sensor enabled");
#else
   send("/home/global", "Gas sensor disabled");
#endif
}

void setupSoilMoistureSensor() {
#ifdef ENABLE_SOIL_MOISTURE_SENSOR
   pinMode(DIGITAL_PIN_SOIL_MOISTURE_SENSOR, INPUT);
   send("/home/global", "Soil moisture enabled");
#else
   send("/home/global", "Soil moisture disabled");
#endif
}

#pragma endregion

int main(void) {
   init();

   setupSerial();
   setupMQTTClient();

   setupDisplay();
   
   setupTemperatureHumiditySensor();
   setupMotionSensor();
   setupRotaryEncoder();
   setupDistanceSensor();
   setupGasSensor();
   setupSoilMoistureSensor();


   while (1) {
      mqttCallback();
      temperatureHumiditySensorCallback();
      motionSensorCallback();
      rotaryEncoderCallback();
      distanceSensorCallback();
      displayCallback();
      gasSensorCallback();
      soilMoistureSensorCallback();
   }
}
