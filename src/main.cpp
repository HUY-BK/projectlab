#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
 #include <SPI.h>


#define bienTro 16

#define nutAn 22

unsigned long lastTime = 0, startTime = 0;

const char *ssid = "Tom Dong Bich";
const char *password = "hi09082015";

#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_PORT 1883

#define MQTT_TOPIC "devID\valve"

WiFiClient wifiClient;
PubSubClient client(wifiClient);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setWiFi()
{
  Serial.print("Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("...");
  }
}
void setBNO055()
{
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Khoi tao BNO055");
  if (!bno.begin())
  {
    Serial.println("ERROR");
    while(1);
  }
  else
  {
    Serial.println("Khoi tao thanh cong");
  }
}
void connect_to_broker()
{
  while ((!client.connected()))
  {
    Serial.println("MQTT connection");
    String clientId = "esp32";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str()))
    {
      Serial.println("NQTT Connected");
      client.subscribe(MQTT_TOPIC);
    }
    else
    {
      Serial.println("FAIED");
      delay(2000);
    }
  }
}
int giatri_bienTro = 0;
void read_bienTro()
{
  giatri_bienTro = map(analogRead(bienTro), 0, 4095, 0, 100);
  
}
double tx, ty, tz, gx, gy, gz;
void readBNO055()
{
  tx=-1000000,ty=-1000000,tz=-1000000, gx=-1000000,gy=-1000000,gz=-1000000;
  sensors_event_t accelerometerData, orientationData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  tx =accelerometerData.acceleration.x;
  ty =accelerometerData.acceleration.y;
  tz =accelerometerData.acceleration.z;
  gx = orientationData.orientation.x;
  gy =orientationData.orientation.y;
  gz = orientationData.orientation.z;

}
void setup()
{
  Serial.begin(9600);
  setWiFi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  connect_to_broker();
  setBNO055();
  pinMode(bienTro, INPUT);
  pinMode(nutAn, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_22, 0);
}
int count = 0;
void loop()
{
 if (!client.connected()) {
    connect_to_broker();
  }
  client.loop();

  if (digitalRead(nutAn) == 0)
  {
    if(millis()- lastTime >= 0.1){
      read_bienTro();
      lastTime = millis();
    }
    readBNO055();

    if ((giatri_bienTro == 0) && (count == 0))
    {
      client.publish(MQTT_TOPIC, "Van Đóng");
      count = 1;
    }
    else if (giatri_bienTro != 0)
    {
      if (count == 1)
      {
        startTime = millis();
      }
      JsonDocument doc;
      doc["f"] = giatri_bienTro;
      doc["t"] = millis() - startTime;

      char jsonBuffer[512];
      serializeJson(doc, jsonBuffer);
      if(millis()- lastTime >= 0.5){
      client.publish(MQTT_TOPIC, jsonBuffer);
      }
      
      count = 0;
    }
  }
  else
  {
    esp_deep_sleep_start();
  }
}