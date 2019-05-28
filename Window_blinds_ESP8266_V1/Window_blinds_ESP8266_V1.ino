#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
extern "C" {
  #include "user_interface.h"
}

//-------------- String declairation--------------------------
const char* ssid = "LakeViewWiFi";
const char* password = "P@ssLakeView";
const char* mqtt_server = "192.168.2.12";
const char* mqtt_uname = "onkar20";
const char* mqtt_pass = "onkar20";
const char* mqtt_device_name = "ESP8266WindowBlind1";
const char* ota_device_name = "Living_Room_Window_Blind";
const char* ota_password = "onkar20";

//-------------variable declaration
const int dirPin = 5;
const int stepPin = 4;
const int enablePin = 2;

const int lightPin = 2;

char msg[50];
int stepper_direction = 0;
int rotationSpeed = 1000;

WiFiClient espClient;
PubSubClient client(espClient);

//----------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();
  setup_OTA();
  pinMode(lightPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  digitalWrite(lightPin, LOW);
  digitalWrite(enablePin, HIGH);
}

//----------------------------------------------------------------------------------------------------
void setup_OTA() {
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(ota_device_name);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)ota_password);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
//----------------------------------------------------------------------------------------------------
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //for Light sleep
  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T); 
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(lightPin, !digitalRead(lightPin));
  }

  digitalWrite(lightPin, HIGH);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//----------------------------------------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived new [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic,"home/livingRoom/windowBlind/command")==0)
  {
    // 0 = stop
    if ((char)payload[0] == '0') 
    {
      stepper_direction = 0;
      client.publish("home/livingRoom/windowBlind/state", "0");
    }
    // 1 = left
    else if ((char)payload[0] == '1')
    {
      stepper_direction = 1;
      client.publish("home/livingRoom/windowBlind/state", "1");
    }
    // 2 = left
    else if ((char)payload[0] == '2')
    {
      stepper_direction = 2;
      client.publish("home/livingRoom/windowBlind/state", "2");
    }
  }

  if (strcmp(topic,"home/livingRoom/windowBlind/speed")==0)
  {
    String speedString = (char*)payload;
    rotationSpeed = speedString.toInt();
  }
}

//----------------------------------------------------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_device_name, mqtt_uname, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe("home/livingRoom/windowBlind/command");
      client.subscribe("home/livingRoom/windowBlind/speed");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//---------------------------------------------------------------------------------------------------
void blinkLED (int noOfTimes) {
  for(int i=0; i< noOfTimes; i++) {
    digitalWrite(lightPin, LOW);
    delay(100);
    digitalWrite(lightPin, HIGH);
    delay(100);
  }
}

//---------------------------------------------------------------------------------------------------
void rotateStepper(int steps)
{
  if (steps == 0)
  {
    digitalWrite(enablePin, HIGH);
    return;
  }
  else if(steps < 0)
  {
    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, LOW);
  }
  else if(steps > 0)
  {
    digitalWrite(enablePin, LOW);
    digitalWrite(dirPin, HIGH);
  }

  for(int i = 0; i <= abs(steps); i++)
  {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(rotationSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(rotationSpeed);
  }
  digitalWrite(enablePin, HIGH);
}
//----------------------------------------------------------------------------------------------------
void loop() {
  ArduinoOTA.handle();
  
  if (!client.connected()) {
    reconnect();
  }

  if(stepper_direction == 0)
  {
    rotateStepper(0);
  }
  else if (stepper_direction == 1)
  {
    rotateStepper(1);
  }
  else if(stepper_direction == 2)
  {
    rotateStepper(-1);
  }
  else
  {
    rotateStepper(0);
  }
  client.loop();
}
