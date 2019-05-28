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
const int leftLimitSw = 12;
const int rightLimitSw = 13;
const int leftSw = 16;
const int rightSw = 14;
const int rotationSpeed = 1000;

char msg[50];
int masterStep = 0;
int maxSteps = 0;

WiFiClient espClient;
PubSubClient client(espClient);

//----------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  setup_wifi();
  setup_OTA();
  
  // Setup DRV8825 pins as output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // Setup limit switches
  pinMode(leftLimitSw, INPUT);
  digitalWrite(leftLimitSw, HIGH);
  pinMode(rightLimitSw, INPUT);
  digitalWrite(rightLimitSw, HIGH);

  // setup two switches
  pinMode(leftSw, INPUT);
  digitalWrite(leftSw, HIGH);
  pinMode(rightSw, INPUT);
  digitalWrite(rightSw, HIGH);
  
  // setup connection with MQTT server 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Perform initial calibration to find min and max position
  calibration();
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
  }

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

  int blindPosition = ((char*)payload).toInt();

  int nextSteps = blindPosition * maxSteps;
  
  // call the rotate stepper function here with relative steps
  
  client.publish("home/livingRoom/windowBlind/state", payload);
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
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//----------------------------------------------------------------------------------------------------
void disableStepper()
{
  // set enable pin high to disable stepper driver
  digitalWrite(enablePin, HIGH);
}
//----------------------------------------------------------------------------------------------------
void enableStepper()
{
  // set enable pin to LOW to enable stepper driver
  digitalWrite(enablePin, LOW);
}
//----------------------------------------------------------------------------------------------------
void directionLeft()
{
  // Set direction to counterclock 
  digitalWrite(dirPin, HIGH);
}
//----------------------------------------------------------------------------------------------------
void directionRight()
{
  // Set direction to clockwise 
  digitalWrite(dirPin, LOW);
}

//----------------------------------------------------------------------------------------------------
void calibration()
{
  // Enable stepper driver
  enableStepper();
  // Set direction to counterclock / left
  directionLeft();

  // Run the loop untill limit SW hit
  while(digitalRead(leftLimitSw))
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(rotationSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(rotationSpeed);
  }

  // Set direction to clockwise / right
  directionRight();
  int localSteps = 0;
  // Run the loop untill limit SW hit
  while(digitalRead(rightLimitSw))
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(rotationSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(rotationSpeed);
    ++localSteps;
  }
  maxSteps = localSteps;
  // Since the blinds are at max position, set masterStep to max steps
  masterStep = localSteps;
  
  disableStepper();
}

//---------------------------------------------------------------------------------------------------
void rotateStepper(int steps)
{
  // No movement needed, disable stepper
  if (steps == 0)
  {
    disableStepper();
    return;
  }
  // -ve steps requested, rotate stepper to left towards -> 0
  else if(steps < 0)
  {
    enableStepper();
    directionLeft();
  }
  // +ve steps requested, rotate stepper to right towards -> 100
  else if(steps > 0)
  {
    enableStepper();
    directionRight();
  }

  // Send number of steps to driver
  for(int i = 0; i <= abs(steps); i++)
  {
    // These 4 lines = 1 Step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(rotationSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(rotationSpeed);
  }
  // disable stepper once movement is done to save power
  disableStepper();
}

//----------------------------------------------------------------------------------------------------
void loop() {
  // This function is called to receive OTA updates
  ArduinoOTA.handle();
  
  // if MQTT connection lost, reconnect
  if (!client.connected()) {
    reconnect();
  }

  // --------------- Application code ---------------------

  
  // --------------- end Application code ---------------------
  // Required!!!, this is to check for new MQTT updates from server
  client.loop();
}
