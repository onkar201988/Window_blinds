#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
extern "C" {
  #include "user_interface.h"
}

//-------------- String declairation--------------------------
const char* ssid                = "LakeViewWiFi";
const char* password            = "P@ssLakeView";
const char* mqtt_server         = "192.168.2.12";
const char* mqtt_uname          = "onkar20";
const char* mqtt_pass           = "onkar20";
const char* mqtt_device_name    = "ESP8266WindowBlind1";
const char* ota_device_name     = "Living_Room_Window_Blind";
const char* ota_password        = "onkar20";

//-------------variable declaration ---------------------------------------------
//-------Pin declaration---------------
const int dirPin          = 5;
const int stepPin         = 4;
const int enablePin       = 2;
const int leftLimitSw     = 12;
const int rightLimitSw    = 13;
const int leftSw          = 16;
const int rightSw         = 14;

//------------ Global variables -------
char msg[50];
int masterStep            = 0;  // Holds the current position of motor in steps
int requestedSteps        = 0;  // Number of steps to left or right requested
int currentSteps          = 0;  // Current counter of steps, to reach requestedSteps
int maxSteps              = 0;  // Right limit for motor in steps

unsigned long leftButtonPressedTime;  // Time stamp when button is pressed
unsigned long leftButtonReleasedTime; // Time stamp when button is released
bool leftButtonPressedFlag = false;   // Flag to store when button is pressed

unsigned long rightButtonPressedTime;   // Time stamp when button is pressed
unsigned long rightButtonReleasedTime;  // Time stamp when button is released
bool rightButtonPressedFlag = false;    // Flag to store when button is pressed

//-------- Global const parameters ------------
// long press time for button press
const unsigned long longPressTime   = 1000; // Time for switch must be pressed
const int rotationSpeed             = 1000; // Delay between two steps

//-------- External object declairation-------------
WiFiClient espClient;
PubSubClient client(espClient);

//----------State machine variables -----------------
enum masterState_E {
  CALIBRATION,
  ROTATE_STEPPER,
  SW_LEFT,
  SW_RIGHT,
  IDEL_ST
};
enum masterState_E masterState = CALIBRATION;

enum calibrationState_E {
  INIT,
  LEFT,
  RIGHT,
  DONE
};
enum calibrationState_E calibration = INIT;
int calibraionLocalSteps = 0;

//---------------------------- Setup Function --------------------------------------
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

  // setup two push button switches
  pinMode(leftSw, INPUT);
  digitalWrite(leftSw, HIGH);
  pinMode(rightSw, INPUT);
  digitalWrite(rightSw, HIGH);
  
  // setup connection with MQTT server 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

//--------------------------------setup_OTA function --------------------------------------------
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

//---------------------------------- setup_wifi function ---------------------------------------
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

//----------------------------------- MQTT callback function --------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived new [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String blindPosition_str = (char*)payload;
  int blindPosition = blindPosition_str.toInt();
  
  // Constrain the value if it is out of range
  blindPosition = constrain(blindPosition, 0, 100);
  
  // Map the value from % to no. steps
  int nextPositionSteps = map(blindPosition, 0, 100, 0, maxSteps);

  requestedSteps = nextPositionSteps - masterStep;

  if(requestedSteps = 0)
  {
    Serial.println("MasterState:MQTT -> IDEL_ST");
    masterState = IDEL_ST;
  }
  else if(requestedSteps < 0)
  {
    requestedSteps = abs(requestedSteps);
    masterState = ROTATE_STEPPER;
    enableStepper();
    directionLeft();
    Serial.println("MasterState:MQTT -> ROTATE_STEPPER left");
  }
  else if(requestedSteps > 0)
  {
    masterState = ROTATE_STEPPER;
    enableStepper();
    directionRight();
    Serial.println("MasterState:MQTT -> ROTATE_STEPPER right");
  }
  
  client.publish("home/livingRoom/windowBlind/state", blindPosition);
}

//------------------------------ MQTT reconnect function ---------------------------------------------
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
bool isLeftLimitReached()
{
  // Return true when left limit sw is triggered
  return !(digitalRead(leftLimitSw));
}
//----------------------------------------------------------------------------------------------------
bool isRightLimitReached()
{
  // Return true when right limit sw is triggered
  return !(digitalRead(rightLimitSw));
}
//----------------------------------------------------------------------------------------------------
bool isLeftButtonPressed()
{
  return !(digitalRead(leftSw));
}
//----------------------------------------------------------------------------------------------------
bool isRightButtonPressed()
{
  return !(digitalRead(rightSw));
}
//----------------------------------------------------------------------------------------------------
void oneStep()
{
  // Rotate stepper one step
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(rotationSpeed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(rotationSpeed); 
}

//----------------------------------------------------------------------------------------------------
void calibrationStateMachine()
{
  switch (calibration) {
    case INIT:
      Serial.println("Calibrating: INIT");
      // Enable stepper driver
      enableStepper();
      // Set direction to counterclock / left
      directionLeft();
      calibration = LEFT;
      Serial.println("Calibrating:LEFT");

    case LEFT:
     // Run the loop until left limit not reacehd
      if(!isLeftLimitReached())
      {
        oneStep();
        Serial.print(".");
      }
      else
      {
        directionRight();
        // Set direction to clockwise / right
        directionRight();
        calibration = RIGHT;
        Serial.println("Calibrating:RIGHT");
      }
      
    case RIGHT:
        // Run the loop until right limit not reached
        if(!isRightLimitReached())
        {
          oneStep();
          ++calibraionLocalSteps;
        }
        else
        {
          maxSteps = calibraionLocalSteps;
          // Since the blinds are at max position, set masterStep to max steps
          masterStep = calibraionLocalSteps;
          calibration = DONE;
          
          Serial.println("Max steps = ");
          Serial.println(maxSteps);
          Serial.println("Calibrating:DONE");
        }
    
    case DONE:
        disableStepper();
        Serial.println("MasterState:Calibration ->IDEL_ST");
        masterState = IDEL_ST;
  }  
}

//---------------------------------------------------------------------------------------------------
void rotateStepperFunction()
{
  if( (currentSteps < requestedSteps) || (!isRightLimitReached()) || (!isLeftLimitReached()) || (!isLeftButtonPressed()) || (!isRightButtonPressed()) )
  {
    oneStep();
    ++currentSteps;
  }
  else
  {
    if(digitalRead(dirPin, HIGH)) // Direction is left
    {
      masterStep = masterStep - currentSteps;
    }
    else  // Direction is right
    {
      masterStep = masterStep + currentSteps;
    }
    currentSteps = 0;
    disableStepper();
    masterState = IDEL_ST;
    
    Serial.println("MasterState:ROTATE_STEPPER -> IDEL_ST");
  } 
}

//---------------------------------------------------------------------------------------------------
void leftSwFunction()
{
  // Left button just pressed, and right button is not pressed
  if(isLeftButtonPressed() && !leftButtonPressedFlag && !rightButtonPressedFlag)
  {
    leftButtonPressedTime = millis();
    leftButtonPressedFlag = true;
    // if button pressed quickly after recent release, that means it is double press
    if((leftButtonPressedTime - leftButtonReleasedTime) < 500)
    {
      // Set the blinds all the way to left
      requestedSteps = masterStep;
      masterState = ROTATE_STEPPER;
      enableStepper();
      directionLeft();
    }
  }
  // Button pressed earlier, but not released
  else if(isLeftButtonPressed() && leftButtonPressedFlag)
  {
    // If button pressed for longer time, then start moving the blinds to left until button is pressed
    if( (millis() - leftButtonPressedTime) > longPressTime)
    {
      directionLeft();
      enableStepper();
      oneStep();
    }
    // Do nothing
    else
    {}
  }
  // button just released 
  else if(!isLeftButtonPressed() && leftButtonPressedFlag)
  {
    leftButtonReleasedTime = millis();
    leftButtonPressedFlag = false;
    disableStepper();
    Serial.println("MasterState:LeftSw ->IDEL_ST");
    masterState = IDEL_ST;
  }
}

//---------------------------------------------------------------------------------------------------
void rightSwFunction()
{
  // Right button just pressed, and left button is not pressed 
  if(isRightButtonPressed() && !rightButtonPressedFlag && !leftButtonPressedFlag)
  {
    rightButtonPressedTime = millis();
    rightButtonPressedFlag = true;
    // if button pressed quickly after recent release, that means it is double press
    if((rightButtonPressedTime - rightButtonReleasedTime) < 500)
    {
      // Set the blinds all the way to right
      requestedSteps = maxSteps - masterStep;
      masterState = ROTATE_STEPPER;
      enableStepper();
      directionRight();
    }
  }
  // Button pressed earlier, but not released
  else if(isRightButtonPressed() && rightButtonPressedFlag)
  {
    // If button pressed for longer time, then start moving the blinds to right until button is pressed
    if( (millis() - rightButtonPressedTime) > longPressTime)
    {
      directionRight();
      enableStepper();
      oneStep();
    }
    // Do nothing
    else
    {}
  }
  // button just released 
  else if(!isRightButtonPressed() && rightButtonPressedFlag)
  {
    rightButtonReleasedTime = millis();
    rightButtonPressedFlag = false;
    disableStepper();
    Serial.println("MasterState:rightSw ->IDEL_ST");
    masterState = IDEL_ST;
  }
}

//---------------------------------------------------------------------------------------------------
void masterStateMachine()
{
  switch (masterState) {
    case CALIBRATION:
      calibrationStateMachine();

    case ROTATE_STEPPER:
      rotateStepperFunction();
    
    case SW_LEFT:
      leftSwFunction();

    case SW_RIGHT:
      rightSwFunction();

    case IDEL_ST:      
      if(isLeftButtonPressed())
      {
        masterState = SW_LEFT;
      }
      else if(isRightButtonPressed())
      {
        masterState = SW_RIGHT;
      }
  }
}

//----------------------------------------------------------------------------------------------------
void loop() {
  // This function is called to receive OTA updates
  ArduinoOTA.handle();
  
  // if MQTT connection lost, reconnect
  if (!client.connected()) {
    reconnect();
  }

  masterStateMachine();

  // Fixed delay of 10 mSec
  delay(10);
  // --------------- end Application code ---------------------
  // Required!!!, this is to check for new MQTT updates from server
  client.loop();
}
