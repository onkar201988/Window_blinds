#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
extern "C" {
  #include "user_interface.h"
}

//#define debug                   // comment this line to remove serial prints
//-------------- String declairation--------------------------
const char* ssid                = "LakeViewWiFi";
const char* password            = "P@ssLakeView";
const char* mqtt_server         = "192.168.2.12";
const char* mqtt_uname          = "onkar20";
const char* mqtt_pass           = "onkar20";
const char* mqtt_device_name    = "ESP8266WindowBlind1";
const char* ota_device_name     = "Living_Room_Window_Blind";
const char* ota_password        = "onkar20";
const char* mqtt_topic_command  = "home/livingRoom/windowBlind/command";
const char* mqtt_topic_state    = "home/livingRoom/windowBlind/state";

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
int nextPositionSteps     = 0;  // Number of steps to left or right requested
//int currentSteps          = 0;  // Current counter of steps, to reach nextPositionSteps
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
const int rotationSpeed             = 200; // Delay between two steps

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
enum calibrationState_E calibrationState = INIT;
int calibrationLocalSteps = 0;

//---------------------------- Setup Function --------------------------------------
void setup() {
  #ifdef debug
    Serial.begin(115200);
  #endif
  setup_wifi();
  setup_OTA();
  
  // Setup DRV8825 pins as output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // Setup limit switches
  pinMode(leftLimitSw, INPUT_PULLUP);
  pinMode(rightLimitSw, INPUT_PULLUP);

  // setup two push button switches
  pinMode(leftSw, INPUT_PULLUP);
  pinMode(rightSw, INPUT_PULLUP);
  
  // setup connection with MQTT server 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
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

//---------------------------------- setup_wifi function ---------------------------------------
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  #ifdef debug
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  #endif
  //for Light sleep
  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T); 
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef debug
      Serial.print(".");
    #endif
  }

  #ifdef debug
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  #endif
}

//--------------------------------setup_OTA function --------------------------------------------
void setup_OTA() {
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(ota_device_name);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)ota_password);

  #ifdef debug
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
  #endif
  ArduinoOTA.begin();
}

//------------------------------ MQTT reconnect function ---------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    #ifdef debug
      Serial.print("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if (client.connect(mqtt_device_name, mqtt_uname, mqtt_pass)) {
      #ifdef debug
        Serial.println("connected");
      #endif
      client.subscribe(mqtt_topic_command);
    } else {
      #ifdef debug
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      #endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//----------------------------------- MQTT callback function --------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  #ifdef debug
    Serial.print("Message arrived new [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  #endif

  payload[length] = '\0';                     // Make payload a string by NULL terminating it.
  int blindPosition = atoi((char *)payload);  // Convert string array to int 
  #ifdef debug
    Serial.print("blindPosition int = ");
    Serial.print(blindPosition);
    Serial.println();
  #endif
  
  // Constrain the value if it is out of range
  blindPosition = constrain(blindPosition, 0, 100);
  #ifdef debug
    Serial.print("blindPosition constraint int = ");
    Serial.print(blindPosition);
    Serial.println();
  #endif
  
  // Map the value from % to no. steps
  nextPositionSteps = map(blindPosition, 0, 100, 0, maxSteps);

  #ifdef debug
    Serial.print("nextPositionSteps = ");
    Serial.print(nextPositionSteps);
    Serial.println();
  #endif
  
  if(nextPositionSteps == masterStep)
  {
    #ifdef debug
      Serial.println("MasterState:MQTT -> IDEL_ST");
    #endif
    masterState = IDEL_ST;
  }
  else if(nextPositionSteps < masterStep)
  {
    directionLeft();
    enableStepper();
    masterState = ROTATE_STEPPER;
    #ifdef debug
      Serial.println("MasterState:MQTT -> ROTATE_STEPPER left");
    #endif
  }
  else if(nextPositionSteps > masterStep)
  {
    directionRight();
    enableStepper();
    masterState = ROTATE_STEPPER;
    #ifdef debug
      Serial.println("MasterState:MQTT -> ROTATE_STEPPER right");
    #endif
  }
}

//----------------------------------------------------------------------------------------------------
void calibrationStateMachine()
{
  switch (calibrationState) {
    case INIT:
      #ifdef debug
        Serial.println("Calibrating: INIT");
      #endif
      // Set direction to counterclock / left
      directionLeft();
      // Enable stepper driver
      enableStepper();
      calibrationState = LEFT;
      #ifdef debug
        Serial.println("Calibrating:LEFT");
      #endif
      break;

    case LEFT:
     // Run the loop until left limit not reacehd
      if(!isLeftLimitReached())
      {
        oneStep();
        #ifdef debug
          Serial.print("->");
        #endif
      }
      else
      {
        directionRight();
        // Set direction to clockwise / right
        directionRight();
        calibrationState = RIGHT;
        #ifdef debug
          Serial.println("Calibrating:RIGHT");
        #endif
      }
      break;
      
    case RIGHT:
        // Run the loop until right limit not reached
        if(!isRightLimitReached())
        {
          oneStep();
          ++calibrationLocalSteps;
          #ifdef debug
            Serial.print("<-");
          #endif
        }
        else
        {
          maxSteps = calibrationLocalSteps;
          // Since the blinds are at max position, set masterStep to max steps
          masterStep = calibrationLocalSteps;
          calibrationState = DONE;
          
          #ifdef debug
            Serial.println("Max steps = ");
            Serial.println(maxSteps);
            Serial.println("Calibrating:DONE");
          #endif
        }
        break;
    
    case DONE:
        disableStepper();
        #ifdef debug
          Serial.println("MasterState:Calibration ->IDEL_ST");
        #endif
        masterState = IDEL_ST;
        break;

    default:
        break;
  }  
}

//---------------------------------------------------------------------------------------------------
void rotateStepperFunction()
{
  if(digitalRead(dirPin)) // Direction is left
  {
    if( ( masterStep > nextPositionSteps) && (!isLeftLimitReached()))
    {
      oneStep();
      --masterStep;
    }
    else
    {
      #ifdef debug
        Serial.print("masterStep = ");
        Serial.print(masterStep);
        Serial.println();
      #endif
      // Send the current state of the blind in %
      client.publish(mqtt_topic_state, String((int)(((float)masterStep / (float)maxSteps)*100)).c_str());

      disableStepper();
      masterState = IDEL_ST;
      
      #ifdef debug
        Serial.println("MasterState:ROTATE_STEPPER -> IDEL_ST");
      #endif
    }
  }
  else
  {
    if( ( masterStep < nextPositionSteps) && (!isRightLimitReached()) )
    {
      oneStep();
      ++masterStep;
    }
    else
    {
      #ifdef debug
        Serial.print("masterStep = ");
        Serial.print(masterStep);
        Serial.println();
      #endif
      // Send the current state of the blind in %
      client.publish(mqtt_topic_state, String((int)(((float)masterStep / (float)maxSteps)*100)).c_str());

      disableStepper();
      masterState = IDEL_ST;
      
      #ifdef debug
        Serial.println("MasterState:ROTATE_STEPPER -> IDEL_ST");
      #endif
    }
  }
}

//---------------------------------------------------------------------------------------------------
void leftSwFunction()
{
  // Left button just pressed, right button press is not possible
  if( isLeftButtonPressed() && !leftButtonPressedFlag )
  {
    leftButtonPressedTime = millis();
    leftButtonPressedFlag = true;
    // if button pressed quickly after recent release, that means it is double press
    if((leftButtonPressedTime - leftButtonReleasedTime) < 500)
    {
      // Set the blinds all the way to left
      nextPositionSteps = masterStep;
      directionLeft();
      enableStepper();
      masterState = ROTATE_STEPPER;
      
      #ifdef debug
        Serial.println("MasterState:LeftSw ->ROTATE_STEPPER");
      #endif
    }
  }
  // Button pressed earlier, but not released
  else if(isLeftButtonPressed() && leftButtonPressedFlag)
  {
    // If button pressed for longer time, then start moving the blinds to left until button is pressed
    if( ((millis() - leftButtonPressedTime) > longPressTime) && (0 != masterStep) )
    {
      directionLeft();
      enableStepper();
      oneStep();
      // Update master steps along with motor rotation
      masterStep--;

      #ifdef debug
        Serial.println("MasterState:LeftSw Long press");
      #endif
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
    masterState = IDEL_ST;
    // Send the current state of the blind in % since long press might be used
    client.publish(mqtt_topic_state, String((int)(((float)masterStep / (float)maxSteps)*100)).c_str());
    
    #ifdef debug
      Serial.println("MasterState:LeftSw ->IDEL_ST");
    #endif
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
      nextPositionSteps = maxSteps - masterStep;
      directionRight();
      enableStepper();
      masterState = ROTATE_STEPPER;
      
      #ifdef debug
        Serial.println("MasterState:rightSw ->ROTATE_STEPPER");
      #endif
    }
  }
  // Button pressed earlier, but not released
  else if(isRightButtonPressed() && rightButtonPressedFlag)
  {
    // If button pressed for longer time, then start moving the blinds to right until button is pressed 
    // or masterSteps != maxSteps
    if( ((millis() - rightButtonPressedTime) > longPressTime) && (maxSteps != masterStep) )
    {
      directionRight();
      enableStepper();
      oneStep();
      // Update master steps along with motor rotation
      masterStep++;
      
      #ifdef debug
        Serial.println("MasterState:RightSw Long press");
      #endif
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
    // Send the current state of the blind in % since long press might be used
    client.publish(mqtt_topic_state, String((int)(((float)masterStep / (float)maxSteps)*100)).c_str());
    
    #ifdef debug
      Serial.println("MasterState:rightSw ->IDEL_ST");
    #endif
    masterState = IDEL_ST;
  }
}

//---------------------------------------------------------------------------------------------------
void masterStateMachine()
{
  switch (masterState) {
    case CALIBRATION:
      calibrationStateMachine();
      break;

    case ROTATE_STEPPER:
      rotateStepperFunction();
      break;
    
    case SW_LEFT:
      leftSwFunction();
      break;

    case SW_RIGHT:
      rightSwFunction();
      break;

    case IDEL_ST:
//      if(isLeftButtonPressed())
//      {
//        masterState = SW_LEFT;
//      }
//      else if(isRightButtonPressed())
//      {
//        masterState = SW_RIGHT;
//      }
      break;
  }
}

//----------------------------------------------------------------------------------------------------
void loop() {
  // This function is called to receive OTA updates
  ArduinoOTA.handle();

  masterStateMachine();

  // --------------- end Application code ---------------------
  // Required!!!, this is to check for new MQTT updates from server
  if(masterState != CALIBRATION)
  {
    // if MQTT connection lost, reconnect
    if (!client.connected()) {
      reconnect();
    }
    
    client.loop();
  }
  // Fixed delay of 10 mSec
  delay(10);
}
