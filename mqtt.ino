#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <WiFi101.h>

/***************************************************
  Adafruit MQTT Library WINC1500 Example
  Adafruit invests time and resources providing this open source code,
  please suplimport Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER "io.adafruit.com" //MQTT Broker link
#define AIO_SERVERPORT 1883 //MQTT Broker comunication port
#define AIO_USERNAME "Savii93" //MQTT Client username used for the login
char alpha[] = "ABCDEFGHIKLMNOPQRSTUVZabcdefghiklmnopqrstuvz1234567890"; //Array used for masquerading purposes and adopted in order to avoid clear-text passwords.
//MQTT Client password for the login 
char AIO_KEY[] = {alpha[0x1B], alpha[0x19], alpha[0x31], alpha[0x1B], alpha[0x1B], alpha[0x35], alpha[0x35], alpha[0x2F], alpha[0x31], alpha[0x1B], alpha[0x1A], alpha[0x2C], alpha[0x2F], alpha[0x32], alpha[0x1B], alpha[0x34], alpha[0x17], alpha[0x2C], alpha[0x31], alpha[0x1A], alpha[0x34], alpha[0x1A], alpha[0x32], alpha[0x2D], alpha[0x35], alpha[0x32], alpha[0x19], alpha[0x33], alpha[0x1B], alpha[0x2C], alpha[0x2C], alpha[0x2D], NULL};

/***************************** Global State **********************************/

char ssid[] = "Savii Wireless Connection 2.4GHz"; //Name of the Wi-Fi network
//Password of the Wi-Fi network
char pass[] = {alpha[0x9], alpha[0x23], alpha[0x18], alpha[0x16], alpha[0x22], alpha[0x1E], alpha[0x2C], alpha[0x32], alpha[0x2C], alpha[0x2C], alpha[0x2C], alpha[0x34], alpha[0x34], alpha[0x2E]}; //Password of the Wi-Fi network
int status = WL_IDLE_STATUS;
boolean admin = false; //Variable used in order to check if the developer status is active
long starttime; //Time value used to manage the main loop operations
long actlaser; //Time value used to store when the laser was activated
int prevuplim; //Previous status of the upper limit switch
int prevdownlim; //Previous status of the down limit switch
int uplim; //Current status of the upper limit switch
int downlim; //Current status of the down limit switch
float temperature; //Temperature value in °C
int light; //Value of light intensity on the photoresistor block

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp", MQTT_QOS_1);
Adafruit_MQTT_Publish motstop = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/finecorsa", MQTT_QOS_1);
Adafruit_MQTT_Publish allarm = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/allarme", MQTT_QOS_1);
const char WILL_FEED[] PROGMEM = AIO_USERNAME "/feeds/arduino-on";
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED, MQTT_QOS_1);

Adafruit_MQTT_Subscribe onoffallarm = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/on-off-all", MQTT_QOS_1);
Adafruit_MQTT_Subscribe motor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/motor", MQTT_QOS_1);

/*************************** Sketch Code ************************************/

void setup()
{
  //Gives the possibility either to login as developer or not
  loginModus();

  //analogReadResolution(12); //ADC resolution setting
  
  pinMode(0, OUTPUT); //Initialization motor up output
  pinMode(1, OUTPUT); //Initialization motor down output
  pinMode(2, OUTPUT); //Initialization laser output
  pinMode(3, INPUT); //Initialization limit switch up input 
  pinMode(4, INPUT); //Initialization limit switch down input
  pinMode(5, OUTPUT); //Initialization allarm output

  //Subscription to allarm switch and to motor controller feeds
  mqtt.subscribe(&onoffallarm);
  mqtt.subscribe(&motor);

  publishBoardOff(); //Last Will message
}

void loop()
{
  connectWIFI(); //Connects the board to the Wi-Fi network
  connectMQTT(); //Connects the Client to the MQTT Broker

  bool first = true; //Variable that signals the first time loop() is executed
  starttime = millis(); //Starttime of loop() is saved

  publishBoardOn();

  //Main loop executed for 60 seconds
  while (!hasTimeElapsed(starttime, 60000))
  {
    //Statement used to check if the login as developer was made; if true, shows the developer menu
    switch (isAdmin())
    {
      case true:
        adminMenu();
        break;

      //Read limit switches status
      case false:
        uplim = digitalRead(3);
        downlim = digitalRead(4);
        break;
    }

    //Computes and publishes the limit switches status
    computeLimitStatus(first);

    //Reads the light value on the photoresistor block
    if (!isAdmin())
      light = analogRead(0);

    //Activation of the DC buzzer and publishment of the status if an intrusion attempt is recognized
    if (isLaserOn() && !isAllarmOn() && light > 690 && hasTimeElapsed(actlaser, 50))
    {
      setAllarm(true);
      publishAllarmOnStatus();
    }

    //If laser device is switched off, the DC buzzer is deactivated
    else if (!isLaserOn() && isAllarmOn())
      setAllarm(false);

    //Read the status of the Subscriptions and perform the related actions
    if(!isAdmin())
      computeSubscriptions();

    //Finish of the first loop() execution
    first = false;
  }

  //Read and compute the temperature value
  if (!isAdmin())
    temperature = computeTemperature(analogRead(1));

  //Publish the temperature value
  publishTemperatureStatus(temperature);
}

//Gives the possibility to login as developer in the first 10 seconds of device execution
void loginModus()
{
  float connectionTime;

  Serial.begin(115200);
  connectionTime = millis();

  while (!hasTimeElapsed(connectionTime, 10000))
  {
    if (Serial.available())
    {
      String txt = Serial.readString();

      if (txt == "admin")
      {
        setAdmin(true);

        Serial.println("Admin mode activated!");
        Serial.println("Select on which object you want to perform an action: ");
        Serial.println("a: Set temperature sensor value (in °C)");
        Serial.println("b: Set up limit switch status (0 for active, 1 for not active)");
        Serial.println("c: Set down limit switch status (0 for active, 1 for not active)");
        Serial.println("d: Set motor status (1 for up, 2 for down, 3 for stop)");
        Serial.println("e: Set laser status (0 for off, 1 for on)");
        Serial.println("f: Set photoresistor light value (0-1100)");
        Serial.println("g: Print photoresistor light value");
        Serial.println("h: Print temperature value");
        Serial.println("i: Print up limit switch value");
        Serial.println("l: Print down limit switch value");
        Serial.println("m: Exit from developer modus");
        break;
      }
    }
  }
}

//Used if the login as developer is correctly performed; gives the possibility to manually manage every I/O device.
void adminMenu()
{
  String input;

  if (Serial.available())
  {
    input = Serial.readString();
    Serial.println(input);

    if (input.startsWith("a"))
      temperature = input.substring(2).toFloat();

    else if (input.startsWith("b"))
      uplim = input.substring(2).toFloat();

    else if (input.startsWith("c"))
      downlim = input.substring(2).toFloat();

    else if (input.startsWith("d"))
    {
      if (input.substring(2) == "1")
        setMotorUp(true);
      else if (input.substring(2) == "2")
        setMotorDown(true);
      else if (input.substring(2) == "3")
      {
        setMotorUp(false);
        setMotorDown(false);
      }
    }

    else if (input.startsWith("e"))
    {
      if (input.substring(2) == "0")
        setLaser(false);

      else if (input.substring(2) == "1")
        setLaser(true);
    }

    else if (input.startsWith("f"))
      light = input.substring(2).toInt();

    else if (input.startsWith("g"))
      Serial.println("Light value: " + (String)light);

    else if (input.startsWith("h"))
      Serial.println("Temperature value: " + (String)temperature);

    else if (input.startsWith("i"))
      Serial.println("Up limit switch value: " + (String)uplim);

    else if (input.startsWith("l"))
      Serial.println("Down limit switch value: " + (String)downlim);

    else if (input.startsWith("m"))
      setAdmin(false);
  }
}

boolean isAdmin()
{
  if (admin)
    return true;
  else
    return false;
}

boolean setAdmin(boolean status)
{
  admin = status;
}

//Reads if there are incoming messages on the subscribed topics (motor control and allarm switch); if yes, executes the related actions.
void computeSubscriptions()
{
  Adafruit_MQTT_Subscribe *subscription = mqtt.readSubscription();

  if (subscription == &onoffallarm)
  {
    Serial.print(F("Allarm switch, got: "));
    Serial.println((char *)onoffallarm.lastread);

    if (0 == strcmp((char *)onoffallarm.lastread, "OFF"))
      setLaser(false);

    else if (0 == strcmp((char *)onoffallarm.lastread, "ON"))
      setLaser(true);
  }

  else if (subscription == &motor)
  {
    Serial.print(F("Motor, got: "));

    if (0 == strcmp((char *)motor.lastread, "5") && uplim == 1)
    {
      Serial.println("UP");
      setMotorUp(true);
    }

    else if (0 == strcmp((char *)motor.lastread, "13") && downlim == 1)
    {
      Serial.println("DOWN");
      setMotorDown(true);
    }

    else if (0 == strcmp((char *)motor.lastread, "6"))
    {
      Serial.println("STOP");
      setMotorUp(false);
      setMotorDown(false);
    }
  }
}

//Sets the allarm output
void setAllarm(boolean stat)
{
  if (stat)
    digitalWrite(5, HIGH);
  else
    digitalWrite(5, LOW);
}

//Sets the laser output and memorizes the activation time
void setLaser(boolean stat)
{
  if (stat)
  {
    actlaser = millis();
    digitalWrite(2, HIGH);
  }
  else
    digitalWrite(2, LOW);
}

//Sets the motor up output; the pin can be set to "1" only if the up limit switch is not active
void setMotorUp(bool stat)
{
  if (stat && uplim != 0)
    digitalWrite(0, HIGH);
  else
    digitalWrite(0, LOW);
}

//Sets the motor down output; the pin can be set to "1" only if the down limit switch is not active
void setMotorDown(bool stat)
{
  if (stat && downlim != 0)
    digitalWrite(1, HIGH);
  else
    digitalWrite(1, LOW);
}

//Method used to check whether a predefined amount of time has passed
boolean hasTimeElapsed(long start, int amount)
{
  if (millis() - start >= amount)
    return true;
  else
    return false;
}

//Signals the state of the DC buzzer
boolean isAllarmOn()
{
  if (digitalRead(5))
    return true;
  else
    return false;
}

//Signals the state of the laser
boolean isLaserOn()
{
  if (digitalRead(2))
    return true;
  else
    return false;
}

//Method used in order to compute the °C out from a digital value
float computeTemperature(float analogIn)
{
  float volt = (analogIn / 1024.0) * 3.3;
  return (volt - 0.5) * 100;
}

void publishBoardOn()
{
  lastwill.publish("ON");
}

//Last Will message
void publishBoardOff()
{
  mqtt.will(WILL_FEED, "OFF");
}

//Publishes the status of allarm active
void publishAllarmOnStatus()
{
  if (!allarm.publish("1"))
    Serial.println(F("Failed"));
  else
    Serial.println(F("Allarm active published."));
}

//Publishes the status of the limit switches (UP, DOWN or ERROR)
void publishLimitStatus(char *data)
{
  if (!motstop.publish(data))
    Serial.println(F("Failed"));

  else
    Serial.println((String) data + " limit published ");
}

//Computes and calls the publication of the limit switches; this method is used in order to prevent continuous pubblications, since there is a check of the previous and actual status
void computeLimitStatus(bool first)
{
  if (first)
  {
    prevuplim = uplim;
    prevdownlim = downlim;
  }

  else
  {
    if (prevuplim != uplim || prevdownlim != downlim)
    {
      if (uplim == 0 && downlim == 1)
        publishLimitStatus("UP");

      else if (downlim == 0 && uplim == 1)
        publishLimitStatus("DOWN");

      else if (downlim == 0 && uplim == 0)
        publishLimitStatus("ERROR");

      else
        publishLimitStatus("OK");
    }
  }

  prevuplim = uplim;
  prevdownlim = downlim;
}

//Publishes the °C
void publishTemperatureStatus(float data)
{
  if (!temp.publish(data))
    Serial.println("Failed");
  else
    Serial.println((String)data + " temperature sent.");
}

//Checks if the Arduino board has a Wi-Fi network card
bool hasWIFIModule()
{
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("Shield not present.");
    return false;
  }
  else
    return true;
}

//Checks whether the Arduino board is connected to the Wi-Fi network
bool isWIFIConnected()
{
  if (WiFi.status() == WL_CONNECTED)
    return true;
  else
    return false;
}

//Connects the Arduino board to the Wi-Fi network
void connectWIFI()
{
  while (!hasWIFIModule());

  while (!isWIFIConnected())
  {
    int count = 10;
    WiFi.begin(ssid, pass);

    while (count != 0 && !isWIFIConnected())
    {
      Serial.println("Retry to connect to WLAN");
      count--;
      delay(1000);
    }
  }

  Serial.print("Connected to: ");
  Serial.println(ssid);
}

//Connects the Client to the MQTT Broker
void connectMQTT()
{
  if (mqtt.connected())
    return;

  while (mqtt.connect() != 0)
  {
    //mqtt.connect will return 0 if connected
    Serial.println("Retrying MQTT connection in 5 seconds ..");
    mqtt.disconnect();
    delay(5000);
  }

  Serial.println("Connected to MQTT Broker!");
}
