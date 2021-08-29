#include <Arduino.h>

//Cloud + Data uploading
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string>

//Servo
#include <Servo_ESP32.h>

//Wifi + cloud
String floor_ = "floor";
String room = "room";
String uuid = "ea2d00b0-8bc8-11eb-89cf-9b66cd16768d";


// Enter your WiFi ssid and password
const char *ssid = "*****";         //Provide your SSID
const char *password = "*****"; // Provide Password

const char *mqtt_server = "a3a4qqsp3xsqud-ats.iot.eu-west-1.amazonaws.com"; // MQTT END point
const int mqtt_port = 8883;

const char *crt = "*****";
const char *private_key = "*****";

String Read_rootca;
String Read_cert;
String Read_privatekey;
#define BUFFER_LEN 256
long lastMsg = 0;
long lastMsg2 = 0;
char msg[BUFFER_LEN];
int count = 1;

WiFiClientSecure espClient;
PubSubClient client(espClient);

//LED
// int LEDpin = 19; //connect to 3.3V
int IN1 = 19;
#define ON 0
#define OFF 1
void relay_init(void);
void relay_SetStatus( unsigned char status_1, unsigned char status_2);

//PIR
int PIRpin = 12; //connect to 5V

//LIGHT
int LIGHTpin = 33; //connect to 5V

//LIGHT2
int LIGHTpin2 = 37; //connect to 5V

//Servo
static const int servoPin = 14; //printed G14 on the board - connect to 5V
Servo_ESP32 servo1;
//int i=0;
int angle = 0;
int angleStep = 5;
int angleMin = 0;
int angleMax = 180;

//Window's BUTTON
#define Button_windows 13 // open-close window button
bool window_is_open = false;

//light's button
#define Button_lights 9 // open-close lights button
bool light_is_open = false;

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String topicStr = topic;
  std::string payload_ = (char *)payload;

  String pubTopic = "sensors/" + uuid;
  String subWindowTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/window/state";
  String subLightsTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/lights/state";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (topicStr == subWindowTopic)
  {
    //Servo
    if (payload_.find("open") != std::string::npos && !window_is_open)
    {
      for (int angle = 180; angle >= angleMin; angle -= angleStep)
      {
        servo1.write(angle);
        delay(20);
      }
      Serial.println("open window");
      pubTopic = "devices/" + uuid + "/window/status";
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"window\",\"number\" : 0, \"status\" : \"open\",\"whoTurn\" : \"us\",\"reason\" : \"\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      window_is_open = true;
    }
    else
    {
      for (int angle = 0; angle <= angleMax; angle += angleStep)
      {
        servo1.write(angle);
        delay(20);
      }
      Serial.println("close window");

      pubTopic = "devices/" + uuid + "/window/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"window\", \"status\" : \"close\"}", uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"window\",\"number\" : 0, \"status\" : \"close\",\"setOffBy\" : \"us\", \"whoTurn\" : \"us\", \"reason\" : \"\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      window_is_open = false;
    }
  }
  //lights
  if (topicStr == subLightsTopic)
  {
    if (payload_.find("on") != std::string::npos && !light_is_open)
    {
      // digitalWrite(LEDpin, LOW);
      // Serial.print("ON");
      relay_SetStatus(ON, OFF);//turn on RELAY_1
      Serial.println("light on");
      pubTopic = "devices/" + uuid + "/lights/status";
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"light\",\"number\" : 0, \"status\" : \"on\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      light_is_open = true;
    }
    else
    {
      // Serial.print("OFF");
      relay_SetStatus(OFF, OFF);//turn on RELAY_1
      // digitalWrite(LEDpin, HIGH);
      Serial.println("light off");
      pubTopic = "devices/" + uuid + "/lights/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"light\", \"status\" : \"off\"}", uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"light\",\"number\" : 0, \"status\" : \"off\",\"setOffBy\" : \"us\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      light_is_open = false;
    }
  }
}

void reconnect()
{
  String subWindowTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/window/state";
  String subLightsTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/lights/state";
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("ei_out", "hello world");
      // ... and resubscribe
      client.subscribe("ei_in");
      client.subscribe(subWindowTopic.c_str());
      client.subscribe(subLightsTopic.c_str());
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);

  byte mac[6];
  char mac_Id[18];

  //PIR Sensor
  pinMode(PIRpin, INPUT);

  //Servo
  servo1.attach(servoPin);

  //Light sensor
  delay(1000);

  //LED
  // pinMode(LEDpin, OUTPUT);
  // digitalWrite(LEDpin, HIGH);
  relay_init();//initialize the relay

  //Buttons
  pinMode(Button_windows, INPUT_PULLUP);
  pinMode(Button_lights, INPUT_PULLUP);


  //connection to wifi and cloud
  setup_wifi();
  delay(1000);
  //=============================================================
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  //=======================================
  //Root CA File Reading.
  File file2 = SPIFFS.open("/AmazonRootCA1.pem", "r");
  if (!file2)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("Root CA File Content:");
  while (file2.available())
  {
    Read_rootca = file2.readString();
    Serial.println(Read_rootca);
  }
  //=============================================
  // Cert file reading
  File file4 = SPIFFS.open(crt, "r");
  if (!file4)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("Cert File Content:");
  while (file4.available())
  {
    Read_cert = file4.readString();
    Serial.println(Read_cert);
  }
  //=================================================
  //Privatekey file reading
  File file6 = SPIFFS.open(private_key, "r");
  if (!file6)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("privateKey File Content:");
  while (file6.available())
  {
    Read_privatekey = file6.readString();
    Serial.println(Read_privatekey);
  }
  //=====================================================

  char *pRead_rootca;
  pRead_rootca = (char *)malloc(sizeof(char) * (Read_rootca.length() + 1));
  strcpy(pRead_rootca, Read_rootca.c_str());

  char *pRead_cert;
  pRead_cert = (char *)malloc(sizeof(char) * (Read_cert.length() + 1));
  strcpy(pRead_cert, Read_cert.c_str());

  char *pRead_privatekey;
  pRead_privatekey = (char *)malloc(sizeof(char) * (Read_privatekey.length() + 1));
  strcpy(pRead_privatekey, Read_privatekey.c_str());

  Serial.println("================================================================================================");
  Serial.println("Certificates that passing to espClient Method");
  Serial.println();
  Serial.println("Root CA:");
  Serial.write(pRead_rootca);
  Serial.println("================================================================================================");
  Serial.println();
  Serial.println("Cert:");
  Serial.write(pRead_cert);
  Serial.println("================================================================================================");
  Serial.println();
  Serial.println("privateKey:");
  Serial.write(pRead_privatekey);
  Serial.println("================================================================================================");

  espClient.setCACert(pRead_rootca);
  espClient.setCertificate(pRead_cert);
  espClient.setPrivateKey(pRead_privatekey);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //====================================================================================================================
  WiFi.macAddress(mac);
  snprintf(mac_Id, sizeof(mac_Id), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(mac_Id);
  //=====================================================================================================================
  delay(2000);
}

void loop()
{
  int Buttonwindows = digitalRead(Button_windows); // read the button pin and store in ButtonState variable
  int Buttonlights = digitalRead(Button_lights);   // read the button pin and store in ButtonState variable
  String pubTopic = "sensors/" + uuid;
  String pubTempTopic = pubTopic;
  //  String m = mac_Id;
  //  String pubTopic = "sensors/" +  m;
  //  String pubTempTopic = pubTopic;

  //PIR Sensor
  int pir = digitalRead(PIRpin);
  Serial.println(pir);
  delay(1000);

  //Light sensor
  int LDR_val = analogRead(LIGHTpin);
  Serial.print("LDR_val: ");
  Serial.println(LDR_val);
  delay(100);

  //Light sensor2
  int LDR_val2 = analogRead(LIGHTpin2);
  Serial.print("LDR_val2: ");
  Serial.println(LDR_val2);
  delay(100);

  //Servo
  if (Buttonwindows == HIGH) // if windows-button is pressed
  {
    if (window_is_open)
    {
      for (int angle = 0; angle <= angleMax; angle += angleStep)
      {
        servo1.write(angle);
        delay(20);
      }
      String pubTopic_ = "devices/" + uuid + "/window/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"window\", \"status\" : \"close\"}", uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"window\",\"number\" : 0, \"status\" : \"close\",\"setOffBy\" : \"user\",\"whoTurn\" : \"user\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.println("close window");
      Serial.println(pubTopic_.c_str());
      window_is_open = false;
    }
    else
    {
      for (int angle = 180; angle >= angleMin; angle -= angleStep)
      {
        servo1.write(angle);
        delay(20);
      }
      String pubTopic_ = "devices/" + uuid + "/window/status";
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"window\",\"number\" : 0, \"status\" : \"open\",\"whoTurn\" : \"user\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.println("open window");
      Serial.println(pubTopic_.c_str());
      window_is_open = true;
    }
  }

  //LED
  if (Buttonlights == HIGH) // if lights-button is pressed
  {
    if (light_is_open)
    {
      // Serial.print("Off");
      relay_SetStatus(OFF, OFF);//turn off RELAY_1
      // digitalWrite(LEDpin, HIGH);
      Serial.println("light off");
      String pubTopic_ = "devices/" + uuid + "/lights/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"light\", \"status\" : \"off\"}", uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"light\",\"number\" : 0, \"status\" : \"off\",\"setOffBy\" : \"user\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.println(pubTopic_.c_str());
      light_is_open = false;
    }
    else
    {
      // Serial.print("ON");
      relay_SetStatus(ON, OFF);//turn on RELAY_1
      // digitalWrite(LEDpin, LOW);
      Serial.println("light on");
      String pubTopic_ = "devices/" + uuid + "/lights/status";
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"light\",\"number\" : 0, \"status\" : \"on\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.println(pubTopic_.c_str());
      light_is_open = true;
    }
  }


  //Connect and Send data to MQTT
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000)
  {
    lastMsg = now;
    //    //==============================================================================================
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"motion\" : %d, \"counter\" : %d}", uuid.c_str(), floor_, room, pir,count);
    pubTempTopic = pubTopic + "/motion";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    delay(2000);
    count = count + 1;
    //==============================================================================================
    // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\" ,\"LDR_val\" : %d}", uuid.c_str(), floor_, room, LDR_val);
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\" ,\"LDR\" : %d, \"type\" : \"in\", \"counter\" : %d}", uuid.c_str(), floor_, room, LDR_val,count);
    pubTempTopic = pubTopic + "/LDR";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    count = count + 1;
    //    //==============================================================================================
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\" ,\"LDR\" : %d, \"type\" : \"out\", \"counter\" : %d}", uuid.c_str(), floor_, room, LDR_val2,count);
    pubTempTopic = pubTopic + "/LDR";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    count = count + 1;
    //    //================================================================================================
    //
  }
  long now2 = millis();
  if (now2 - lastMsg2 > 120000)
  {
    lastMsg2 = now;
    if (light_is_open)
    {
      String p = "close";
      if (window_is_open)
      {
        p = "open";
      }
      snprintf(msg, BUFFER_LEN, "{\"type\" : \"lights\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"lights\" : \"on\",\"LDR_val\" : \"%d\",\"window\" : \"%s\", \"counter\" : %d}", uuid.c_str(), floor_, room, LDR_val, p,count);
      pubTempTopic = "study/" + uuid + "/lights";
      client.publish(pubTempTopic.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTempTopic.c_str());
      delay(2000);
      count = count + 1;
    }
  }
}

void relay_init(void)//initialize the relay
{
 //set all the relays OUTPUT
 pinMode(IN1, OUTPUT);
// pinMode(IN2, OUTPUT);
 relay_SetStatus(OFF, OFF); //turn off all the relay
}
//set the status of relays
void relay_SetStatus( unsigned char status_1, unsigned char status_2)
{
 digitalWrite(IN1, status_1);
// digitalWrite(IN2, status_2);
}
