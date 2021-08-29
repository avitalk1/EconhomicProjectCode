#include <Arduino.h>

//Cloud + Data uploading
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string>

//water temprature
#include <OneWire.h>
#include <DallasTemperature.h>

//waterflow
#include "SPIFFS.h"
#include <SPI.h>
#include <Wire.h>

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

//the relays connect to
int IN1 = 33; //pump
int IN2 = 32; //kettle
#define ON 0
#define OFF 1

//Water temprature
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float Celcius = 0;
float Fahrenheit = 0;

//Pump
int is_pump_on = 0;
#define Button_Pump 14 // button is connected with gpio 14 (PURPLE WIRE)

//Kettle
int is_kettle_on = 0;
#define Button_Kettle 13 // button is connected with gpio 14 (PURPLE WIRE)

//Waterflow
#define WATERFLOWpin 27
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned long flowMilliLitres;
unsigned int totalMilliLitres;
float flowLitres;
float totalLitres;

void relay_init(void); //initialize the relay
void relay_SetStatus(unsigned char status);
void relay_SetStatus2(unsigned char status);

void callback(char *topic, byte *payload, unsigned int length)
{
  // int ON =0;
  // int OFF = 1;
  String topicStr = topic;
  std::string payload_ = (char *)payload;

  String pubTopic = "devices/" + uuid;
  String subPumpTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/pump/state";
  // String subKittleTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/kittle/state";
  String subBoilerTopic = "jobs/" + uuid + "/boiler/state";
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  //Pump
  if (subPumpTopic == topic)
  {
    if (payload_.find("on") != std::string::npos && is_pump_on == 0)
    {
      Serial.println("pump on");
      relay_SetStatus(ON); //turn on RELAY_1
      is_pump_on = 1;
      pubTopic = "devices/" + uuid + "/pump/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"pump\", \"status\" : \"on\"}",uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"sink\", \"number\" : 0, \"status\" : \"on\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic.c_str());
      // delay(2000);
      count = count + 1;
    }
    else
    {
      Serial.println("pump off");
      relay_SetStatus(OFF); //turn off RELAY_1
      is_pump_on = 0;
      pubTopic = "devices/" + uuid + "/pump/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"sink\",\"number\" : 0, \"status\" : \"on\"}",uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"sink\",\"number\" : 0, \"status\" : \"off\",\"setOffBy\" : \"us\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic.c_str());
      // delay(2000);
      count = count + 1;
    }
  }

  //Boiler
  // if (subKittleTopic == topic)
  if (subBoilerTopic == topic)
  {
    if (payload_.find("off") != std::string::npos && is_kettle_on == 1)
    {
      Serial.println("Boiler off");
      relay_SetStatus2(OFF); //turn off RELAY_1
      is_kettle_on = 0;
      // pubTopic = "devices/" + uuid + "/kittle/status";
      pubTopic = "devices/" + uuid + "/boiler/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"kittle\", \"status\" : \"off\"}",uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\", \"type\" : \"boiler\", \"status\" : \"off\",\"setOffBy\" : \"us\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic.c_str());
      // delay(2000);
      count = count + 1;
    }
  }
}

void reconnect()
{
  String subPumpTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/pump/state";
  // String subKittleTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/kittle/state";
  String subBoilerTopic = "jobs/" + uuid + "/boiler/state";
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
      client.subscribe(subPumpTopic.c_str());
      // client.subscribe(subKittleTopic.c_str());
      client.subscribe(subBoilerTopic.c_str());
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

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

void setup()
{
  Serial.begin(115200);

  byte mac[6];
  char mac_Id[18];

  //Water temprature
  sensors.begin();

  //WaterFlow Sensor
  pinMode(WATERFLOWpin, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(WATERFLOWpin), pulseCounter, FALLING);

  //Relay
  relay_init(); //initialize the relay

  //Button Pump
  pinMode(Button_Pump, INPUT_PULLUP);

  //Button Boiler
  pinMode(Button_Kettle, INPUT_PULLUP);

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
  String pubTopic = "sensors/" + uuid;
  String pubTempTopic = pubTopic;

  //Pump Button
  int ButtonStatePump = digitalRead(Button_Pump); // read the Button_Pump pin and store in ButtonStatePump variable
  if (ButtonStatePump == HIGH)                    // if button is pressed
  {
    if (is_pump_on == 0)
    {
      Serial.println("pump on");
      relay_SetStatus(ON); //turn on RELAY_1
      is_pump_on = 1;
      String pubTopic_ = "devices/" + uuid + "/pump/status";
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"sink\",\"number\" : 0, \"status\" : \"on\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      // delay(2000);
      count = count + 1;
    }
    else
    {
      Serial.println("pump off");
      relay_SetStatus(OFF); //turn off RELAY_1
      is_pump_on = 0;
      String pubTopic_ = "devices/" + uuid + "/pump/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"sink\",\"number\" : 0, \"status\" : \"off\"}",uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"sink\",\"number\" : 0, \"status\" : \"off\",\"setOffBy\" : \"user\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      // delay(2000);
      count = count + 1;
    }
    Serial.println("pump button pressed");
  }

  //Boiler Button
  int ButtonStateKettle = digitalRead(Button_Kettle); // read the Button_Kettle pin and store in ButtonStateKettle variable
  if (ButtonStateKettle == HIGH)                      // if button is pressed
  {
    if (is_kettle_on == 0)
    {
      Serial.println("Boiler on");
      relay_SetStatus2(ON); //turn on RELAY_1
      is_kettle_on = 1;
      // String pubTopic_ = "devices/" + uuid + "/kittle/status";
      String pubTopic_ = "devices/" + uuid + "/boiler/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"kittle\", \"status\" : \"on\"}",uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\", \"type\" : \"boiler\", \"status\" : \"on\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      // delay(2000);
      count = count + 1;
    }
    else
    {
      Serial.println("Boiler off");
      relay_SetStatus2(OFF); //turn off RELAY_1
      is_kettle_on = 0;
      // String pubTopic_ = "devices/" + uuid + "/kittle/status";
      String pubTopic_ = "devices/" + uuid + "/boiler/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"kittle\", \"status\" : \"off\"}", uuid.c_str(),floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\", \"type\" : \"boiler\", \"status\" : \"off\",\"setOffBy\" : \"user\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      // delay(2000);
      count = count + 1;
    }
    Serial.println("Kettle button pressed");
  }

  // Water temprature
  sensors.requestTemperatures();
  Celcius = sensors.getTempCByIndex(0);
  Fahrenheit = sensors.toFahrenheit(Celcius);
  Serial.print(" C  ");
  Serial.print(Celcius);
  Serial.print(" F  ");
  Serial.println(Fahrenheit);
  delay(1000);

  //Water Flow
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {

    pulse1Sec = pulseCount;
    pulseCount = 0;

    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    flowMilliLitres = (flowRate / 60) * 1000;
    flowLitres = (flowRate / 60);

    totalMilliLitres += flowMilliLitres;
    totalLitres += flowLitres;

    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(float(flowRate)); // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t"); // Print tab space

    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.print("mL / ");
    Serial.print(totalLitres);
    Serial.println("L");

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
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"waterTemperature\" : %.4f, \"counter\" : %d}", uuid.c_str(), floor_, room, Celcius,count);
    pubTempTopic = pubTopic + "/waterTemperature";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    delay(2000);
    count = count + 1;
    //==============================================================================================
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\" ,\"number\" : 0,\"waterFlowRate\" : %.4f, \"counter\" : %d}", uuid.c_str(), floor_, room, float(flowRate),count);
    pubTempTopic = pubTopic + "/waterFlowRate";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    count = count + 1;
    
    if(is_pump_on){
      snprintf(msg, BUFFER_LEN, "{\"DeviceName\" : \"Sink\",\"DeviceType\" : \"water\",\"sensor\" : \"waterFlow\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\" ,\"number\" : 0,\"waterConsumption\" : \"%.4f\",\"isOn\" :\"on\", \"counter\" : %d }", uuid.c_str(), floor_, room, float(flowRate),count);
    }
    else{
      snprintf(msg, BUFFER_LEN, "{\"DeviceName\" : \"Sink\",\"DeviceType\" : \"water\",\"sensor\" : \"waterFlow\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\" ,\"number\" : 0,\"waterConsumption\" : \"%.4f\",\"isOn\" : \"off\", \"counter\" : %d}", uuid.c_str(), floor_, room, float(flowRate),count);
    }
    pubTempTopic = pubTopic + "/consumption";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    count = count + 1;
  }

  long now2 = millis();
  if (now2 - lastMsg2 > 120000)
  {
    lastMsg2 = now;
    if (is_kettle_on)
    {
      snprintf(msg, BUFFER_LEN, "{\"type\" : \"boiler\",\"uuid\" : \"%s\",\"floor\" : \"\",\"room\" : \"\",\"waterTemperature\" : %.4f, \"counter\" : %d}", uuid.c_str(), Celcius,count);
      pubTempTopic = "study/" + uuid + "/boiler";
      client.publish(pubTempTopic.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTempTopic.c_str());
      delay(2000);
      count = count + 1;
    }
    if (is_pump_on)
    {
      snprintf(msg, BUFFER_LEN, "{\"type\" : \"pump\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\",\"number\" : 0,\"waterFlowRate\" : %.4f, \"counter\" : %d}", uuid.c_str(), floor_, room, float(flowRate),count);
      pubTempTopic = "study/" + uuid + "/pump";
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

void relay_init(void) //initialize the relay
{
  //set all the relays OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  relay_SetStatus(OFF);  //turn off  relay1
  relay_SetStatus2(OFF); //turn off relay2
}

//set the status of relays

//set status on relay1
void relay_SetStatus(unsigned char status)
{
  digitalWrite(IN1, status);
}
//set status on relay2
void relay_SetStatus2(unsigned char status)
{
  digitalWrite(IN2, status);
}