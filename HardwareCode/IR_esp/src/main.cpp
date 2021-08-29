#include <Arduino.h>

//Cloud + Data uploading
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string>

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

//IR sensors
int irPin1=37;
int irPin2=38;
int countIR=0;
boolean flag1=0;
boolean flag2=0;
boolean flagIN=0;
boolean flagOUT=0;

void IRAM_ATTR check_one();
void IRAM_ATTR check_two();

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
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
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

void setup() {
Serial.begin(115200);
byte mac[6];
char mac_Id[18];

//IR sensors
  pinMode(irPin1, INPUT);
  attachInterrupt(digitalPinToInterrupt(irPin1),check_one,FALLING);
  pinMode(irPin2, INPUT); 
  attachInterrupt(digitalPinToInterrupt(irPin2),check_two,FALLING); 

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

void loop() {
  String pubTopic = "sensors/" + uuid;
  String pubTempTopic = pubTopic;

  if(flagIN==1){
    delay(2000);
    flagIN=0;
//    attachInterrupt(digitalPinToInterrupt(irPin1),check_one,FALLING);
    attachInterrupt(digitalPinToInterrupt(irPin2),check_two,FALLING);
  }
  if(flagOUT==1){
    delay(2000);
    flagOUT=0;
    attachInterrupt(digitalPinToInterrupt(irPin1),check_one,FALLING);
//    attachInterrupt(digitalPinToInterrupt(irPin2),check_two,FALLING);
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
    //=============================================================================================
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"amount\" : %d, \"counter\" : %d}", uuid.c_str(), floor_, room, countIR,count);
    pubTempTopic = pubTopic + "/peopleAmount";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
//    delay(2000);
    count = count + 1;
  }
    //==============================================================================================
}

void IRAM_ATTR check_one(){
  detachInterrupt(digitalPinToInterrupt(irPin1));
  Serial.println("One");

    if(flag2==1){
      if(countIR>0){
        countIR--;
      }
      else{
        countIR=0;
      }
      Serial.print("OUT: ");
      Serial.println(countIR);
      flag1=0;
      flag2=0;
      attachInterrupt(digitalPinToInterrupt(irPin2),check_two,FALLING);
      flagOUT=1;
    }
    else{
      flag1=1;
    }
    Serial.print("check_one:\t");
    Serial.print(" flag1: ");
    Serial.print(flag1);
    Serial.print(" flag2: ");
    Serial.println(flag2);
}

void IRAM_ATTR check_two(){
  detachInterrupt(digitalPinToInterrupt(irPin2)); 
  Serial.println("Two");


    if(flag1==1){
      countIR++;
      Serial.print("IN: ");
      Serial.println(countIR);
      flag1=0;
      flag2=0;
      attachInterrupt(digitalPinToInterrupt(irPin1),check_one,FALLING);
      flagIN=1;
    }
    else{
      flag2=1;
    }
    Serial.print("check_two:\t");
    Serial.print(" flag1: ");
    Serial.print(flag1);
    Serial.print(" flag2: ");
    Serial.println(flag2);
}