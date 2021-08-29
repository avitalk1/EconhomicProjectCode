#include <Arduino.h>

//Cloud + Data uploading
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string>

//lcd
#include <LiquidCrystal_I2C.h>

//DHT
#include <DHT.h> // library for getting data from DHT
#include <Adafruit_Sensor.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//LCD and Buttons
#define Button_Plus 13 // button is connected with gpio 22 (PURPLE WIRE)
#define Button_Minus 9 // button is connected with gpio 10 (PURPLE WIRE)
int ac_count = 22;

//LED
#define LED 12

//New Fan
#define LED_GPIO   10
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000
int PWM1_DutyCycle = 0;

//Fan's BUTTON
#define Button_fan 15 // open-close fan button
bool fan_is_on = false;

//DHT
#define DHTPIN 33 //pin where the DHT11 is connected - connect to 5V
DHT dht(DHTPIN, DHT11);

//Current sensor
int powerPin = 35;
double Vout = 0;
double Current = 0;
const double scale_factor = 0.185; // 5A
const double vRef = 5.00;
const double resConvert = 4096;
double resADC = vRef / resConvert;
double zeroPoint = vRef / 2;
double powerAmount = 0;

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

void callback(char *topic, byte *payload, unsigned int length)
{
  String topicStr = topic;
  std::string payload_ = (char *)payload;

  String pubTopic = "devices/" + uuid;
  String subFanTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/ac/state";

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (topicStr == subFanTopic)
  {
    //Fan
    if (payload_.find("set_status") != std::string::npos)
    {

      if (payload_.find("on") != std::string::npos && !fan_is_on)
      {
        ledcWrite(PWM1_Ch, 255);
        //ledcWrite(FanChannel, 255);
        delay(200);
        Serial.println("Fan turned ON");

        pubTopic = "devices/" + uuid + "/ac/status";
        // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count);
        snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d, \"opType\" : \"acState\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count,count);
        client.publish(pubTopic.c_str(), msg);
        Serial.print("Publish message: ");
        Serial.print(count);
        Serial.println(msg);
        Serial.print("To: ");
        Serial.println(pubTopic.c_str());
        delay(2000);
        // count = count + 1;
        fan_is_on = true;

        lcd.clear(); // clear previous values from screen
        lcd.setCursor(0, 0);
        lcd.print("A/C Temprature:");
        lcd.setCursor(7, 1);
        lcd.print(ac_count);
      }
      else
      {
        ledcWrite(PWM1_Ch, 0);
        //ledcWrite(FanChannel, 0);
        delay(200);

        Serial.println("Fan turned OFF");
        pubTopic = "devices/" + uuid + "/ac/status";
        // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"off\",\"setOffBy\" : \"us\"}", uuid.c_str(), floor_.c_str(), room.c_str());
        snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"off\",\"setOffBy\" : \"us\", \"opType\" : \"acState\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
        client.publish(pubTopic.c_str(), msg);
        Serial.print("Publish message: ");
        Serial.print(count);
        Serial.println(msg);
        Serial.print("To: ");
        Serial.println(pubTopic.c_str());
        delay(2000);
        // count = count + 1;
        fan_is_on = false;

        lcd.clear(); // clear previous values from screen
        lcd.setCursor(2, 0);
        lcd.print("A/C is OFF");
        digitalWrite(LED, HIGH); // led will be OFF
      }
    }

    else
    {
      if (payload_.find("set_temperature") != std::string::npos && fan_is_on == true)
      {
        int i = payload_.find("\"set_temperature\": '");
        int temp = String("\"set_temperature\": '").length() + i;
        String num = "";
        while (payload_[temp] != '"' && payload_[temp] != ' ')
        {
          num += payload_[temp];
          temp++;
        }
        sscanf(num.c_str(), "%d", &ac_count);
        Serial.print("ac_count: ");
        Serial.println(ac_count);
        pubTopic = "devices/" + uuid + "/ac/status";
        // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d,,\"setTemperatureBy\" : \"us\"}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count);
        snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d,\"setTemperatureBy\" : \"us\", \"opType\" : \"temperatureSet\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count,count);
        client.publish(pubTopic.c_str(), msg);
        Serial.print("Publish message: ");
        Serial.print(count);
        Serial.println(msg);
        Serial.print("To: ");
        Serial.println(pubTopic.c_str());
        delay(2000);
        lcd.clear(); // clear previous values from screen
        lcd.setCursor(0, 0);
        lcd.print("A/C Temprature:");
        lcd.setCursor(7, 1);
        lcd.print(ac_count);
        //count = count + 1;
      }
    }
  }
}
void reconnect()
{
  String subFanTopic = "jobs/" + uuid + '/' + floor_ + "/" + room + "/ac/state";
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
      client.subscribe(subFanTopic.c_str());
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

  //LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // led will be OFF

  //LCD and buttons
  pinMode(Button_Plus, INPUT_PULLUP);
  pinMode(Button_Minus, INPUT_PULLUP);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("econHOMEic");
  delay(3000);

  lcd.clear(); // clear previous values from screen
  lcd.setCursor(2, 0);
  lcd.print("A/C is OFF");

  //DHT
  dht.begin();


//New Fan
  ledcAttachPin(LED_GPIO, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  ledcWrite(PWM1_Ch, 0);


  //Fan's Button
  pinMode(Button_fan, INPUT_PULLUP);

  //Connection to wifi and cloud
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

  //LCD and buttons
  int ButtonStatePlus = digitalRead(Button_Plus); // read the button pin and store in ButtonState variable
  if (fan_is_on)
  {
    if (ButtonStatePlus == HIGH) // if button plus is pressed
    {
      lcd.clear(); // clear previous values from screen
      if (ac_count < 28)
      {
        ac_count++;
      }
      lcd.setCursor(0, 0);
      lcd.print("A/C Temprature:");
      lcd.setCursor(7, 1);
      lcd.print(ac_count);
      delay(500);
      // digitalWrite(LED, LOW); // led will be ON
      Serial.print("A/C Temp is: ");
      Serial.print(ac_count);
      Serial.println("° Plus Pressed");
      String pubTopic_ = "devices/" + uuid + "/ac/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d,\"setTemperatureBy\" : \"user\"}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count);
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d,\"setTemperatureBy\" : \"user\", \"opType\" : \"temperatureSet\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count,count);
      //need to add the curennt ac_temp
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      delay(2000);
      count = count + 1;
      fan_is_on = true;
    }
    else // otherwise
    {
      // digitalWrite(LED, HIGH); // LED will be OFF
      // Serial.print("PLUS else Loop -> Count: ");
      // Serial.print(count);
      // Serial.println(" LOW");
    }

    int ButtonStateMinus = digitalRead(Button_Minus); // read the button pin and store in ButtonState variable
    if (ButtonStateMinus == HIGH)                     // if button minnus is pressed
    {
      lcd.clear(); // clear previous values from screen
      if (ac_count > 16)
      {
        ac_count--;
      }
      lcd.setCursor(0, 0);
      lcd.print("A/C Temprature:");
      lcd.setCursor(7, 1);
      lcd.print(ac_count);
      delay(500);
      // digitalWrite(LED, LOW); // led will be ON
      Serial.print("A/C Temp is: ");
      Serial.print(ac_count);
      Serial.println("° Minus Pressed");
      String pubTopic_ = "devices/" + uuid + "/ac/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d,\"setTemperatureBy\" : \"user\"}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count);
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d,\"setTemperatureBy\" : \"user\", \"opType\" : \"temperatureSet\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count,count);
      //need to add the curennt ac_temp
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      delay(2000);
      count = count + 1;
      fan_is_on = true;
    }
    else // otherwise
    {
      // digitalWrite(LED, HIGH); // LED will be OFF
      // Serial.print("MINUS else Loop -> Count: ");
      // Serial.print(count);
      // Serial.println(" LOW");
    }
  }

  //FAN
  int ButtonStateFan = digitalRead(Button_fan); // read the button pin and store in ButtonState variable
  if (ButtonStateFan == HIGH)                   // if button state is pressed
  {
    if (fan_is_on == false) //if fan is off, turn it on
    {
      ledcWrite(PWM1_Ch, 255);
      //ledcWrite(FanChannel, 255);
      digitalWrite(LED, LOW); // led will be ON
      delay(500);
      fan_is_on = true;
      String pubTopic_ = "devices/" + uuid + "/ac/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count);
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"on\", \"ac_temp\" : %d, \"opType\" : \"acState\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(), ac_count,count);
      //need to add the curennt ac_temp
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      delay(2000);
      count = count + 1;

      Serial.println("A/C turned ON");

      //clear lcd screen and update it to ac ON
      lcd.clear(); // clear previous values from screen
      lcd.setCursor(0, 0);
      lcd.print("A/C Temprature:");
      lcd.setCursor(7, 1);
      lcd.print(ac_count);
    }
    else // otherwise, turn fan off
    {
      ledcWrite(PWM1_Ch, 0);
      //ledcWrite(FanChannel, 0);
      digitalWrite(LED, HIGH); // led will be OFF
      delay(500);
      fan_is_on = false;
      String pubTopic_ = "devices/" + uuid + "/ac/status";
      // snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"off\",\"setOffBy\" : \"user\"}", uuid.c_str(), floor_.c_str(), room.c_str());
      snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"type\" : \"ac\", \"status\" : \"off\",\"setOffBy\" : \"user\", \"opType\" : \"acState\", \"counter\" : %d}", uuid.c_str(), floor_.c_str(), room.c_str(),count);
      //need to add the curennt ac_temp
      client.publish(pubTopic_.c_str(), msg);
      Serial.print("Publish message: ");
      Serial.print(count);
      Serial.println(msg);
      Serial.print("To: ");
      Serial.println(pubTopic_.c_str());
      delay(2000);
      count = count + 1;

      Serial.println("A/C turned OFF");

      //clear lcd screen and update it to ac off
      lcd.clear(); // clear previous values from screen
      lcd.setCursor(2, 0);
      lcd.print("A/C is OFF");
    }
  }
  // ledcWrite(FanChannel, 150);
  // delay(3000);
  // ledcWrite(FanChannel, 255);
  // delay(3000);

  //DHT
  float h = dht.readHumidity();    // Reading Temperature form DHT sensor
  float t = dht.readTemperature(); // Reading Humidity form DHT sensor
                                   //  float tF = (t * 1.8) + 32;
  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
  }
  else
  {
    Serial.print("Temprature: ");
    Serial.print(t);
    Serial.print("  Humidity: ");
    Serial.println(h);
    delay(200);
  }

  //Current sensor
  // Vout is read 1000 Times for precision
  for (int i = 0; i < 1000; i++)
  {
    Vout = (Vout + (resADC * analogRead(powerPin)));
    delay(1);
  }
  // Get Vout in mv
  Vout = Vout / 1000;
  // Convert Vout into Current using Scale Factor
  Current = (Vout - zeroPoint) / scale_factor;
  powerAmount = Current * 5; 

  if(Vout< 0){
    Vout = 0;
  }
  if(Current< 0){
    Current = 0;
  }
  if(powerAmount< 0){
    powerAmount = 0;
  }

  // Print Vout and Current to two Current = ");
  Serial.print("Vout = ");
  Serial.print(Vout, 2);
  Serial.print(" Volts");
  Serial.print("\t Current = ");
  Serial.print(Current, 2);
  Serial.print(" Amps");
  Serial.print("\t Power = ");
  Serial.println(powerAmount);
  delay(300);

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
    //DHT Temprature
    snprintf(msg, BUFFER_LEN, "{\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"temperature\" : %f, \"counter\" : %d}", uuid.c_str(), floor_, room, t,count);
    pubTempTopic = pubTopic + "/temperature";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    delay(2000);
    count = count + 1;
    //    //==============================================================================================
    //Current sensor
    if(fan_is_on){
      snprintf(msg, BUFFER_LEN, "{\"DeviceType\" : \"electricity\",\"DeviceName\" : \"Fan\",\"sensor\" : \"power\",\"isOn\" : \"on\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"volt\" :\"%.4f\" , \"electricitConsumption\" :\"%.4f\", \"counter\" : %d}", uuid.c_str(), floor_, room,Vout,Current,count);
    }
    else{
      snprintf(msg, BUFFER_LEN, "{\"DeviceType\" : \"electricity\",\"DeviceName\" : \"Fan\",\"sensor\" : \"power\",\"isOn\" : \"off\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"volt\" :\"%.4f\" , \"electricitConsumption\" :\"%.4f\", \"counter\" : %d}", uuid.c_str(), floor_, room,Vout,Current,count);
    
    }
    pubTempTopic = pubTopic + "/consumption";
    client.publish(pubTempTopic.c_str(), msg);
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    Serial.print("To: ");
    Serial.println(pubTempTopic.c_str());
    delay(2000);
    count = count + 1;
    //==============================================================================================
  }

  long now2 = millis();
  if (now2 - lastMsg2 > 20000)
  {
    lastMsg2 = now2;
    if (fan_is_on==true)
    {
      snprintf(msg, BUFFER_LEN, "{\"type\" : \"ac\",\"uuid\" : \"%s\",\"floor\" : \"%s\",\"room\" : \"%s\", \"temperature\" : %d, \"counter\" : %d}", uuid.c_str(), floor_, room, ac_count,count);
      pubTempTopic = "study/" + uuid + "/ac";
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