/*
  MQTT Sample

 This sketch connects to an MQTT broker with publish and subscribe actions.

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the Wifi.begin() call accordingly.
 */

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

char ssid[] = "DanielSSID";
char pass[] = "DanielIstToll";
int keyIndex = 0;            // your network key Index number (needed only for WEP)

char mqttServerAddress[] = "test.mosquitto.org";
int mqttServerPort = 1883;

int status = WL_IDLE_STATUS;

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;
PubSubClient mqttClient(client);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  // check firmware version. 1.1.0 is mandatory
  // wifi shield is usually delivered with 1.0.0
  // and an upgrade to 1.1.0 is necessary !!!
  String fv = WiFi.firmwareVersion();
  Serial.print("firmwareVersion: ");
  Serial.println(fv);
  if ( fv != "1.1.0" )
    Serial.println("Please upgrade the firmware"); 

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.print("\n***Starting connection to MQTT broker: ");
  Serial.println(mqttServerAddress);
 
  mqttClient.setServer(mqttServerAddress, mqttServerPort);
  mqttClient.setCallback(callback);
}

void loop()
{
  if(!mqttClient.connected()){
    reconnect();
  }

  mqttClient.loop();

  delay(5000);
  mqttClient.publish("hcc/daniel/outTopic", "Hi guys ;-)");
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// reconnect logic for MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("danielClient")) {
      Serial.println("connected");

    mqttClient.publish("hcc/daniel/outTopic", "I bims 1 Daniel", true);

    mqttClient.subscribe("hcc/+/inTopic");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}






