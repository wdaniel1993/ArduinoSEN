#include<SPI.h>
#include<WiFi.h>

void setup() {
  Serial.begin(9600);

  Serial.println("init wifi");
  printMacAddress();
}

void loop() {
  Serial.println("scanning networks");
  listNetworks();

  delay(10000);
}

void listNetworks() {
  byte numSsid = WiFi.scanNetworks();

  Serial.print("# of networks:");
  Serial.println(numSsid);

  for(int i = 0; i< numSsid; i++){
    Serial.print(i);
    Serial.print(") ");
    Serial.print(WiFi.SSID(i));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(i));
    Serial.print(" dbm");
    Serial.print("\tEncryption: ");
    Serial.println(WiFi.encryptionType(i));
  }
}

void printMacAddress() {
  byte mac[6];                     

   // print your MAC address:
   WiFi.macAddress(mac);
   Serial.print("MAC: ");
   Serial.print(mac[5],HEX);
   Serial.print(":");
   Serial.print(mac[4],HEX);
   Serial.print(":");
   Serial.print(mac[3],HEX);
   Serial.print(":");
   Serial.print(mac[2],HEX);
   Serial.print(":");
   Serial.print(mac[1],HEX);
   Serial.print(":");
   Serial.println(mac[0],HEX);
}

