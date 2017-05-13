//example: http://api.openweathermap.org/data/2.5/weather?q=Linz&mode=xml&APPID=9814a0b3d429b53576623315c7944469

#include<SPI.h>
#include<WiFi.h>

char ssid[] = "DanielSSID";
char pass[] = "DanielIstToll";

int status = WL_IDLE_STATUS;

char server[] = "api.openweathermap.org";
char city[] = "Krems";
char appKey[] = "9814a0b3d429b53576623315c7944469";

WiFiClient client;

void setup() {
  Serial.begin(9600);

  while(status != WL_CONNECTED){
    Serial.print("Connect to network: ");
    Serial.println(ssid);
    
    status = WiFi.begin(ssid,pass);
    delay(5000);
  }

  Serial.println("connected sucessfully");
  printWifiStatus();

  Serial.println("connect to server...");

  Serial.println(server);

  if(client.connect(server, 80))
  {
      Serial.println("connected to server!");
      client.print("GET /data/2.5/weather?q=");
      client.print(city);
      client.print("&mode=json&units=metrics&APPID=");
      client.print(appKey);
      client.println(" HTTP/1.1");
      client.print("Host: ");
      client.println(server);
      client.println("Connection: close");
      client.println();
  }
  else 
  {
    Serial.println("ERROR: no server connection");
  }
}

void loop() {
  while(client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  if(!client.connected()){
    Serial.println("disconnected");
    client.stop();

    while(true);
  }
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
