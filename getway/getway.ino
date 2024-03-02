/*
 * Project: River monitoring with LoRa
 * Device: LoRa Generic RX
 * Author: Wilson Cosmo
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>


const char* ssid = "iPhone Rommel";
const char* password = "acesso1111";

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

//Endnodes W:
String tx_id_1 = "T1";
String tx_id_2 = "T2";
String tx_id_3 = "T3";

//Endnodes T:
String tx_id_4 = "T4";
String tx_id_5 = "T5";
String tx_id_6 = "T6";

//Endnodes Wendell:
String tx_id_7 = "T7";
String tx_id_8 = "T8";

//Endnode debug
String tx_id_9 = "T9";
String tx_id_10 = "T0";

//Your Domain name with URL path or IP address with path
String serverName = "https://api.thingspeak.com/update?api_key=";
String thingspeak_key = "6NHWH4LJEZT4JXAB";  /* Coloque aqui sua chave de escrita do seu canal */

void set_api_key(String idd){
  if(idd == tx_id_10){
    //thingspeak_key = "JM2EWASN5EKGF930";
    thingspeak_key = "6NHWH4LJEZT4JXAB";
  }
  if(idd == tx_id_2){
    thingspeak_key = "090H4X9YYS3SKDYE";
  }
  if(idd == tx_id_3){
    thingspeak_key = "YK14OHD0NP7WQWHJ";
  }
  if(idd == tx_id_4){
    thingspeak_key = "CKMNFCJ05MICVPTQ";
  }
  if(idd == tx_id_5){
    thingspeak_key = "";
  }
  if(idd == tx_id_6){
    thingspeak_key = "";
  }
  if(idd == tx_id_7){
    thingspeak_key = "7SJ4XZC282NUI7FU";
  }
  if(idd == tx_id_8){
    thingspeak_key = "";
  }
  if(idd == tx_id_1){
    thingspeak_key = "6NHWH4LJEZT4JXAB";
  }

}

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 2 seconds (5000)
unsigned long timerDelay = 2000;

void wifi_start(){

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

}

void setup() {
  
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=====================================");    
  Serial.println("Device: LoRa Generic RX");  
  Serial.println("=====================================");

  LoRa.setPins(SS,RST,DI0);

  if (!LoRa.begin(915000000)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  Serial.println("LoRa receiver ok"); 

  wifi_start();

  Serial.println("\nLoRa Gateway ready.\n\n");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    String incoming = "";    

    // read packet
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    
    if(incoming.startsWith(tx_id_1) || incoming.startsWith(tx_id_2) || incoming.startsWith(tx_id_3) || incoming.startsWith(tx_id_4) || incoming.startsWith(tx_id_5) || incoming.startsWith(tx_id_6) || incoming.startsWith(tx_id_7) || incoming.startsWith(tx_id_8) || incoming.startsWith(tx_id_9)){
      //Serial.print("Received packet: '");
      Serial.print(incoming);
      Serial.print(";");
      Serial.print(String(LoRa.packetRssi()));
      Serial.print(";");
      Serial.println(String(LoRa.packetSnr()));
      //Serial.println("'"); 

      String s_payload = incoming;

      //Tx ID:
      int f1 = s_payload.indexOf(';');
      //Serial.print("Tx ID: ");
      //Serial.println(s_payload.substring(0,f1));
      //c_id = s_payload.substring(0,f1);
      //set_api_key(s_payload.substring(0,f1));

      //Tag:
      int f2 = s_payload.indexOf(';', f1 + 1 );
      //Serial.print("Tag: ");
      //Serial.println(s_payload.substring(f1+1,f2));        
      
      //Battery Voltage
      int f3 = s_payload.indexOf(';', f2 + 1 );
      //Serial.print("Battery Voltage: ");
      //Serial.println(s_payload.substring(f2+1,f3));

      //Temperature
      int f4 = s_payload.indexOf(';', f3 + 1 );
      //Serial.print("Temperature: ");
      //Serial.println(s_payload.substring(f3+1,f4));

      //ph
      int f5 = s_payload.indexOf(';', f4 + 1 );
      //Serial.print("Solar Panel Voltage: ");
      //Serial.println(s_payload.substring(f4+1,f5));

      //do
      int f6 = s_payload.indexOf(';', f5 + 1 );
      //Serial.print("Solar Panel Voltage: ");
      //Serial.println(s_payload.substring(f5+1,f6));

      //Solar Panel Voltage
      int f7 = s_payload.indexOf(';', f6 + 1 );
      //Serial.print("Solar Panel Voltage: ");
      //Serial.println(s_payload.substring(f6+1,f7));
      

      //Send via HTTP post
      if ((millis() - lastTime) > timerDelay) {
      //Check WiFi connection status
      if(WiFi.status()== WL_CONNECTED){
        HTTPClient http;

        String serverPath = serverName + thingspeak_key + "&field1=" + s_payload.substring(f1+1,f2) + "&field2=" + s_payload.substring(f2+1,f3) + "&field3=" + s_payload.substring(f3+1,f4) + "&field4=" + s_payload.substring(f4+1,f5) + "&field5=" + s_payload.substring(f5+1,f6) + "&field6=" + s_payload.substring(f6+1,f7) + "&field7=" + String(LoRa.packetRssi()) + "&field8=" + String(LoRa.packetSnr());
        Serial.print("HTTP post: ");
        Serial.println(serverPath);
        
        // Your Domain name with URL path or IP address with path
        http.begin(serverPath.c_str());
        
        // If you need Node-RED/server authentication, insert user and password below
        //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
        
        // Send HTTP GET request
        int httpResponseCode = http.GET();
        
        if (httpResponseCode>0) {
          Serial.print("HTTP Response code: ");
          Serial.println(httpResponseCode);
          String payload = http.getString();
          Serial.println(payload);
        }
        else {
          Serial.print("Error code: ");
          Serial.println(httpResponseCode);
        }
        // Free resources
        http.end();
      }
      else {
        Serial.println("WiFi Disconnected");
        wifi_start();
      }
      lastTime = millis();
    }

      
    }
    else{
      Serial.println("Invalid ID!");
    }
    

  
  }
}
