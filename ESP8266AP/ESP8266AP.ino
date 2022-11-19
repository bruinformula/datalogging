// progress: .ino finished, working on index.h for website formatting
// reference: https://community.appinventor.mit.edu/t/esp32-sends-data-to-the-app-over-wifi-in-realtime-javascript-ajax/46307
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include "index.h"  //Web page header file

ESP8266WebServer server(80);

// Replace with your network credentials
const char* ssid     = "ESP-AP-SH-FSAE";
const char* password = "11223344";

char caracter = '0';
String texto = "";
String texto_send = "";


//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot(){
  String s = MAIN_page; //Read HTML contents //read and resend webpage
  server.send(200, "text/html", s); //Send web page
}

void handleADC(){
  server.send(200, "text/plane", texto_send); // send text to server
}

//===============================================================
// Setup
//===============================================================

void setup(void){
  Serial.begin(9600); //single serial port for TX/RX and serial monitor
  Serial.println();
  Serial.println("Booting Sketch...");

  //ESP8266 as soft access point -----------------------------------
  WiFi.softAP(ssid, password);
    
  //show IP address in serial monitor
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
//----------------------------------------------------------------
 
  server.on("/", handleRoot);      //This is display page
  server.on("/readADC", handleADC);//To get update of ADC Value only
 
  server.begin();                  //Start server
  Serial.println("HTTP server started");
}

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void loop(void){
  server.handleClient();
  
  //////////////// TEXT FROM SERIAL MONITOR or TX/RX  ////////////
  // data should be in form of d0|d1|d20,d21,d22|d3|d4|... (end with '|')
  if (Serial.available() > 0){
    caracter = Serial.read(); // read a single character
    texto += caracter; // append to texto
      
    if (caracter == '\n') 
    { 
      // if line ends
      texto_send = texto; // confirm text to send
      //Serial.println("Write a text in Serial Monitor and 'Send'");
      Serial.print(texto); // print out what has been sent
      texto = ""; // clean texto
    }
  }
}
/////////////////////////////////////////////////////
