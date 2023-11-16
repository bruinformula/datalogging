// board: ESP8266 Board 3.0.2 (or newer) -> Generic ESP8266 Module
// reference: https://community.appinventor.mit.edu/t/esp32-sends-data-to-the-app-over-wifi-in-realtime-javascript-ajax/46307
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include "index.h"  //Web page header file

ESP8266WebServer server(80);

// Replace with your network credentials
const char* ssid     = "Mk8 Telemetry";
const char* password = "bruinformula";

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
      if(texto[0]=='B' && texto[1]=='F' && texto[2]=='R'){// verlify if it's our message
        texto_send = texto.substring(3); // confirm text to send
        //Serial.println("Write a text in Serial Monitor and 'Send'");
        Serial.print(texto); // print out what has been sent
      }
      texto = ""; // clean texto
    }
  }
}
/////////////////////////////////////////////////////
