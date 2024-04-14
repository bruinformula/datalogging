
#include "constants.h"
#include <Arduino.h>

#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"


#include "driver/twai.h"



static float mlx90640To[768];
paramsMLX90640 mlx90640;

unsigned long str_gag_milli = 0;
unsigned long shock_pot_milli = 0;
unsigned long wheel_speed_milli = 0;
unsigned long brake_temp_milli = 0;
unsigned long wheel_temp_milli = 0;
    


void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("----------");
  Serial.print(F("Starting Corner PCB "));
  Serial.println(BOARD_INDEX);


  /*          TWAI SETUP          */

    // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  /*  TWAI SETUP END */


  pinMode(WHEEL_SPEED_PIN, INPUT);

  pinMode(BLUE_LED, OUTPUT);

  Serial1.begin (115200, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  while (!Serial1);
  Serial.println(("Brake Temp Serial Working"));


  //temp sensor stuff

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz


  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    // while (1);
  }else{
    Serial.println("MLX90640 online!");
  }
  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

}

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

void readWriteWheelTemp(){
  if (millis() < wheel_temp_milli) return;

  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  for (int x = start_pixel ; x < end_pixel ; x++)
  {
    
    uint32_t val = mlx90640To[x];

     uint8_t msg[8] = {
      (uint8_t)((val & 0xFF000000) >> 24), 
      (uint8_t)((val & 0x00FF0000) >> 16), 
      (uint8_t)((val & 0x0000FF00) >> 8), 
      (uint8_t)((val & 0x000000FF) >> 0), 
      0, 0, 0, 0};


      sendCanMsg(CAN_ID_FRAME + WHEEL_TEMP_ID_START + (x-start_pixel), msg);

  }

  
  

  
  wheel_temp_milli = millis() + WHEEL_TEMP_READ_INT;
}

void readWriteBrakeTemp(){
  if (millis() < brake_temp_milli) return;

  uint32_t val;
  if (Serial1.available()) {
    String val_as_string = Serial1.readStringUntil ('\n');

    val = (val_as_string.toInt());
  }

  uint8_t msg[8] = {
      (uint8_t)((val & 0xFF000000) >> 24), 
      (uint8_t)((val & 0x00FF0000) >> 16), 
      (uint8_t)((val & 0x0000FF00) >> 8), 
      (uint8_t)((val & 0x000000FF) >> 0), 
      0, 0, 0, 0};

  sendCanMsg(CAN_ID_FRAME + BRAKE_TEMP_ID, msg);
  

  brake_temp_milli = millis() + BRAKE_TEMP_READ_INT;
}



void readWriteWheelSpeed(){
  if (millis() < wheel_speed_milli) return;
  uint8_t val = digitalRead(WHEEL_SPEED_PIN);

  uint8_t msg[8] = {(uint8_t)(val), 0, 0, 0, 0, 0, 0, 0};

  sendCanMsg(CAN_ID_FRAME + WHEEL_SPEED_ID, msg);

  wheel_speed_milli = millis() + WHEEL_SPEED_READ_INT;
}


void readWriteStrainGauges(){
  if (millis() < str_gag_milli) return;

  uint16_t strgagOut1 = analogRead(STR_GAG_PIN_1);
  uint16_t strgagOut2 = analogRead(STR_GAG_PIN_2);
  uint16_t strgagOut3 = analogRead(STR_GAG_PIN_3);

  strgagOut1 = ((strgagOut1*(str_gag_range/1023.0))-str_gag_zero)*(1023.0/str_gag_range);;
  strgagOut2 = ((strgagOut2*(str_gag_range/1023.0))-str_gag_zero)*(1023.0/str_gag_range);;
  strgagOut3 = ((strgagOut3*(str_gag_range/1023.0))-str_gag_zero)*(1023.0/str_gag_range);;

  uint8_t msg1[8] = {(uint8_t)(strgagOut1>> 8), (uint8_t)(strgagOut1), 0, 0, 0, 0, 0, 0};
  uint8_t msg2[8] = {(uint8_t)(strgagOut2 >> 8), (uint8_t)(strgagOut2), 0, 0, 0, 0, 0, 0};
  uint8_t msg3[8] = {(uint8_t)(strgagOut3 >> 8), (uint8_t)(strgagOut3), 0, 0, 0, 0, 0, 0};


  sendCanMsg(CAN_ID_FRAME + CAN_STR_GAG_ID_1, msg1);
  sendCanMsg(CAN_ID_FRAME + CAN_STR_GAG_ID_2, msg2);
  sendCanMsg(CAN_ID_FRAME + CAN_STR_GAG_ID_3, msg3);


  str_gag_milli = millis() + STR_GAG_READ_INT;
}

void readWriteShockPot(){
  if (millis() < shock_pot_milli) return;

  uint16_t shockPotOut = analogRead(SHOCK_POT_PIN);

  shockPotOut = ((shockPotOut*(shock_pot_range/1023.0))-shock_pot_zero)*(1023.0/shock_pot_range);

  // Serial.println(shockPotOut);

  uint8_t msg[8] = {(uint8_t)(shockPotOut>> 8), (uint8_t)(shockPotOut), 0, 0, 0, 0, 0, 0};

  sendCanMsg(CAN_ID_FRAME + CAN_SHOCK_POT_ID, msg);

  str_gag_milli = millis() + STR_GAG_READ_INT;
  // Serial.println(str_gag_milli);
}



void loop() {

  digitalWrite(BLUE_LED, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay(1000);                      // wait for a second
  // digitalWrite(BLUE_LED, LOW);   // turn the LED off by making the voltage LOW
  // delay(1000);     

  readWriteStrainGauges();
  readWriteShockPot();
  readWriteWheelSpeed();
  // readWriteBrakeTemp();
  // readWriteWheelTemp();


  if (TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
          if (message.extd) {
        Serial.println("Message is in Extended Format");
      } else {
        Serial.println("Message is in Standard Format");
      }
      Serial.printf("ID: %lx\nByte:", message.identifier);
      if (!(message.rtr)) {
        for (int i = 0; i < message.data_length_code; i++) {
          Serial.printf(" %d = %02x,", i, message.data[i]);
        }
        Serial.println("");
      }
    }
  }
  
}

void sendCanMsg(uint32_t id, uint8_t msg[]){


    twai_message_t message;
    message.identifier = id;
    message.data_length_code = 8;
    for (int i = 0; i < 8; i++) {
      message.data[i] = msg[i];
    }

    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.print("Message queued for transmission\n");
    } else {
      Serial.println("Failed to queue message for transmission\n");
    }

    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    Serial.println(status_info.state);
}



