/*
  Read the temperature pixels from the MLX90640 IR array
this code is a rewrite driver for the MLX90640 to allow working on arudino. 
no inclusion is noteworthy at this point, based on MIT liscencing using wire.h examples for i2c
driver files were included from sparkfun. sparkfun sensor can be found at https://www.sparkfun.com/products/14768
and sensor examples from melexis. might use some example driver code but it is being simplified and only code i'll use for sensor will be included.

driver updates and simplifications written by james villeneuve
all info above must be included with driver updates and use.
//when this device is first used, data for sensors needs to be moved into eeprom, this is done by typing capital 'U',
when responce is 'data written' the sensor can be used with this code. and stored calibration data will be easily accessed.

  Hardware Connections:
 (https://www.sparkfun.com/products/14425)

*/
#include <Wire.h>
#include <avr/pgmspace.h>
#include "factoryCalData.h" //this contains data that we dumped from eeprom on the sensor and placed into progmem space
#include "MLX90640_SimpleDriver.h"
//below lines change important settings of sensor
#define MLX90640_address 0x33 //Default 7-bit unshifted address of the MLX90640
#define wireClockSpeed 400000 //we define here easier to set from here.
#define SerialBaudRate 115200 //this is speed of serial protocol. be sure enough time is set to allow messages to be sent thru before using i2c
#define continuousmode true //true is default when sensor is bought, however we want step mode. this is checked by the code and only written to if it is different than value here. it is here for experimentation.
#define hzMode 7//0=0.5hz,1=1hz,2=2hz,3=4hz,4=8hz,5=16hz,6=32hz,7=64hz 
#define adSensorResolution 3 //0=16bit,it 1=17bit, 2=18bit, 3=19b
#define MLX90640_mirror true //this flips direction of sensor in case used in camera mode
#define pixelmodeTrueTestModeFalse true //true outputs data in celcius
long timerDataCalc = 0;//this is used to time calculations so we can optimize 
////************************* calibration store. first we run program that dumps the data, and then we cut and paste it here in bracekts below*********
//language words repeated so often easier to add in as const define
void serial_mxl90460 (){Serial.print(F("MLX90640"));} //we use the defined word as a reusabel variable
void serial_reset_message(){Serial.println(F("rebooting so we can verify it took."));}
void serial_set_same_as_flash_msg(){Serial.println(F(" is set the same as #define settings in flash"));}
void serial_HZ_msg(){Serial.print(F(" HZ"));}
void serial_Data_should_now_be_changes_msg(){Serial.println(F("Data should now be changed."));}
void setting_is_different(){Serial.println(F("setting is different in firmware. we are going to write data now."));}
void Device_is_in_mode_of(){Serial.println(F(" Device is in mode of "));}
void(* resetFunc) (void) = 0; //declare reset function @ address 0 //we reset if needed for hz change, allows reboot
void reset_and_msg(){serial_Data_should_now_be_changes_msg();delay(200);serial_reset_message();delay(2000);resetFunc();}  //call reset
byte status =0;//we use for status
//int dataArray[64]; //we store 64 words for 64 values to make sensor work like amg8833. we will advance this later on...

int kVddfield_A=0;//these are the voltage fields for passes 
float Vddfield_A=0;//these are the voltage fields for passes 
int Vdd25field_A=0;//these are the voltage fields for passes 
int kVddfield_B=0;//these are the voltage fields for passes 
float Vddfield_B=0;//these are the voltage fields for passes 
int Vdd25field_B=0;//these are the voltage fields for passes 
float tempb;
float tempa;
void setup()
{

   Wire.begin(MLX90640_address);
  Wire.setClock(wireClockSpeed); //Increase I2C clock speed to 400kHz
 Serial.begin(SerialBaudRate);//we init this first because i think it gives priority on data bus
 for (byte i=0;i<64;i++){Serial.println();}//we clear screen in terminal in case reset.
  serial_mxl90460; Serial.println(F(" IR Array Example"));
//  if (isConnected() == false)
 // {
 //   Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
 //   while (1);
 // }
  
  serial_mxl90460; Serial.println(F(" online!"));
  delay(500);//make sure serial done
   Wire.onReceive(receiveEvent); // register event

  if (status != 0)
    Serial.println(F("Failed to load system parameters"));

//  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println(F("Parameter extraction failed"));

    //here we read eeprom
 Serial.println();
 Serial.println(F("This shows a single image from entire sensor. since we have the start count delay, the memory will already be filled with sensor data"));
 Serial.println();
 delay(200);//serial data to clear
  }
//long  timert =millis();

uint16_t startvalue[8];//array

float ta ;//we use 4 corner sensors to determin chip temp at start up, then we take lowest pixel on screen and use that.
float tr;//reflecitve temp. this is open air shift. going with documentation and using -8
float vddpage0;//we need voltage per page
float vddpage1;//we need voltage per page
//float emissivity = 0.95;
bool runonce =true;//singleshot
void loop()
{ if (runonce){ 
    for (int i=0;i<832;i++){//gets 0x2400 (9216) to 0x273f (10047)
MLX90640_I2CRead(MLX90640_address, 0x2400+i, 1,worddata);
delay(1);
Serial.print(worddata[0]);
Serial.print((", "));delay(1);
if ((i&15)==15){Serial.print(F(" register location:"));Serial.print(i+0x2400-15,HEX);Serial.print("-");Serial.println(i+0x2400,HEX);}//every 16 data make a line
delay(2);
    }//next
    Serial.println(F("data above is from epprom. it will be used later on for calibration"));
Serial.println(F("We will now test the eeprom of the sensor with the flash data. if it does not match it will error but keep working using the included settings."));

    for (int i=0;i<832;i++){
MLX90640_I2CRead(MLX90640_address, 0x2400+i, 1,worddata);
if (worddata[0] !=pgm_read_word_near(factoryCalData+i)){Serial.print(F("Error at (in HEXMODE):"));
Serial.print(0x2400+i,HEX);Serial.print("sensor:");Serial.print(worddata[0],HEX);
Serial.print("PROGMEMVALUE:");Serial.println(pgm_read_word_near(factoryCalData+i),HEX);delay(2);//allows serial to finish before i2c read
}
    }//next
Serial.print(F("Flash memory check complete. if errors troubleshoot and fix for best results"));
  
byte  testMode=MLX90640_GetCurMode(MLX90640_address);
  Serial.print(F("Device is in "));
  if (testMode==0){Serial.print(F("interleaved"));}
  if (testMode==1){Serial.print(F("checkered"));}
  Serial.println(F(" pattern mode"));
 testMode=MLX90640_GetCurResolution(MLX90640_address);//we get resolution of ad converter 16-19 bit resolution
Serial.print(F("Device ad resoltuion set to ....")); 
 Serial.print(16+testMode);Serial.println(F(".... bit"));
if (adSensorResolution==testMode){Serial.println(F("analog resolution")); serial_set_same_as_flash_msg();}
else{//if resolution is different we change resolution
MLX90640_I2CRead(MLX90640_address,  0x800D,  1,worddata);//we read control register
delay(1);  
worddata[0]=worddata[0]&62463;//1111 0011 1111 1111 //we clear bits so we can just add changes
worddata[0]=worddata[0]+1024*adSensorResolution;  
MLX90640_I2CWrite(MLX90640_address, 0x800D, worddata[0]);//we write modified data back to register and make it single mode
delay(5);// we add delay soit can finish write
reset_and_msg();
}
 testMode=MLX90640_GetRefreshRate(MLX90640_address);//get refresh in hz
Serial.print(F("HZ is set to "));
if (testMode==0){Serial.print(F("0.5"));}
if (testMode>0){
if (testMode==1){Serial.print(F("1"));}
if (testMode==2){Serial.print(F("2"));}
if (testMode==3){Serial.print(F("4"));}
if (testMode==4){Serial.print(F("8"));}
if (testMode==5){Serial.print(F("16"));}
if (testMode==6){Serial.print(F("32"));}
if (testMode==7){Serial.print(F("64"));}
Serial.print(F(".0"));
}
serial_HZ_msg();
if (hzMode==testMode){serial_HZ_msg(); serial_set_same_as_flash_msg();}
else{//this means setting is different
serial_HZ_msg();setting_is_different();

MLX90640_I2CRead(MLX90640_address,  0x800D,  1,worddata);//we read control register
delay(1);
worddata[0] = worddata[0]&64639 ;//we make 1111110001111111 the zeros are the control registers for refresh rate. we set zero here for ease of or of 1's later
worddata[0]+=128*hzMode;
//ok we have data changes to matcht the new time change, now we need to write it
MLX90640_I2CWrite(MLX90640_address, 0x800D, worddata[0]);//we write modified data back to register and make it single mode
delay(5);// we add delay soit can finish write
reset_and_msg();
}//end of else
delay(200);
Serial.print(F("register:"));
delay(10);
MLX90640_I2CRead(MLX90640_address,  0x8000,  1,worddata); 
delay(10);  
MLX90640_I2CRead(MLX90640_address,  0x800D,  1,worddata);//we get status of step mode. default it is not in step mode, but we only want to write to eeprom if it is not in step mode
delay(5);//we wait ample time for i2c to complete
printBits(highByte(worddata[0]));
printBits(lowByte(worddata[0]));
if ((worddata[0]&2) ==0){Device_is_in_mode_of();Serial.println(F("'continueous'"));
if (!continuousmode){//here is where we change it if it is set different
setting_is_different();
delay(5);//we wait ample time for i2c to complete

worddata[0]=worddata[0]|2;//we make this and 0000000000000010; bit2 is now high this is needed at 800D HEX location
delay(5);
Serial.println("new values:");
printBits(highByte(worddata[0]));
printBits(lowByte(worddata[0]));
Serial.println();
delay(5000);
 MLX90640_I2CWrite(MLX90640_address, 0x800D, worddata[0]);//we write modified data back to register and make it single mode
  delay(5);//we wait ample time for i2c to complete
 serial_Data_should_now_be_changes_msg();
 delay(5);//we wait ample time for i2c to complete
}
}

else
{Device_is_in_mode_of();Serial.println(F("'step'"));
if (continuousmode){//here is where we change it if it is set different
setting_is_different();
delay(5);//we wait ample time for i2c to complete
worddata[0]=worddata[0]&65533;//we make this and 1111111111111101; bit2 is now low this is needed at 800D HEX location
delay(5);
Serial.println(F("new values:"));
printBits(highByte(worddata[0]));
printBits(lowByte(worddata[0]));
Serial.println();
delay(5000);
 MLX90640_I2CWrite(MLX90640_address, 0x800D, worddata[0]);//we write modified data back to register and make it single mode
  delay(5);//we wait ample time for i2c to complete
 serial_Data_should_now_be_changes_msg();
 delay(5);//we wait ample time for i2c to complete
}
}
delay(10);
Serial.print(F("mirrored mode is :"));
#if MLX90640_mirror == false
Serial.println(F("off"));
#else
Serial.println(F("on"));
#endif
//this prints out register data
printBits(highByte(worddata[0]));
printBits(lowByte(worddata[0]));
Serial.print(F(". This means that page is on:"));
Serial.println(1&worddata[0]);//it can be zero or 1
 Serial.print(F("seconds before startup:"));
 for (byte i=7;i>0;i--){Serial.print(i);Serial.print(F(".."));delay(1000);}//7 seconds to read message before start

//we force two scans so we get preuse data
if (!continuousmode){//if we are in step mode we need to tell sensor to scan page
MLX90640_I2CRead(MLX90640_address,  0x8000,  1,worddata);//we read sensor 
delay(600);
worddata[0]=  worddata[0]|32;//we set 000000000100000 wich is register that starts measurement
MLX90640_I2CWrite(MLX90640_address, 0x8000, worddata[0]);//we write modified values
delay(600);
worddata[0]=  worddata[0]|32;//we set 000000000100000 wich is register that starts measurement
MLX90640_I2CWrite(MLX90640_address, 0x8000, worddata[0]);//we write modified values
}
//we need ontime gain read
ExtractGain();//value stored here SensorGaincommon
Getpixeldata(12,10);
  MLX90640_I2CRead(MLX90640_address,  0x0400,  32,startvalue);runonce=false;}//we get values one time only
   #if pixelmodeTrueTestModeFalse  != true //this allows for raw output **************************
   for (int i=8; i<16;i++){//this samples and draws entire page of data. sensor is 
  MLX90640_I2CRead(MLX90640_address,  0x0400+32*i,  32, mydata);//read 32 places in memory
  #if MLX90640_mirror == false 

  // originally this is 1-31, but now it is 12-17
  for (int x = 12 ; x < 20 ; x+=1) //this is normal sensor mode
  #else 
  for (int x = 20 ; x >12 ; x-=1) //this is mirrored sensor mode
  #endif

  { //we compare active one row to see at output
    int testx=(50+mydata[x]-startvalue[x]);
    char outputx=46;
    if (testx>60){outputx=44;}//+
    if (testx>70){outputx=111;}//o
    if (testx>80){outputx=79;}//O
    if (testx>100){outputx=64;}//@
   // Serial.print  (outputx);Serial.print(F("  "));
   Serial.print(testx); Serial.print(" ");//this gives a numeric value
  }
MLX90640_I2CRead(MLX90640_address,  0x8000,  1,worddata);
delay(10);   
Serial.print(F("mempage (we read all 32 even if old data):"));
Serial.println(1&worddata[0]);//it can be zero or 1
Serial.println();
  delay(5);//allow serial to finish
}// do loop thru all 24 lines!-//we are using only 8 for now
#else //***********************************************************************************
//this reads 8 x8 sensors of data
ExtractVDDParameters();
ExtractAmbientTemp();
ExtractGain();//value stored here SensorGaincommon
//Getpixeldata(12,10);
  for (int i=0; i<32;i++){//this samples and draws entire page of data. sensor is 
  //MLX90640_I2CRead(MLX90640_address,  0x0400+32*i,  32, mydata);//read 32 places in memory
  #if MLX90640_mirror == false 

  // originally this is 1-31, but now it is 12-17
  for (int x = 0 ; x < 24 ; x+=1) //this is normal sensor mode
  #else 
  for (int x = 23 ; x >0 ; x-=1) //this is mirrored sensor mode
  #endif
  { //we compare active one row to see at output
   //Getpixeldata(i,x);delay(5);
   Serial.print(CalculateTo(i,x),2);; Serial.print("|");//this gives a numeric value
   delay(5);
  }
Serial.println("");
Serial.println("-------------------------------------------------------");
  delay(5);//allow serial to finish
}// do loop thru all 24 lines!-//we are using only 8 for now
#endif //************************************************************************************
//we alread see screen. now lets refresh it before we look at data again
if (!continuousmode){//if we are in step mode we need to tell sensor to scan page
MLX90640_I2CRead(MLX90640_address,  0x8000,  1,worddata);//we read sensor 
delay(50);
worddata[0]=  worddata[0]|32;//we set 000000000100000 wich is register that starts measurement
worddata[0]=  worddata[0]|16;//we set 000000000101000 auto next page
delay(50);
ExtractVDDParameters();
kVddfield_A=kVdd; 
Vddfield_A=Vdd;
Vdd25field_A=Vdd25;
tempa=AmbientTemp;
delay(5);
MLX90640_I2CWrite(MLX90640_address, 0x8000, worddata[0]);//we write modified values
delay(16);

worddata[0]=  worddata[0]|32;//we set 000000000100000 wich is register that starts measurement
worddata[0]=  worddata[0]|16;//we set 000000000101000 auto next page
MLX90640_I2CWrite(MLX90640_address, 0x8000, worddata[0]);//we write modified values
delay(50);
delay(5);
#if pixelmodeTrueTestModeFalse == false
ExtractVDDParameters();
ExtractAmbientTemp();
#endif
kVddfield_B=kVdd; 
Vddfield_B=Vdd;
Vdd25field_B=Vdd25;
tempb=AmbientTemp;
delay(5);
}



Serial.print("********kVdd = ");Serial.print(kVddfield_A);Serial.print(" vdd25 =  ");Serial.print(Vdd25field_A);Serial.print(" vdd =  ");Serial.print(Vddfield_A,4);
Serial.print(" Gain =");Serial.print(SensorGaincommon);
Serial.print(" TEMPAFRame_Silicone =");Serial.print(tempa);Serial.println("   **********");

Serial.print("********kVdd = ");Serial.print(kVddfield_B);Serial.print(" vdd25 =  ");Serial.print(Vdd25field_B);Serial.print(" vdd =  ");Serial.print(Vddfield_B,4);

//Serial.print(" KVPTAT= ");Serial.print(KVPTAT,3);
//Serial.print(" KTPTAT= ");Serial.print(KTPTAT,3);
//Serial.print(" VPTAT25 = ");Serial.print(VPTAT25);
Serial.print(" Gain =");Serial.println(SensorGaincommon);
Serial.print(" TEMPBFRame_Silicone =");Serial.print(tempb);Serial.print("   **********");
timerDataCalc=micros();
;

delay(5);//we ad to make sure i2c buffer empty before sending serial
timerDataCalc=micros()-timerDataCalc;
Serial.print(" kta =");Serial.print(kta);
Serial.print(" KV =");Serial.print(KV);
Serial.print(" PixGain =");Serial.print(PixGain);
Serial.print(" pixelTemp =");Serial.println(CalculateTo(12,10),9);
Serial.println("");   
Serial.print(F("pages done. if in step mode memory page will always read 1, because it scans twice and ends up on page 1       calc time microseconds:"));
Serial.println(timerDataCalc);
delay(1800);//2hz is speed of sensor read by default


}

/*
//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
*/
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}





