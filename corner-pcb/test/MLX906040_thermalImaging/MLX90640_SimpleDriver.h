/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <math.h>
#include "factoryCalData.h" //this contains data that we dumped from eeprom on the sensor and placed into progmem space
//I2C_BUFFER_LENGTH is defined in Wire.H
#define MLX90640_address 0x33 //Default 7-bit unshifted address of the MLX90640
//#define I2C_BUFFER_LENGTH BUFFER_LENGTH


#define I2C_BUFFER_LENGTH 32
 int16_t kVdd;
 int16_t Vdd25;
 float Vdd;//used to calc voltage across chip during sensor read
 float KVPTAT;//temp calc
 float KTPTAT;//temp calc
 int16_t VPTAT25;//temp calc
 float ALPHAPTAT;
 float AmbientTemp;//use to know temp of entire silicon. needed so we can use difference to find out actual temp of sensors
 float pixelTemp;
 float SensorGaincommon;//we might convert to int values late on
 float  kta=0.0;
 float KV;
 float KVscale;
 float kGain;
float cpAlpha[2];
int16_t cpOffset[2];
float ilChessC[3];
float KsTa;//need to solve
float tgc=0.0;//11.1.16 usually set to zero
float alphapixel;//normalize
float PixGain;//individual pixel data
float offsetpixel;
 float emissivity=0.95;//
 float To=0.0;
 
int MLX90640_I2CWrite(uint8_t _deviceAddress, unsigned int writeAddress, uint16_t data)
{
  Wire.beginTransmission((uint8_t)_deviceAddress);
  Wire.write(writeAddress >> 8); //MSB
  Wire.write(writeAddress & 0xFF); //LSB
  Wire.write(data >> 8); //MSB
  Wire.write(data & 0xFF); //LSB
  
  if (Wire.endTransmission() != 0)
  {
    //Sensor did not ACK
    Serial.println("Error: Sensor did not ack");
    return (-1);
  }

  uint16_t dataCheck;
int   MLX90640_I2CRead(_deviceAddress, writeAddress, 1, &dataCheck);
  if (dataCheck != data)
  {
    //Serial.println("The write request didn't stick");
    return -2;
  }

  return (0); //Success
}

void ProgmemExtractVDDParameters(uint16_t datastatus){

  //since we have data directly in epprom, all we do here is verify it matches sensor data and return status
  //rest of code will look at internal flash of progmem directly. it will however generate an error here
  //is we do not have values to extract or they do not match system.
}




int MLX90640_I2CRead(uint8_t _deviceAddress, unsigned int startAddress, unsigned int nWordsRead, uint16_t *data)
{

  //Caller passes number of 'unsigned ints to read', increase this to 'bytes to read'
  uint16_t bytesRemaining = nWordsRead * 2;

  //It doesn't look like sequential read works. Do we need to re-issue the address command each time?

  uint16_t dataSpot = 0; //Start at beginning of array

  //Setup a series of chunked I2C_BUFFER_LENGTH byte reads
  while (bytesRemaining > 0)
  {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(startAddress >> 8); //MSB
    Wire.write(startAddress & 0xFF); //LSB
    if (Wire.endTransmission(false) != 0) //Do not release bus
    {
      Serial.println("No ack read");
      return (0); //Sensor did not ACK
    }

    uint16_t numberOfBytesToRead = bytesRemaining;
    if (numberOfBytesToRead > I2C_BUFFER_LENGTH) numberOfBytesToRead = I2C_BUFFER_LENGTH;

    Wire.requestFrom((uint8_t)_deviceAddress, numberOfBytesToRead);
    if (Wire.available())
    {
      for (uint16_t x = 0 ; x < numberOfBytesToRead / 2; x++)
      {
        //Store data into array
        data[dataSpot] = Wire.read() << 8; //MSB
        data[dataSpot] |= Wire.read(); //LSB

        dataSpot++;
      }
    }

    bytesRemaining -= numberOfBytesToRead;

    startAddress += numberOfBytesToRead / 2;
  }

  return (0); //Success
}

int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
     return MLX90640_I2CRead(slaveAddr, 0x2400, 832, eeData);
}



int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData)
{
    uint16_t dataReady = 1;
    uint16_t controlRegister1;
    uint16_t statusRegister;
    int error = 1;
    uint8_t cnt = 0;
    Serial.println(".");
    dataReady = 0;
    while(dataReady == 0)
    {
        error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
    }       
        
    while(dataReady != 0 && cnt < 5)
    { 
        error = MLX90640_I2CWrite(slaveAddr, 0x8000, 0x0030);
        if(error == -1)
        {
            return error;
        }
            
        error = MLX90640_I2CRead(slaveAddr, 0x0400, 832, frameData); 
        if(error != 0)
        {
            return error;
        }
                   
        error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
        cnt = cnt + 1;
    }
    
    if(cnt > 4)
    {
        return -8;
    }    
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    frameData[832] = controlRegister1;
    frameData[833] = statusRegister & 0x0001;
    
    if(error != 0)
    {
        return error;
    }
    
    return frameData[10];    
}
int MLX90640_GetCurResolution(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int resolutionRAM;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    resolutionRAM = (controlRegister1 & 0x0C00) >> 10;
    
    return resolutionRAM; 
}

int MLX90640_GetCurMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int modeRAM;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    modeRAM = (controlRegister1 & 0x1000) >> 12;
    
    return modeRAM; 
}

int MLX90640_GetRefreshRate(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int refreshRate;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    refreshRate = (controlRegister1 & 0x0380) >> 7;
    
    return refreshRate;
}
void printBits(byte myByte){
 for(byte mask = 0x80; mask; mask >>= 1){
   if(mask  & myByte)
       Serial.print('1');
   else
       Serial.print('0');
 }
}
void  ExtractGain(){
worddata[0]= pgm_read_word_near(factoryCalData+0x0030);//we can get this value from eeprom, faster and more reliably

if (worddata[0]> 32767){worddata[0]=worddata[0]-65536;}//per document 11.2.2.4

   kGain=worddata[0];
  MLX90640_I2CRead(MLX90640_address, 0x070A, 1, worddata);//we get from ram

 if ( worddata[0]> 32767){worddata[0]=worddata[0]-65536;}
  kGain=  worddata[0]/ kGain;
   SensorGaincommon=kGain;
}

void ExtractVDDParameters()
{
   worddata[0]= pgm_read_word_near(factoryCalData+0x0033);//we can get this value from eeprom, faster and more reliably
    kVdd =worddata[0];
    kVdd =kVdd & 0xFF00;//we do some required conversion (11.2.2.2 in document)
    kVdd =kVdd >>8 ;//we divide by 2^8 
    if (kVdd >127){kVdd -=256;}//if >127 we subtract 256 as per document
    kVdd =kVdd *32 ;//we now multiply by 32    
    Vdd25=worddata[0];
    Vdd25=Vdd25 & 0x00FF;
    Vdd25=  ((Vdd25 -256)<<5)-8192;
    //we need to get this from ram on sensor because it changes all the time!
 MLX90640_I2CRead(MLX90640_address, 0x072A, 1, worddata);//we get vdd used real time from senso
    Vdd=worddata[0];
    if (Vdd>32767 ){Vdd=Vdd-65536;}//we do this to normalize the value

    
uint16_t storedRes= pgm_read_word_near(factoryCalData+56);//we can get this value from eeprom, faster and more reliably
 storedRes= storedRes& 0x3000;
  storedRes= storedRes>>12;
//we need analog resolution
 MLX90640_I2CRead(MLX90640_address, 0x800D, 1, worddata);//we read register memory
 worddata[0]=(worddata[0]& 0x0C00)>>10;//we get ad resolution detail
 Serial.print("ad data================") ;Serial.println(worddata[0]);  
float resolutionfix=0.0;
if (worddata[0]==0){resolutionfix=3.84;}//this is about right.
if (worddata[0]==1){resolutionfix=1.92;}
if (worddata[0]==2){resolutionfix=0.96;}
if (worddata[0]==3){resolutionfix=0.48;}
Vdd= (resolutionfix * Vdd - Vdd25) / kVdd + 3.3;
}

void ExtractAmbientTemp()
{

worddata[0]= pgm_read_word_near(factoryCalData+0x0032);//we can get this value from eeprom, faster and more reliably


   worddata[0]=worddata[0] & 0xFC00;// as per 11.2.2.3
   KVPTAT=worddata[0]>>10;
   if (KVPTAT<31) {KVPTAT-=64;}   
   KVPTAT = KVPTAT/4096;
   
worddata[0]= pgm_read_word_near(factoryCalData+0x0032);//we can get this value from eeprom, faster and more reliably


    KTPTAT = worddata[0]& 0x03FF;
    if(KTPTAT > 511)
    {
        KTPTAT = KTPTAT - 1024;
    }
    KTPTAT= KTPTAT/8;

VPTAT25= pgm_read_word_near(factoryCalData+0x0031);//we can get this value from eeprom, faster and more reliably
  

worddata[0]= pgm_read_word_near(factoryCalData+0x0010);//we can get this value from eeprom, faster and more reliably

    ALPHAPTAT=(worddata[0] & 0xF000)/pow(2, (double)14) + 8.0f;//as per documentation. wil simplify later on
    float ptat;
    float ptatArt;
    float vdd;
    float ta;   
    vdd = Vdd;//Vdd has already been calculated or should have been
    MLX90640_I2CRead(MLX90640_address, 0x0720, 1, worddata);//we read register memory
    ptat= worddata[0];
    if (ptat> 32767 ){ptat = ptat - 65536;}//documented method
    
    MLX90640_I2CRead(MLX90640_address, 0x0700, 1, worddata);//we read register memory
    ptatArt =worddata[0];
   if(ptatArt > 32767){ptatArt = ptatArt - 65536;}
       ptatArt = (ptat / (ptat * ALPHAPTAT + ptatArt)) * pow(2, 18);
    
    ta = (ptatArt / (1 + KVPTAT * (vdd - 3.3)) - VPTAT25);

    
    ta = ta / KTPTAT + 25;
    AmbientTemp=ta;
}

/*
void Getpixeldata(byte x,byte y){
  while (y>32) {y=y-24;}//we want only 32 values of y, so if over we wrap around
  while (x>32) {x=x-32;}//we want only 32 values of y, so if over we wrap around
  uint16_t AdressOfData= 0x0400+y*32<<5+x;//this gives us ram location of data from x,y value
//this gets pixel x,y and stores value in ram
byte KVvalue;
//kTA_RC_EE; 
float dataValue; //this stores ram value of data
//IR compensation -offset,VDD and Ta(temp average) as per documented section 11.2.2.5.3
//x<----> y^v
//we need to process differently for KTA depending on pixel location
//X odd, Y odd =(0x2436 & FF00)>>8 
//x even, y odd = & 00FF
//x odd  , y even =  0x2437 & 00FF
//x even , y even =(0x2437 & FF00)>>8
//if (x & b00000001)==1 odd
//(x & b00000001)==0 even
 //    kta_RC_EE=
//dataValue =


//return dataValue  
}
*/
//get pixel value ad conversion 19bits
float Getpixeldata(byte x, byte y)
{   
  MLX90640_I2CRead(MLX90640_address,  0x0400+32 * y +x,  1,worddata);//we get pixel gain from ram, 11.2.2.5.1
   PixGain=worddata[0]*kGain;//this is individual pixel gain and uses overal average gain as well 11.2.2.4

    if(PixGain > 32767)
    {
        PixGain = PixGain- 65536;
    }
    
    //we have global store of PixGain
  
  kta=0.0;
    int p = 0;
    int8_t KtaRC[4];
    int8_t KtaRoCo= (pgm_read_word_near(factoryCalData+0x0036) & 0xFF00)>>8 ;//MLX90640 page 26 document
    int8_t KtaRoCe= (pgm_read_word_near(factoryCalData+0x0037) & 0xFF00)>>8 ;//MLX90640 page 26 document
    int8_t KtaReCo= pgm_read_word_near(factoryCalData+0x0036) & 0x00FF ;//MLX90640 page 26 document
    int8_t KtaReCe =pgm_read_word_near(factoryCalData+0x0037) & 0x00FF ;//MLX90640 page 26 document
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
//0x2436
   // KtaRoCo = (eeData[54] & 0xFF00) >> 8;
    if (KtaRoCo > 127)
    {
        KtaRoCo = KtaRoCo - 256;
    }
    KtaRC[0] = KtaRoCo;
    
  //  KtaReCo = (eeData[54] & 0x00FF);
    if (KtaReCo > 127)
    {
        KtaReCo = KtaReCo - 256;
    }
    KtaRC[2] = KtaReCo;
      
  //  KtaRoCe = (eeData[55] & 0xFF00) >> 8;
    if (KtaRoCe > 127)
    {
        KtaRoCe = KtaRoCe - 256;
    }
    KtaRC[1] = KtaRoCe;
      
   // KtaReCe = (eeData[55] & 0x00FF);
    if (KtaReCe > 127)
    {
        KtaReCe = KtaReCe - 256;
    }
    KtaRC[3] = KtaReCe;
  
    ktaScale1 = ((pgm_read_word_near(factoryCalData+0x0038) & 0x00F0) >> 4) + 8;
    ktaScale2 = pgm_read_word_near(factoryCalData+0x0038) & 0x000F;

            p = 32 * y +x;
            split = 2*(p/32 - (p/64)*2) + p%2;
            kta= (pgm_read_word_near(factoryCalData+0x0040 + p) & 0x000E) >> 1;// doc page 20 offset data 0x2440
            if (kta > 3)
            {
               kta = kta - 8;
            }
            kta = kta * (1 << ktaScale2);
            kta = KtaRC[split] + kta;
            kta = kta / pow(2,(double)ktaScale1);
 //KTA=Kta;


    p = 0;
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvRoCe;
    int8_t KvReCo;
    int8_t KvReCe;
    uint8_t kvScale;
    split;

    KvRoCo = (pgm_read_word_near(factoryCalData+0x0034) & 0xF000) >> 12;
    if (KvRoCo > 7)
    {
        KvRoCo = KvRoCo - 16;
    }
    KvT[0] = KvRoCo;
    
    KvReCo = (pgm_read_word_near(factoryCalData+0x0034) & 0x0F00) >> 8;
    if (KvReCo > 7)
    {
        KvReCo = KvReCo - 16;
    }
    KvT[2] = KvReCo;
      
    KvRoCe = (pgm_read_word_near(factoryCalData+0x0034) & 0x00F0) >> 4;
    if (KvRoCe > 7)
    {
        KvRoCe = KvRoCe - 16;
    }
    KvT[1] = KvRoCe;
      
    KvReCe = (pgm_read_word_near(factoryCalData+0x0034) & 0x000F);
    if (KvReCe > 7)
    {
        KvReCe = KvReCe - 16;
    }
    KvT[3] = KvReCe;
  
    kvScale = (pgm_read_word_near(factoryCalData+0x0038) & 0x0F00) >> 8;


            p = 32 * y +x;//y (24) x(32)
            split = 2*(p/32 - (p/64)*2) + p%2;
           KV = KvT[split];
           KV = KV / pow(2,(double)kvScale);
//we calc offsets //as per 11.2.2.5.2
    p = 0;//we reuse p
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;
    uint8_t  occRow;//stores row calibtration for pixel
    uint8_t occColumn;//stores col calibration for pixel information
    occRemScale = (pgm_read_word_near(factoryCalData+0x0010) & 0x000F);
    occColumnScale = (pgm_read_word_near(factoryCalData+0x0010) & 0x00F0) >> 4;
    occRowScale = (pgm_read_word_near(factoryCalData+0x0010) & 0x0F00) >> 8;
    offsetRef = pgm_read_word_near(factoryCalData+0x0011);
    if (offsetRef > 32767) {offsetRef = offsetRef - 65536;}
//we need to do real work here to get correct value i would suspect it is evry 4 values or something like that.>>2?

//this calculates occRow for pixel
        p = x & B00011100; //x=0,4,8,12,16,20
        p>>2;//we make the values lower 0,1,2,3,4,5,6,
        if (x & B00000011==0){occRow=(pgm_read_word_near(factoryCalData+0x0012+p) & 0x000F); }
        if (x & B00000011==1){(occRow=pgm_read_word_near(factoryCalData+0x0012+p) & 0x00F0)>>4; }
        if (x & B00000011==2){(occRow=pgm_read_word_near(factoryCalData+0x0012+p) & 0x0F00)>>8; }
        if (x & B00000011==3){(occRow=pgm_read_word_near(factoryCalData+0x0012+p) & 0xF000)>>12; }
        if (occRow>7){occRow=occRow-16;}

        p = y & B00011100; //x=0,4,8,12,16,20
        p>>2;//we make the values lower 0,1,2,3,4,5,6,
        if (y & B00000011==0){occColumn=(pgm_read_word_near(factoryCalData+0x001B+p) & 0x000F); }
        if (y & B00000011==1){(occColumn=pgm_read_word_near(factoryCalData+0x001B+p) & 0x00F0)>>4; }
        if (y & B00000011==2){(occColumn=pgm_read_word_near(factoryCalData+0x001B+p) & 0x0F00)>>8; }
        if (y & B00000011==3){(occColumn=pgm_read_word_near(factoryCalData+0x001B+p) & 0xF000)>>12; }
        if (occColumn>7){occColumn=occColumn-16;}      

offsetpixel=offsetpixel*(1 << occRemScale);//we have extracted offset for pixel data
  offsetpixel=(offsetRef + (occRow << occRowScale) + (occColumn << occColumnScale) +  offsetpixel);  //we have extracted offset for pixel data   



  //now we get ir compensation data
      
    uint8_t calibrationModeEE;
    
    calibrationModeEE = (pgm_read_word_near(factoryCalData+0x000A) & 0x0800) >> 4;//10 is 000A
    calibrationModeEE = calibrationModeEE ^ 0x80;

    ilChessC[0] = (pgm_read_word_near(factoryCalData+0x0035) & 0x003F);//53 is 0x0035
    if (ilChessC[0] > 31)
    {
        ilChessC[0] = ilChessC[0] - 64;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;
    
    ilChessC[1] = (pgm_read_word_near(factoryCalData+0x0035) & 0x07C0) >> 6;
    if (ilChessC[1] > 15)
    {
        ilChessC[1] = ilChessC[1] - 32;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;
    
    ilChessC[2] = (pgm_read_word_near(factoryCalData+0x0035) & 0xF800) >> 11;
    if (ilChessC[2] > 15)
    {
        ilChessC[2] = ilChessC[2] - 32;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;
    
    calibrationModeEE = calibrationModeEE;//redundant but here for explenation
    ilChessC[0] = ilChessC[0];
    ilChessC[1] = ilChessC[1];
    ilChessC[2] = ilChessC[2];
//here we normalize and calculate alpha paramiters
      p = 0;
    int accRow;//24
    int accColumn;//32
     

uint8_t    accRemScale =     pgm_read_word_near(factoryCalData+0x0020) & 0x000F;//32
uint8_t    accColumnScale = (pgm_read_word_near(factoryCalData+0x0020)  & 0x00F0) >> 4;
uint8_t    accRowScale =    (pgm_read_word_near(factoryCalData+0x0020)  & 0x0F00) >> 8;
uint8_t    alphaScale =    ((pgm_read_word_near(factoryCalData+0x0020)  & 0xF000) >> 12) + 30;
int        alphaRef =        pgm_read_word_near(factoryCalData+0x0021) ;//33

        p = x & B00011100; //x=0,4,8,12,16,20
        p>>2;//we make the values lower 0,1,2,3,4,5,6,
        if (x & B00000011==0){accRow=(pgm_read_word_near(factoryCalData+0x0022+p) & 0x000F); }
        if (x & B00000011==1){(accRow=pgm_read_word_near(factoryCalData+0x0022+p) & 0x00F0)>>4; }
        if (x & B00000011==2){(accRow=pgm_read_word_near(factoryCalData+0x0022+p) & 0x0F00)>>8; }
        if (x & B00000011==3){(accRow=pgm_read_word_near(factoryCalData+0x0022+p) & 0xF000)>>12; }
        if (accRow>7){accRow=accRow-16;}

        p = y & B00011100; //x=0,4,8,12,16,20
        p>>2;//we make the values lower 0,1,2,3,4,5,6,
        if (y & B00000011==0){accColumn=(pgm_read_word_near(factoryCalData+0x0028+p) & 0x000F); }
        if (y & B00000011==1){(accColumn=pgm_read_word_near(factoryCalData+0x0028+p) & 0x00F0)>>4; }
        if (y & B00000011==2){(accColumn=pgm_read_word_near(factoryCalData+0x0028+p) & 0x0F00)>>8; }
        if (y & B00000011==3){(accColumn=pgm_read_word_near(factoryCalData+0x0028+p) & 0xF000)>>12; }
        if (accColumn>7){accColumn=accColumn-16;} 
        p = 32*y+x;   
        alphapixel=(pgm_read_word_near(factoryCalData+0x0040+p) & 0x03F0)>>4;//11.2.2.8
        if (alphapixel>31){alphapixel=alphapixel-64;}
            alphapixel = alphapixel*(1 << accRemScale);
            alphapixel = (alphaRef + (accRow << accRowScale) + (accColumn << accColumnScale) +alphapixel);
            alphapixel = alphapixel / pow(2,(double)alphaScale);

            //ok all this to get to the meat! the To for this pixel cell in degrees C!
    int KsToScale;
    int8_t step;
    int ct[4];
    int ksTo[4];
    step = ((pgm_read_word_near(factoryCalData+0x003F) & 0x3000) >> 12) * 10;  
    ct[0] = -40;
    ct[1] = 0;
    ct[2] = ((pgm_read_word_near(factoryCalData+0x003F)) & 0x00F0) >> 4;
    ct[3] = ((pgm_read_word_near(factoryCalData+0x003F)) & 0x0F00) >> 8;
    
    ct[2] = ct[2]*step;
    ct[3] = ct[2] + ct[3]*step;
    
    KsToScale = ((pgm_read_word_near(factoryCalData+0x003F))& 0x000F) + 8;
    KsToScale = 1 << KsToScale;
    
    ksTo[0] = (pgm_read_word_near(factoryCalData+0x003D)) & 0x00FF;
    ksTo[1] = (pgm_read_word_near(factoryCalData+0x003D) & 0xFF00) >> 8;
    ksTo[2] = (pgm_read_word_near(factoryCalData+0x003E)) & 0x00FF;
    ksTo[3] = (pgm_read_word_near(factoryCalData+0x003E) & 0xFF00) >> 8;
    



          
}

float CalculateTo(byte x, byte y)
{float vdd;float ta;
float ptat;float ptatArt; 
   worddata[0]= pgm_read_word_near(factoryCalData+0x0033);//we can get this value from eeprom, faster and more reliably
    kVdd =worddata[0];kVdd =kVdd & 0xFF00;kVdd =kVdd >>8 ;if (kVdd >127){kVdd -=256;};kVdd =kVdd *32 ;Vdd25=worddata[0];Vdd25=Vdd25 & 0x00FF;Vdd25=  ((Vdd25 -256)<<5)-8192;
    Vdd25=worddata[0];Vdd25=Vdd25 & 0x00FF;Vdd25=  ((Vdd25 -256)<<5)-8192;MLX90640_I2CRead(MLX90640_address, 0x072A, 1, worddata);
    Vdd=worddata[0];if (Vdd>32767 ){Vdd=Vdd-65536;}//we do this to normalize the value
     uint16_t storedRes= pgm_read_word_near(factoryCalData+56); storedRes= storedRes& 0x3000;storedRes= storedRes>>12;
 MLX90640_I2CRead(MLX90640_address, 0x800D, 1, worddata);worddata[0]=(worddata[0]& 0x0C00)>>10;
float resolutionfix=0.0;if (worddata[0]==0){resolutionfix=3.84;};if (worddata[0]==1){resolutionfix=1.92;};if (worddata[0]==2){resolutionfix=0.96;};if (worddata[0]==3){resolutionfix=0.48;};
Vdd= (resolutionfix * Vdd - Vdd25) / kVdd + 3.3;



worddata[0]= pgm_read_word_near(factoryCalData+0x0032);//we can get this value from eeprom, faster and more reliably
worddata[0]=worddata[0] & 0xFC00;KVPTAT=worddata[0]>>10;if (KVPTAT<31) {KVPTAT-=64;};KVPTAT = KVPTAT/4096;// as per 11.2.2.3
worddata[0]= pgm_read_word_near(factoryCalData+0x0032);//we can get this value from eeprom, faster and more reliably
KTPTAT = worddata[0]& 0x03FF;if(KTPTAT > 511){KTPTAT = KTPTAT - 1024;};KTPTAT= KTPTAT/8;
VPTAT25= pgm_read_word_near(factoryCalData+0x0031);
worddata[0]= pgm_read_word_near(factoryCalData+0x0010);//we can get this value from eeprom, faster and more reliably

    ALPHAPTAT=(worddata[0] & 0xF000)/pow(2, (double)14) + 8.0f;
     
   
    MLX90640_I2CRead(MLX90640_address, 0x0720, 1, worddata);//we read register memory
    ptat= worddata[0];
    if (ptat> 32767 ){ptat = ptat - 65536;}//documented method
    
    MLX90640_I2CRead(MLX90640_address, 0x0700, 1, worddata);//we read register memory
    ptatArt =worddata[0];if(ptatArt > 32767){ptatArt = ptatArt - 65536;};ptatArt = (ptat / (ptat * ALPHAPTAT + ptatArt)) * pow(2, 18);
    ta = (ptatArt / (1 + KVPTAT * (Vdd - 3.3)) - VPTAT25);ta = ta / KTPTAT + 25;


//******************************************************************  
    float tr=ta-8;//as per documentation 11.2.2.9
   // float emissivity=1;//
    float ksTo[4];//we somehow need this value. 
    float ct[4];
//******************************************************************    


    int KsToScale;
    int8_t step;
    
    step = ((pgm_read_word_near(factoryCalData+63) & 0x3000) >> 12) * 10;
    
    ct[0] = -40;
    ct[1] = 0;
    ct[2] = (pgm_read_word_near(factoryCalData+63) & 0x00F0) >> 4;
    ct[3] = (pgm_read_word_near(factoryCalData+63) & 0x0F00) >> 8;
    
    ct[2] = ct[2]*step;
    ct[3] = ct[2] + ct[3]*step;
    
    KsToScale = (pgm_read_word_near(factoryCalData+63) & 0x000F) + 8;
    KsToScale = 1 << KsToScale;
    
    ksTo[0] = pgm_read_word_near(factoryCalData+61) & 0x00FF;
    ksTo[1] = (pgm_read_word_near(factoryCalData+61)& 0xFF00) >> 8;
    ksTo[2] = pgm_read_word_near(factoryCalData+62) & 0x00FF;
    ksTo[3] = (pgm_read_word_near(factoryCalData+62) & 0xFF00) >> 8;
    
    
    for(int i = 0; i < 4; i++)
    {
        if(ksTo[i] > 127)
        {
           ksTo[i] = ksTo[i] -256;
        }
        ksTo[i] = ksTo[i] / KsToScale;
    } 


//******************************************************************
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    //float To;
    float alphaCorrR[4];
    int8_t range;
    uint16_t subPage;

   if  ( (x+y) & B00000010 ==2 ) {subPage =0;}
   if  ( (x+y) & B00000010 !=2 ) {subPage =1;}

    
   
   // vdd = MLX90640_GetVdd(frameData, params);
   // ta = MLX90640_GetTa(frameData, params);
    ta4 = pow((ta + 273.15), (double)4);
    tr4 = pow((tr + 273.15), (double)4);
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    alphaCorrR[0] = 1 / (1 + ksTo[0] * 40);
    alphaCorrR[1] = 1 ;
    alphaCorrR[2] = (1 + ksTo[2] * ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + ksTo[3] * (ct[3] - ct[2]));
    
//------------------------- Gain calculation -----------------------------------    

worddata[0]= pgm_read_word_near(factoryCalData+0x0030);//we can get this value from eeprom, faster and more reliably

if (worddata[0]> 32767){worddata[0]=worddata[0]-65536;}//per document 11.2.2.4

   gain=worddata[0];
  MLX90640_I2CRead(MLX90640_address, 0x070A, 1, worddata);//we get from ram

 if ( worddata[0]> 32767){worddata[0]=worddata[0]-65536;}
  gain=  worddata[0]/ gain;
   SensorGaincommon=gain;//we use this for testing currently
//-------------------------------------
// ExtractCPParameters

    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = ((pgm_read_word_near(factoryCalData+32) & 0xF000) >> 12) + 27;
    
    offsetSP[0] = (pgm_read_word_near(factoryCalData+58) & 0x03FF);
    if (offsetSP[0] > 511)
    {
        offsetSP[0] = offsetSP[0] - 1024;
    }
    
    offsetSP[1] = (pgm_read_word_near(factoryCalData+58) & 0xFC00) >> 10;
    if (offsetSP[1] > 31)
    {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0]; 
    
    alphaSP[0] = (pgm_read_word_near(factoryCalData+57) & 0x03FF);
    if (alphaSP[0] > 511)
    {
        alphaSP[0] = alphaSP[0] - 1024;
    }
    alphaSP[0] = alphaSP[0] /  pow(2,(double)alphaScale);
    
    alphaSP[1] = (pgm_read_word_near(factoryCalData+57) & 0xFC00) >> 10;
    if (alphaSP[1] > 31)
    {
        alphaSP[1] = alphaSP[1] - 64;
    }
    alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0];
    
    cpKta = (pgm_read_word_near(factoryCalData+59) & 0x00FF);
    if (cpKta > 127)
    {
        cpKta = cpKta - 256;
    }
    ktaScale1 = ((pgm_read_word_near(factoryCalData+56) & 0x00F0) >> 4) + 8;    
    cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = (pgm_read_word_near(factoryCalData+59) & 0xFF00) >> 8;
    if (cpKv > 127)
    {
        cpKv = cpKv - 256;
    }
    kvScale = (pgm_read_word_near(factoryCalData+56) & 0x0F00) >> 8;
    cpKv = cpKv / pow(2,(double)kvScale);
       
    cpAlpha[0] = alphaSP[0];
    cpAlpha[1] = alphaSP[1];
    cpOffset[0] = offsetSP[0];
    cpOffset[1] = offsetSP[1]; 
//---------------------------------------
//we find KsTa as per 11.1.8 

 KsTa = (pgm_read_word_near(factoryCalData+60)  & 0xFF00) >> 8;//get from flash epprom
    if(KsTa > 127)
    {
        KsTa = KsTa -256;
    }
    KsTa = KsTa / 8192.0f;
    
    

//------------------------- To calculation -------------------------------------

    uint8_t calibrationModeEE;
    
    calibrationModeEE = (pgm_read_word_near(factoryCalData+0x000A) & 0x0800) >> 4;//10 is 000A
    calibrationModeEE = calibrationModeEE ^ 0x80;

    
    MLX90640_I2CRead(MLX90640_address,0x2740, 1, worddata);//we read register memory    
    mode = (worddata[0] & 0x1000) >> 5;//0x2400+832 9216+832 0x2740 hex
    MLX90640_I2CRead(MLX90640_address,0x2708, 1, worddata);//we read register memory    

    irDataCP[0] = worddata[0];  //9216+776 9992 or 0x2708
     MLX90640_I2CRead(MLX90640_address,0x2728, 1, worddata);//we read register memory    
    irDataCP[1] = worddata[0];  //9216+808 10024 or 0x2728
    for( int i = 0; i < 2; i++)
    {
        if(irDataCP[i] > 32767)
        {
            irDataCP[i] = irDataCP[i] - 65536;
        }
        irDataCP[i] = irDataCP[i] * gain;
    }
    irDataCP[0] = irDataCP[0] - cpOffset[0] * (1 + cpKta * (ta - 25)) * (1 + cpKv * (vdd - 3.3));
    if( mode ==  calibrationModeEE)// calibration is set from above
    {
        irDataCP[1] = irDataCP[1] - cpOffset[1] * (1 + cpKta * (ta - 25)) * (1 + cpKv * (vdd - 3.3));
    }
    else
    {
      irDataCP[1] = irDataCP[1] - (cpOffset[1] + ilChessC[0]) * (1 + cpKta * (ta - 25)) * (1 + cpKv * (vdd - 3.3));
    }

    //for( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
    //{
       // ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2; 
       // chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2); 
       // conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);
        /*
        if(mode == 0)
        {
          pattern = ilPattern; 
        }
        else 
        {
          pattern = chessPattern; 
        } 
           */           
         MLX90640_I2CRead(MLX90640_address,0x0341, 1, worddata);//we read register memory 
        if(pattern == worddata[0]) ;;//0341
        {    MLX90640_I2CRead(MLX90640_address,  0x0400+32 * y +x,  1,worddata);//we get pixel gain from ram, 11.2.2.5.1 
            irData =worddata[0];
            if(irData > 32767)
            {
                irData = irData - 65536;
            }
            irData = irData * gain;
          
            irData = irData -offsetpixel*(1 + kta*(ta - 25))*(1 + KV*(vdd - 3.3));
            if(mode !=  calibrationModeEE)
            {
              irData = irData + ilChessC[2] * (2 * ilPattern - 1) - ilChessC[1] * conversionPattern; 
            }
            
            irData = irData / emissivity;
    
            irData = irData - tgc * irDataCP[subPage];
            
            alphaCompensated = (alphapixel - tgc * cpAlpha[subPage])*(1 + KsTa * (ta - 25));
            
            Sx = pow((double)alphaCompensated, (double)3) * (irData + alphaCompensated * taTr);
            Sx = sqrt(sqrt(Sx)) * ksTo[1];
            To = sqrt(sqrt(irData/(alphaCompensated * (1 - ksTo[1] * 273.15) + Sx) + taTr)) - 273.15;
            To = sqrt(sqrt(1/(alphaCompensated * (1 - 1 * 273.15) + Sx) + taTr)) - 273.15;
                    
            if(To < ct[1])
            {
                range = 0;
            }
            else if(To < ct[2])   
            {
                range = 1;            
            }   
            else if(To < ct[3])
            {
                range = 2;            
            }
            else
            {
                range = 3;            
            }      
            
            To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + ksTo[range] * (To - ct[range]))) + taTr)) - 273.15;
            return To;
           
        }
        
  // }
  

}

