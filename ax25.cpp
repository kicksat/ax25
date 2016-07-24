#include "ax25.h"

  // Calls constructor GenericSPIClass encapsulates the standard Arduino hardware and other
  // hardware SPI interfaces. Defaults to Frequency1MHz, BitOrderMSBFirst, DataMode0.
  // Calls RH_RF22 constructor with defaults of SS pin and pin 2 for interrupt
AX25::AX25(uint8_t slaveSelectPin, uint8_t interruptPin, uint8_t _shutdownPin)
 : spi(RHHardwareSPI()), radio(RH_RF22(slaveSelectPin, interruptPin, spi)) {
  shutdownPin = _shutdownPin;
}

// Power cycles the radio, then initalizes the radio
bool AX25::powerAndInit() {
  // Need a delay before turning on radio
  // so that power supply can stabilize
  digitalWrite(shutdownPin, HIGH);
  delay(2000);
  digitalWrite(shutdownPin, LOW);
  delay(500);

  if (!radio.init()) {
    return false;
    Serial.println("init failed");
  } else {
    return true;
    Serial.println("init success");
  }

}


// Formats and transmit messages and then puts radio to sleep
void AX25::transmit(char* message1, uint16_t size) {
  Index = 0;
  arrayInit();
  setCallsignAndSsid();
  // Serial.println(message);
  // strcpy(message, message1);

  memcpy(message, message1, size);
  // Serial.print("messge size: "); Serial.println(sizeof(message));
  // Serial.print("messge: "); Serial.println(message);
  formatPacket(size);
  sendPacket();
  radio.waitPacketSent();
  radio.sleep();
}

bool AX25::available() {
  return radio.available();
}

void AX25::setRxMode() {
  // radio.sleep();
  radio.setModeRx();

  arrayInit();
  setCallsignAndSsid();
  radio.setModeIdle();
  radio.setFrequency(437.505);  //TODO: FIX
  radio.setModemRegisters(&FSK1k2);
}

void AX25::setTxMode() {
  radio.setModeTx();
}

void AX25::sendPacket() {
  radio.setModeIdle();
  radio.setFrequency(437.505);  //TODO: FIX
  radio.setModemRegisters(&FSK1k2);
  radio.setTxPower(RH_RF22_RF23BP_TXPOW_29DBM); //TODO FIX
  // radio.setTxPower(0x02); //TODO FIX
  radio.send(finalSequence, Index);  
}

bool AX25::receive(uint8_t* buf, uint8_t* len) {
  *len = MAX_LENGTH_FINAL;
  return radio.recv(buf, len);
}

void AX25::setSSIDsource(byte ssid_src) { ssid_source = ssid_src;}
void AX25::setSSIDdest(byte ssid_dest) { ssid_destination = ssid_dest;}
void AX25::setFromCallsign(char *fromcallsign){strcpy(SrcCallsign,fromcallsign);}
void AX25::setToCallsign(char *tocallsign){strcpy(DestCallsign,tocallsign);}

// void setFrequency(float freq) {radio.setFrequency(freq);}
// void setPower(byte pwr) {radio.setTxPower(pwr);}

void AX25::addHeader(byte *Buffer) {
    //Shift bits 1 place to the left in order to allow for HDLC extension bit
    for (int i=0; i < strlen(DestCallsign) ; i++) Buffer[Index++] = DestCallsign[i]<<1;

    // Append SSID Destination
    Buffer[Index++] = ssid_destination;  

    //Append Source Callsign
    for (int i=0; i < strlen(SrcCallsign) ; i++) Buffer[Index++] = SrcCallsign[i]<<1;
    
    //Append SSID Source
    Buffer[Index++] = ssid_source;
   
    //Append Control bits
    Buffer[Index++] = AX25_CONTROL;
    
    //Append Protocol Identifier
    Buffer[Index++] = AX25_PROTOCOL;
}

void AX25::formatPacket(uint16_t size) {
  
  // Add Header
  addHeader(bitSequence);

  //Add Message
  for (int i=0; i < size ; i++) bitSequence[Index++] = message[i];

  //Convert bit sequence from MSB to LSB
  for (int i=0; i < Index ; i++) bitSequence[i] = MSB_LSB_swap_8bit(bitSequence[i]);

  //Compute Frame check sequence : CRC
  FCS = crcCcitt(bitSequence, Index);

  // Serial.print("CRC:"); Serial.println(FCS, HEX);
  
  //Add FCS in MSB form
  //Add MS byte
  bitSequence[Index++] = (FCS >> 8) & 0xff;
  //Add LS byte
  bitSequence[Index++] = FCS & 0xff;

    
  // radio.printBuffer("Init Message:", bitSequence, Index);

    
  //Bit Processing...Bit stuff, add FLAG and do NRZI enconding...
  bitProcessing(bitSequence,Index);

  // Serial.println("mesg; ");
  // for (int i=0; i< MAX_LENGTH_FINAL ;i++) Serial.println(finalSequence[i]);
  // Serial.println("");
  
}



void AX25::bitProcessing(byte *Buffer, uint8_t bytelength) {
  
    byte BitSequence[bytelength*8+1];
    byte BitSequenceStuffed[bytelength*8+bytelength*8/5+1];
    int k = 0; //general counter
    int _size = 0;
    int s = 0; //stuffed sequence counter
    uint8_t cnt = 0 ;//Bit stuff counter  
    uint8_t remBits = 0;
    byte temp = 0;
    byte byte_temp[255*8];//max message lenght 255 bytes
    
    k = 0;
    //Convert bits to byte size
    for (int i = 0; i< bytelength ; i++)
    {
      for (register uint8_t t=128; t>0 ; t = t/2) {
        if (Buffer[i] & t) BitSequence[k++] = 0x01;
        else BitSequence[k++] = 0x00;
       }       
     }
     
     // stuff a 0 after five consecutive 1s.
     for (int i = 0; i < k ; i++)
     {
        if (BitSequence[i] == 0x01) cnt++;
        else cnt = 0; // restart count at 1
     
        BitSequenceStuffed[s++] = BitSequence[i]; // add the bit to the final sequence

        if (cnt == 5) // there are five consecutive bits of the same value
        {
            BitSequenceStuffed[s++] = 0x00; // stuff with a zero bit
            cnt = 0; // and reset cnt to zero

        }
      }
      
      _size = 0;
       //Recreate 0b01111110 (FLAG) in byte size
      for (int i=0; i < 64 ; i++)
      { 
         Buffer[_size++] = 0x00;
         for (int j=0; j < 6 ; j++) 
         {
           Buffer[_size++] = 0x01;
         }
         Buffer[_size++] = 0x00;
      }
              
      for (int i=0; i < s ; i++) Buffer[_size++] = BitSequenceStuffed[i];
      
      //Insert 0b01111110 (FLAG)
       Buffer[_size++] = 0x00;
       for (int j=0; j < 6 ; j++) 
       {
         Buffer[_size++] = 0x01;
       }
       Buffer[_size++] = 0x00;
            
      for (int i = 0; i< 255*8 ; i++) byte_temp[i] = 0x00;
      
      //NRZI encoding
      for (int i=0; i < _size ; i++) 
      {
         if (Buffer[i] == 0x00) 
         {
           byte_temp[i+1] = ! byte_temp[i];
         }
         else 
         {
           byte_temp[i+1] = byte_temp[i];
         }
      }

      //extrabits = (_size+1) % 8;
      if (((_size+1) % 8) > 0) remBits = 8 - ((_size+1) % 8);
      
      for (int i = (_size + 1) ; i < (_size + 1 + remBits ) ; i++)
      {
         byte_temp[i] = 0x01;
      }

      //Convert to bit after NRZI and added remaining bits to form byte array
      Index = 0;
      for (int i = 0; i < (_size + 1 + remBits); i = i + 8)
      {
        temp = 0;
        if  (byte_temp[i] == 0x01)   temp = temp + 0b10000000;
        if  (byte_temp[i+1] == 0x01) temp = temp + 0b01000000;
        if  (byte_temp[i+2] == 0x01) temp = temp + 0b00100000;
        if  (byte_temp[i+3] == 0x01) temp = temp + 0b00010000;
        if  (byte_temp[i+4] == 0x01) temp = temp + 0b00001000;
        if  (byte_temp[i+5] == 0x01) temp = temp + 0b00000100;
        if  (byte_temp[i+6] == 0x01) temp = temp + 0b00000010;
        if  (byte_temp[i+7] == 0x01) temp = temp + 0b00000001;
        finalSequence[Index++] = temp;
      }
}

char* AX25::demod(byte *Buffer, uint8_t bytelength) {
  
    byte BitSequence[bytelength*8];
    byte ByteSequence[bytelength];
    byte BitSequence_temp[bytelength*8];
    byte ByteSequence_temp[bytelength];
    char Message[256];
    byte Checksum[3];
    char DestCS[7];
    char SourceCS[7];
    int k = 0; //general counter
    int _size = 0;
    int s = 0;
    uint8_t cnt = 0 ;//Bit stuff counter  
    uint8_t extraBit = 0;
    uint8_t extraByte = 0;
    byte temp = 0;
    boolean pastFlag;
    boolean BitFound;

    //Initialization
    for (int i=0; i < bytelength*8 ; i++) BitSequence[i] = 0x00;  
    for (int i=0; i < bytelength*8 ; i++) BitSequence_temp[i] = 0x00; 
    for (int i=0; i < bytelength ; i++) ByteSequence[i] = 0x00; 
    for (int i=0; i < bytelength ; i++) ByteSequence_temp[i] = 0x00; 
   
    //Convert bits to byte size
    for (int i = 0; i< bytelength ; i++)
    {
      for (register uint8_t t=128; t>0 ; t = t/2) {
        if (Buffer[i] & t) BitSequence[_size++] = 0x01;
        else BitSequence[_size++] = 0x00;
       }       
    }
   
    for (int i=1; i < _size ; i++) 
    {
       if (BitSequence[i] == BitSequence[i-1]) 
       {
         BitSequence_temp[i-1] = 0x01;
       } else BitSequence_temp[i-1] = 0x00;
       
    }

    //Convert bit to Byte
    k = 0;
    for (int i = 0; i < _size-1; i = i + 8)
    {
        temp = 0;
        if  (BitSequence_temp[i] == 0x01)   temp = temp + 0b10000000;
        if  (BitSequence_temp[i+1] == 0x01) temp = temp + 0b01000000;
        if  (BitSequence_temp[i+2] == 0x01) temp = temp + 0b00100000;
        if  (BitSequence_temp[i+3] == 0x01) temp = temp + 0b00010000;
        if  (BitSequence_temp[i+4] == 0x01) temp = temp + 0b00001000;
        if  (BitSequence_temp[i+5] == 0x01) temp = temp + 0b00000100;
        if  (BitSequence_temp[i+6] == 0x01) temp = temp + 0b00000010;
        if  (BitSequence_temp[i+7] == 0x01) temp = temp + 0b00000001;
        ByteSequence[k++] = temp;
    }
//Test
   // radio.printBuffer("NRZI:", ByteSequence, k);

    pastFlag = false;
    cnt = 0;
    //Find and Remove Flags
    for (int i = 0; i < k; i++)
    {
       Serial.println(ByteSequence[i], HEX);


       if (ByteSequence[i] != AX25_FLAG)
       {
          pastFlag = true;
          ByteSequence_temp[cnt++] = ByteSequence[i]; 
       } else if (pastFlag) break;
    }
    
//Test
   // radio.printBuffer("Removed Flags:", ByteSequence_temp, cnt);
    
    //Re-init
    for (int i=0; i < bytelength*8 ; i++) BitSequence[i] = 0x00;
    k = 0;
    
    //Convert bits to byte size
    for (int i = 0; i< cnt ; i++)
    {
      for (register uint8_t t=128; t>0 ; t = t/2) {
        if (ByteSequence_temp[i] & t) BitSequence[k++] = 0x01;
        else BitSequence[k++] = 0x00;
       }       
    }
   
   //Re-init
   for (int i=0; i < bytelength*8 ; i++) BitSequence_temp[i] = 0x00;
   
    // Remove end flag
  for (int i = 0; i < k ; i++)
   { 
      if (BitSequence[i] == 0x01) cnt++;
      else cnt = 0; // restart count at 1

      if (cnt == 6) // there are five consecutive bits of the same value
      {
        k = i - 6;
        break;
      }
    }

   //Bit unstuff : Remove 0 after five consecutive 1s.
   cnt = 0;
   s = 0;
   BitFound = false;
   extraBit = 0;

   for (int i = 0; i < k ; i++)
   {
      if (BitFound) 
      {
        BitFound = false;
        extraBit++;
        continue;
      }
      
      if (BitSequence[i] == 0x01) cnt++;
      else cnt = 0; // restart count at 1

      if (cnt == 5) // there are five consecutive bits of the same value
      {
          BitFound = true;
          cnt = 0; // and reset cnt to zero

          // Serial.println("Zero removed");
      }
      BitSequence_temp[s++] = BitSequence[i]; // add the bit to the final sequence
    }
    
    extraByte = (extraBit / 8);
    if ( ((extraBit) % 8) > 0) extraByte++ ;
    
    //Re-init ByteSequence
    for (int i=0; i < bytelength ; i++) ByteSequence[i] = 0x00; 

    //Convert bit to Byte
    k = 0;
    // for (int i = 0; i < s - extraByte*8; i = i + 8)
    for (int i = 0; i < s ; i = i + 8)
      {
        temp = 0;
        if  (BitSequence_temp[i] == 0x01)   temp = temp + 0b10000000;
        if  (BitSequence_temp[i+1] == 0x01) temp = temp + 0b01000000;
        if  (BitSequence_temp[i+2] == 0x01) temp = temp + 0b00100000;
        if  (BitSequence_temp[i+3] == 0x01) temp = temp + 0b00010000;
        if  (BitSequence_temp[i+4] == 0x01) temp = temp + 0b00001000;
        if  (BitSequence_temp[i+5] == 0x01) temp = temp + 0b00000100;
        if  (BitSequence_temp[i+6] == 0x01) temp = temp + 0b00000010;
        if  (BitSequence_temp[i+7] == 0x01) temp = temp + 0b00000001;
        ByteSequence[k++] = temp;
      }
    
   // radio.printBuffer("received:", ByteSequence, k);
    
    //Check if message has errors
    //Compute FCS on received byte stream
    FCS = 0;
    FCS = crcCcitt(ByteSequence, k-2);
    
    Checksum[1] = ByteSequence[k-2];
    Checksum[2] = ByteSequence[k-1];
    
    // Serial.println("Checksums : ");
    // Serial.println(Checksum[1],HEX);
    // Serial.println(Checksum[2],HEX);
    // Serial.println("FCS in LSB: ");
    // Serial.print(FCS,HEX);
    // Serial.println("Checksums computed: ");
    // Serial.print((FCS >> 8) & 0xff,HEX);
    // Serial.print(FCS & 0xff,HEX);
    
    if (Checksum[1] != ((FCS >> 8) & 0xff))
    {
      Serial.println("Error in Checksum 1 : ");
      Serial.print(Checksum[1]);Serial.print(" != ");Serial.println((FCS >> 8) & 0xff);
    } 
    if (Checksum[2] != (FCS & 0xff))
    {
      Serial.println("Error in Checksum 2: ");
      Serial.print(Checksum[2]);Serial.print(" != ");Serial.println(FCS & 0xff);
    }
    
    //Convert form LSB to MSB
    for (int i=0; i < bytelength ; i++) ByteSequence_temp[i] = 0x00; 
    for (int i=0; i < k-2 ; i++) ByteSequence_temp[i] = MSB_LSB_swap_8bit(ByteSequence[i]);
    
    cnt = 0;
    //Recover header
    for (int i=0; i < 6; i++) DestCS[i] = char(ByteSequence_temp[cnt++]>>1);

    //SSID Destination
    cnt++; 

    //Append Source Callsign
    for (int i=0; i < 6; i++) SourceCS[i] = char(ByteSequence_temp[cnt++]>>1);

    //Append SSID Source
    cnt++;
    //Append Control bits
    cnt++;
    //Append Protocol Identifier
    cnt++;
    //Recover message
    s = k-2-cnt;


    // Serial.println("Final decoded Message");
    // for (int i=0; i < s; i++) 
    // {
    //   Message[i] = char(ByteSequence_temp[cnt++]);
    //   Serial.print(Message[i]);
    // }
    // Serial.println("");
    return Message;
    
}

boolean AX25::logicXOR(boolean a, boolean b) {
  return (a||b) && !(a && b); 
}

// unsigned int AX25::crcCcitt (byte *Buffer, uint8_t bytelength) {
//   uint8_t OutBit = 0;
//   unsigned int XORMask = 0x0000;
//   unsigned int SR = 0xFFFF;
  
//   for (int i=0; i<bytelength ; i++)
//   {
//     for (uint8_t b = 128 ; b > 0 ; b = b/2) {
       
//       OutBit = SR & 1 ? 1 : 0; //Bit shifted out of shift register
    
//       SR = SR>>1; // Shift the register to the right and shift a zero in

//       XORMask = logicXOR((Buffer[i] & b),OutBit) ? MSB_LSB_swap_16bit(CRC_POLYGEN) : 0x0000;

//       SR = SR ^ XORMask;
//     }
//   }
//   return  MSB_LSB_swap_16bit(~SR);  
// }

uint16_t AX25::crcCcitt (byte *Buffer, uint8_t bytelength) {
  uint8_t OutBit = 0;
  uint16_t XORMask = 0x0000;
  uint16_t SR = 0xFFFF;
  
  for (int i=0; i<bytelength ; i++)
  {
    for (uint8_t b = 128 ; b > 0 ; b = b/2) {
       
      OutBit = SR & 1 ? 1 : 0; //Bit shifted out of shift register
    
      SR = SR>>1; // Shift the register to the right and shift a zero in

      XORMask = logicXOR((Buffer[i] & b),OutBit) ? MSB_LSB_swap_16bit(CRC_POLYGEN) : 0x0000;

      SR = SR ^ XORMask;
    }
  }
  return  MSB_LSB_swap_16bit(~SR);  
}

byte AX25::MSB_LSB_swap_8bit(byte v) {
  // swap odd and even bits
  v = ((v >> 1) & 0x55) | ((v & 0x55) << 1);
  // swap consecutive pairs
  v = ((v >> 2) & 0x33) | ((v & 0x33) << 2);
  // swap nibbles ... 
  v = ((v >> 4) & 0x0F) | ((v & 0x0F) << 4);
  return v;
}

// unsigned int AX25::MSB_LSB_swap_16bit(unsigned int v) {
//   // swap odd and even bits
//   v = ((v >> 1) & 0x5555) | ((v & 0x5555) << 1);
//   // swap consecutive pairs
//   v = ((v >> 2) & 0x3333) | ((v & 0x3333) << 2);
//   // swap nibbles ... 
//   v = ((v >> 4) & 0x0F0F) | ((v & 0x0F0F) << 4);
//   // swap bytes
//   v = ((v >> 8) & 0x00FF) | ((v & 0x00FF) << 8);  
//   return v;
// }

uint16_t AX25::MSB_LSB_swap_16bit(uint16_t v) {
  // swap odd and even bits
  v = ((v >> 1) & 0x5555) | ((v & 0x5555) << 1);
  // swap consecutive pairs
  v = ((v >> 2) & 0x3333) | ((v & 0x3333) << 2);
  // swap nibbles ... 
  v = ((v >> 4) & 0x0F0F) | ((v & 0x0F0F) << 4);
  // swap bytes
  v = ((v >> 8) & 0x00FF) | ((v & 0x00FF) << 8);  
  return v;
}

void AX25::arrayInit() {
  for (int i=0; i< MAX_LENGTH * 8 ;i++) bitSequence[i] = 0;
  for (int i=0; i< MAX_LENGTH_FINAL ;i++) finalSequence[i] = 0;
  for (int j=0; j< 256 ;j++) strcpy(message," ");
}

void AX25::setCallsignAndSsid() {
  setFromCallsign("KD2BHC"); //TODO replace
  setToCallsign("CQ    ");
  setSSIDdest(AX25_SSID_DESTINATION);
  setSSIDsource(AX25_SSID_SOURCE);
}




