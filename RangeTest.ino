#include <SPI.h>
#include <RH_RF22.h>
#include <RHGenericDriver.h>>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
///                  Teensy      RFM-22B
///                  GND----------GND-\ (ground in)
///                               SDN-/ (shutdown in)
///                  3V3----------VCC   (3.3V in)
///  interrupt 2 pin D2-----------NIRQ  (interrupt request out)
///           SS pin D10----------NSEL  (chip select in)
///          SCK pin D13----------SCK   (SPI clock in)
///         MOSI pin D11----------SDI   (SPI Data in)
///         MISO pin D12----------SDO   (SPI data out)
///                            /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                            \--TX_ANT (TX antenna control in) RFM22B only
///                            /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                            \--RX_ANT (RX antenna control in) RFM22B only
/// For an Arduino Mega:
/// \code
///                 Mega         RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 0 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D53----------NSEL  (chip select in)
///         SCK pin D52----------SCK   (SPI clock in)
///        MOSI pin D51----------SDI   (SPI Data in)
///        MISO pin D50----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
///
/// For connecting an Arduino to an RFM23BP module. Note that the antenna control pins are reversed 
/// compared to the RF22.
/// \code
///                 Arduino      RFM-23BP
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 5V-----------VCC   (5V in)
/// interrupt 0 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control receiver antenna RXON)
///                           \--RXON   (RX antenna control in)
///                           /--GPIO1 (GPIO1 out to control transmitter antenna TXON)
///                           \--TXON   (TX antenna control in)

#define FLAG 0x7E
//Protocol Identifier. 0xF0 means : No Layer 3 protocol implemented
#define PID 0xF0
//Control Field Type for Unnumbered Information Frames : 0x03
#define CONTROL 0x03
#define MAX_LENGTH 280
#define MAX_LENGTH_FINAL 450
//CRC-CCITT
#define CRC_POLYGEN     0x1021

char SrcCallsign[7] = "000000";
char DestCallsign[7] = "000000";

byte ssid_source = 0;
byte ssid_destination = 0;

byte bitSequence[280*8];
byte finalSequence[450];
byte RcvSequence[450];
uint8_t len = sizeof(RcvSequence);
char message[256];

int Index = 0;
int numReceived = 0;

// CRC-CCITT
unsigned int FCS = 0;

#define RADIO_SS       10
#define RADIO_INT       2
#define RADIO_SDN       9
#define TEENSY_LED     13

RHHardwareSPI spi;
RH_RF22 radio = RH_RF22(RADIO_SS, RADIO_INT, spi);

RH_RF22::ModemConfig FSK1k2 = {
  0x2B, //reg_1c
  0x03, //reg_1f
  0x41, //reg_20
  0x60, //reg_21
  0x27, //reg_22
  0x52, //reg_23
  0x00, //reg_24
  0x9F, //reg_25
  0x2C, //reg_2c - Only matters for OOK mode
  0x11, //reg_2d - Only matters for OOK mode
  0x2A, //reg_2e - Only matters for OOK mode
  0x80, //reg_58
  0x60, //reg_69
  0x09, //reg_6e
  0xD5, //reg_6f
  0x24, //reg_70
  0x22, //reg_71
  0x01  //reg_72
};

inline void setSSIDsource(byte ssid_src) { ssid_source = ssid_src;}
inline void setSSIDdest(byte ssid_dest) { ssid_destination = ssid_dest;}
inline void setFromCallsign(char *fromcallsign){strcpy(SrcCallsign,fromcallsign);}
inline void setToCallsign(char *tocallsign){strcpy(DestCallsign,tocallsign);}

void AddHeader(byte *Buffer);
void BitProcessing(byte *Buffer,uint8_t bytelength);
void Demod(byte *Buffer,uint8_t bytelength);

//CRC_CCITT
unsigned int CRC_CCITT (byte *Buffer,uint8_t bytelength);
boolean logicXOR(boolean a, boolean b);
unsigned int MSB_LSB_swap_16bit(unsigned int v);
byte MSB_LSB_swap_8bit(byte v);

void setup() 
{ 
  Serial.begin(9600);
  pinMode(TEENSY_LED, OUTPUT);
  pinMode(RADIO_SDN, OUTPUT);

  //Need a delay before turning on radio
  //so that power supply can stabilize
  digitalWrite(RADIO_SDN, HIGH);
  delay(500);
  digitalWrite(RADIO_SDN, LOW);
  delay(500);
  
  //Only init once...since Interrupt flag will increment
  if (!radio.init()) Serial.println("init failed");
  delay(1000);
}

void loop()
{  
  Index = 0;
  
  //Array Initialization
  for (int i=0; i< MAX_LENGTH * 8 ;i++) bitSequence[i] = 0;
  for (int i=0; i< MAX_LENGTH_FINAL ;i++) finalSequence[i] = 0;
  for (int i=0; i< MAX_LENGTH_FINAL ;i++) RcvSequence[i] = 0;
  for (int j=0; j< 256 ;j++) strcpy(message," ");
  
  //Set CallSign and SSID
  setFromCallsign("KD2BHC");
  setToCallsign("CQ    ");
  setSSIDdest(0x60);
  setSSIDsource(0x61);

  //MESSAGE to transmit
  //strcpy(message,"Test 0");  
  sprintf(message, "%d Received", numReceived);
  
  //----------------------- Start Ax25 Packet format ----------------------//
  
  //Add Header
  AddHeader(bitSequence);
      
  //Add Message
  for (int i=0; i < strlen(message) ; i++) bitSequence[Index++] = message[i];
     
  //Convert bit sequence from MSB to LSB
  for (int i=0; i < Index ; i++) bitSequence[i] = MSB_LSB_swap_8bit(bitSequence[i]);
    
  //Compute Frame check sequence : CRC
  FCS = CRC_CCITT(bitSequence, Index);
  
  //Add FCS in MSB form
  //Add MS byte
  bitSequence[Index++] = (FCS >> 8) & 0xff;
  //Add LS byte
  bitSequence[Index++] = FCS & 0xff;
    
  //radio.printBuffer("Init Message:", bitSequence, Index);
    
  //Bit Processing...Bit stuff, add FLAG and do NRZI enconding...
  BitProcessing(bitSequence,Index);
  
   //----------------------- End Ax25 Packet format ----------------------//
  radio.setModeIdle();
  radio.setFrequency(437.505);
  radio.setModemRegisters(&FSK1k2);
  radio.setTxPower(RH_RF22_RF23BP_TXPOW_29DBM); 
  radio.send(finalSequence, Index); 
  radio.waitPacketSent();

  radio.sleep();
  //delay(5000);
  
  if (radio.waitAvailableTimeout(5000))
  { 
    // Should be a reply message for us now   
    if (radio.recv(RcvSequence, &len))
    {
      ++numReceived;
      Serial.println("got reply: ");
      for (int i=0 ; i < len; i++) Serial.print(RcvSequence[i]);
      Demod(RcvSequence,450);
    }
    else Serial.println("recv failed");
  }
  else Serial.println("No reply...");
  
  delay(2000);
  
}

void AddHeader(byte *Buffer)
{

    //Shift bits 1 place to the left in order to allow for HDLC extension bit
    for (int i=0; i < strlen(DestCallsign) ; i++) Buffer[Index++] = DestCallsign[i]<<1;

    // Append SSID Destination
    Buffer[Index++] = ssid_destination;  

    //Append Source Callsign
    for (int i=0; i < strlen(SrcCallsign) ; i++) Buffer[Index++] = SrcCallsign[i]<<1;
    
    //Append SSID Source
    Buffer[Index++] = ssid_source;
   
    //Append Control bits
    Buffer[Index++] = CONTROL;
    
    //Append Protocol Identifier
    Buffer[Index++] = PID;
}

void BitProcessing(byte *Buffer, uint8_t bytelength)
{
  
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

void Demod(byte *Buffer, uint8_t bytelength)
{
  
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
//    radio.printBuffer("NRZI:", ByteSequence, k);

    pastFlag = false;
    cnt = 0;
    //Find and Remove Flags
    for (int i = 0; i < k; i++)
    {
       if (ByteSequence[i] != FLAG)
       {
          pastFlag = true;
          ByteSequence_temp[cnt++] = ByteSequence[i]; 
       } else if (pastFlag) break;
    }
    
//Test
//    radio.printBuffer("Removed Flags:", ByteSequence_temp, cnt);
    
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
      }
      BitSequence_temp[s++] = BitSequence[i]; // add the bit to the final sequence
    }
    
    extraByte = (extraBit / 8);
    if ( ((extraBit) % 8) > 0) extraByte++ ;
    
    //Re-init ByteSequence
    for (int i=0; i < bytelength ; i++) ByteSequence[i] = 0x00; 
    //Convert bit to Byte
    k = 0;
    for (int i = 0; i < s - extraByte*8; i = i + 8)
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
    
 //   Serial.println("Received Stream"); 
 //   radio.printBuffer("received:", ByteSequence, k);
    //for (int i=0 ; i < k; i++) Serial.print(ByteSequence[i],HEX);
    Serial.println(""); 
    
    //Check if message has errors
    //Compute FCS on received byte stream
    FCS = 0;
    FCS = CRC_CCITT(ByteSequence, k-2);
    
    Checksum[1] = ByteSequence[k-2];
    Checksum[2] = ByteSequence[k-1];
    
    //Serial.println("Checksums : ");
    //Serial.println(Checksum[1],HEX);
    //Serial.println(Checksum[2],HEX);
    //Serial.println("FCS in LSB: ");
    //Serial.print(FCS,HEX);
    //Serial.println("Checksums computed: ");
    //Serial.print((FCS >> 8) & 0xff,HEX);
    //Serial.print(FCS & 0xff,HEX);
    
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
    Serial.println("Final decoded Message");
    for (int i=0; i < s; i++) 
    {
      Message[i] = char(ByteSequence_temp[cnt++]);
      Serial.print(Message[i]);
    }
    Serial.println("");
}


boolean logicXOR(boolean a, boolean b)
{
  return (a||b) && !(a && b); 
}

unsigned int CRC_CCITT (byte *Buffer, uint8_t bytelength)
{
  uint8_t OutBit = 0;
  unsigned int XORMask = 0x0000;
  unsigned int SR = 0xFFFF;
  
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

byte MSB_LSB_swap_8bit(byte v)
{
  // swap odd and even bits
  v = ((v >> 1) & 0x55) | ((v & 0x55) << 1);
  // swap consecutive pairs
  v = ((v >> 2) & 0x33) | ((v & 0x33) << 2);
  // swap nibbles ... 
  v = ((v >> 4) & 0x0F) | ((v & 0x0F) << 4);
  return v;
}

unsigned int MSB_LSB_swap_16bit(unsigned int v)
{
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



