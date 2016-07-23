#ifndef AX25_h
#define AX25_h

#include <SPI.h>
#include "RH_RF22.h"
#include <RHGenericDriver.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Arduino.h"

//Magic Constants
#define AX25_FLAG				0x7E
#define AX25_SSID_SOURCE		0x61
#define AX25_SSID_DESTINATION	0x60
#define AX25_CONTROL			0x03
#define AX25_PROTOCOL			0xF0

//Maximum payload size of an AX.25 packet in bytes (determined by RF22 library)
// DIFFERENT
#define AX25_MAX_PAYLOAD_SIZE   160
#define CRC_POLYGEN     	    0x1021

#define MAX_LENGTH 				280
#define MAX_LENGTH_FINAL 		450

class AX25 {
public:
	//Default Constructor
	// AX25(char* fromCallsign, char* toCallsign);
	AX25(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, uint8_t _shutdownPin = 9);

	RH_RF22 radio;

	bool powerAndInit();

	void transmit(char* payload, uint16_t size); // void transmit(char* payload, unsigned int len)

	bool available();

	void setRxMode();

	void setTxMode();

	bool receive(uint8_t* buf, uint8_t* len);

	char* demod(byte *Buffer, uint8_t bytelength);

	uint8_t shutdownPin;

	

	// TODO: figure out
	// inline void setFrequency(float freq);

	// inline void setPower(byte pwr);

private:

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

	

	RHHardwareSPI spi;

	char SrcCallsign[7];
	char DestCallsign[7];

	byte ssid_source;
	byte ssid_destination;

	byte bitSequence[MAX_LENGTH*8];
	byte finalSequence[MAX_LENGTH_FINAL];
	char message[256];

	int Index = 0;
	unsigned int FCS = 0;

	void radioSetup();

	void setSSIDsource(byte ssid_src);
	void setSSIDdest(byte ssid_dest);
	void setFromCallsign(char *fromcallsign);
	void setToCallsign(char *tocallsign);

	void addHeader( byte *Buffer);

	void bitProcessing(byte *Buffer, uint8_t bytelength);

	

	boolean logicXOR(boolean a, boolean b);

	//Calculate the 2-byte CRC on the data
	// void crcCcitt(char* crc, char* data, unsigned int len);
	uint16_t crcCcitt (byte *Buffer,uint8_t bytelength);

	//Flips the order of bytes from MSB first to LSB first
	byte MSB_LSB_swap_8bit(byte v);

	uint16_t MSB_LSB_swap_16bit(uint16_t v);

	// Initializes arrays with zeros
	void arrayInit();

	void setCallsignAndSsid();

	//Perform bit stuffing on the input array (add an extra 0 after 5 sequential 1's)
	// unsigned int bitStuff(char* out, char* in, unsigned int inLen);
	void formatPacket(uint16_t size);

	void sendPacket();	


};

#endif //AX25_h