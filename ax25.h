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
	AX25(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2);

	void transmit(char* payload); // void transmit(char* payload, unsigned int len)

	void setFrequency(float freq);

	void setPower(byte pwr);

private:

	RH_RF22::ModemConfig FSK1k2;

	RH_RF22 radio;

	RHHardwareSPI spi;

	char SrcCallsign[7];
	char DestCallsign[7];

	byte ssid_source;
	byte ssid_destination;

	byte bitSequence[280*8];
	byte finalSequence[450];
	byte RcvSequence[450];
	uint8_t len = sizeof(RcvSequence);
	char message[256];

	int Index = 0;
	unsigned int FCS = 0;

	void radioSetup();
	inline void setSSIDsource(byte ssid_src);
	inline void setSSIDdest(byte ssid_dest);
	inline void setFromCallsign(char *fromcallsign);
	inline void setToCallsign(char *tocallsign);

	void addHeader( byte *Buffer);

	void bitProcessing(byte *Buffer, uint8_t bytelength);

	void demod(byte *Buffer, uint8_t bytelength);

	//Flips the order of bytes from MSB first to LSB first
	// void lsbFirst(char* out, char* in, unsigned int len);

	boolean logicXOR(boolean a, boolean b);

	//Calculate the 2-byte CRC on the data
	// void crcCcitt(char* crc, char* data, unsigned int len);
	unsigned int crcCcitt (byte *Buffer,uint8_t bytelength);

	byte MSB_LSB_swap_8bit(byte v);

	unsigned int MSB_LSB_swap_16bit(unsigned int v);

	void arrayInit();

	void setCallsignSsid();

	void formatPacket();

	void sendPacket();

	//Perform bit stuffing on the input array (add an extra 0 after 5 sequential 1's)
	// unsigned int bitStuff(char* out, char* in, unsigned int inLen);

	// void nrzi(char* out, char* in, unsigned int len);

};

#endif //AX25_h