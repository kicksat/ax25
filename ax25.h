#ifndef AX25_h
#define AX25_h

#include "RadioHead/RH_RF22.h"

//Magic Constants
#define AX25_FLAG				0x7E
#define AX25_SSID_SOURCE		0x61
#define AX25_SSID_DESTINATION	0x60
#define AX25_CONTROL			0x03
#define AX25_PROTOCOL			0xF0

//Maximum payload size of an AX.25 packet in bytes (determined by RF22 library)
#define AX25_MAX_PAYLOAD_SIZE 160

class AX25 {
public:

	void setFromCallsign(char* callsign);

	void setToCallsign(char* callsign);

	void transmit(char* payload, unsigned int len);

private:

	RH_RF22::ModemConfig m_modemConfig;

	char m_buffer1[];

	char m_buffer2[];

	//Flips the order of bytes from MSB first to LSB first
	void lsbFirst(char* out, char* in, unsigned int len);

	//Calculate the 2-byte CRC on the data
	void crc_ccitt(char* crc, char* data, unsigned int len);

	//Perform bit stuffing on the input array (add an extra 0 after 5 sequential 1's)
	unsigned int bitStuff(char* out, char* in, unsigned int inLen);

	void nrzi(char* out, char* in, unsigned int len);

};

#endif //AX25_h