// RH_TCP.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_TCP.cpp,v 1.2 2014/05/15 10:55:57 mikem Exp mikem $

#include <RadioHead.h>

// This can only build on Linux and compatible systems
#if (RH_PLATFORM == RH_PLATFORM_SIMULATOR) 

#include <RH_TCP.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>

RH_TCP::RH_TCP(const char* server)
    : _server(server),
      _rxBufLen(0),
      _rxBufValid(false),
      _socket(-1)
{
}

bool RH_TCP::init()
{   
    _socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket < 0)
    {
	fprintf(stderr,"RH_TCP::init failed to create socket: %s\n", strerror(errno));
	return false;
    }

    // Connect to the etherServer
    const char* server = "127.0.0.1"; // FIXME
    short port = 4000; // Default
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(server);
    servaddr.sin_port = htons(port);
    
    if (connect(_socket, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0)
    {
	// Failed to connect
	fprintf(stderr,"RH_TCP::init failed to connect to %s:%d. %s\n", server, port, strerror(errno));
	close(_socket);
	_socket = -1;
	return false;
    }

    // Now make the socket non-blocking
    int on = 1;
    int rc = ioctl(_socket, FIONBIO, (char *)&on);
    if (rc < 0)
    {
	fprintf(stderr,"RH_TCP::init failed to set socket non-blocking: %s\n", strerror(errno));
	close(_socket);
	_socket = -1;
	return false;
    }

    return sendThisAddress(_thisAddress);
}

void RH_TCP::clearRxBuf()
{
    _rxBufValid = false;
    _rxBufLen = 0;
}

void RH_TCP::checkForEvents()
{
    #define RH_TCP_SOCKETBUF_LEN 500
    static uint8_t socketBuf[RH_TCP_SOCKETBUF_LEN]; // Room for several messages
    static uint16_t socketBufLen = 0;

    // Read at most the amount of space we have left in the buffer
    ssize_t count = read(_socket, socketBuf + socketBufLen, sizeof(socketBuf) - socketBufLen);
    if (count < 0)
    {
	if (errno != EAGAIN)
	{
	    fprintf(stderr,"RH_TCP::checkForEvents read error: %s\n", strerror(errno));
	    exit(1);
	}
    }
    else if (count == 0)
    {
	// End of file
	fprintf(stderr,"RH_TCP::checkForEvents unexpected end of file on read\n");
	exit(1);
    }
    else
    {
	socketBufLen += count;
	while (socketBufLen >= 5)
	{
	    RHTcpTypeMessage* message = ((RHTcpTypeMessage*)socketBuf);
	    uint32_t len = ntohl(message->length);
	    uint32_t messageLen = len + sizeof(message->length);
	    if (len > sizeof(socketBuf) - sizeof(message->length))
	    {
		// Bogus length
		fprintf(stderr, "RH_TCP::checkForEvents read ridiculous length: %d. Corrupt message stream? Aborting\n", len);
		exit(1);
	    }
	    if (socketBufLen >= len + sizeof(message->length))
	    {
		// Got at least all of this message
		if (message->type == RH_TCP_MESSAGE_TYPE_PACKET && len >= 5)
		{
		    // REVISIT: need to check if we are actually receiving?
		    // Its a new packet, extract the headers and payload
		    RHTcpPacket* packet = ((RHTcpPacket*)socketBuf);
		    _rxHeaderTo    = packet->to;
		    _rxHeaderFrom  = packet->from;
		    _rxHeaderId    = packet->id;
		    _rxHeaderFlags = packet->flags;
		    uint32_t payloadLen = len - 5;
		    if (payloadLen <= sizeof(_rxBuf))
		    {
			// Enough room in our receiver buffer
			memcpy(_rxBuf, packet->payload, payloadLen);
			_rxBufLen = payloadLen;
			_rxBufFull = true;
		    }
		}
		// check for other message types here
		// Now remove the used message by copying the trailing bytes (maybe start of a new message?)
		// to the top of the buffer
		memcpy(socketBuf, socketBuf + messageLen, sizeof(socketBuf) - messageLen);
		socketBufLen -= messageLen;
	    }
	}
    }
}

void RH_TCP::validateRxBuf()
{
    // The headers have already been extracted
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

bool RH_TCP::available()
{
    if (_socket < 0)
	return false;
    checkForEvents();
    if (_rxBufFull)
    {
	validateRxBuf();
	_rxBufFull= false;
    }
    return _rxBufValid;
}

bool RH_TCP::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	if (*len > _rxBufLen)
	    *len = _rxBufLen;
	memcpy(buf, _rxBuf, *len);
    }
    clearRxBuf();
    return true;
}

bool RH_TCP::send(const uint8_t* data, uint8_t len)
{
    bool ret = sendPacket(data, len);
    delay(10); // Wait for transmit to succeed. REVISIT: depends on length and speed
    return ret;
}

uint8_t RH_TCP::maxMessageLength()
{
    return RH_TCP_MAX_MESSAGE_LEN;
}

void RH_TCP::setThisAddress(uint8_t address)
{
    RHGenericDriver::setThisAddress(address);
    sendThisAddress(_thisAddress);
}

bool RH_TCP::sendThisAddress(uint8_t thisAddress)
{
    if (_socket < 0)
	return false;
    RHTcpThisAddress m;
    m.length = htonl(2);
    m.type = RH_TCP_MESSAGE_TYPE_THISADDRESS;
    m.thisAddress = thisAddress;
    ssize_t sent = write(_socket, &m, sizeof(m));
    return sent > 0;
}

bool RH_TCP::sendPacket(const uint8_t* data, uint8_t len)
{
    if (_socket < 0)
	return false;
    RHTcpPacket m;
    m.length = htonl(len + 4);
    m.type  = RH_TCP_MESSAGE_TYPE_PACKET;
    m.to    = _txHeaderTo;
    m.from  = _txHeaderFrom;
    m.id    = _txHeaderId;
    m.flags = _txHeaderFlags;
    memcpy(m.payload, data, len);
    ssize_t sent = write(_socket, &m, len + 8);
    return sent > 0;
}

#endif
