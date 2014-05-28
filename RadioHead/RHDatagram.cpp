// RHDatagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.cpp,v 1.5 2014/04/28 23:07:14 mikem Exp mikem $

#include <RHDatagram.h>

RHDatagram::RHDatagram(RHGenericDriver& driver, uint8_t thisAddress) 
    :
    _driver(driver),
    _thisAddress(thisAddress)
{
}

////////////////////////////////////////////////////////////////////
// Public methods
bool RHDatagram::init()
{
    bool ret = _driver.init();
    if (ret)
	setThisAddress(_thisAddress);
    return ret;
}

void RHDatagram::setThisAddress(uint8_t thisAddress)
{
    _driver.setThisAddress(thisAddress);
    // Use this address in the transmitted FROM header
    setHeaderFrom(thisAddress);
    _thisAddress = thisAddress;
}

bool RHDatagram::sendto(uint8_t* buf, uint8_t len, uint8_t address)
{
    setHeaderTo(address);
    return _driver.send(buf, len);
}

bool RHDatagram::recvfrom(uint8_t* buf, uint8_t* len, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{
    if (_driver.recv(buf, len))
    {
	if (from)  *from =  headerFrom();
	if (to)    *to =    headerTo();
	if (id)    *id =    headerId();
	if (flags) *flags = headerFlags();
	return true;
    }
    return false;
}

bool RHDatagram::available()
{
    return _driver.available();
}

void RHDatagram::waitAvailable()
{
    _driver.waitAvailable();
}

bool RHDatagram::waitPacketSent()
{
    return _driver.waitPacketSent();
}

bool RHDatagram::waitPacketSent(uint16_t timeout)
{
    return _driver.waitPacketSent(timeout);
}

bool RHDatagram::waitAvailableTimeout(uint16_t timeout)
{
    return _driver.waitAvailableTimeout(timeout);
}

uint8_t RHDatagram::thisAddress()
{
    return _thisAddress;
}

void RHDatagram::setHeaderTo(uint8_t to)
{
    _driver.setHeaderTo(to);
}

void RHDatagram::setHeaderFrom(uint8_t from)
{
    _driver.setHeaderFrom(from);
}

void RHDatagram::setHeaderId(uint8_t id)
{
    _driver.setHeaderId(id);
}

void RHDatagram::setHeaderFlags(uint8_t flags)
{
    _driver.setHeaderFlags(flags);
}

uint8_t RHDatagram::headerTo()
{
    return _driver.headerTo();
}

uint8_t RHDatagram::headerFrom()
{
    return _driver.headerFrom();
}

uint8_t RHDatagram::headerId()
{
    return _driver.headerId();
}

uint8_t RHDatagram::headerFlags()
{
    return _driver.headerFlags();
}



