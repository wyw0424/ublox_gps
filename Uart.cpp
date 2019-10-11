#include "Uart.h"
#include <stdio.h>
#include <memory.h>

using namespace wincomm;

Uart::~Uart()
{
    close();
}

bool Uart::open(const std::string& comName, int baudrate)
{
    try
    {
        _port.Open(comName.c_str()) ;
    }
    catch (const LibSerial::OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl ;
        return false ;
    }

    configureDCB(baudrate, -1, -1, -1, FLOWCONTROL_IGNORE);
    configureTimeouts(50, 50, 50);
    
    _comString = comName;
    return refresh();
}

void Uart::close()
{
    _port.Close();
}

bool Uart::refresh()
{
    return true;
}

int Uart::writeFromBuffer(const char* buffer, int reqSize)
{
    std::string data(buffer, reqSize);
    _port.Write(data);
    return reqSize;
}

int Uart::readToBuffer(char* buffer, int reqSize)
{
    std::string data;
    _port.Read(data, reqSize, 0);
    memcpy(buffer, data.data(), data.size());
    return data.size();
}

void Uart::configureDCB(int baudrate, int numBits, int parity, int stopBits, FlowControl flowCtrl)
{
    _port.SetBaudRate(LibSerial::BaudRate::BAUD_115200) ;
    _port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8) ;
    _port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE) ;
    _port.SetParity(LibSerial::Parity::PARITY_NONE) ;
    _port.SetStopBits(LibSerial::StopBits::STOP_BITS_1) ;
}

void Uart::configureTimeouts(int interval, int readTimeout, int writeTimeout)
{
}
