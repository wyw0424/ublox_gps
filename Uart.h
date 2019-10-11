#ifndef WINCOMM_UART_HEADER
#define WINCOMM_UART_HEADER 1

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <vector>
#include <string>

namespace wincomm
{

class Uart
{
public:
    Uart()  {}
    virtual ~Uart();
    
    bool open(const std::string& comName, int baudrate);
    bool refresh();
    void close();
    
    int writeFromBuffer(const char* buffer, int reqSize);
    int readToBuffer(char* buffer, int reqSize);
    
    const std::string& portName() const { return _comString; }
    bool valid() { return _port.IsOpen(); }
    
    enum FlowControl { FLOWCONTROL_IGNORE = 0, FLOWCONTROL_XONOFF, FLOWCONTROL_HARDWARE, FLOWCONTROL_NONE };
    void configureDCB(int baudrate, int numBits, int parity, int stopBits, FlowControl flowCtrl);
    void configureTimeouts(int interval, int readTimeout, int writeTimeout);
    
protected:
    LibSerial::SerialPort _port;
    std::string _comString;
};

}

#endif
