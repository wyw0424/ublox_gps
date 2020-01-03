#ifndef WINCOMM_UART_HEADER
#define WINCOMM_UART_HEADER 1

#include <map>
#include <vector>
#include <string>

namespace serial { class Serial; }

namespace wincomm
{

class Uart
{
public:
    Uart() : _serialImpl(NULL) {}
    virtual ~Uart();
    
    typedef std::map<std::string, std::string> PortAttributeMap;
    static int queryPorts(std::vector<std::string>& ports, std::vector<PortAttributeMap>& portAttr);

    bool open(const std::string& comName, int baudrate);
    void close();
    
    int writeFromBuffer(const char* buffer, int reqSize);
    int readToBuffer(char* buffer, int reqSize);
    
    const std::string& portName() const { return _comString; }
    bool valid() const { return _serialImpl != NULL; }
    
    enum FlowControl { FLOWCONTROL_NONE = 0, FLOWCONTROL_SOFTWARE, FLOWCONTROL_HARDWARE };
    void configureDCB(int baudrate, int numBits, int parity, int stopBits, FlowControl flowCtrl);
    void configureTimeouts(int interval, int readTimeout, int writeTimeout);
    
protected:
    serial::Serial* _serialImpl;
    std::string _comString;
};

}

#endif
