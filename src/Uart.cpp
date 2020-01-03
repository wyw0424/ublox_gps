#include "serial/serial.h"
#include "Uart.h"

using namespace wincomm;

Uart::~Uart()
{
    close();
}

int Uart::queryPorts(std::vector<std::string>& ports, std::vector<PortAttributeMap>& portAttr)
{
    std::vector<serial::PortInfo> devices = serial::list_ports();
    for (unsigned int i = 0; i < devices.size(); ++i)
    {
        PortAttributeMap attr;
        attr["DEVICEDESC"] = devices[0].description;
        attr["HARDWAREID"] = devices[0].hardware_id;
        attr["LOCATION"] = devices[0].location;

        ports.push_back(devices[0].port);
        portAttr.push_back(attr);
    }
    return devices.size();
}

bool Uart::open(const std::string& comName, int baudrate)
{
    try
    {
        if (_serialImpl != NULL) close();
#if _WIN32
        _serialImpl = new serial::Serial(
            ("\\\\.\\" + comName), baudrate, serial::Timeout::simpleTimeout(100));
#else
        _serialImpl = new serial::Serial(
            comName, baudrate, serial::Timeout::simpleTimeout(100));
#endif
    }
    catch (std::exception &e)
    {
        printf("Failed to open port %s: %s\n", _comString.c_str(), e.what());
        return false;
    }

    _comString = comName;
    return true;
}

void Uart::close()
{
    delete _serialImpl;
    _serialImpl = NULL; _comString = "";
}

int Uart::writeFromBuffer(const char* buffer, int reqSize)
{
    if (_serialImpl != NULL)
    {
        try
        {
            return _serialImpl->write((const uint8_t*)buffer, reqSize);
        }
        catch (std::exception &e)
        { printf("Failed to set timeouts of port %s: %s\n", _comString.c_str(), e.what()); }
    }
    return 0;
}

int Uart::readToBuffer(char* buffer, int reqSize)
{
    if (_serialImpl != NULL)
    {
        try
        {
            return _serialImpl->read((uint8_t*)buffer, reqSize);
        }
        catch (std::exception &e)
        { printf("Failed to set timeouts of port %s: %s\n", _comString.c_str(), e.what()); }
    }
    return 0;
}

void Uart::configureDCB(int baudrate, int numBits, int parity, int stopBits, FlowControl flowCtrl)
{
    if (_serialImpl != NULL)
    {
        try
        {
            _serialImpl->setBaudrate(baudrate);
            _serialImpl->setBytesize((serial::bytesize_t)numBits);   // 5 - 8
            _serialImpl->setParity((serial::parity_t)parity);        // none = 0, odd, even, mark, space = 4
            _serialImpl->setStopbits((serial::stopbits_t)stopBits);  // 1 / 2
            _serialImpl->setFlowcontrol((serial::flowcontrol_t)flowCtrl);
        }
        catch (std::exception &e)
        { printf("Failed to set timeouts of port %s: %s\n", _comString.c_str(), e.what()); }
    }
}

void Uart::configureTimeouts(int interval, int readTimeout, int writeTimeout)
{
    if (_serialImpl != NULL)
    {
        try
        { _serialImpl->setTimeout(interval, readTimeout, 1, writeTimeout, 1); }
        catch (std::exception &e)
        { printf("Failed to set timeouts of port %s: %s\n", _comString.c_str(), e.what()); }
    }
}
