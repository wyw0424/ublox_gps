#ifndef WINCOMM_UDPSOCKET_HEADER
#define WINCOMM_UDPSOCKET_HEADER 1

#include <vector>
#include <string>

namespace wincomm
{

class Socket
{
public:
    enum Type { TCP, UDP };
    
    Socket()
    :   _socket(NULL), _lastActiveClientSocket(NULL), _remoteHost(0), _remotePort(0),
        _type(TCP), _serverSide(false) {}
    
    virtual ~Socket() { disconnect(); }
    
    /** Bind to a local port */
    bool bind(int port, Type t);
    
    /** Connect to a remote address/port */
    bool connect(const std::string& address, int port, Type t);
    
    bool disconnect();
    bool isConnected() const;
    
    /** Check for new connected clients in TCP bind mode */
    bool checkNewConnection(int retryTimes = 0);
    unsigned int getNumConnected() const { return _connectedSockets.size(); }
    
    /** A static method to check all current sockets to see if they are readable
        Only need to be called once every frame to check all sockets
    */
    static bool checkReadingStates(unsigned int ms = 0);
    
    /** Read from the socket
        If doCheck is set to true, it will call checkReadingStates() automatically.
        It is redundant if more than one sockets read and check the same states
    */
    int read(void* dest, int maxToRead, bool doCheck = true);
    
    /** Write to the socket, UDP bind mode should first receive before writing */
    int write(const void* source, int numToWrite);
    
    /** Write to active client found in last read(), for TCP server only */
    int writeToLastActiveClient(const void* source, int numToWrite);
    
    /** Write to the socket at specified IP, for UDP mode only */
    int write(const std::string& address, int port, const void* source, int numToWrite);
    
protected:
    void* _socket, *_lastActiveClientSocket;
    std::vector<void*> _connectedSockets;
    int _remoteHost, _remotePort;
    Type _type;
    bool _serverSide;
};

}

#endif
