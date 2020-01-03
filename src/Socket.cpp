/* The MIT License:

Copyright (c) 2008 Ivan Gagis

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE. */

// (c) Ivan Gagis, 2008
// e-mail: igagis@gmail.com

// Description:
//          cross platfrom C++ Sockets wrapper
//

//system specific defines, typedefs and includes
#ifdef _WIN32
#pragma warning(disable : 4290)
#pragma warning(disable : 4996)
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <windows.h>

typedef SOCKET T_Socket;
#define M_INVALID_SOCKET INVALID_SOCKET
#define M_SOCKET_ERROR SOCKET_ERROR
#define M_EINTR WSAEINTR
#define M_FD_SETSIZE FD_SETSIZE

#else //assume linux/unix

#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
typedef int T_Socket;
#define M_INVALID_SOCKET (-1)
#define M_SOCKET_ERROR (-1)
#define M_EINTR EINTR
#define M_FD_SETSIZE FD_SETSIZE

#endif

#include <exception>
#include <iostream>
#include <stdio.h>
#define M_DECLSPEC

/**
@brief the main namespace of sckt library.
All the declarations of sckt library are made inside this namespace.
*/
namespace sckt{

//=================
//= Static Assert =
//=================
#ifndef DOC_DONT_EXTRACT //direction to doxygen not to extract this class
template <bool b> struct StaticAssert{
    virtual void STATIC_ASSERTION_FAILED()=0;
    virtual ~StaticAssert(){};
};
template <> struct StaticAssert<true>{};
#define M_SCKT_STATIC_ASSERT_II(x, l) struct StaticAssertInst_##l{ \
    sckt::StaticAssert<x> STATIC_ASSERTION_FAILED;};
#define M_SCKT_STATIC_ASSERT_I(x, l) M_SCKT_STATIC_ASSERT_II(x, l)
#define M_SCKT_STATIC_ASSERT(x)  M_SCKT_STATIC_ASSERT_I(x, __LINE__)
#endif //~DOC_DONT_EXTRACT
//= ~Static Assert=

//some typedefs, these are system specific, so maybe there will appear some #ifdef s in the future, for different systems.
typedef unsigned int uint;///< platform native unsigned integer
typedef unsigned long long int u64;///< 64 bit unsigned integer
typedef unsigned int u32;///< 32 bit unsigned integer
typedef unsigned short u16;///< 16 bit unsigned integer
typedef unsigned char byte;///< 8bit unsigned integer

M_SCKT_STATIC_ASSERT( sizeof(u64)==8 )
M_SCKT_STATIC_ASSERT( sizeof(u32)==4 )
M_SCKT_STATIC_ASSERT( sizeof(u16)==2 )
M_SCKT_STATIC_ASSERT( sizeof(byte)==1 )

//forward declarations
class SocketSet;
class IPAddress;

//function prototypes
void ToNetworkFormat16(u16 value, byte* out_buf);
void ToNetworkFormat32(u32 value, byte* out_buf);
u16 FromNetworkFormat16(const byte* buf);
u32 FromNetworkFormat32(const byte* buf);


/**
@brief Basic exception class.
This is a basic exception class of the library. All other exception classes are derived from it.
*/
class M_DECLSPEC Exc : public std::exception{
    char *msg;
public:
    /**
    @brief Exception constructor.
    @param message Pointer to the exception message null-terminated string. Constructor will copy the string into objects internal memory buffer.
    */
    Exc(const char* message = 0) throw(std::bad_alloc);
    virtual ~Exc()throw();

    /**
    @brief Returns a pointer to exception message.
    @return a pointer to objects internal memory buffer holding the exception message null-terminated string.
            Note, that after the exception object is destroyed the pointer returned by this method become invalid.
    */
    const char *What()const{
        return this->msg?this->msg:("sckt::Exc(): Unknown exception");
    };

private:
    //override from std::exception
    const char *what()const throw(){
        return this->What();
    };
};

/**
@brief a structure which holds IP address
*/
class M_DECLSPEC IPAddress{
public:
    u32 host;///< IP address
    u16 port;///< IP port number

    inline IPAddress() :
        host(0),
        port(0)
    {}

    /**
    @brief Create IP address specifying exact ip address and port number.
    @param h - IP address. For example, 0x100007f represents "127.0.0.1" IP address value.
    @param p - IP port number.
    */
    inline IPAddress(u32 h, u16 p) :
            host(h),
            port(p)
    {}

    /**
    @brief Create IP address specifying exact ip address as 4 bytes and port number.
    The ip adress can be specified as 4 separate byte values, for example:
    @code
    sckt::IPAddress ip(127, 0, 0, 1, 80); //"127.0.0.1" port 80
    @endcode
    @param h1 - 1st triplet of IP address.
    @param h2 - 2nd triplet of IP address.
    @param h3 - 3rd triplet of IP address.
    @param h4 - 4th triplet of IP address.
    @param p - IP port number.
    */
    inline IPAddress(byte h1, byte h2, byte h3, byte h4, u16 p) :
            host(u32(h1) + (u32(h2)<<8) + (u32(h3)<<16) + (u32(h4)<<24)),
            port(p)
    {}

    /**
    @brief Create IP address specifying ip address as string and port number.
    @param ip - IP address null-terminated string. Example: "127.0.0.1".
    @param p - IP port number.
    */
    inline IPAddress(const char* ip, u16 p) throw(sckt::Exc) :
            host(IPAddress::ParseString(ip)),
            port(p)
    {}

    /**
    @brief compares two IP addresses for equality.
    @param ip - IP address to compare with.
    @return true if hosts and ports of the two IP addresses are equal accordingly.
    @return false otherwise.
    */
    inline bool operator==(const IPAddress& ip){
        return (this->host == ip.host) && (this->port == ip.port);
    }
private:
    //parse IP address from string
    static u32 ParseString(const char* ip) throw(sckt::Exc);
};

/**
@brief Library singletone class.
This is a library singletone class. Creating an object of this class initializes the library
while destroying this object deinitializes it. So, the convenient way of initializing the library
is to create an object of this class on the stack. Thus, when the object goes out of scope its
destructor will be called and the library will be deinitialized automatically.
This is what C++ RAII is all about ;-).
*/
//singletone
class M_DECLSPEC Library{
    static Library *instance;
public:
    Library()throw(sckt::Exc);
    ~Library();

    /**
    @brief Get reference to singletone object.
    This static method returns a reference to Sockets singletone object.
    If the object was not created before then this function will throw sckt::Exc.
    @return reference to Sockets singletone object.
    */
    static Library& Inst()throw(sckt::Exc){
        if(!Library::instance)
            throw sckt::Exc("Sockets::Inst(): singletone sckt::Library object is not created");
        return *Library::instance;
    };

    /**
    @brief Resolve host IP by its name.
    This function resolves host IP address by its name. If it fails resolving the IP address it will throw sckt::Exc.
    @param hostName - null-terminated string representing host name. Example: "www.somedomain.com".
    @param port - IP port number which will be placed in the resulting IPAddress structure.
    @return filled IPAddress structure.
    */
    IPAddress GetHostByName(const char *hostName, u16 port)throw(sckt::Exc);
private:
    static void InitSockets()throw(sckt::Exc);
    static void DeinitSockets();
};

/**
@brief Basic socket class.
This is a base class for all socket types such as TCP sockets or UDP sockets.
*/
class M_DECLSPEC Socket{
    friend class SocketSet;

public:
    //this type will hold system specific socket handle.
    //this buffer should be large enough to hold socket handle in different systems.
    //sizeof(u64) looks enough so far (need to work on 64-bit platforms too).
#ifndef DOC_DONT_EXTRACT //direction to doxygen not to extract this class
    struct SystemIndependentSocketHandle{
        sckt::byte buffer[sizeof(u64)];
    };
#endif//~DOC_DONT_EXTRACT

protected:
    bool isReady;

    SystemIndependentSocketHandle socket;

    Socket();

    Socket& operator=(const Socket& s);

public:
    virtual ~Socket(){
        this->Close();
    };

    /**
    @brief Tells whether the socket is opened or not.
    TODO: write some detailed description.
    @return Returns true if the socket is opened or false otherwise.
    */
    bool IsValid()const;

    /**
    @brief Tells whether there is some activity on the socket or not.
    See sckt::SocketSet class description for more info on how to use this method properly.
    @return
        - true if there is some data to read on the socket or if the remote socket has disconnected.
            The latter case can be detected by checking if subsequent Recv() method returns 0.
        - false if there is no any activity on the socket.
    */
    inline bool IsReady()const{
        return this->isReady;
    };

    /**
    @brief Closes the socket disconnecting it if necessary.
    */
    void Close();

    /**
    @brief Returns local port this socket is bound to.
    @return local port number to which this socket is bound,
            0 means that the socket is not bound.
    */
    u16 GetLocalPort() throw(sckt::Exc);
};

/**
@brief a class which represents a TCP socket.
*/
class M_DECLSPEC TCPSocket : public Socket{
    friend class TCPServerSocket;
public:
    /**
    @brief Constructs an invalid TCP socket object.
    */
    TCPSocket(){};

    /**
    @brief A copy constructor.
    Copy constructor creates a new socket object which refers to the same socket as s.
    After constructor completes the s becomes invalid.
    In other words, the behavior of copy constructor is similar to one of std::auto_ptr class from standard C++ library.
    @param s - other TCP socket to make a copy from.
    */
    //copy constructor
    TCPSocket(const TCPSocket& s){
        //NOTE: that operator= calls destructor, so this->socket should be invalid, base class constructor takes care about it.
        this->operator=(s);//same as auto_ptr
    }

    /**
    @brief A constructor which automatically calls sckt::TCPSocket::Open() method.
    This constructor creates a socket and calls its sckt::TCPSocket::Open() method.
    So, it creates an already opened socket.
    @param ip - IP address to 'connect to/listen on'.
    @param disableNaggle - enable/disable Naggle algorithm.
    */
    TCPSocket(const IPAddress& ip, bool disableNaggle = false) throw(sckt::Exc){
        this->Open(ip, disableNaggle);
    }

    /**
    @brief Assignment operator, works similar to std::auto_ptr::operator=().
    After this assignment operator completes this socket object refers to the socket the s objejct referred, s become invalid.
    It works similar to std::auto_ptr::operator=() from standard C++ library.
    @param s - socket to assign from.
    */
    TCPSocket& operator=(const TCPSocket& s){
        this->Socket::operator=(s);
        return *this;
    }

    /**
    @brief Connects the socket.
    This method connects the socket to remote TCP server socket.
    @param ip - IP address.
    @param disableNaggle - enable/disable Naggle algorithm.
    */
    void Open(const IPAddress& ip, bool disableNaggle = false) throw(sckt::Exc);

    /**
    @brief Send data to connected socket.
    Sends data on connected socket. This method blocks until all data is completely sent.
    @param data - pointer to the buffer with data to send.
    @param size - number of bytes to send.
    @return the number of bytes sent. Note that this value should normally be equal to the size argument value.
    */
    uint Send(const byte* data, uint size) throw(sckt::Exc);

    /**
    @brief Receive data from connected socket.
    Receives data available on the socket.
    If there is no data available this function blocks until some data arrives.
    @param buf - pointer to the buffer where to put received data.
    @param maxSize - maximal number of bytes which can be put to the buffer.
    @return if returned value is not 0 then it represents the number of bytes written to the buffer.
    @return 0 returned value indicates disconnection of remote socket.
    */
    //returns 0 if connection was closed by peer
    uint Recv(byte* buf, uint maxSize) throw(sckt::Exc);

    /**
    @brief Get local IP address and port.
    @return IP address and port of the local socket.
    */
    IPAddress GetLocalAddress() throw(sckt::Exc);

    /**
    @brief Get remote IP address and port.
    @return IP address and port of the peer socket.
    */
    IPAddress GetRemoteAddress() throw(sckt::Exc);

private:
    void DisableNaggle() throw(sckt::Exc);
};

/**
@brief a class which represents a TCP server socket.
TCP server socket is the socket which can listen for new connections
and accept them creating an ordinary TCP socket for it.
*/
class M_DECLSPEC TCPServerSocket : public Socket{
    bool disableNaggle;//this flag indicates if accepted sockets should be created with disabled Naggle
public:
    /**
    @brief Creates an invalid (unopened) TCP server socket.
    */
    TCPServerSocket() :
            disableNaggle(false)
    {};

    /**
    @brief A copy constructor.
    Copy constructor creates a new socket object which refers to the same socket as s.
    After constructor completes the s becomes invalid.
    In other words, the behavior of copy constructor is similar to one of std::auto_ptr class from standard C++ library.
    @param s - other TCP socket to make a copy from.
    */
    //copy constructor
    TCPServerSocket(const TCPServerSocket& s) :
            disableNaggle(s.disableNaggle)
    {
        //NOTE: that operator= calls destructor, so this->socket should be invalid, base class constructor takes care about it.
        this->operator=(s);//same as auto_ptr
    };

    /**
    @brief Assignment operator, works similar to std::auto_ptr::operator=().
    After this assignment operator completes this socket object refers to the socket the s objejct referred, s become invalid.
    It works similar to std::auto_ptr::operator=() from standard C++ library.
    @param s - socket to assign from.
    */
    TCPServerSocket& operator=(const TCPServerSocket& s){
        this->Socket::operator=(s);
        return *this;
    };

    /**
    @brief A constructor which automatically calls sckt::TCPServerSocket::Open() method.
    This constructor creates a socket and calls its sckt::TCPServerSocket::Open() method.
    So, it creates an already opened socket listening on the specified port.
    @param port - IP port number to listen on.
    @param disableNaggle - enable/disable Naggle algorithm for all accepted connections.
    */
    TCPServerSocket(u16 port, bool disableNaggle = false) throw(sckt::Exc){
        this->Open(port, disableNaggle);
    };

    /**
    @brief Connects the socket or starts listening on it.
    This method starts listening on the socket for incoming connections.
    @param port - IP port number to listen on.
    @param disableNaggle - enable/disable Naggle algorithm for all accepted connections.
    */
    void Open(u16 port, bool disableNaggle = false) throw(sckt::Exc);

    /**
    @brief Accepts one of the pending connections, non-blocking.
    Accepts one of the pending connections and returns a TCP socket object which represents
    either a valid connected socket or an invalid socket object.
    This function does not block if there is no any pending connections, it just returns invalid
    socket object in this case. One can periodically check for incoming connections by calling this method.
    @return sckt::TCPSocket object. One can later check if the returned socket object
        is valid or not by calling sckt::Socket::IsValid() method on that object.
        - if the socket is valid then it is a newly connected socket, further it can be used to send or receive data.
        - if the socket is invalid then there was no any connections pending, so no connection was accepted.
    */
    TCPSocket Accept() throw(sckt::Exc);
};

class M_DECLSPEC UDPSocket : public Socket{
public:
    UDPSocket(){};

    ~UDPSocket(){
        this->Close();
    };

    /**
    @brief Open the socket.
    This mthod opens the socket, this socket can further be used to send or receive data.
    After the socket is opened it becomes a valid socket and Socket::IsValid() will return true for such socket.
    After the socket is closed it becomes invalid.
    In other words, a valid socket is an opened socket.
    In case of errors this method throws sckt::Exc.
    @param port - IP port number on which the socket will listen for incoming datagrams.
        This is useful for server-side sockets, for client-side sockets use UDPSocket::Open().
    */
    void Open(u16 port) throw(sckt::Exc);


    inline void Open() throw(sckt::Exc){
        this->Open(0);
    };

    //returns number of bytes sent, should be less or equal to size.
    uint Send(const byte* buf, u16 size, IPAddress destinationIP) throw(sckt::Exc);

    //returns number of bytes received, 0 if connection was gracefully closed (???).
    uint Recv(byte* buf, u16 maxSize, IPAddress &out_SenderIP) throw(sckt::Exc);
};


/**
@brief Socket set class for checking multiple sockets for activity.
This class represents a set of sockets which can be checked for any activity
such as incoming data received or remote socket has disconnected.
Note, that the socket set holds only references to socket objects, so it is up to you
to make sure that all the socket objects you add to a particular socket set will not be
destroyed without prior removing them from socket set.
*/
class M_DECLSPEC SocketSet{
    Socket** set;
    uint maxSockets;
    uint numSockets;
  public:

    /**
    @brief Creates a socket set of the specified size.
    Creates a socket set which can hold the specified number of sockets at maximum.
    @param maxNumSocks - maximum number of sockets this socket set can hold.
    */
    SocketSet(uint maxNumSocks) throw(sckt::Exc, std::bad_alloc);

    /**
    @brief Destroys the socket set.
    Note, that it does not destroy the sockets this set holds references to.
    */
    ~SocketSet(){
        delete[] set;
    };

    /**
    @brief Returns number of sockets the socket set currently holds.
    @return number of sockets the socket set currently holds.
    */
    inline uint NumSockets()const{return this->numSockets;};

    /**
    @brief Returns maximal number of sockets this set can hold.
    @return maximal number of sockets this set can hold.
    */
    inline uint MaxSockets()const{return this->maxSockets;};

    /**
    @brief Add a socket to socket set.
    @param sock - pointer to the socket object to add.
    */
    void AddSocket(Socket *sock) throw(sckt::Exc);

    /**
    @brief Remove socket from socket set.
    @param sock - pointer to socket object which we want to remove from the set.
    */
    void RemoveSocket(Socket *sock) throw(sckt::Exc);

    /**
    @brief Check sokets from socket set for activity.
    This method checks sockets for activities of incoming data ready or remote socket has disconnected.
    This method sets ready flag for all sockets with activity which can later be checked by
    sckt::Socket::IsReady() method. The ready flag will be cleared by subsequent sckt::TCPSocket::Recv() function call.
    @param timeoutMillis - maximum number of milliseconds to wait for socket activity to appear.
        if 0 is specified the function will not wait and will return immediately.
    @return true if there is at least one socket with activity.
    @return false if there are no any sockets with activities.
    */
    //This function checks to see if data is available for reading on the
    //given set of sockets.  If 'timeout' is 0, it performs a quick poll,
    //otherwise the function returns when either data is available for
    //reading, or the timeout in milliseconds has elapsed, which ever occurs
    //first.  This function returns true if there are any sockets ready for reading,
    //or false if there was an error with the select() system call.
    bool CheckSockets(uint timeoutMillis);
};

}//~namespace sckt

using namespace sckt;

Library* Library::instance = 0;

inline static T_Socket& CastToSocket(sckt::Socket::SystemIndependentSocketHandle& s){
    M_SCKT_STATIC_ASSERT( sizeof(s) >= sizeof(T_Socket) )
    return *reinterpret_cast<T_Socket*>(&s);
};

inline static const T_Socket& CastToSocket(const sckt::Socket::SystemIndependentSocketHandle& s){
    return CastToSocket(const_cast<sckt::Socket::SystemIndependentSocketHandle&>(s));
};

//static
void Library::InitSockets()throw(sckt::Exc){
#ifdef _WIN32
    WORD versionWanted = MAKEWORD(2,2);
    WSADATA wsaData;
    if(WSAStartup(versionWanted, &wsaData) != 0 )
        throw sckt::Exc("sdlw::InitSockets(): Winsock 2.2 initialization failed");
#else //assume linux/unix
    // SIGPIPE is generated when a remote socket is closed
    void (*handler)(int);
    handler = signal(SIGPIPE, SIG_IGN);
    if(handler != SIG_DFL)
        signal(SIGPIPE, handler);
#endif
};

//static
void Library::DeinitSockets(){
#ifdef _WIN32
    // Clean up windows networking
    if(WSACleanup() == M_SOCKET_ERROR)
        if(WSAGetLastError() == WSAEINPROGRESS){
            WSACancelBlockingCall();
            WSACleanup();
        }
#else //assume linux/unix
    // Restore the SIGPIPE handler
    void (*handler)(int);
    handler = signal(SIGPIPE, SIG_DFL);
    if(handler != SIG_IGN)
        signal(SIGPIPE, handler);
#endif
};

IPAddress Library::GetHostByName(const char *hostName, u16 port)throw(sckt::Exc){
    if(!hostName)
        throw sckt::Exc("Sockets::GetHostByName(): pointer passed as argument is 0");

    IPAddress addr;
    addr.host = inet_addr(hostName);
    if(addr.host == INADDR_NONE){
        struct hostent *hp;
        hp = gethostbyname(hostName);
        if(hp)
            memcpy(&(addr.host), hp->h_addr, sizeof(addr.host)/* hp->h_length */);
        else
            throw sckt::Exc("Sockets::GetHostByName(): gethostbyname() failed");
    }
    addr.port = port;
    return addr;
};

sckt::Exc::Exc(const char* message) throw(std::bad_alloc){
    if(message==0)
        message = "unknown exception";

    int len = strlen(message);
    this->msg = new char[len+1];
    memcpy(this->msg, message, len);
    this->msg[len] = 0;//null-terminate
};

sckt::Exc::~Exc()throw(){
    delete[] this->msg;
};

Library::Library()throw(sckt::Exc){
    if(Library::instance != 0)
        throw sckt::Exc("Library::Library(): sckt::Library singletone object is already created");
    Library::InitSockets();
    this->instance = this;
};

Library::~Library(){
    //this->instance should not be null here
    Library::DeinitSockets();
    this->instance = 0;
};

Socket::Socket() :
        isReady(false)
{
    CastToSocket(this->socket) = M_INVALID_SOCKET;
}



bool Socket::IsValid()const{
    return CastToSocket(this->socket) != M_INVALID_SOCKET;
}



Socket& Socket::operator=(const Socket& s){
    if(this == &s)//detect self-assignment
        return *this;

    this->~Socket();
    CastToSocket(this->socket) = CastToSocket(s.socket);
    this->isReady = s.isReady;
    CastToSocket( const_cast<Socket&>(s).socket ) = M_INVALID_SOCKET;//same as std::auto_ptr
    return *this;
}



u16 Socket::GetLocalPort() throw(sckt::Exc){
    if(!this->IsValid())
        throw sckt::Exc("Socket::GetLocalPort(): socket is not valid");

    sockaddr_in addr;

#ifdef _WIN32
    int len = sizeof(addr);
#else//assume linux/unix
    socklen_t len = sizeof(addr);
#endif

    if(getsockname(
            CastToSocket(this->socket),
            reinterpret_cast<sockaddr*>(&addr),
            &len
        ) < 0)
    {
        throw sckt::Exc("Socket::GetLocalPort(): getsockname() failed");
    }

    return u16(ntohs(addr.sin_port));
}



IPAddress TCPSocket::GetLocalAddress() throw(sckt::Exc){
    if(!this->IsValid())
        throw sckt::Exc("Socket::GetLocalPort(): socket is not valid");
 
    sockaddr_in addr;

#ifdef _WIN32
    int len = sizeof(addr);
#else//assume linux/unix
    socklen_t len = sizeof(addr);
#endif

    if(getsockname(
            CastToSocket(this->socket),
            reinterpret_cast<sockaddr*>(&addr),
            &len
        ) < 0)
    {
        throw sckt::Exc("Socket::GetLocalPort(): getsockname() failed");
    }

    return IPAddress(
            u32(ntohl(addr.sin_addr.s_addr)),
            u16(ntohs(addr.sin_port))
        );
}



IPAddress TCPSocket::GetRemoteAddress() throw(sckt::Exc){
    if(!this->IsValid())
        throw sckt::Exc("TCPSocket::GetRemoteAddress(): socket is not valid");

    sockaddr_in addr;

#ifdef _WIN32
    int len = sizeof(addr);
#else//assume linux/unix
    socklen_t len = sizeof(addr);
#endif

    if(getpeername(
            CastToSocket(this->socket),
            reinterpret_cast<sockaddr*>(&addr),
            &len
        ) < 0)
    {
        throw sckt::Exc("TCPSocket::GetRemoteAddress(): getpeername() failed");
    }

    return IPAddress(
            u32(ntohl(addr.sin_addr.s_addr)),
            u16(ntohs(addr.sin_port))
        );
}



//static
sckt::u32 IPAddress::ParseString(const char* ip) throw(sckt::Exc){
    if(!ip)
        throw sckt::Exc("IPAddress::ParseString(): pointer passed as argument is 0");

    //subfunctions
    struct sf{
        static void ThrowInvalidIP(){
            throw sckt::Exc("IPAddress::ParseString(): string is not a valid IP address");
        }
    };

    u32 h = 0;//parsed host
    const char *curp = ip;
    for(uint t = 0; t < 4; ++t){
        uint digits[3];
        uint numDgts;
        for(numDgts = 0; numDgts < 3; ++numDgts){
            if( *curp == '.' || *curp == 0 ){
                if(numDgts==0)
                    sf::ThrowInvalidIP();
                break;
            }else{
                if(*curp < '0' || *curp > '9')
                    sf::ThrowInvalidIP();
                digits[numDgts] = uint(*curp) - uint('0');
            }
            ++curp;
        }

        if(t < 3 && *curp != '.')//unexpected delimiter or unexpected end of string
            sf::ThrowInvalidIP();
        else if(t == 3 && *curp != 0)
            sf::ThrowInvalidIP();

        uint xxx = 0;
        for(uint i = 0; i < numDgts; ++i){
            uint ord = 1;
            for(uint j = 1; j < numDgts - i; ++j)
               ord *= 10;
            xxx += digits[i] * ord;
        }
        if(xxx > 255)
            sf::ThrowInvalidIP();

        h |= (xxx << (8 * t));

        ++curp;
    }
    return h;
};

/* Open a TCP network server socket
   This creates a local server socket on the given port.
*/
void TCPServerSocket::Open(u16 port, bool disableNaggle) throw(sckt::Exc){
    if(this->IsValid())
        throw sckt::Exc("TCPServerSocket::Open(): socket already opened");

    this->disableNaggle = disableNaggle;

    CastToSocket(this->socket) = ::socket(AF_INET, SOCK_STREAM, 0);
    if(CastToSocket(this->socket) == M_INVALID_SOCKET)
        throw sckt::Exc("TCPServerSocket::Open(): Couldn't create socket");

    sockaddr_in sockAddr;
    memset(&sockAddr, 0, sizeof(sockAddr));
    sockAddr.sin_family = AF_INET;
    sockAddr.sin_addr.s_addr = INADDR_ANY;
    sockAddr.sin_port = htons(port);

    // allow local address reuse
    {
        int yes = 1;
        setsockopt(CastToSocket(this->socket), SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));
    }

    // Bind the socket for listening
    if( bind(CastToSocket(this->socket), reinterpret_cast<sockaddr*>(&sockAddr), sizeof(sockAddr)) == M_SOCKET_ERROR ){
        char buffer[128]; snprintf( buffer, 128, "TCPServerSocket::Open(): Couldn't bind to local port (%d)",
#ifdef _WIN32
        WSAGetLastError() );
#else //linux/unix
        errno );
#endif
        this->Close(); throw sckt::Exc(buffer);
    }

    if( listen(CastToSocket(this->socket), 5) == M_SOCKET_ERROR ){
        char buffer[128]; snprintf( buffer, 128, "TCPServerSocket::Open(): Couldn't listen to local port (%d)",
#ifdef _WIN32
        WSAGetLastError() );
#else //linux/unix
        errno );
#endif
        this->Close(); throw sckt::Exc(buffer);
    }

    //Set the socket to non-blocking mode for accept()
#if defined(__BEOS__) && defined(SO_NONBLOCK)
    // On BeOS r5 there is O_NONBLOCK but it's for files only
    {
        long b = 1;
        setsockopt(CastToSocket(this->socket), SOL_SOCKET, SO_NONBLOCK, &b, sizeof(b));
    }
#elif defined(O_NONBLOCK)
    {
        fcntl(CastToSocket(this->socket), F_SETFL, O_NONBLOCK);
    }
#elif defined(WIN32)
    {
        u_long mode = 1;
        ioctlsocket(CastToSocket(this->socket), FIONBIO, &mode);
    }
#elif defined(__OS2__)
    {
        int dontblock = 1;
        ioctl(CastToSocket(this->socket), FIONBIO, &dontblock);
    }
#else
#warning How do we set non-blocking mode on other operating systems?
#endif
}



/* Open a TCP network socket.
   A TCP connection to the remote host and port is attempted.
*/
void TCPSocket::Open(const IPAddress& ip, bool disableNaggle) throw(sckt::Exc){
    if(this->IsValid())
        throw sckt::Exc("TCPSocket::Open(): socket already opened");

    CastToSocket(this->socket) = ::socket(AF_INET, SOCK_STREAM, 0);
    if(CastToSocket(this->socket) == M_INVALID_SOCKET)
        throw sckt::Exc("TCPSocket::Open(): Couldn't create socket");

    //Connecting to remote host

    sockaddr_in sockAddr;
    memset(&sockAddr, 0, sizeof(sockAddr));
    sockAddr.sin_family = AF_INET;
    sockAddr.sin_addr.s_addr = ip.host;
    sockAddr.sin_port = htons(ip.port);

    // Connect to the remote host
    if( connect(CastToSocket(this->socket), reinterpret_cast<sockaddr *>(&sockAddr), sizeof(sockAddr)) == M_SOCKET_ERROR ){
        char buffer[128]; snprintf( buffer, 128, "TCPSocket::Open(): Couldn't connect to remote host (%d)",
#ifdef _WIN32
        WSAGetLastError() );
#else //linux/unix
        errno );
#endif
        this->Close(); throw sckt::Exc(buffer);
    }

    //Disable Naggle algorithm if required
    if(disableNaggle)
        this->DisableNaggle();

    this->isReady = false;
}



void TCPSocket::DisableNaggle() throw(sckt::Exc){
    if(!this->IsValid())
        throw sckt::Exc("TCPSocket::DisableNaggle(): socket is not opened");

    int yes = 1;
    setsockopt(CastToSocket(this->socket), IPPROTO_TCP, TCP_NODELAY, (char*)&yes, sizeof(yes));
}



void Socket::Close(){
    if(this->IsValid()){
#ifdef _WIN32
        //Closing socket in Win32.
        //refer to http://tangentsoft.net/wskfaq/newbie.html#howclose for details
        shutdown(CastToSocket(this->socket), SD_BOTH);
        closesocket(CastToSocket(this->socket));
#else //assume linux/unix
        close(CastToSocket(this->socket));
#endif
    }
    this->isReady = false;
    CastToSocket(this->socket) = M_INVALID_SOCKET;
}



TCPSocket TCPServerSocket::Accept() throw(sckt::Exc){
    if(!this->IsValid())
        throw sckt::Exc("TCPServerSocket::Accept(): the socket is not opened");

    this->isReady = false;

    sockaddr_in sockAddr;

#ifdef _WIN32
    int sock_alen = sizeof(sockAddr);
#else //linux/unix
    socklen_t sock_alen = sizeof(sockAddr);
#endif

    TCPSocket sock;//allocate a new socket object

    CastToSocket(sock.socket) = accept(CastToSocket(this->socket), reinterpret_cast<sockaddr*>(&sockAddr),
#ifdef USE_GUSI_SOCKETS
                (unsigned int *)&sock_alen);
#else
                &sock_alen);
#endif

    if(CastToSocket(sock.socket) == M_INVALID_SOCKET)
        return sock;//no connections to be accepted, return invalid socket

    //set blocking mode
#ifdef _WIN32
    {
        /* passing a zero value, socket mode set to block on */
        u_long mode = 0;
        ioctlsocket(CastToSocket(sock.socket), FIONBIO, &mode);
    }
#elif defined(O_NONBLOCK)
    {
        int flags = fcntl(CastToSocket(sock.socket), F_GETFL, 0);
        fcntl(CastToSocket(sock.socket), F_SETFL, flags & ~O_NONBLOCK);
    }
#else
#error do not know how to set blocking mode to socket
#endif //#ifdef WIN32

    if(this->disableNaggle)
        sock.DisableNaggle();

    return sock;//return a newly created socket
}



sckt::uint TCPSocket::Send(const sckt::byte* data, uint size) throw(sckt::Exc){
    if(!this->IsValid())
        throw sckt::Exc("TCPSocket::Send(): socket is not opened");

    int sent = 0,
        left = int(size);

    //Keep sending data until it's sent or an error occurs
    int errorCode = 0;

    int res;
    do{
        res = send(CastToSocket(this->socket), reinterpret_cast<const char*>(data), left, 0);
        if(res == M_SOCKET_ERROR){
#ifdef _WIN32
            errorCode = WSAGetLastError();
#else //linux/unix
            errorCode = errno;
#endif
        }else{
            sent += res;
            left -= res;
            data += res;
        }
    }while( (left > 0) && ((res != M_SOCKET_ERROR) || (errorCode == M_EINTR)) );

    if(res == M_SOCKET_ERROR)
        throw sckt::Exc("TCPSocket::Send(): send() failed");

    return uint(sent);
}



sckt::uint TCPSocket::Recv(sckt::byte* buf, uint maxSize) throw(sckt::Exc){
    //this flag shall be cleared even if this function fails to avoid subsequent
    //calls to Recv() because it indicates that there's activity.
    //So, do it at the beginning of the function.
    this->isReady = false;

    if(!this->IsValid())
        throw sckt::Exc("TCPSocket::Send(): socket is not opened");

    int len;
    int errorCode = 0;

    do{
        len = recv(CastToSocket(this->socket), reinterpret_cast<char *>(buf), maxSize, 0);
        if(len == M_SOCKET_ERROR){
#ifdef _WIN32
            errorCode = WSAGetLastError();
#else //linux/unix
            errorCode = errno;
#endif
        }
    }while(errorCode == M_EINTR);

    if(len == M_SOCKET_ERROR)
        throw sckt::Exc("TCPSocket::Recv(): recv() failed");

    return uint(len);
};

void UDPSocket::Open(u16 port) throw(sckt::Exc){
    if(this->IsValid())
        throw sckt::Exc("UDPSocket::Open(): the socket is already opened");

    CastToSocket(this->socket) = ::socket(AF_INET, SOCK_DGRAM, 0);
    if(CastToSocket(this->socket) == M_INVALID_SOCKET)
    throw sckt::Exc("UDPSocket::Open(): ::socket() failed");

    /* Bind locally, if appropriate */
    if(port != 0){
        struct sockaddr_in sockAddr;
        memset(&sockAddr, 0, sizeof(sockAddr));
        sockAddr.sin_family = AF_INET;
        sockAddr.sin_addr.s_addr = INADDR_ANY;
        sockAddr.sin_port = htons(port);

        // Bind the socket for listening
        if(bind(CastToSocket(this->socket), reinterpret_cast<struct sockaddr*>(&sockAddr), sizeof(sockAddr)) == M_SOCKET_ERROR){
            char buffer[128]; snprintf( buffer, 128, "UDPSocket::Open(): could not bind to local port (%d)",
#ifdef _WIN32
            WSAGetLastError() );
#else //linux/unix
            errno );
#endif
            this->Close(); throw sckt::Exc(buffer);
        }
    }
#ifdef SO_BROADCAST
    //Allow LAN broadcasts with the socket
    {
        int yes = 1;
        setsockopt(CastToSocket(this->socket), SOL_SOCKET, SO_BROADCAST, (char*)&yes, sizeof(yes));
    }
#endif

    this->isReady = false;
}



sckt::uint UDPSocket::Send(const sckt::byte* buf, u16 size, IPAddress destinationIP) throw(sckt::Exc){
    sockaddr_in sockAddr;
    int sockLen = sizeof(sockAddr);

    sockAddr.sin_addr.s_addr = destinationIP.host;
    sockAddr.sin_port = htons(destinationIP.port);
    sockAddr.sin_family = AF_INET;
    int res = sendto(CastToSocket(this->socket), reinterpret_cast<const char*>(buf), size, 0, reinterpret_cast<struct sockaddr*>(&sockAddr), sockLen);

    if(res == M_SOCKET_ERROR)
        throw sckt::Exc("UDPSocket::Send(): sendto() failed");

    return res;
}



sckt::uint UDPSocket::Recv(sckt::byte* buf, u16 maxSize, IPAddress &out_SenderIP) throw(sckt::Exc){
    //this flag shall be cleared even if this function fails to avoid subsequent
    //calls to Recv() because it indicates that there's activity.
    //So, do it at the beginning of the function.
    this->isReady = false;

    sockaddr_in sockAddr;

#ifdef _WIN32
    int sockLen = sizeof(sockAddr);
#else //linux/unix
    socklen_t sockLen = sizeof(sockAddr);
#endif

    int res = recvfrom(CastToSocket(this->socket), reinterpret_cast<char*>(buf), maxSize, 0, reinterpret_cast<sockaddr*>(&sockAddr), &sockLen);

    if(res == M_SOCKET_ERROR)
        throw sckt::Exc("UDPSocket::Recv(): recvfrom() failed");

    out_SenderIP.host = ntohl(sockAddr.sin_addr.s_addr);
    out_SenderIP.port = ntohs(sockAddr.sin_port);
    return res;
}



SocketSet::SocketSet(uint maxNumSocks) throw(sckt::Exc, std::bad_alloc):
        maxSockets(maxNumSocks),
        numSockets(0)
{
    if(this->maxSockets > M_FD_SETSIZE)
        throw sckt::Exc("SocketSet::SocketSet(): socket size reuqested is too large");
    this->set = new Socket*[this->maxSockets];
};

void SocketSet::AddSocket(Socket *sock) throw(sckt::Exc){
    if(!sock)
        throw sckt::Exc("SocketSet::AddSocket(): null socket pointer passed as argument");

    if(this->numSockets == this->maxSockets)
        throw sckt::Exc("SocketSet::AddSocket(): socket set is full");

    for(uint i=0; i<this->numSockets; ++i){
        if(this->set[i] == sock)
            return;
    }

    this->set[this->numSockets] = sock;
    ++this->numSockets;
};

void SocketSet::RemoveSocket(Socket *sock) throw(sckt::Exc){
    if(!sock)
        throw sckt::Exc("SocketSet::RemoveSocket(): null socket pointer passed as argument");

    uint i;
    for(i=0; i<this->numSockets; ++i)
        if(this->set[i]==sock)
            break;

    if(i==this->numSockets)
        return;//socket sock not found in the set

    --this->numSockets;//decrease numsockets before shifting the sockets
    //shift sockets
    for(;i<this->numSockets; ++i){
        this->set[i] = this->set[i+1];
    }
};

bool SocketSet::CheckSockets(uint timeoutMillis){
    if(this->numSockets == 0)
        return false;

    T_Socket maxfd = 0;

    //Find the largest file descriptor
    for(uint i = 0; i<this->numSockets; ++i){
        if(CastToSocket(this->set[i]->socket) > maxfd)
            maxfd = CastToSocket(this->set[i]->socket);
    }

    int retval;
    fd_set readMask;

    //Check the file descriptors for available data
    int errorCode = 0;
    do{
        //Set up the mask of file descriptors
        FD_ZERO(&readMask);
        for(uint i=0; i<this->numSockets; ++i){
            T_Socket socketHnd = CastToSocket(this->set[i]->socket);
            FD_SET(socketHnd, &readMask);
        }

        // Set up the timeout
        //TODO: consider moving this out of do{}while() loop
        timeval tv;
        tv.tv_sec = timeoutMillis/1000;
        tv.tv_usec = (timeoutMillis%1000)*1000;

        retval = select(maxfd+1, &readMask, NULL, NULL, &tv);
        if(retval == M_SOCKET_ERROR){
#ifdef _WIN32
            errorCode = WSAGetLastError();
#else
            errorCode = errno;
#endif
        }
    }while(errorCode == M_EINTR);

    // Mark all file descriptors ready that have data available
    if(retval != 0 && retval != M_SOCKET_ERROR){
        int numSocketsReady = 0;
        for(uint i=0; i<this->numSockets; ++i){
            T_Socket socketHnd = CastToSocket(this->set[i]->socket);
            if( (FD_ISSET(socketHnd, &readMask)) ){
                this->set[i]->isReady = true;
                ++numSocketsReady;
            }
        }

        //on Win32 when compiling with mingw there are some strange things,
        //sometimes retval is not zero but there is no any sockets marked as ready in readMask.
        //I do not know why this happens on win32 and mingw. The workaround is to calculate number
        //of active sockets mnually, ignoring the retval value.
        if(numSocketsReady > 0)
            return true;
    }
    return false;
}



void sckt::ToNetworkFormat16(u16 value, byte* out_buf){
    *reinterpret_cast<u16*>(out_buf) = value;//assume little-endian
}



void sckt::ToNetworkFormat32(u32 value, byte* out_buf){
    *reinterpret_cast<u32*>(out_buf) = value;//assume little-endian
}



u16 sckt::FromNetworkFormat16(const byte* buf){
    return *reinterpret_cast<const u16*>(buf);//assume little-endian
}



u32 sckt::FromNetworkFormat32(const byte* buf){
    return *reinterpret_cast<const u32*>(buf);//assume little-endian
}

///////////////////////////////////////////////

#include "Socket.h"
#include <algorithm>
#include <functional>
#include <iostream>
#include <sstream>

sckt::Library g_socketSingleton;

class SocketChecker
{
public:
    static SocketChecker* instance()
    {
        static SocketChecker s_instance;
        return &s_instance;
    }
    
    SocketChecker()
    {
        _socketSet = new sckt::SocketSet(32);  // TODO: max to 32 sockets in one application?
    }
    
    virtual ~SocketChecker()
    {
        delete _socketSet;
        _socketSet = NULL;
    }
    
    void add(sckt::Socket* socket) { _socketSet->AddSocket(socket); }
    void remove(sckt::Socket* socket) { _socketSet->RemoveSocket(socket); }
    bool check(unsigned int ms) { return _socketSet->CheckSockets(ms); }
    
protected:
    sckt::SocketSet* _socketSet;
};

bool wincomm::Socket::checkReadingStates(unsigned int ms)
{
    return SocketChecker::instance()->check(ms);
}

bool wincomm::Socket::bind(int port, Type t)
{
    _type = t;
    _serverSide = true;
    if (_type == TCP)
    {
        sckt::TCPServerSocket* s = new sckt::TCPServerSocket;
        s->Open(port);
        _socket = (void*)s;
        if (s->IsValid())
        {
            SocketChecker::instance()->add(s);
            return true;
        }
    }
    else
    {
        sckt::UDPSocket* s = new sckt::UDPSocket;
        s->Open(port);
        _socket = (void*)s;
        if (s->IsValid())
        {
            SocketChecker::instance()->add(s);
            return true;
        }
    }
    return false;
}

bool wincomm::Socket::connect(const std::string& address, int port, Type t)
{
    sckt::IPAddress remoteIP(address.c_str(), port);
    _remoteHost = remoteIP.host;
    _remotePort = remoteIP.port;
    _type = t;
    _serverSide = false;
    if (_type == TCP)
    {
        sckt::TCPSocket* s = new sckt::TCPSocket;
        s->Open(remoteIP);
        _socket = (void*)s;
        if (s->IsValid())
        {
            SocketChecker::instance()->add(s);
            return true;
        }
    }
    else
    {
        sckt::UDPSocket* s = new sckt::UDPSocket;
        s->Open();
        _socket = (void*)s;
        if (s->IsValid())
        {
            SocketChecker::instance()->add(s);
            return true;
        }
    }
    return false;
}

bool wincomm::Socket::disconnect()
{
    if (_socket)
    {
        if (_type == TCP)
        {
            for (unsigned int i = 0; i<_connectedSockets.size(); ++i)
            {
                sckt::TCPServerSocket* c = (sckt::TCPServerSocket*)_connectedSockets[i];
                c->Close();
                SocketChecker::instance()->remove(c);
                delete c;
            }
            _connectedSockets.clear();
        }

        sckt::Socket* s = (sckt::Socket*)_socket;
        s->Close();
        SocketChecker::instance()->remove(s);
        _socket = NULL;
        return true;
    }
    return false;
}

bool wincomm::Socket::isConnected() const
{
    if (!_socket) return false;
    return ((sckt::Socket*)_socket)->IsValid();
}

bool wincomm::Socket::checkNewConnection(int retryTimes)
{
    if (!_socket) return false;
    else if (_type == TCP)
    {
        sckt::TCPServerSocket* s = (sckt::TCPServerSocket*)_socket;
        sckt::TCPSocket clientSocket;
        if (!retryTimes)
        {
            while (!clientSocket.IsValid()) clientSocket = s->Accept();
        }
        else
        {
            int t = 0;
            while (t++<retryTimes && !clientSocket.IsValid())
                clientSocket = s->Accept();
            if (!clientSocket.IsValid()) return false;
        }

        sckt::TCPSocket* c = new sckt::TCPSocket(clientSocket);
        SocketChecker::instance()->add(c);
        _connectedSockets.push_back(c);
        return true;
    }
    else return true;
}

int wincomm::Socket::read(void* dest, int maxToRead, bool doCheck)
{
    if (!_socket) return 0;
    if (doCheck) checkReadingStates();

    int size = 0;
    if (_type == TCP)
    {
        if (!_serverSide)
        {
            sckt::TCPSocket* c = (sckt::TCPSocket*)_socket;
            if (!c->IsValid() || !c->IsReady()) return 0;

            try
            { size = c->Recv((sckt::byte*)dest, maxToRead); }
            catch (sckt::Exc& e)
            { std::cerr << "[Socket] " << e.What() << std::endl; }
            return size;
        }

        for (std::vector<void*>::iterator itr = _connectedSockets.begin();
            itr != _connectedSockets.end(); )
        {
            sckt::TCPSocket* c = (sckt::TCPSocket*)(*itr);
            if (!c->IsValid() || !c->IsReady()) { itr++; continue; }

            try
            { size = c->Recv((sckt::byte*)dest, maxToRead); }
            catch (sckt::Exc& e)
            { std::cerr << "[Socket] " << e.What() << std::endl; }
            if (size <= 0)
            {
                SocketChecker::instance()->remove(c);
                if (_lastActiveClientSocket == c) _lastActiveClientSocket = NULL;
                c->Close(); delete c;
                itr = _connectedSockets.erase(itr);  // remove the client
            }
            else
            {
                _lastActiveClientSocket = (void*)c;
                return size;
            }
        }
        return 0;
    }
    else
    {
        sckt::IPAddress senderIP;
        sckt::UDPSocket* s = (sckt::UDPSocket*)_socket;
        if (!s->IsReady()) return 0;

        try
        { size = s->Recv((sckt::byte*)dest, maxToRead, senderIP); }
        catch (sckt::Exc& e)
        { std::cerr << "[Socket] " << e.What() << std::endl; }
        if (_serverSide && size > 0)
        {
            // UDP server will record last client IP and ready to send data to it
            _remoteHost = senderIP.host;
            _remotePort = senderIP.port;

            // FIXME: should we always swap bytes here?
            _remoteHost = (_remoteHost & 0x000000ff) << 24 | (_remoteHost & 0x0000ff00) << 8 |
                          (_remoteHost & 0x00ff0000) >> 8 | (_remoteHost & 0xff000000) >> 24;
        }
        return size;
    }
}

int wincomm::Socket::writeToLastActiveClient(const void* source, int numToWrite)
{
    if (!_socket || _type != TCP) return 0;
    if (!_serverSide || !_lastActiveClientSocket) return 0;

    sckt::TCPSocket* c = (sckt::TCPSocket*)_lastActiveClientSocket;
    try
    {
        if (!c->IsValid()) return 0;
        return c->Send((const sckt::byte*)source, numToWrite);
    }
    catch (sckt::Exc& e)
    { std::cerr << "[Socket] " << e.What() << std::endl; }
    return 0;
}

int wincomm::Socket::write(const void* source, int numToWrite)
{
    if (!_socket) return 0;
    else if (_type == TCP)
    {
        if (!_serverSide)
        {
            sckt::TCPSocket* c = (sckt::TCPSocket*)_socket;
            if (!c->IsValid()) return 0;

            int size = 0;
            try
            { size = c->Send((const sckt::byte*)source, numToWrite); }
            catch (sckt::Exc& e)
            { std::cerr << "[Socket] " << e.What() << std::endl; }
            return size;
        }

        int totalSize = 0; std::vector<void*> toRemove;
        for (unsigned int i = 0; i<_connectedSockets.size(); ++i)
        {
            sckt::TCPSocket* c = (sckt::TCPSocket*)_connectedSockets[i];
            if (!c->IsValid()) continue;

            try
            { totalSize += c->Send((const sckt::byte*)source, numToWrite); }
            catch (sckt::Exc& e)
            {
                toRemove.push_back((void*)c);
                std::cerr << "[Socket] " << e.What() << std::endl;
            }
        }

        for (unsigned int i = 0; i < toRemove.size(); ++i)
        {
            // Remove outdated client sockets
            std::vector<void*>::iterator itr = std::find(
                _connectedSockets.begin(), _connectedSockets.end(), toRemove[i]);
            if (itr != _connectedSockets.end()) _connectedSockets.erase(itr);
        }
        return totalSize;
    }
    else
    {
        sckt::UDPSocket* s = (sckt::UDPSocket*)_socket;
        if (!_remoteHost)
        {
            std::cerr << "[Socket] UDP socket can't write without"
                      << " knowing the opposite's IP" << std::endl;
            return 0;
        }

        int size = 0;
        try
        {
            size = s->Send((const sckt::byte*)source, numToWrite,
                           sckt::IPAddress(_remoteHost, _remotePort));
        }
        catch (sckt::Exc& e)
        { std::cerr << "[Socket] " << e.What() << std::endl; }
        return size;
    }
}

int wincomm::Socket::write(const std::string& address, int port, const void* source, int numToWrite)
{
    if (!_socket) return 0;
    else if (_type == TCP) return write(source, numToWrite);
    else
    {
        sckt::UDPSocket* s = (sckt::UDPSocket*)_socket;
        int size = 0;
        try
        {
            size = s->Send((const sckt::byte*)source, numToWrite,
                           sckt::IPAddress(address.c_str(), port));
        }
        catch (sckt::Exc& e)
        { std::cerr << "[Socket] " << e.What() << std::endl; }
        return size;
    }
}
