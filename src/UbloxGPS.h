#ifndef WINCOMM_UBLOXGPS_HEADER
#define WINCOMM_UBLOXGPS_HEADER 1

#include <ublox/Message.h>
#include <ublox/frame/UbloxFrame.h>
#include <ublox/message/NavHpposllh.h>
#include <ublox/message/NavPosllh.h>
#include <ublox/message/NavVelned.h>
#include <ublox/message/NavSat.h>
#include <ublox/message/NavStatus.h>
#include <functional>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>

namespace wincomm
{

class Uart;
class Socket;

class UbloxTimer
{
public:
    typedef std::chrono::milliseconds Interval;
    typedef std::function<void (void*)> Timeout;
    UbloxTimer() : _running(false) {}
    
    void start(const Interval& interval, const Timeout& timeout, void* data)
    {
        _running = true;
        _thread = std::thread([=]()
        {
            while (_running)
            {
                std::this_thread::sleep_for(interval);
                timeout(data);
            }
        });
    }

    void stop()
    {
        _running = false;
        _thread.join();
    }
    
protected:
    std::thread _thread;
    bool _running;
};
class UbloxGPS
{
public:
    UbloxGPS() : _uart(NULL), _ntripSocket(NULL), _dirtyMessageTypes(0), satellites_signal_threshold_(40) {}
    virtual ~UbloxGPS();
    
    void start(const std::string& filename, int linesPerRead, int reqPositionMS = 100);
    void start(Uart* uart, int reqPositionMS = 100);
    void stop() { _timer.stop(); }
    bool readFromBuffer();

    bool connectNtripCaster(const char* host, int port, const char* mountpoint,
                            const char* appName, const char* auth);
    bool updateNtripCaster(std::vector<unsigned char>& casterData, bool sendToGPS);
    void disconnectNtripCaster();
    
    using InMessage = ublox::Message<comms::option::ReadIterator<const std::uint8_t*>,
                                     comms::option::Handler<UbloxGPS> >;
    void handle(InMessage& msg);
    
    using InNavPosllh = ublox::message::NavPosllh<InMessage>;
    virtual void handle(InNavPosllh& msg);
    void requestPositionPoll();  /// Require position data (auto)

    using InNavVelocity = ublox::message::NavVelned<InMessage>;
    virtual void handle(InNavVelocity& msg);
    void requestVelocityPoll();  /// Require velocity data

    using InNavSat = ublox::message::NavSat<InMessage>;
    virtual void handle(InNavSat& msg);
    void requestSatellitePoll();  /// Require satellite information data

    using InNavStatus = ublox::message::NavStatus<InMessage>;
    virtual void handle(InNavStatus& msg);
    void requestStatusPoll();   /// Require fix mode data

    enum MessageTypeFlag { MSG_POSITION = 0x1, MSG_VELOCITY = 0x2, MSG_SATELLITE = 0x4, MSG_STATUS = 0x8 };
    int getDirtyMessageTypes() const { return _dirtyMessageTypes; }
    void clearDirtyMessageTypes() { _dirtyMessageTypes = 0; }
    
    struct NavigationData
    {
        double latitude, longitude, altitude;
        double hAccuracy, vAccuracy, timeOfWeekMS;
        NavigationData() : timeOfWeekMS(-1.0) {}
    };
    const NavigationData& getLatestNavigation() const { return _navData; }

    struct VelocityData
    {
        double velocityN, velocityE, velocityD, timeOfWeekMS;
        VelocityData() : timeOfWeekMS(-1.0) {}
    };
    const VelocityData& getLatestVelocity() const { return _velocityData; }
    
    struct SatelliteData
    {
        struct SatelliteInfo
        {
            std::string gnssID;
            int satelliteID, signalCNO;
            double elevation, azimuth;
        };
        std::vector<SatelliteInfo> infoList;
        double timeOfWeekMS;
        double score;
        int valid_count;
        SatelliteData() : timeOfWeekMS(-1.0) {}
    };
    const SatelliteData& getLatestSatellites() const { return _satelliteData; }

    struct StatusData
    {
        enum FixMode { NoFix = 0, DeadReckon, Fix2D, Fix3D, GpsAndDeadReckon, TimeOnlyFix };
        FixMode fixMode;
        bool gpsFixValid, diffCorrectionValid;
        double timeToFirstFix, timeSinceStart, timeOfWeekMS;
        StatusData() : timeOfWeekMS(-1.0) {}
    };
    const StatusData& getLatestStatus() const { return _statusData; }
    
protected:
    using OutBuffer = std::vector<std::uint8_t>;
    using OutMessage = ublox::Message<comms::option::IdInfoInterface,
                                      comms::option::WriteIterator<std::back_insert_iterator<OutBuffer> >,
                                      comms::option::LengthInfoInterface>;
    
    // Remember to add requests here
    using AllInMessages = std::tuple<InNavPosllh, InNavStatus, InNavVelocity, InNavSat>;
    using Frame = ublox::frame::UbloxFrame<InMessage, AllInMessages>;

    void sendMessage(const OutMessage& msg);
    void configureUbxOutput();
    
    Uart* _uart;
    Socket* _ntripSocket;
    std::ifstream _emulatedFile;
    int _linesReadFromEmuFile;

    UbloxTimer _timer;
    NavigationData _navData;
    VelocityData _velocityData;
    SatelliteData _satelliteData;
    StatusData _statusData;
    int _dirtyMessageTypes;
    
    Frame _frame;
    std::string _message_GPGGA_GNGGA;
    std::vector<std::uint8_t> _dataBuffer;
    int satellites_signal_threshold_;
};

}

#endif
