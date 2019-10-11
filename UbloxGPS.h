#ifndef WINCOMM_UBLOXGPS_HEADER
#define WINCOMM_UBLOXGPS_HEADER 1

#include <ublox/Message.h>
#include <ublox/frame/UbloxFrame.h>
#include <ublox/message/NavHpposllh.h>
#include <ublox/message/NavPosllh.h>
#include <ublox/message/NavVelned.h>
#include <ublox/message/NavSat.h>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>

namespace wincomm
{

class Uart;

class UbloxTimer
{
public:
    typedef std::chrono::milliseconds Interval;
    typedef std::function<void (void*)> TimeoutFunction;
    UbloxTimer() : _running(false) {}
    
    void start(const Interval& interval, const TimeoutFunction& timeoutFunc, void* data)
    {
        _running = true;
        _thread = std::thread([=]()
        {
            while (_running)
            {
                std::this_thread::sleep_for(interval);
                timeoutFunc(data);
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
    UbloxGPS() : _uart(NULL), _dirtyMessageTypes(0) {}
    virtual ~UbloxGPS();
    
    void start(Uart* uart, int reqPositionMS = 1000);
    void stop() { _timer.stop(); }
    bool readFromBuffer();
    
    using InMessage = ublox::Message<comms::option::ReadIterator<const std::uint8_t*>,
                                     comms::option::Handler<UbloxGPS> >;
    void handle(InMessage& msg);
    
    using InNavPosllh = ublox::message::NavPosllh<InMessage>;
    virtual void handle(InNavPosllh& msg);
    void requestPositionPoll();

    using InNavVelocity = ublox::message::NavVelned<InMessage>;
    virtual void handle(InNavVelocity& msg);
    void requestVelocityPoll();

    using InNavSat = ublox::message::NavSat<InMessage>;
    virtual void handle(InNavSat& msg);
    void requestSatellitePoll();
    
    enum MessageTypeFlag { MSG_POSITION = 0x1, MSG_VELOCITY = 0x2, MSG_SATELLITE = 0x4 };
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
        SatelliteData() : timeOfWeekMS(-1.0) {}
    };
    const SatelliteData& getLatestSatellites() const { return _satelliteData; }
    
protected:
    using OutBuffer = std::vector<std::uint8_t>;
    using OutMessage = ublox::Message<comms::option::IdInfoInterface,
                                      comms::option::WriteIterator<std::back_insert_iterator<OutBuffer> >,
                                      comms::option::LengthInfoInterface>;
    using AllInMessages = std::tuple<InNavPosllh, InNavVelocity, InNavSat>;
    using Frame = ublox::frame::UbloxFrame<InMessage, AllInMessages>;

    void sendMessage(const OutMessage& msg);
    void configureUbxOutput();
    
    Uart* _uart;
    UbloxTimer _timer;
    NavigationData _navData;
    VelocityData _velocityData;
    SatelliteData _satelliteData;
    int _dirtyMessageTypes;
    
    Frame _frame;
    std::vector<std::uint8_t> _dataBuffer;
};

}

#endif
