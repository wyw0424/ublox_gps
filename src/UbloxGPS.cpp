#include "Socket.h"
#include "Uart.h"
#include "UbloxGPS.h"
#include <ublox/field/GnssId.h>
#include <ublox/message/CfgPrtUsb.h>
#include <ublox/message/NavPosllhPoll.h>
#include <ublox/message/NavHpposllhPoll.h>
#include <ublox/message/NavVelnedPoll.h>
#include <ublox/message/NavSatPoll.h>
#include <comms/units.h>
#include <comms/process.h>
#include <iostream>
#include <sstream>

using namespace wincomm;

static void threadFunction(void* userData)
{
    UbloxGPS* gps = (UbloxGPS*)userData;
    gps->requestPositionPoll();
    //gps->requestSatellitePoll();
}

UbloxGPS::~UbloxGPS()
{
    
    disconnectNtripCaster();
    stop();
}

void UbloxGPS::start(const std::string& filename, int linesPerRead, int reqPositionMS)
{
    _uart = NULL; _linesReadFromEmuFile = linesPerRead;
    _emulatedFile = std::ifstream(filename.c_str());
    if (reqPositionMS > 0)
        _timer.start(std::chrono::milliseconds(reqPositionMS), threadFunction, this);
    configureUbxOutput();
}

void UbloxGPS::start(Uart* uart, int reqPositionMS)
{
    _uart = uart; _linesReadFromEmuFile = 0;
    if (reqPositionMS > 0)
        _timer.start(std::chrono::milliseconds(reqPositionMS), threadFunction, this);
    configureUbxOutput();
}

bool UbloxGPS::readFromBuffer()
{
    static char buffer[128] = "";
    int length = _uart ? _uart->readToBuffer(buffer, 128) : 0;
    if (length > 0)
        _dataBuffer.insert(_dataBuffer.end(), buffer, buffer + length);
    
    if (_emulatedFile && _linesReadFromEmuFile > 0)
    {
        int cnt = _linesReadFromEmuFile; std::string line;
        while (cnt > 0 && std::getline(_emulatedFile, line))
        {
            cnt--; if (cnt > 0) line += "\r\n";
            _dataBuffer.insert(_dataBuffer.end(), line.begin(), line.end());
        }
    }

    using FrameType = typename std::decay<decltype(_frame)>::type;
    using MsgPtr = typename FrameType::MsgPtr;
    std::size_t consumed = 0, msgDispatched = 0;
    while (consumed < _dataBuffer.size())
    {
        auto begIter = &(_dataBuffer[0]) + consumed;
        auto iter = begIter; MsgPtr msg;

        auto es = comms::processSingleWithDispatch(iter, _dataBuffer.size() - consumed, _frame, msg, *this);
        consumed += std::distance(begIter, iter);
        if (es == comms::ErrorStatus::NotEnoughData) break;
        else if (es == comms::ErrorStatus::Success) msgDispatched++;
    }

    if (_emulatedFile.eof()) exit(1);

    if (consumed > 0)
    {
        std::string msgBuffer((char*)&(_dataBuffer[0]), consumed);
        size_t gpggaStart = msgBuffer.find("$GPGGA");
        if (gpggaStart == std::string::npos) gpggaStart = msgBuffer.find("$GNGGA");

        size_t gpggaEndCRC = msgBuffer.find("*", gpggaStart);
        if (gpggaStart != std::string::npos && gpggaEndCRC != std::string::npos)
        {
            _message_GPGGA_GNGGA = msgBuffer.substr(gpggaStart, gpggaEndCRC + 3 - gpggaStart);
        }
    }
    _dataBuffer.erase(_dataBuffer.begin(), _dataBuffer.begin() + consumed);
    return msgDispatched > 0;
}

bool UbloxGPS::connectNtripCaster(const char* host, int port, const char* mountpoint,
                                  const char* appName, const char* auth)
{
    if (!host || !mountpoint || !appName || !auth)
    {
        std::cout << "[UbloxGPS] No enough parameters to start NTRIP client" << std::endl;
        return false;
    }

    if (_ntripSocket != NULL) delete _ntripSocket;
    _ntripSocket = new Socket;
    try
    {
        if (!_ntripSocket->connect(host, port, wincomm::Socket::TCP))
        {
            std::cout << "[UbloxGPS] Failed to connect to NTRIP caster" << std::endl;
            return false;
        }
    }
    catch (std::exception& e)
    {
        std::cout << "[UbloxGPS] " << e.what() << std::endl;
        return false;
    }

    std::stringstream ss;
    ss << "GET /" << mountpoint << " HTTP/1.0\r\n"
       << "User-Agent: " << appName << "\r\n"
       << "Accept: */*\r\n" << "Connection: close\r\n"
       << "Authorization: Basic " << auth << "\r\n\r\n";
    _ntripSocket->write(ss.str().c_str(), ss.str().size());
    return true;
}

bool UbloxGPS::updateNtripCaster(std::vector<unsigned char>& casterData, bool sendToGPS)
{
    char buffer[1024] = "";
    if (!_ntripSocket) return false;

    int length = _ntripSocket->read(buffer, 1024);
    if (length > 0)
    {
        casterData.insert(casterData.end(), buffer, buffer + length);
        if (sendToGPS && _uart) _uart->writeFromBuffer(buffer, length);
    }

    if (!_message_GPGGA_GNGGA.empty())
    {
        _message_GPGGA_GNGGA += "\r\n";
        _ntripSocket->write(_message_GPGGA_GNGGA.c_str(), _message_GPGGA_GNGGA.size());
        _message_GPGGA_GNGGA = "";
    }
    return true;
}

void UbloxGPS::disconnectNtripCaster()
{
    if (!_ntripSocket) return;
    delete _ntripSocket;
    _ntripSocket = NULL;
}

void UbloxGPS::requestPositionPoll()
{ sendMessage(ublox::message::NavPosllhPoll<OutMessage>()); }

void UbloxGPS::handle(InNavPosllh& msg)
{
    _navData.latitude = comms::units::getDegrees<double>(msg.field_lat());
    _navData.longitude = comms::units::getDegrees<double>(msg.field_lon());
    _navData.altitude = comms::units::getMeters<double>(msg.field_height());
    _navData.hAccuracy = comms::units::getMeters<double>(msg.field_hAcc());
    _navData.vAccuracy = comms::units::getMeters<double>(msg.field_vAcc());
    _navData.timeOfWeekMS = comms::units::getMilliseconds<double>(msg.field_itow());
    _dirtyMessageTypes |= MSG_POSITION;
}

void UbloxGPS::requestVelocityPoll()
{ sendMessage(ublox::message::NavVelnedPoll<OutMessage>()); }

void UbloxGPS::handle(InNavVelocity& msg)
{
    _velocityData.velocityN = comms::units::details::getSpeed<double, comms::traits::units::MetersRatio>(msg.field_velN());
    _velocityData.velocityE = comms::units::details::getSpeed<double, comms::traits::units::MetersRatio>(msg.field_velE());
    _velocityData.velocityD = comms::units::details::getSpeed<double, comms::traits::units::MetersRatio>(msg.field_velD());
    _velocityData.timeOfWeekMS = comms::units::getMilliseconds<double>(msg.field_itow());
    _dirtyMessageTypes |= MSG_VELOCITY;
}

void UbloxGPS::requestSatellitePoll()
{ sendMessage(ublox::message::NavSatPoll<OutMessage>()); }

void UbloxGPS::handle(InNavSat& msg)
{
    auto satList = msg.field_list().value();
    _satelliteData.infoList.resize(satList.size());
    
    //std::cout << "satllite number: " << satList.size() << std::endl;
    
    _satelliteData.timeOfWeekMS = comms::units::getMilliseconds<double>(msg.field_itow());
    _satelliteData.score = 0;
    _satelliteData.valid_count = 0;

    for (size_t i = 0; i < satList.size(); ++i)
    {
        auto element = satList.at(i);
        _satelliteData.infoList[i].gnssID = ublox::field::GnssId<>::valueName(element.field_gnssId().value());
        _satelliteData.infoList[i].satelliteID = element.field_svid().value();
        _satelliteData.infoList[i].elevation = comms::units::getDegrees<double>(element.field_elev());
        _satelliteData.infoList[i].azimuth = comms::units::getDegrees<double>(element.field_azim());
        _satelliteData.infoList[i].signalCNO = element.field_cno().value();
        //std::cout << i << ": " << _satelliteData.infoList[i].signalCNO << std::endl;

        if(_satelliteData.infoList[i].signalCNO > satellites_signal_threshold_)
        {
            _satelliteData.valid_count++;
        }
        _satelliteData.score = _satelliteData.score + _satelliteData.infoList[i].signalCNO;

        //std::cout << _satelliteData.infoList[i].gnssID << ", " << _satelliteData.infoList[i].satelliteID << ", " << _satelliteData.infoList[i].elevation << ", " << _satelliteData.infoList[i].azimuth << ", " << _satelliteData.infoList[i].signalCNO <<std::endl;
    }
    _dirtyMessageTypes |= MSG_SATELLITE;
}

void UbloxGPS::requestStatusPoll()
{ sendMessage(ublox::message::NavStatusPoll<OutMessage>()); }

void UbloxGPS::handle(InNavStatus& msg)
{
    _statusData.timeOfWeekMS = comms::units::getMilliseconds<double>(msg.field_itow());
    _statusData.timeToFirstFix = comms::units::getMilliseconds<double>(msg.field_ttff());
    _statusData.timeSinceStart = comms::units::getMilliseconds<double>(msg.field_msss());
    _statusData.fixMode = (StatusData::FixMode)msg.field_gpsFix().value();
    _statusData.gpsFixValid = msg.field_flags().getBitValue_gpsFixOk();
    _statusData.diffCorrectionValid = msg.field_flags().getBitValue_diffSoln();
    _dirtyMessageTypes |= MSG_STATUS;
}

void UbloxGPS::handle(InMessage& msg)
{
    static_cast<void>(msg); // ignore
}

void UbloxGPS::sendMessage(const UbloxGPS::OutMessage& msg)
{
    OutBuffer buf;
    buf.reserve(_frame.length(msg));
    
    auto iter = std::back_inserter(buf);
    auto es = _frame.write(msg, iter, buf.max_size());
    if (es == comms::ErrorStatus::UpdateRequired)
    {
        auto* updateIter = &buf[0];
        es = _frame.update(updateIter, buf.size());
    }
    
    if (_uart && es == comms::ErrorStatus::Success)
        _uart->writeFromBuffer(reinterpret_cast<const char*>(&buf[0]), buf.size());
}

void UbloxGPS::configureUbxOutput()
{
    ublox::message::CfgPrtUsb<OutMessage> msg;
    auto& outProtoMaskField = msg.field_outProtoMask();
    auto& inProtoMaskField = msg.field_inProtoMask();

    using OutProtoMaskField = typename std::decay<decltype(outProtoMaskField)>::type;
    outProtoMaskField.setBitValue(OutProtoMaskField::BitIdx_outUbx, true);
    outProtoMaskField.setBitValue(OutProtoMaskField::BitIdx_outNmea, false);

    using InProtoMaskField = typename std::decay<decltype(inProtoMaskField)>::type;
    inProtoMaskField.setBitValue(InProtoMaskField::BitIdx_inUbx, true);
    inProtoMaskField.setBitValue(InProtoMaskField::BitIdx_inNmea, false);
    sendMessage(msg);
}
