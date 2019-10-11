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

using namespace wincomm;

static void threadFunction(void* userData)
{
    UbloxGPS* gps = (UbloxGPS*)userData;
    gps->requestPositionPoll();
}

UbloxGPS::~UbloxGPS()
{
    _timer.stop();
}

void UbloxGPS::start(Uart* uart, int reqPositionMS)
{
    _uart = uart;
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

    _dataBuffer.erase(_dataBuffer.begin(), _dataBuffer.begin() + consumed);
    return msgDispatched > 0;
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
    _satelliteData.timeOfWeekMS = comms::units::getMilliseconds<double>(msg.field_itow());
    
    for (size_t i = 0; i < satList.size(); ++i)
    {
        auto element = satList.at(i);
        _satelliteData.infoList[i].gnssID = ublox::field::GnssId<>::valueName(element.field_gnssId().value());
        _satelliteData.infoList[i].satelliteID = element.field_svid().value();
        _satelliteData.infoList[i].elevation = comms::units::getDegrees<double>(element.field_elev());
        _satelliteData.infoList[i].azimuth = comms::units::getDegrees<double>(element.field_azim());
        _satelliteData.infoList[i].signalCNO = element.field_cno().value();
    }
    _dirtyMessageTypes |= MSG_SATELLITE;
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
