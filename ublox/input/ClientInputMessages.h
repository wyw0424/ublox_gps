/// @file
/// @brief Contains definition of all client input messages bundle.

#pragma once

#include <tuple>
#include "ublox/message/AckAck.h"
#include "ublox/message/AckNak.h"
#include "ublox/message/AidAlm.h"
#include "ublox/message/AidAlpStatus.h"
#include "ublox/message/AidAlpsrv.h"
#include "ublox/message/AidAlpsrvToServer.h"
#include "ublox/message/AidAop.h"
#include "ublox/message/AidAopU8.h"
#include "ublox/message/AidEph.h"
#include "ublox/message/AidHui.h"
#include "ublox/message/AidIni.h"
#include "ublox/message/CfgAnt.h"
#include "ublox/message/CfgDat.h"
#include "ublox/message/CfgDgnss.h"
#include "ublox/message/CfgDosc.h"
#include "ublox/message/CfgEkf.h"
#include "ublox/message/CfgEsrc.h"
#include "ublox/message/CfgFxn.h"
#include "ublox/message/CfgGeofence.h"
#include "ublox/message/CfgGnss.h"
#include "ublox/message/CfgHnr.h"
#include "ublox/message/CfgInf.h"
#include "ublox/message/CfgItfm.h"
#include "ublox/message/CfgLogfilter.h"
#include "ublox/message/CfgMsg.h"
#include "ublox/message/CfgMsgCurrent.h"
#include "ublox/message/CfgNav5.h"
#include "ublox/message/CfgNavx5.h"
#include "ublox/message/CfgNavx5V2.h"
#include "ublox/message/CfgNavx5V3.h"
#include "ublox/message/CfgNmea.h"
#include "ublox/message/CfgNmeaV0.h"
#include "ublox/message/CfgNmeaV1.h"
#include "ublox/message/CfgOdo.h"
#include "ublox/message/CfgPm.h"
#include "ublox/message/CfgPm2.h"
#include "ublox/message/CfgPm2V2.h"
#include "ublox/message/CfgPms.h"
#include "ublox/message/CfgPrtDdc.h"
#include "ublox/message/CfgPrtSpi.h"
#include "ublox/message/CfgPrtUart.h"
#include "ublox/message/CfgPrtUsb.h"
#include "ublox/message/CfgRate.h"
#include "ublox/message/CfgRinv.h"
#include "ublox/message/CfgRxm.h"
#include "ublox/message/CfgSbas.h"
#include "ublox/message/CfgSmgr.h"
#include "ublox/message/CfgTmode.h"
#include "ublox/message/CfgTmode2.h"
#include "ublox/message/CfgTmode3.h"
#include "ublox/message/CfgTp.h"
#include "ublox/message/CfgTp5.h"
#include "ublox/message/CfgUsb.h"
#include "ublox/message/EsfIns.h"
#include "ublox/message/EsfMeas.h"
#include "ublox/message/EsfRaw.h"
#include "ublox/message/EsfStatus.h"
#include "ublox/message/HnrPvt.h"
#include "ublox/message/InfDebug.h"
#include "ublox/message/InfError.h"
#include "ublox/message/InfNotice.h"
#include "ublox/message/InfTest.h"
#include "ublox/message/InfWarning.h"
#include "ublox/message/LogFindtimeResp.h"
#include "ublox/message/LogInfo.h"
#include "ublox/message/LogRetrievepos.h"
#include "ublox/message/LogRetrieveposextra.h"
#include "ublox/message/LogRetrievestring.h"
#include "ublox/message/MgaAck.h"
#include "ublox/message/MgaDbd.h"
#include "ublox/message/MgaFlashAck.h"
#include "ublox/message/MonGnss.h"
#include "ublox/message/MonHw.h"
#include "ublox/message/MonHw2.h"
#include "ublox/message/MonIo.h"
#include "ublox/message/MonMsgpp.h"
#include "ublox/message/MonPatch.h"
#include "ublox/message/MonRxbuf.h"
#include "ublox/message/MonRxr.h"
#include "ublox/message/MonSmgr.h"
#include "ublox/message/MonTxbuf.h"
#include "ublox/message/MonVer.h"
#include "ublox/message/NavAopstatus.h"
#include "ublox/message/NavAopstatusUblox8.h"
#include "ublox/message/NavAtt.h"
#include "ublox/message/NavClock.h"
#include "ublox/message/NavDgps.h"
#include "ublox/message/NavDop.h"
#include "ublox/message/NavEoe.h"
#include "ublox/message/NavGeofence.h"
#include "ublox/message/NavHpposecef.h"
#include "ublox/message/NavHpposllh.h"
#include "ublox/message/NavOdo.h"
#include "ublox/message/NavOrb.h"
#include "ublox/message/NavPosecef.h"
#include "ublox/message/NavPosllh.h"
#include "ublox/message/NavPvt.h"
#include "ublox/message/NavPvt_u8.h"
#include "ublox/message/NavRelposned.h"
#include "ublox/message/NavSat.h"
#include "ublox/message/NavSbas.h"
#include "ublox/message/NavSol.h"
#include "ublox/message/NavStatus.h"
#include "ublox/message/NavSvin.h"
#include "ublox/message/NavSvinfo.h"
#include "ublox/message/NavTimebds.h"
#include "ublox/message/NavTimegal.h"
#include "ublox/message/NavTimeglo.h"
#include "ublox/message/NavTimegps.h"
#include "ublox/message/NavTimels.h"
#include "ublox/message/NavTimeutc.h"
#include "ublox/message/NavVelecef.h"
#include "ublox/message/NavVelned.h"
#include "ublox/message/RxmAlm.h"
#include "ublox/message/RxmEph.h"
#include "ublox/message/RxmImes.h"
#include "ublox/message/RxmMeasx.h"
#include "ublox/message/RxmRaw.h"
#include "ublox/message/RxmRawx.h"
#include "ublox/message/RxmRlmLong.h"
#include "ublox/message/RxmRlmShort.h"
#include "ublox/message/RxmRtcm.h"
#include "ublox/message/RxmSfrb.h"
#include "ublox/message/RxmSfrbx.h"
#include "ublox/message/RxmSvsi.h"
#include "ublox/message/SecSign.h"
#include "ublox/message/SecUniqid.h"
#include "ublox/message/TimDosc.h"
#include "ublox/message/TimFchg.h"
#include "ublox/message/TimSmeas.h"
#include "ublox/message/TimSvin.h"
#include "ublox/message/TimTm2.h"
#include "ublox/message/TimTos.h"
#include "ublox/message/TimTp.h"
#include "ublox/message/TimVcocal.h"
#include "ublox/message/TimVrfy.h"
#include "ublox/message/UpdSosAck.h"
#include "ublox/message/UpdSosRestored.h"
#include "ublox/options/DefaultOptions.h"

namespace ublox
{

namespace input
{

/// @brief Messages of the protocol in ascending order.
/// @tparam TBase Base class of all the messages.
/// @tparam TOpt Protocol definition options.
template <typename TBase, typename TOpt = ublox::options::DefaultOptions>
using ClientInputMessages =
    std::tuple<
        ublox::message::NavPosecef<TBase, TOpt>,
        ublox::message::NavPosllh<TBase, TOpt>,
        ublox::message::NavStatus<TBase, TOpt>,
        ublox::message::NavDop<TBase, TOpt>,
        ublox::message::NavAtt<TBase, TOpt>,
        ublox::message::NavSol<TBase, TOpt>,
        ublox::message::NavPvt_u8<TBase, TOpt>,
        ublox::message::NavPvt<TBase, TOpt>,
        ublox::message::NavOdo<TBase, TOpt>,
        ublox::message::NavVelecef<TBase, TOpt>,
        ublox::message::NavVelned<TBase, TOpt>,
        ublox::message::NavHpposecef<TBase, TOpt>,
        ublox::message::NavHpposllh<TBase, TOpt>,
        ublox::message::NavTimegps<TBase, TOpt>,
        ublox::message::NavTimeutc<TBase, TOpt>,
        ublox::message::NavClock<TBase, TOpt>,
        ublox::message::NavTimeglo<TBase, TOpt>,
        ublox::message::NavTimebds<TBase, TOpt>,
        ublox::message::NavTimegal<TBase, TOpt>,
        ublox::message::NavTimels<TBase, TOpt>,
        ublox::message::NavSvinfo<TBase, TOpt>,
        ublox::message::NavDgps<TBase, TOpt>,
        ublox::message::NavSbas<TBase, TOpt>,
        ublox::message::NavOrb<TBase, TOpt>,
        ublox::message::NavSat<TBase, TOpt>,
        ublox::message::NavGeofence<TBase, TOpt>,
        ublox::message::NavSvin<TBase, TOpt>,
        ublox::message::NavRelposned<TBase, TOpt>,
        ublox::message::NavAopstatus<TBase, TOpt>,
        ublox::message::NavAopstatusUblox8<TBase, TOpt>,
        ublox::message::NavEoe<TBase, TOpt>,
        ublox::message::RxmRaw<TBase, TOpt>,
        ublox::message::RxmSfrb<TBase, TOpt>,
        ublox::message::RxmSfrbx<TBase, TOpt>,
        ublox::message::RxmMeasx<TBase, TOpt>,
        ublox::message::RxmRawx<TBase, TOpt>,
        ublox::message::RxmSvsi<TBase, TOpt>,
        ublox::message::RxmAlm<TBase, TOpt>,
        ublox::message::RxmEph<TBase, TOpt>,
        ublox::message::RxmRtcm<TBase, TOpt>,
        ublox::message::RxmRlmLong<TBase, TOpt>,
        ublox::message::RxmRlmShort<TBase, TOpt>,
        ublox::message::RxmImes<TBase, TOpt>,
        ublox::message::InfError<TBase, TOpt>,
        ublox::message::InfWarning<TBase, TOpt>,
        ublox::message::InfNotice<TBase, TOpt>,
        ublox::message::InfTest<TBase, TOpt>,
        ublox::message::InfDebug<TBase, TOpt>,
        ublox::message::AckNak<TBase, TOpt>,
        ublox::message::AckAck<TBase, TOpt>,
        ublox::message::CfgPrtDdc<TBase, TOpt>,
        ublox::message::CfgPrtUart<TBase, TOpt>,
        ublox::message::CfgPrtUsb<TBase, TOpt>,
        ublox::message::CfgPrtSpi<TBase, TOpt>,
        ublox::message::CfgMsg<TBase, TOpt>,
        ublox::message::CfgMsgCurrent<TBase, TOpt>,
        ublox::message::CfgInf<TBase, TOpt>,
        ublox::message::CfgDat<TBase, TOpt>,
        ublox::message::CfgTp<TBase, TOpt>,
        ublox::message::CfgRate<TBase, TOpt>,
        ublox::message::CfgFxn<TBase, TOpt>,
        ublox::message::CfgRxm<TBase, TOpt>,
        ublox::message::CfgEkf<TBase, TOpt>,
        ublox::message::CfgAnt<TBase, TOpt>,
        ublox::message::CfgSbas<TBase, TOpt>,
        ublox::message::CfgNmeaV1<TBase, TOpt>,
        ublox::message::CfgNmeaV0<TBase, TOpt>,
        ublox::message::CfgNmea<TBase, TOpt>,
        ublox::message::CfgUsb<TBase, TOpt>,
        ublox::message::CfgTmode<TBase, TOpt>,
        ublox::message::CfgOdo<TBase, TOpt>,
        ublox::message::CfgNavx5V3<TBase, TOpt>,
        ublox::message::CfgNavx5V2<TBase, TOpt>,
        ublox::message::CfgNavx5<TBase, TOpt>,
        ublox::message::CfgNav5<TBase, TOpt>,
        ublox::message::CfgTp5<TBase, TOpt>,
        ublox::message::CfgPm<TBase, TOpt>,
        ublox::message::CfgRinv<TBase, TOpt>,
        ublox::message::CfgItfm<TBase, TOpt>,
        ublox::message::CfgPm2V2<TBase, TOpt>,
        ublox::message::CfgPm2<TBase, TOpt>,
        ublox::message::CfgTmode2<TBase, TOpt>,
        ublox::message::CfgGnss<TBase, TOpt>,
        ublox::message::CfgLogfilter<TBase, TOpt>,
        ublox::message::CfgHnr<TBase, TOpt>,
        ublox::message::CfgEsrc<TBase, TOpt>,
        ublox::message::CfgDosc<TBase, TOpt>,
        ublox::message::CfgSmgr<TBase, TOpt>,
        ublox::message::CfgGeofence<TBase, TOpt>,
        ublox::message::CfgDgnss<TBase, TOpt>,
        ublox::message::CfgTmode3<TBase, TOpt>,
        ublox::message::CfgPms<TBase, TOpt>,
        ublox::message::UpdSosRestored<TBase, TOpt>,
        ublox::message::UpdSosAck<TBase, TOpt>,
        ublox::message::MonIo<TBase, TOpt>,
        ublox::message::MonVer<TBase, TOpt>,
        ublox::message::MonMsgpp<TBase, TOpt>,
        ublox::message::MonRxbuf<TBase, TOpt>,
        ublox::message::MonTxbuf<TBase, TOpt>,
        ublox::message::MonHw<TBase, TOpt>,
        ublox::message::MonHw2<TBase, TOpt>,
        ublox::message::MonRxr<TBase, TOpt>,
        ublox::message::MonPatch<TBase, TOpt>,
        ublox::message::MonGnss<TBase, TOpt>,
        ublox::message::MonSmgr<TBase, TOpt>,
        ublox::message::AidIni<TBase, TOpt>,
        ublox::message::AidHui<TBase, TOpt>,
        ublox::message::AidAlm<TBase, TOpt>,
        ublox::message::AidEph<TBase, TOpt>,
        ublox::message::AidAlpsrv<TBase, TOpt>,
        ublox::message::AidAlpsrvToServer<TBase, TOpt>,
        ublox::message::AidAopU8<TBase, TOpt>,
        ublox::message::AidAop<TBase, TOpt>,
        ublox::message::AidAlpStatus<TBase, TOpt>,
        ublox::message::TimTp<TBase, TOpt>,
        ublox::message::TimTm2<TBase, TOpt>,
        ublox::message::TimVrfy<TBase, TOpt>,
        ublox::message::TimSvin<TBase, TOpt>,
        ublox::message::TimDosc<TBase, TOpt>,
        ublox::message::TimTos<TBase, TOpt>,
        ublox::message::TimSmeas<TBase, TOpt>,
        ublox::message::TimVcocal<TBase, TOpt>,
        ublox::message::TimFchg<TBase, TOpt>,
        ublox::message::EsfMeas<TBase, TOpt>,
        ublox::message::EsfRaw<TBase, TOpt>,
        ublox::message::EsfStatus<TBase, TOpt>,
        ublox::message::EsfIns<TBase, TOpt>,
        ublox::message::MgaFlashAck<TBase, TOpt>,
        ublox::message::MgaAck<TBase, TOpt>,
        ublox::message::MgaDbd<TBase, TOpt>,
        ublox::message::LogInfo<TBase, TOpt>,
        ublox::message::LogRetrievepos<TBase, TOpt>,
        ublox::message::LogRetrievestring<TBase, TOpt>,
        ublox::message::LogFindtimeResp<TBase, TOpt>,
        ublox::message::LogRetrieveposextra<TBase, TOpt>,
        ublox::message::SecSign<TBase, TOpt>,
        ublox::message::SecUniqid<TBase, TOpt>,
        ublox::message::HnrPvt<TBase, TOpt>
    >;

} // namespace input

} // namespace ublox


