/// @file
/// @brief Contains definition of <b>"NAV-VELNED (Poll)"</b> message and its fields.

#pragma once

#include <tuple>
#include "comms/MessageBase.h"
#include "ublox/MsgId.h"
#include "ublox/options/DefaultOptions.h"

namespace ublox
{

namespace message
{

/// @brief Fields of @ref NavVelnedPoll.
/// @tparam TOpt Extra options
/// @see @ref NavVelnedPoll
/// @headerfile "ublox/message/NavVelnedPoll.h"
template <typename TOpt = ublox::options::DefaultOptions>
struct NavVelnedPollFields
{
    /// @brief All the fields bundled in std::tuple.
    using All = std::tuple<
    >;
};

/// @brief Definition of <b>"NAV-VELNED (Poll)"</b> message class.
/// @details
///     See @ref NavVelnedPollFields for definition of the fields this message contains.
/// @tparam TMsgBase Base (interface) class.
/// @tparam TOpt Extra options
/// @headerfile "ublox/message/NavVelnedPoll.h"
template <typename TMsgBase, typename TOpt = ublox::options::DefaultOptions>
class NavVelnedPoll : public
    comms::MessageBase<
        TMsgBase,
        typename TOpt::message::NavVelnedPoll,
        comms::option::def::StaticNumIdImpl<ublox::MsgId_NavVelned>,
        comms::option::def::FieldsImpl<typename NavVelnedPollFields<TOpt>::All>,
        comms::option::def::MsgType<NavVelnedPoll<TMsgBase, TOpt> >,
        comms::option::def::HasName
    >
{
    // Redefinition of the base class type
    using Base =
        comms::MessageBase<
            TMsgBase,
            typename TOpt::message::NavVelnedPoll,
            comms::option::def::StaticNumIdImpl<ublox::MsgId_NavVelned>,
            comms::option::def::FieldsImpl<typename NavVelnedPollFields<TOpt>::All>,
            comms::option::def::MsgType<NavVelnedPoll<TMsgBase, TOpt> >,
            comms::option::def::HasName
        >;

public:
    // Compile time check for serialisation length.
    static const std::size_t MsgMinLen = Base::doMinLength();
    static const std::size_t MsgMaxLen = Base::doMaxLength();
    static_assert(MsgMinLen == 0U, "Unexpected min serialisation length");
    static_assert(MsgMaxLen == 0U, "Unexpected max serialisation length");
    
    /// @brief Name of the message.
    static const char* doName()
    {
        return "NAV-VELNED (Poll)";
    }
    
    
};

} // namespace message

} // namespace ublox

