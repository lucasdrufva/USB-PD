#pragma once
#include <USB-PD.h>

//TODO remove
//#include "oldtcpm.h"

namespace PD
{

} // namespace PD

class TcpmDriver
{
public:
    virtual void begin() = 0;

    virtual void reset() = 0;

    virtual void sendMessage(uint16_t header, const uint32_t *data, PD::Destination destination) = 0;

    virtual int getMessageID() = 0;

    virtual void setReceivedMessageCB(void (*cb)(PD::Message)) = 0;

    virtual void setIsSource(bool isSource) = 0;

    // //TODO change return value to more readable
    virtual int getConnection() = 0;
};