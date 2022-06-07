#include "USB-PD.h"

PD_Engine pd;

void handleMessageCB(PD::Message msg)
{
    pd.handleMessage(msg);
}

void registerMessageCB(TcpmDriver *tcpm)
{
    tcpm->setReceivedMessageCB(&handleMessageCB);
}

uint8_t PD::parseHeaderMessageType(uint16_t header)
{
    return header & 0xF;
}

void PD_Engine::begin(TcpmDriver *tcpm)
{
    this->tcpm = tcpm;
    tcpm->begin();
    registerMessageCB(tcpm);
}

void PD_Engine::reset()
{
    this->tcpm->reset();
}

void PD_Engine::configureAsSource()
{
    this->tcpm->setIsSource(true);

    //is this needed?
    //Seems to prevent bugs sometimes
    //tcpm->resetPD();
}

void PD_Engine::configureAsSink()
{
    tcpm->setIsSource(false);
}

PD_Engine::Error PD_Engine::requestIdentity(PD::Destination dest, void (*cb)(PD::Identity))
{
    this->identityCB = cb;

    // //TODO use TCPM_DETECTION_RA
    // // if (!(tcpm->getConnection() & (1 << 3)))
    // // {
    // //     return PD_Engine::Error::DISCONNECT;
    // // }

    uint32_t VDM_header = 0;
    VDM_header |= PD::VDM::DISCOVER_IDENTITY;
    VDM_header |= PD::VDM::COMMAND_TYPE_REQ;
    VDM_header |= PD::VDM::STRUCTURED;
    VDM_header |= PD::VDM::STRUCTURED_VERSION;
    VDM_header |= PD::VDM::POWER_DELIVERY_SVID;

    uint16_t header = 0;

    //Message type: VDM
    header |= 0x0F;
    //Revision
    header |= 1 << 6;
    //Cable plug role
    header |= 0 << 8;
    //Message id
    header |= (this->tcpm->getMessageID() & 0x07) << 9;
    //Number of data objects
    header |= 1 << 12;

    this->tcpm->sendMessage(header, &VDM_header, dest);

    return PD_Engine::Error::NONE;
}

PD_Engine::Error PD_Engine::requestSourceCap(void (*cb)(PD::Capabilities))
{
    this->capabilitiesCB = cb;
}

PD_Engine::Error PD_Engine::requestSinkCap(void (*cb)(PD::Capabilities))
{
    uint16_t header = 0;

    //Message type: Get_Source_Cap
    header |= 0x7;
    //Revision
    header |= 1 << 6;
    //Cable plug role
    header |= 0 << 8;
    //Message id
    header |= (this->tcpm->getMessageID() & 0x07) << 9;
    //Number of data objects
    header |= 0 << 12;

    // Serial.print("Message id: ");
    // Serial.println((this->messageID & 0x07));

    this->tcpm->sendMessage(header, 0, PD::Destination::SOP);
}

void PD_Engine::registerCapCB(void (*cb)(PD::Capabilities))
{
    this->capabilitiesCB = cb;
}

void PD_Engine::sendSourceCap(PD::Capabilities cap)
{
    uint16_t header = 0;

    //Message type: Source_Capabilities
    header |= 0x1;
    //Revision
    header |= 1 << 6;
    //Port Power Role: Source
    header |= 1 << 8;
    //Message id
    header |= (this->tcpm->getMessageID() & 0x07) << 9;
    //Number of data objects
    header |= cap.length << 12;

    uint32_t body[7];

    for(int i = 0; i < cap.length; i++)
    {
        PD::PDO::PDO data = cap.dataObjects[i];
        if(data.type == PD::PDO::SOURCE_FIXED)
        {
            body[i] |= 0x0 << 30;
            body[i] |= data.sourceFixed.dualRolePower << 29;
            body[i] |= data.sourceFixed.suspend << 28;
            body[i] |= data.sourceFixed.unconstrained << 27;
            body[i] |= data.sourceFixed.commCapable << 26;
            body[i] |= data.sourceFixed.dualRoleData << 25;

            body[i] |= data.sourceFixed.peakCurrent << 20;
            body[i] |= (data.sourceFixed.voltage / 50) << 10;
            body[i] |= (data.sourceFixed.current / 10) << 0;
        }
    }

    this->tcpm->sendMessage(header, body, PD::Destination::SOP);
}

void PD_Engine::requestPower(int object_position)
{
    uint16_t header = 0;

    //Message type: Request
    header |= 0x2;
    //Revision
    header |= 1 << 6;
    //Port Power Role: Sink
    header |= 0 << 8;
    //Message id
    header |= (this->tcpm->getMessageID() & 0x07) << 9;
    //Number of data objects: 1
    header |= 1 << 12;

    uint32_t body;

    body |= object_position+1 << 28;

    // Operating current in 10mA units
    body |= (this->last_cap_recieved.dataObjects[object_position].sourceFixed.current / 10) << 10;
    // Maximum Operating Current 10mA units, see 6.4.2
    body |= (this->last_cap_recieved.dataObjects[object_position].sourceFixed.current / 10) << 0;

    this->tcpm->sendMessage(header, &body, PD::Destination::SOP);
}

void PD_Engine::handleMessage(PD::Message msg)
{
    // this->capabilitiesCB(msg.dataMessage.sourceCap);
    if (msg.type == PD::MessageType::DATA)
    {
        if (msg.dataMessage.type == PD::DataMessageType::VDM_TYPE)
        {
            if (msg.dataMessage.vdm.type == PD::VdmType::IDENTITY)
            {
                this->identityCB(msg.dataMessage.vdm.id);
            }
        }
        else if (msg.dataMessage.type == PD::DataMessageType::SOURCE_CAP)
        {
            this->last_cap_recieved = msg.dataMessage.sourceCap;
            this->capabilitiesCB(msg.dataMessage.sourceCap);
        }
        else if (msg.dataMessage.type == PD::DataMessageType::SINK_CAP)
        {
            this->capabilitiesCB(msg.dataMessage.sinkCap);
        }
    }
}