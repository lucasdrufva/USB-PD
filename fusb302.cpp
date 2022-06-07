#include "fusb302.h"

// //TODO: remove
// #include "display.h"
// extern Display display;

void FUSB302::begin()
{
    //Reset device and register values to default
    this->i2c_write(FUSB_RESET, FUSB_RESET_SW_RES);

    //Enable auto retries and set amount to 3
    this->i2c_set_bits(FUSB_CONTROL3, FUSB_CONTROL3_AUTO_RETRY | FUSB_CONTROL3_RETRIES(3));

    this->interrupt_init();

    //Enable sop' packets
    this->i2c_set_bits(FUSB_CONTROL1, FUSB_CONTROL1_ENSOP1);

    //Turn on power for all internals
    //TODO: save power and start in lower power mode
    this->i2c_write(FUSB_POWER, FUSB_POWER_ALL);

    // //Reset pd
    this->i2c_write(0x0C, 1 << 1);

    //TODO: remove as this is only fot testing
    //Configure device as source
    //this->set_pull(1);

    this->interrupt_clear();

    uint8_t data;
}

/*
 * initialize interrupt on the chip
 * - unmasked interrupts: TXSENT, GCRCSENT
 */
void FUSB302::interrupt_init()
{
    //Enable interupts
    this->i2c_clear_bits(FUSB_CONTROL0, FUSB_CONTROL0_INT_MASK);

    this->i2c_write(FUSB_MASKA, 0xFF & ~FUSB_MASKA_TXSENT);
    this->i2c_write(FUSB_MASKB, 0xFF & ~FUSB_MASKB_GCRCSENT);
    this->i2c_write(FUSB_MASK, 0xFF);
}

//0 = down, 1 = up
void FUSB302::set_pull(int direction)
{
    //down
    if (direction == 0)
    {
        //disable pull up
        this->i2c_clear_bits(FUSB_SWITCHES0, FUSB_SWITCHES0_PU_EN1 | FUSB_SWITCHES0_PU_EN2);
        //enable pull down
        this->i2c_set_bits(FUSB_SWITCHES0, FUSB_SWITCHES0_PDWN1 | FUSB_SWITCHES0_PDWN1);
    }
    else if (direction == 1)
    {
        //disable pull down
        this->i2c_clear_bits(FUSB_SWITCHES0, FUSB_SWITCHES0_PDWN1 | FUSB_SWITCHES0_PDWN2);
        //enable pull up
        this->i2c_set_bits(FUSB_SWITCHES0, FUSB_SWITCHES0_PU_EN1 | FUSB_SWITCHES0_PU_EN2);
    }
}

// cc1 = 0, cc2 = 1
int FUSB302::measure_cc_pin_source(int pin)
{
    //Save old state to return to after meassuring
    uint8_t oldSwitchesState;
    this->i2c_read(FUSB_SWITCHES0, &oldSwitchesState);

    uint8_t reg;
    int cc_lvl;

    this->i2c_clear_bits(FUSB_SWITCHES0, 0xFF);

    this->set_pull(1);

    this->i2c_set_bits(FUSB_SWITCHES0, pin == 0 ? FUSB_SWITCHES0_MEAS_CC1 : FUSB_SWITCHES0_MEAS_CC2);

    /* Set MDAC for Open vs Rd/Ra comparison */
    //fusbWrite(0x04, (((1600)/42) & 0x3f));
    this->i2c_write(0x04, 60 & 0x3f);

    /* Wait on measurement */
    delay(1);

    /* Read status register */
    this->i2c_read(0x40, &reg);

    /* Assume open */
    cc_lvl = 0;

    /* CC level is below the 'no connect' threshold (vOpen) */
    if ((reg & 1 << 5) == 0)
    {
        /* Set MDAC for Rd vs Ra comparison */
        this->i2c_write(0x04, (((200) / 42) & 0x3f));
        //fusbWrite(0x04, 0x05);

        /* Wait on measurement */
        delay(1);

        /* Read status register */
        this->i2c_read(0x40, &reg);

        cc_lvl = (reg & 1 << 5) ? 2
                                : 1;
    }

    /* Restore SWITCHES0 register to its value prior */
    this->i2c_write(0x02, oldSwitchesState);

    Serial.print("Measured this on");
    Serial.print(pin);
    Serial.print(" : ");
    Serial.println(cc_lvl);

    return cc_lvl;
}

// cc1 = 0, cc2 = 1
int FUSB302::measure_cc_pin_sink(int pin)
{
    //Save old state to return to after meassuring
    uint8_t oldSwitchesState;
    this->i2c_read(FUSB_SWITCHES0, &oldSwitchesState);

    uint8_t reg;
    int cc_lvl;

    this->i2c_clear_bits(FUSB_SWITCHES0, 0xFF);

    this->i2c_set_bits(FUSB_SWITCHES0, pin == 0 ? FUSB_SWITCHES0_MEAS_CC1 : FUSB_SWITCHES0_MEAS_CC2);

    /* Wait on measurement */
    delay(1);

    /* Read status register */
    this->i2c_read(0x40, &reg);

    cc_lvl = reg & 0x2;

    /* Restore SWITCHES0 register to its value prior */
    this->i2c_write(0x02, oldSwitchesState);

    Serial.print("Measured this on");
    Serial.print(pin);
    Serial.print(" : ");
    Serial.println(cc_lvl);

    return cc_lvl;
}

void FUSB302::i2c_write(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(FUSB_I2C_SLAVE_ADDRESS);
    Wire.write(reg & 0xFF);
    Wire.write(val & 0xFF);
    Wire.endTransmission();
}

void FUSB302::i2c_mask_write(uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t data;

    this->i2c_read(reg, &data);
    data &= ~mask;
    data |= val;
    this->i2c_write(reg, data);
}

void FUSB302::i2c_set_bits(uint8_t reg, uint8_t bits)
{
    this->i2c_mask_write(reg, bits, 0x00);
}

void FUSB302::i2c_clear_bits(uint8_t reg, uint8_t bits)
{
    this->i2c_mask_write(reg, 0x00, bits);
}

void FUSB302::i2c_read(uint8_t reg, uint8_t *val)
{
    Wire.beginTransmission(FUSB_I2C_SLAVE_ADDRESS);
    Wire.write(reg & 0xFF);
    Wire.endTransmission(false);
    Wire.requestFrom(FUSB_I2C_SLAVE_ADDRESS, 1, true);
    *val = Wire.read();
}

void FUSB302::i2c_transfer(const uint8_t *out, int out_size,
                           uint8_t *in, int in_size, bool stop)
{
    Wire.beginTransmission(FUSB_I2C_SLAVE_ADDRESS);
    for (; out_size > 0; out_size--)
    {
        Wire.write(*out);
        out++;
    }

    if (in_size)
    {
        Wire.endTransmission(false);
        Wire.requestFrom(FUSB_I2C_SLAVE_ADDRESS, in_size, stop);
        for (; in_size > 0; in_size--)
        {
            *in = Wire.read();
            in++;
        }
    }
    else
    {
        Wire.endTransmission(stop);
    }
}

void FUSB302::softReset()
{
    this->i2c_write(FUSB_RESET, FUSB_RESET_SW_RES);
    messageID = 0;
}

void FUSB302::pdReset()
{
    this->i2c_write(0x0C, 1 << 1);
    messageID = 0;
}

void FUSB302::reset()
{
    this->pdReset();
}

uint8_t FUSB302::checkVConn(){
    uint8_t data;
    this->i2c_read(FUSB_SWITCHES0, &data);
    return data & (FUSB_SWITCHES0_VCONN_CC1 | FUSB_SWITCHES0_VCONN_CC2);
}


void FUSB302::detectConnectionAndOrientation()
{
    this->connection = 0;

    if (this->isSource)
    {
        int cc1_lvl = this->measure_cc_pin_source(0);

        if (cc1_lvl == CC_VOLT_RA)
        {
            connection |= TCPM_DETECTION_RA + TCPM_DETECTION_POLARITY(1);
        }
        else if (cc1_lvl == CC_VOLT_RD)
        {
            connection |= TCPM_DETECTION_RD + TCPM_DETECTION_POLARITY(0);
        }

        int cc2_lvl = this->measure_cc_pin_source(1);

        if (cc2_lvl == CC_VOLT_RA)
        {
            connection |= TCPM_DETECTION_RA + TCPM_DETECTION_POLARITY(0);
        }
        else if (cc2_lvl == CC_VOLT_RD)
        {
            connection |= TCPM_DETECTION_RD + TCPM_DETECTION_POLARITY(1);
        }
    }
    else
    {
        //is sink
        int cc1_lvl = this->measure_cc_pin_sink(0);

        if (cc1_lvl != 0)
        {
            connection |= TCPM_DETECTION_RP + TCPM_DETECTION_POLARITY(0);
        }

        int cc2_lvl = this->measure_cc_pin_sink(1);

        if (cc2_lvl != 0)
        {
            connection |= TCPM_DETECTION_RP + TCPM_DETECTION_POLARITY(1);
        }
    }

    if (connection != 0)
    {
        this->set_polarity(connection & TCPM_DETECTION_POLARITY(1));
        this->enable_tx();
    }
}

int FUSB302::getConnection()
{
    this->detectConnectionAndOrientation();
    return this->connection;
}

void FUSB302::requestIdentity(int destination)
{
    uint32_t VDM_header = 0;
    VDM_header |= PD_DISCOVER_IDENTITY;
    VDM_header |= PD_COMMAND_TYPE_REQ;
    VDM_header |= PD_STRUCTURED_VDM;
    VDM_header |= PD_STRUCTURED_VDM_VERSION;
    VDM_header |= PD_POWER_DELIVERY_SVID;

    uint16_t header = 0;

    //Message type: VDM
    header |= 0x0F;
    //Revision
    header |= 1 << 6;
    //Cable plug role
    header |= 0 << 8;
    //Message id
    header |= (this->messageID & 0x07) << 9;
    //Number of data objects
    header |= 1 << 12;

    this->send_message(header, &VDM_header, destination);
}

void FUSB302::requestIdentity(int destination, void (*cb)(PD_ID))
{
    this->identity_cb = cb;
    this->requestIdentity(destination);
}

void FUSB302::sendPing(int destination)
{
    uint16_t header = 0;

    //Message type: Ping
    header |= 0x5;
    //Revision
    header |= 1 << 6;
    //Cable plug role
    header |= 0 << 8;
    //Message id
    header |= (this->messageID & 0x07) << 9;
    //Number of data objects
    header |= 0 << 12;

    Serial.println("Message id: ");
    Serial.println((this->messageID & 0x07));

    this->send_message(header, 0, destination);
}

void FUSB302::send_message(uint16_t header, const uint32_t *data, int destination)
{
    this->flush_tx();

    /*
     * this is the buffer that will be burst-written into the fusb302
     * maximum size necessary =
     * 1: FIFO register address
     * 4: SOP* tokens
     * 1: Token that signifies "next X bytes are not tokens"
     * 30: 2 for header and up to 7*4 = 28 for rest of message
     * 1: "Insert CRC" Token
     * 1: EOP Token
     * 1: "Turn transmitter off" token
     * 1: "Star Transmission" Command
     * -
     * 40: 40 bytes worst-case
     */
    uint8_t buf[40];
    int buf_pos = 0;

    int reg;
    int len;

    /* put register address first for of burst tcpc write */
    buf[buf_pos++] = FUSB_FIFOS;

    buf_pos = this->write_ordered_set(destination, buf, buf_pos);

    len = get_num_bytes(header);

    Serial.print("Sent header bytes len: ");
    Serial.println(len);

    /*
     * packsym tells the TXFIFO that the next X bytes are payload,
     * and should not be interpreted as special tokens.
     * The 5 LSBs represent X, the number of bytes.
     */
    reg = FUSB_TOKEN_PACKSYM;
    reg |= (len & 0x1F);

    buf[buf_pos++] = reg;

    /* write in the header */
    reg = header;
    buf[buf_pos++] = reg & 0xFF;

    reg >>= 8;
    buf[buf_pos++] = reg & 0xFF;

    /* header is done, subtract from length to make memcpy simpler */
    len -= 2;

    /* write data objects, if present */
    memcpy(&buf[buf_pos], data, len);
    buf_pos += len;

    /* put in the CRC */
    buf[buf_pos++] = FUSB_TOKEN_JAMCRC;

    /* put in EOP */
    buf[buf_pos++] = FUSB_TOKEN_EOP;

    /* Turn transmitter off after sending message */
    buf[buf_pos++] = FUSB_TOKEN_TXOFF;

    /* Start transmission */
    buf[buf_pos++] = FUSB_TOKEN_TXON;

    /* burst write for speed! */
    this->i2c_transfer(buf, buf_pos, 0, 0, true);

    Serial.println("Message sent...");
}

void FUSB302::sendMessage(uint16_t header, const uint32_t *data, PD::Destination destination)
{
    this->flush_tx();

    /*
     * this is the buffer that will be burst-written into the fusb302
     * maximum size necessary =
     * 1: FIFO register address
     * 4: SOP* tokens
     * 1: Token that signifies "next X bytes are not tokens"
     * 30: 2 for header and up to 7*4 = 28 for rest of message
     * 1: "Insert CRC" Token
     * 1: EOP Token
     * 1: "Turn transmitter off" token
     * 1: "Star Transmission" Command
     * -
     * 40: 40 bytes worst-case
     */
    uint8_t buf[40];
    int buf_pos = 0;

    int reg;
    int len;

    /* put register address first for of burst tcpc write */
    buf[buf_pos++] = FUSB_FIFOS;

    buf_pos = this->write_ordered_set(destination, buf, buf_pos);

    len = get_num_bytes(header);

    Serial.print("Sent header bytes len: ");
    Serial.println(len);

    /*
     * packsym tells the TXFIFO that the next X bytes are payload,
     * and should not be interpreted as special tokens.
     * The 5 LSBs represent X, the number of bytes.
     */
    reg = FUSB_TOKEN_PACKSYM;
    reg |= (len & 0x1F);

    buf[buf_pos++] = reg;

    /* write in the header */
    reg = header;
    buf[buf_pos++] = reg & 0xFF;

    reg >>= 8;
    buf[buf_pos++] = reg & 0xFF;

    /* header is done, subtract from length to make memcpy simpler */
    len -= 2;

    /* write data objects, if present */
    memcpy(&buf[buf_pos], data, len);
    buf_pos += len;

    /* put in the CRC */
    buf[buf_pos++] = FUSB_TOKEN_JAMCRC;

    /* put in EOP */
    buf[buf_pos++] = FUSB_TOKEN_EOP;

    /* Turn transmitter off after sending message */
    buf[buf_pos++] = FUSB_TOKEN_TXOFF;

    /* Start transmission */
    buf[buf_pos++] = FUSB_TOKEN_TXON;

    /* burst write for speed! */
    this->i2c_transfer(buf, buf_pos, 0, 0, true);

    Serial.println("Message sent...");
}

void FUSB302::flush_tx()
{
    this->i2c_set_bits(FUSB_CONTROL0, FUSB_CONTROL0_TX_FLUSH);
}

void FUSB302::flush_rx()
{
    this->i2c_set_bits(FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH);
}

int FUSB302::write_ordered_set(int destination, uint8_t *buf, int buf_pos)
{
    if (destination == FUSB_DESTINATION_SOP)
    {
        buf[buf_pos++] = FUSB_TOKEN_SYNC1;
        buf[buf_pos++] = FUSB_TOKEN_SYNC1;
        buf[buf_pos++] = FUSB_TOKEN_SYNC1;
        buf[buf_pos++] = FUSB_TOKEN_SYNC2;
    }
    else if (destination == FUSB_DESTINATION_SOP_PRIME)
    {
        buf[buf_pos++] = FUSB_TOKEN_SYNC1;
        buf[buf_pos++] = FUSB_TOKEN_SYNC1;
        buf[buf_pos++] = FUSB_TOKEN_SYNC3;
        buf[buf_pos++] = FUSB_TOKEN_SYNC3;
    }
    return buf_pos;
}

int FUSB302::get_num_bytes(uint16_t header)
{
    int rv;

    /* Grab the Number of Data Objects field.*/
    rv = (header >> 12) & 7;

    /* Multiply by four to go from 32-bit words -> bytes */
    rv *= 4;

    /* Plus 2 for header */
    rv += 2;

    return rv;
}

void FUSB302::handleInterrupt()
{
    uint8_t data;
    int rv = 0;

    this->i2c_read(FUSB_INTERRUPT_A, &data);
    if (data & FUSB_INTERRUPT_A_TOGDONE)
    {
        //Signal toggle done?
    }
    if (data & FUSB_INTERRUPT_A_TXSENT)
    {
        Serial.println("Recieved good crc on last message");
        this->read_message();
    }

    this->i2c_read(FUSB_INTERRUPT_B, &data);
    if (data & FUSB_INTERRUPT_B_GCRCSENT)
    {
        Serial.println("Responded good crc");
        this->read_message();
    }

    this->i2c_read(FUSB_INTERRUPT, &data);

    this->interrupt_clear();
}

//TODO: Handle soft reset correctly
void (*resetFunc)(void) = 0;

void FUSB302::read_message()
{
    uint8_t status;

    this->i2c_read(FUSB_STATUS1, &status);

    if (status & FUSB_STATUS1_RX_EMPTY)
    {
        Serial.println("No messages");
        return;
    }

    uint8_t temp;
    uint8_t token;
    int len = 0;
    int dataObjects;
    uint16_t header;
    uint32_t data[10];
    uint8_t buffer[40];

    this->i2c_read(FUSB_FIFOS, &token);

    this->i2c_read(FUSB_FIFOS, &temp);

    header |= temp;

    this->i2c_read(FUSB_FIFOS, &temp);

    header |= temp << 8;

    len = this->get_num_bytes(header);

    dataObjects = (header >> 12) & 7;

    uint8_t reg[] = {FUSB_FIFOS};

    this->i2c_transfer(reg, 1, buffer, len + 2, true);

    memcpy(data, buffer, len);

    Serial.print("header recieved: 0x");
    Serial.println(header, HEX);

    Serial.print("Rec message id: ");
    Serial.println((header >> 9) & 0x7);

    //this->messageID++;

    //TODO: Move parsing logic to PD engine
    PD::Message message;

    int messageType =  PD::parseHeaderMessageType(header);
    //Control message
    if (dataObjects == 0)
    {
        message.type = PD::MessageType::CONTROL;
        if (messageType == PD::ControlMessageType::GOODCRC)
        {
            Serial.println("Message GCRC");
            this->messageID++;
        }
        else if (messageType == PD::ControlMessageType::ACCEPT)
        {
            Serial.println("Message ACCEPT");
        }
        else if (messageType == PD::ControlMessageType::REJECT)
        {
            Serial.println("Message REJECT");
        }
        else if (messageType == PD::ControlMessageType::PS_RDY)
        {
            Serial.println("Message PS_RDY");
            Serial.print("VBUS voltage: ");
            Serial.println(this->measureVBus());
        }
        else if (messageType == PD::ControlMessageType::SOFT_RESET)
        {
            Serial.println("Message Soft reset");
            resetFunc();
        }
    }
    //Data message
    else
    {
        message.type = PD::MessageType::DATA;
        //Source capabilities
        if (messageType == PD::DataMessageType::SOURCE_CAP)
        {
            message.dataMessage.type = PD::DataMessageType::SOURCE_CAP;
            for (int i = 0; i < dataObjects; i++)
            {
                Serial.print("Source cap: 0x");
                Serial.println(data[i], HEX);
            }

            // this->requestPower(data[0], 1);

            //TODO: Parse source cap

            PD::Capabilities sourceCap;

            for (int i = 0; i < dataObjects; i++)
            {
                //TODO: Use real type
                sourceCap.dataObjects[i].type = PD::PDO::Types::SOURCE_FIXED;
                sourceCap.dataObjects[i].sourceFixed.voltage = ((data[i] >> 10) & 0x3FF) * 50;
                sourceCap.dataObjects[i].sourceFixed.current = ((data[i]) & 0x3FF) * 10;
            }
            sourceCap.length = dataObjects;

            message.dataMessage.sourceCap = sourceCap;

            //display.debug();
            receivedMessageCB(message);
            //display.debug();
        }

        else if( messageType == PD::DataMessageType::SINK_CAP)
        {
            message.dataMessage.type = PD::DataMessageType::SINK_CAP;

            //TODO: Parse sink cap
            PD::Capabilities sinkCap;

            for (int i = 0; i < dataObjects; i++)
            {
                //TODO: Use real type
                sinkCap.dataObjects[i].type = PD::PDO::Types::SINK_FIXED;
                sinkCap.dataObjects[i].sinkFixed.voltage = ((data[i] >> 10) & 0x3FF) * 50;
                sinkCap.dataObjects[i].sinkFixed.current = ((data[i]) & 0x3FF) * 10;
            }
            sinkCap.length = dataObjects;

            message.dataMessage.sinkCap = sinkCap;
            receivedMessageCB(message);
        }

        else if (messageType == PD::DataMessageType::REQUEST)
        {
            Serial.print("Request recieved");
        }

        //Vendor defined message
        else if (messageType == PD::DataMessageType::VDM_TYPE)
        {
             message.dataMessage.type = PD::DataMessageType::VDM_TYPE;

            Serial.print("Vendor header: 0x");
            Serial.println(data[0], HEX);
            if ((data[0] & 0x1F) == 1)
            {
                //Identity
                Serial.println("Identity VDM");
                if (((data[0] >> 6) & 0x3) == 0)
                {
                    //Request
                    //TODO: use destination request recieved from
                    Serial.println("Respond identity");
                    this->respond_identity_req_message(PD::Destination::SOP);
                }
                else
                {
                    message.dataMessage.vdm.type = PD::VdmType::IDENTITY;
                    //TODO: support more than only passive cable
                    PD::Identity id;
                    id.host = data[1] & PD::VDM::ID_USB_HOST;
                    id.device = data[1] & PD::VDM::ID_USB_DEVICE;
                    id.modal = data[1] & PD::VDM::ID_MODAL;
                    id.vendorId = data[1] & 0xFFFF;
                    id.productId = (data[3] & 0xFFFF0000) >> 16;
                    id.bcdDevice = data[3] & 0xF;
                    id.productType = PD::IdentityProductType::PASSIVE_CABLE;

                    PD::PassiveCable cable;
                    cable.hwFirmwareVersion = (data[4] >> 24) & 0xFF;
                    cable.latency = (data[4] >> 13) & 0xF;
                    cable.Vconn = data[4] & PD::VDM::ID_VCONN;
                    cable.vbusCurrent = static_cast<PD::VbusCurrent>((data[4] >> 5) & 0x3);
                    cable.usbSpeed = static_cast<PD::UsbSpeed>(data[4] & 0x7);

                    id.passiveCable = cable;

                    message.dataMessage.vdm.id = id;

                    Serial.println("Calling cb from read_message");
                    //display.debug();

                    //TODO move to end of function
                    receivedMessageCB(message);

                    // TODO call cb
                    //identity_cb(id);

                }
            }
        }
    }

    this->i2c_read(FUSB_STATUS1, &status);

    if (!(status & FUSB_STATUS1_RX_EMPTY))
    {
        Serial.println("More messages");
        this->read_message();
    }
}

//0 = cc1, 1 = cc2
void FUSB302::set_polarity(int polarity)
{
    this->i2c_clear_bits(FUSB_SWITCHES0, FUSB_SWITCHES0_VCONN_CC1 | FUSB_SWITCHES0_VCONN_CC2 | FUSB_SWITCHES0_MEAS_CC1 | FUSB_SWITCHES0_MEAS_CC2);

    this->i2c_set_bits(FUSB_SWITCHES0, polarity == 0 ? FUSB_SWITCHES0_MEAS_CC1 : FUSB_SWITCHES0_MEAS_CC2);

    if (this->isSource && this->vconnEnabled)
    {
        this->i2c_set_bits(FUSB_SWITCHES0, polarity == 0 ? FUSB_SWITCHES0_VCONN_CC2 : FUSB_SWITCHES0_VCONN_CC1);
    }

    this->polarity = polarity;
}

void FUSB302::enable_tx()
{
    this->i2c_set_bits(FUSB_SWITCHES1, FUSB_SWITCHES1_AUTOCRC);

    this->i2c_clear_bits(FUSB_SWITCHES1, FUSB_SWITCHES1_TXCC1 | FUSB_SWITCHES1_TXCC2);
    if (this->polarity == 0)
    {
        this->i2c_set_bits(FUSB_SWITCHES1, FUSB_SWITCHES1_TXCC1);
    }
    else
    {
        this->i2c_set_bits(FUSB_SWITCHES1, FUSB_SWITCHES1_TXCC2);
    }

    if (this->isSource)
    {
        this->i2c_set_bits(FUSB_SWITCHES1, FUSB_SWITCHES1_DATAROLE | FUSB_SWITCHES1_POWERROLE);
    }
    else
    {
        this->i2c_clear_bits(FUSB_SWITCHES1, FUSB_SWITCHES1_DATAROLE | FUSB_SWITCHES1_POWERROLE);
    }

    this->flush_tx();
    this->flush_rx();
}

uint8_t FUSB302::getIdentity()
{
    uint8_t data;
    this->i2c_read(FUSB_DEVICE_ID, &data);
    return data;
}

void FUSB302::interrupt_clear()
{
    uint8_t temp;

    this->i2c_read(FUSB_INTERRUPT_A, &temp);
    this->i2c_read(FUSB_INTERRUPT_B, &temp);
    this->i2c_read(FUSB_INTERRUPT, &temp);
}

void FUSB302::setIsSource(bool setIsSource)
{
    this->isSource = setIsSource;

    this->set_pull(isSource);
    this->set_polarity(this->polarity);

}

void FUSB302::respond_identity_req_message(int destination)
{
    uint16_t MESSAGE_HEADER;

    //Message type: VDM
    MESSAGE_HEADER |= 0x0F;
    //Revision
    MESSAGE_HEADER |= 1 << 6;
    //Port power role
    //TODO: diffrent depending on source or sink
    MESSAGE_HEADER |= 0 << 8;
    //Message id
    MESSAGE_HEADER |= (this->messageID & 0x07) << 9;
    //Number of data objects
    MESSAGE_HEADER |= 0x4 << 12;

    uint32_t VDM_HEADER;
    VDM_HEADER |= PD_DISCOVER_IDENTITY;
    VDM_HEADER |= PD_COMMAND_TYPE_ACK;
    VDM_HEADER |= PD_STRUCTURED_VDM_VERSION;
    VDM_HEADER |= PD_STRUCTURED_VDM;
    VDM_HEADER |= PD_POWER_DELIVERY_SVID;

    //TODO: Use same as onboard programmer
    const uint16_t USB_VENDOR_ID = 0x1209;
    const uint16_t USB_PRODUCT_ID = 0x2003;

    uint32_t ID_HEADER_VDO;

    ID_HEADER_VDO |= PD_ID_HEADER_USB_DEVICE;
    ID_HEADER_VDO |= PD_ID_HEADER_PRODUCT_TYPE_UFP_PERIPHERAL;
    ID_HEADER_VDO |= USB_VENDOR_ID;

    uint32_t CERT_STAT_VDO = 0;

    uint32_t PRODUCT_VDO;
    PRODUCT_VDO |= USB_PRODUCT_ID << 16;

    uint32_t data[] = {VDM_HEADER, ID_HEADER_VDO, CERT_STAT_VDO, PRODUCT_VDO};

    this->send_message(MESSAGE_HEADER, data, destination);
}

void FUSB302::requestPower(uint32_t pdo, int position)
{
    uint16_t MESSAGE_HEADER = 0;

    //Message type: Request
    MESSAGE_HEADER |= PD_HEADER_MESSAGE_TYPE_REQUEST;
    //Revision
    MESSAGE_HEADER |= 1 << 6;
    //Port power role
    //TODO: diffrent depending on source or sink
    MESSAGE_HEADER |= 0 << 8;
    //Message id
    MESSAGE_HEADER |= (this->messageID & 0x07) << 9;
    //Number of data objects
    MESSAGE_HEADER |= 1 << 12;

    Serial.print("Message id: ");
    Serial.println((this->messageID & 0x07));

    uint32_t REQUEST = 0;

    // REQUEST |= position<<28;
    // REQUEST |= (pdo&0x3FF);
    // REQUEST |= (pdo&0x3FF)<<10;

    int profile = (this->messageID % 4 == 0 ? 1 : 2);

    Serial.print("Power profile: ");
    Serial.println(profile);

    REQUEST |= profile << 28;
    REQUEST |= 50 << 0;
    REQUEST |= 50 << 10;

    uint32_t data[] = {REQUEST};

    this->send_message(MESSAGE_HEADER, data, FUSB_DESTINATION_SOP);

    Serial.println("Requested voltage");
}

void FUSB302::requestSourceCap()
{
    uint16_t header = 0;

    //Message type: Get_Source_Cap
    header |= 0x7;
    //Revision
    header |= 1 << 6;
    //Cable plug role
    header |= 0 << 8;
    //Message id
    header |= (this->messageID & 0x07) << 9;
    //Number of data objects
    header |= 0 << 12;

    Serial.print("Message id: ");
    Serial.println((this->messageID & 0x07));

    this->send_message(header, 0, FUSB_DESTINATION_SOP);
}

void FUSB302::requestSourceCap(void (*cb)(PD_Source_Cap))
{
    this->sourceCap_cb = cb;
    this->requestSourceCap();
}

float FUSB302::measureVBus()
{
    // uint8_t old;
    // this->i2c_read(FUSB_SWITCHES0, &old);
    // this->i2c_clear_bits(FUSB_SWITCHES0, (FUSB_SWITCHES0_MEAS_CC1 | FUSB_SWITCHES0_MEAS_CC2));
    // this->i2c_set_bits(FUSB_MEASURE, FUSB_MEASURE_VBUS);
    // delay(1);
    // uint8_t data;
    // this->i2c_read(FUSB_MEASURE, &data);
    // this->i2c_clear_bits(FUSB_MEASURE, FUSB_MEASURE_VBUS);
    // this->i2c_write(FUSB_SWITCHES0, old);
    // return data &0x3F;

    return analogRead(A1) * (3.3 / 1023.0) * (1.0 + (10.0 / 1.5));
}

void FUSB302::registerIdentityCB(void (*cb)(PD_ID))
{
    this->identity_cb = cb;
}

void FUSB302::registerSourceCapCB(void (*cb)(PD_Source_Cap))
{
    this->sourceCap_cb = cb;
}

int FUSB302::getMessageID()
{
    return this->messageID;
}

void FUSB302::setReceivedMessageCB(void (*cb)(PD::Message)){
    this->receivedMessageCB = cb;
}