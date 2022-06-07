#pragma once
//TODO: refactor away arduino specific code
#include <Arduino.h>
#include <Wire.h>
#include <future>

#include "fusb302_reg.h"

#include "oldtcpm.h"

#include <tcpm.h>
//#include <USB-PD.h>

#define FUSB_DESTINATION_SOP 0
#define FUSB_DESTINATION_SOP_PRIME 1

struct PD_ID;
struct PD_Source_Cap;

enum cc_voltage_status
{
    CC_VOLT_OPEN = 0,
    CC_VOLT_RA = 1,
    CC_VOLT_RD = 2,
    CC_VOLT_SNK_DEF = 5,
    CC_VOLT_SNK_1_5 = 6,
    CC_VOLT_SNK_3_0 = 7,
};

int fusbWrite(int reg, int val);
int fusbRead(int reg, int *val);

class FUSB302: public TcpmDriver
{
public:
    void begin();
    void sendMessage(uint16_t header, const uint32_t *data, PD::Destination destination);
    int getMessageID();
    void setReceivedMessageCB(void (*cb)(PD::Message));

    void init();
    void softReset();
    void pdReset();
    void reset();
    void handleInterrupt();

    void setIsSource(bool isSource);

    uint8_t getIdentity();

    int getConnection();

    void requestIdentity(int destination);
    void requestIdentity(int destination, void (*cb)(PD_ID));
    void sendPing(int destination);
    void requestPower(uint32_t pdo, int position);
    void requestSourceCap();
    void requestSourceCap(void (*cb)(PD_Source_Cap));

    float measureVBus();

    void read_message();

    void registerIdentityCB(void (*cb)(PD_ID));
    void registerSourceCapCB(void (*cb)(PD_Source_Cap));

    volatile bool interrupt_flag;

    //for debug
    uint8_t checkVConn();

private:
     void (*receivedMessageCB)(PD::Message);

    void i2c_write(uint8_t reg, uint8_t val);
    void i2c_mask_write(uint8_t reg, uint8_t val, uint8_t mask);
    void i2c_set_bits(uint8_t reg, uint8_t bits);
    void i2c_clear_bits(uint8_t reg, uint8_t bits);
    void i2c_read(uint8_t reg, uint8_t *val);
    void i2c_transfer(const uint8_t *out, int out_size,
                      uint8_t *in, int in_size, bool stop);

    void interrupt_init();
    void interrupt_clear();

    int measure_cc_pin_source(int pin);
    int measure_cc_pin_sink(int pin);
    void detectConnectionAndOrientation();

    void set_pull(int direction);
    void set_polarity(int polarity);
    void enable_tx();

    void flush_tx();
    void flush_rx();
    int write_ordered_set(int destination, uint8_t *buf, int buf_pos);
    int get_num_bytes(uint16_t header);

    void send_message(uint16_t header, const uint32_t *data, int destination);
    void handle_identity_message(const uint32_t *data);
    void respond_identity_req_message(int destination);

    int connection;
    int polarity = 0;
    bool isSource = true;
    bool vconnEnabled = true;
    int messageID = 0;

    void (*identity_cb)(PD_ID);
    void (*sourceCap_cb)(PD_Source_Cap);
};