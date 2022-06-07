#pragma once
#include "fusb302.h"

#define TCPM_DETECTION_POLARITY(polarity) (polarity<<0) 
#define TCPM_DETECTION_RD BIT(1)
#define TCPM_DETECTION_RP BIT(2)
#define TCPM_DETECTION_RA BIT(3)

#define PD_HEADER_MESSAGE_TYPE(header) (header&0xF)
#define PD_HEADER_MESSAGE_TYPE_GoodCRC 0x1
#define PD_HEADER_MESSAGE_TYPE_ACCEPT 0x3
#define PD_HEADER_MESSAGE_TYPE_REJECT 0x4
#define PD_HEADER_MESSAGE_TYPE_PS_RDY 0x6
#define PD_HEADER_MESSAGE_TYPE_SOFT_RESET 0xD
#define PD_HEADER_MESSAGE_TYPE_SOURCE_CAP 0x1
#define PD_HEADER_MESSAGE_TYPE_REQUEST 0x2
#define PD_HEADER_MESSAGE_TYPE_VDM 0xF

#define PD_DISCOVER_IDENTITY 1 << 0
#define PD_COMMAND_TYPE_REQ 0 << 6
#define PD_COMMAND_TYPE_ACK 1 << 6
#define PD_STRUCTURED_VDM 1 << 15
#define PD_STRUCTURED_VDM_VERSION 0 << 13
#define PD_POWER_DELIVERY_SVID 0xFF00 << 16

#define PD_ID_HEADER_USB_HOST BIT(31)
#define PD_ID_HEADER_USB_DEVICE BIT(30)
#define PD_ID_HEADER_PRODUCT_TYPE_UFP_PERIPHERAL 0x2 << 27
#define PD_ID_HEADER_MODAL_OPERATION BIT(26)

#define PD_PRODUCT_VCONN BIT(11)

enum IDType {
    PASSIVE_CABLE,
    ACTIVE_CABLE,
    UFP,
};

enum VbusCurrent {
    A_3 = 0x1,
    A_5 = 0x2,
};

enum USBSpeed {
    USB20 = 0x0,
    USB31 = 0x1,
    USB32 = 0x2,
    USB4 = 0x3,
};

struct PD_passive_cable {
    uint8_t HW_firmware_version;
    uint8_t latency;
    bool Vconn;
    VbusCurrent Vbus_current;
    USBSpeed USB_speed;
};

struct PD_ID {
    bool USB_host;
    bool USB_device;
    bool modal_operation;
    uint16_t USB_vendor_id;
    uint16_t USB_product_id;
    uint16_t bcdDevice;
    IDType type;
    PD_passive_cable passive_cable;
};

struct Source_DO{
    float voltage;
    float current;
};

struct PD_Source_Cap {
    Source_DO dataObjects[7];
    int length;
};

class TCPM {
public:
    void begin();
    int detectConnectionAndOrientation();
    void setDeviceIsSource();
    void setDeviceIsSink();

private:
    //FUSB302 fusb;
};

extern TCPM tcpm;