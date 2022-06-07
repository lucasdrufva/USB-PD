#pragma once
#include <stdint.h>

//TODO: Rename all enums to follow https://google.github.io/styleguide/cppguide.html

namespace PD
{
    // Power Data Object
    // First one should always be vSafe5V
    // Then in increasing order of voltage
    namespace PDO
    {
        const uint8_t MAX_OBJECTS = 7;

        enum Types
        {
            SOURCE_FIXED,
            SINK_FIXED,
            VARIBLE,
            BATTERY
        };

        typedef struct SourceFixed
        {
            bool dualRolePower;  // Dual role power cabable
            bool suspend;        // USB Suspend supported
            bool unconstrained;  // Externally powered
            bool commCapable;    // USB Comunications Capable
            bool dualRoleData;  // Data role swap command supported
            uint8_t peakCurrent; // Default 0, See USB PD rev2 6.4.1.2.3.6
            uint16_t voltage;    // Voltage in milliVolts
            uint16_t current;    // Current in milliAmpere
        };

        typedef struct SinkFixed
        {
            bool dualRolePower;  // Dual role power caoable
            bool higherCap;      // Shall be set if sink needs more than vSafe5V, See USB PD rev2 6.4.1.3.1.2
            bool unconstrained;  // Externally powered
            bool commCapable;    // USB Comunications Capable
            bool dualRoleData;   // Data role swap command supported
            uint8_t peakCurrent; // Default 0, See USB PD rev2 6.4.1.2.3.6
            uint16_t voltage;    // Voltage in milliVolts
            uint16_t current;    // Current in milliAmpere
        };

        typedef struct Varible
        {
            uint16_t maxVolt; // Maximum voltage in milliVolts
            uint16_t minVolt; // Minimum voltage in milliVolts
            uint16_t current; // Current in milliAmpere
        };

        typedef struct Battery
        {
            uint16_t maxVolt; // Maximum voltage in milliVolts
            uint16_t minVolt; // Minimum voltage in milliVolts
            uint16_t power;   // Max Power in milliWatt
        };

        typedef struct PDO
        {
            Types type;
            union
            {
                SourceFixed sourceFixed;
                SinkFixed sinkFixed;
                Varible varible;
                Battery battery;
            };
        };

    }

    struct Capabilities
    {
        PDO::PDO dataObjects[PDO::MAX_OBJECTS];
        uint8_t length;
    };

    enum Destination
    {
        SOP = 0,
        SOP_PRIME = 1
    };

    enum IdentityProductType
    {
        UNDEFINED,
        PDUSB_HUB,
        PDUSB_PERIPHERAL,
        ALTERNATE_MODE_ADAPTER,
        PASSIVE_CABLE,
        ACTIVE_CABLE
    };

    enum VbusCurrent
    {
        A_3 = 0x1,
        A_5 = 0x2,
    };

    enum UsbSpeed
    {
        USB20 = 0x0,
        USB31 = 0x1,
        USB32 = 0x2,
        USB4 = 0x3,
    };

    struct PassiveCable
    {
        uint8_t hwFirmwareVersion; // 4 MSB Hardware, 4 LSB Firmare version
        uint8_t latency;           // In units of 10ns ~ 1m
        bool Vconn;                // Vconn required
        VbusCurrent vbusCurrent;   // Max supported current
        UsbSpeed usbSpeed;
    };

    struct Identity
    {
        bool host;   // USB Communications Capable as USB Host
        bool device; // USB Communications Capable as a USB Device
        IdentityProductType productType;
        bool modal;         // Product supports Modes
        uint16_t vendorId;  // Assigned by USB-IF
        uint16_t productId; // Specific to product
        uint16_t bcdDevice; // Version and revision of product
        union               // Type specific data
        {
            PassiveCable passiveCable;
        };
    };

    namespace VDM
    {
        const uint32_t DISCOVER_IDENTITY = 1 << 0;
        const uint32_t COMMAND_TYPE_REQ = 0 << 6;
        const uint32_t COMMAND_TYPE_ACK = 1 << 6;
        const uint32_t STRUCTURED = 1 << 15;
        const uint32_t STRUCTURED_VERSION = 0 << 13;
        const uint32_t POWER_DELIVERY_SVID = 0xFF00 << 16;

        const uint32_t ID_USB_HOST = 1 << 31;
        const uint32_t ID_USB_DEVICE = 1 << 30;
        const uint32_t ID_MODAL = 1 << 26;
        const uint32_t ID_PRODUCT_TYPE_UFP_PERIPHERAL = 0x2 << 27;
        const uint32_t ID_VCONN = 1 << 11;
    }

    uint8_t parseHeaderMessageType(uint16_t header);

    enum MessageType
    {
        CONTROL,
        DATA,
    };

    enum ControlMessageType
    {
        GOODCRC = 0x1u,
        GOTOMIN = 0x2u,
        ACCEPT = 0x3u,
        REJECT = 0x4u,
        PS_RDY = 0x6u,
        SOFT_RESET = 0xDu, 
    };

    struct ControlMessage
    {
        ControlMessageType type;
    };

    enum DataMessageType
    {
        SOURCE_CAP = 0x1u,
        REQUEST = 0x2u,
        SINK_CAP = 0x4u,
        VDM_TYPE = 0xFu,
    };

    enum VdmType
    {
        IDENTITY,
    };

    struct VdmMessage
    {
        VdmType type;
        union 
        {
            Identity id;
        };
        
    };

    struct DataMessage
    {
        DataMessageType type;
        union
        {
            Capabilities sourceCap;
            Capabilities sinkCap;
            VdmMessage vdm;
        };
    };

    struct Message
    {
        MessageType type;
        union
        {
            ControlMessage controlMessage;
            DataMessage dataMessage;
        };
    };
}

#include "tcpm.h"

class TcpmDriver;

class PD_Engine
{
public:
    enum Error{
        NONE,
        DISCONNECT,
    };

    void begin(TcpmDriver *tcpm);
    void reset();

    // No capabilites advertised
    void configureAsSource();
    void configureAsSource(PD::Capabilities cap, void (*sourceRequestHandler)());

    // No capabilites advertised
    void configureAsSink();
    void configureAsSink(PD::Capabilities cap);

    Error requestIdentity(PD::Destination dest, void (*cb)(PD::Identity));

    void registerCapCB(void (*cb)(PD::Capabilities));
    Error requestSourceCap(void (*cb)(PD::Capabilities));
    Error requestSinkCap(void (*cb)(PD::Capabilities));
    void requestPower(int object_position);

    void sendSourceCap(PD::Capabilities cap);

    void getConnectionStatus();

    //For use by tcpm
    void handleMessage(PD::Message msg);

private:
    TcpmDriver *tcpm;

    void (*identityCB)(PD::Identity);
    void (*capabilitiesCB)(PD::Capabilities);

    PD::Capabilities last_cap_recieved;

};