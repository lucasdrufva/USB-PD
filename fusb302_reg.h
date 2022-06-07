#define BIT(bit) (1<<bit)

#define FUSB_I2C_SLAVE_ADDRESS 0x22

#define FUSB_DEVICE_ID 0x01

#define FUSB_SWITCHES0 0x02
#define FUSB_SWITCHES0_PDWN1 BIT(0)
#define FUSB_SWITCHES0_PDWN2 BIT(1)
#define FUSB_SWITCHES0_MEAS_CC1 BIT(2)
#define FUSB_SWITCHES0_MEAS_CC2 BIT(3)
#define FUSB_SWITCHES0_VCONN_CC1 BIT(4)
#define FUSB_SWITCHES0_VCONN_CC2 BIT(5)
#define FUSB_SWITCHES0_PU_EN1 BIT(6)
#define FUSB_SWITCHES0_PU_EN2 BIT(7)

#define FUSB_SWITCHES1 0x03
#define FUSB_SWITCHES1_TXCC1 BIT(0)
#define FUSB_SWITCHES1_TXCC2 BIT(1)
#define FUSB_SWITCHES1_AUTOCRC BIT(2)
#define FUSB_SWITCHES1_DATAROLE BIT(4)
#define FUSB_SWITCHES1_POWERROLE BIT(7)

#define FUSB_MEASURE 0x04
#define FUSB_MEASURE_VBUS BIT(6)

#define FUSB_CONTROL0 0x06
#define FUSB_CONTROL0_INT_MASK BIT(5)
#define FUSB_CONTROL0_TX_FLUSH BIT(6)

#define FUSB_CONTROL1 0x07
#define FUSB_CONTROL1_ENSOP1 BIT(0)
#define FUSB_CONTROL1_RX_FLUSH BIT(2)

#define FUSB_CONTROL3 0x09
#define FUSB_CONTROL3_AUTO_RETRY BIT(0)
#define FUSB_CONTROL3_RETRIES(n) (n&3)<<1

#define FUSB_MASK 0x0A

#define FUSB_POWER 0x0B
#define FUSB_POWER_ALL 0xF

#define FUSB_RESET 0x0C
#define FUSB_RESET_SW_RES BIT(0)

#define FUSB_MASKA 0x0E
#define FUSB_MASKA_TXSENT BIT(2)

#define FUSB_MASKB 0x0F
#define FUSB_MASKB_GCRCSENT BIT(0)

#define FUSB_INTERRUPT_A 0x3E
#define FUSB_INTERRUPT_A_TXSENT BIT(2)
#define FUSB_INTERRUPT_A_TOGDONE BIT(6)

#define FUSB_INTERRUPT_B 0x3F
#define FUSB_INTERRUPT_B_GCRCSENT BIT(0)

#define FUSB_STATUS0 0x40
#define FUSB_STATUS0_COMP BIT(5)

#define FUSB_STATUS1 0x41
#define FUSB_STATUS1_RX_EMPTY BIT(5)

#define FUSB_INTERRUPT 0x42

#define FUSB_FIFOS 0x43

#define FUSB_TOKEN_TXON 0xA1
#define FUSB_TOKEN_SYNC1 0x12
#define FUSB_TOKEN_SYNC2 0x13
#define FUSB_TOKEN_SYNC3 0x1B
#define FUSB_TOKEN_PACKSYM 0x80
#define FUSB_TOKEN_JAMCRC 0xFF
#define FUSB_TOKEN_EOP 0x14
#define FUSB_TOKEN_TXOFF 0xFE

