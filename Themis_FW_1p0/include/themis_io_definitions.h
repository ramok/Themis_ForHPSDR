// header file for themis IO definitions	


#define	ON	1
#define OFF 0
#define ENABLE	1
#define DISABLE 0

#define O1PPS_OUTPUT_6EN		GPIO0
#define O10MHZ_6TIN_EN		GPIO1
#define I2C_ENABLE_280		GPIO2
#define PROG_STATUS			GPIO3
#define O1PPS_OUT			GPIO4
#define O1PPS_OUT_280_EN		GPIO5
#define SYNC_INPUT			GPIO6
#define RX_WIRELESS			GPIO7
#define EN_OSC_X1 			GPIO8
#define O10MHz_Out_I			GPIO9
#define O10MHz_CLK_EN		GPIO10
#define CLKIN_HRCAP			GPIO11
#define TX_WIRELESS			GPIO12
#define UBLOX_IRQ			GPIO13
#define SPI_CLK_280			GPIO14
#define SPI_CS_280			GPIO15
#define AD_SPI_DATA			GPIO16
#define AD_SPI_CSLW			GPIO17
#define AD_SPI_CLK			GPIO18
#define XCLKIN				GPIO19
#define O1PPS_SWITCH_CTRL	GPIO20
#define USB_EN				GPIO21
#define RXD_UBLOX_I			GPIO22
#define TXD_UBLOX_I			GPIO23
#define O1PPS_IN				GPIO24
#define OCXO_OUT_MII		GPIO25
#define USB_TI_D_P			GPIO26
#define USB_TI_D_N			GPIO27
#define SDA2_SPI_MOSI_280T	GPIO28
#define SCL2_SPI_MISO_280T	GPIO29
#define PWM_DRV_DAC_I		GPIO30
#define PFLT				GPIO31
#define RESET_UBLOX			GPIO32
#define USB_ID				GPIO33
#define GPIO_34				GPIO34
#define TDI					GPIO35
#define TMS					GPIO36
#define TDO					GPIO37
#define TCK					GPIO38
#define	ERROR				GPIO39
#define	O10MHz_280_Out		GPIO40
#define O10MHz_280_Out_EN	GPIO41
#define O1PPS_IN_EN			GPIO42
#define VBUS_DET			GPIO43
#define SYNC_IN_EN			GPIO44
//
#define PGA_CS				GPIO50
#define ADI_SPI_CSHW		GPIO51
#define O10MHz_SWITCH_CTRL	GPIO52
#define OCXO_OUT_EN			GPIO53
#define CLKIN_HRCAP_54			GPIO54
#define CLKIN_OCXO			GPIO55
#define CLKIN_6TIN			GPIO56
#define O1PPS_IN_57			GPIO57
#define O10MHZ_OUT6T_EN		GPIO58


void themis_gpio_init(void);
void themis_usb_gpio_init(void);

