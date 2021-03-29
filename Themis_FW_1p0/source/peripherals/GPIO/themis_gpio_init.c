// file for GPIO init of Themis board --- jcw

//
// Included Files
//
#include "DSP28x_Project.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "inc/hw_usb.h"
#include "include/usblib.h"
#include "include/usbcdc.h"
#include "driverlib/usb_hal.h"
#include "include/device/usbdevice.h"
#include "include/device/usbdcdc.h"
// #include "../../usb_serial_structs.h"
#include "usb_serial_structs.h"
#include "themis_io_definitions.h"
#include "F2806x_GlobalPrototypes.h"

extern void Gpio_select(void);
extern void delay_loop(void);



void themis_gpio_init(void)
{


	Gpio_select();                          // init GPIO

    EALLOW;

    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = DISABLE;                // Error
    GpioDataRegs.GPBSET.bit.GPIO39 = ENABLE;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = ENABLE;
  //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = ENABLE;
#if 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = DISABLE;                // ENABLE MEMS OSC - 90 MHz
    GpioDataRegs.GPASET.bit.GPIO8 = ENABLE;
    GpioCtrlRegs.GPADIR.bit.GPIO8 = ENABLE;
  //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = ENABLE;

// #if 0
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = DISABLE;                  // XCLKIN
   // GpioDataRegs.GPASET.bit.GPIO8 = DISABLE;
        GpioCtrlRegs.GPADIR.bit.GPIO19 = DISABLE;
      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
        GpioCtrlRegs.GPAPUD.bit.GPIO19 = ENABLE;                // this means NO pullup is enabled
        GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#endif

    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = DISABLE;              // OCXO OUT En
    GpioDataRegs.GPBSET.bit.GPIO53 = DISABLE;
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = DISABLE;
    //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = ENABLE;                // this means NO pullup is enabled
    // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = ENABLE;              // 10MHz OUT In - ECAP3
       GpioDataRegs.GPASET.bit.GPIO9 = DISABLE;
       GpioCtrlRegs.GPADIR.bit.GPIO9 = DISABLE;
       //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
       GpioCtrlRegs.GPAPUD.bit.GPIO9 = ENABLE;                // this means NO pullup is enabled
       // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

       GpioCtrlRegs.GPAMUX1.bit.GPIO0 = DISABLE;                // 1PPS 6T Out En
       GpioDataRegs.GPASET.bit.GPIO0 = DISABLE;                  // active high
//       GpioDataRegs.GPASET.bit.GPIO0 = ENABLE;
       GpioCtrlRegs.GPADIR.bit.GPIO0 = ENABLE;
   //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
       GpioCtrlRegs.GPAPUD.bit.GPIO0 = ENABLE;

        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = DISABLE;                // I2C En
        GpioDataRegs.GPASET.bit.GPIO2 = DISABLE;                  // active high
        GpioCtrlRegs.GPADIR.bit.GPIO2 = ENABLE;
    //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
        GpioCtrlRegs.GPAPUD.bit.GPIO2 = ENABLE;

#if 0
       GpioCtrlRegs.GPAMUX2.bit.GPIO17 = DISABLE;              // AD SPI CSL
        GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
        GpioCtrlRegs.GPADIR.bit.GPIO17 = ENABLE;
        GpioDataRegs.GPADAT.bit.GPIO17 = DISABLE;
        GpioCtrlRegs.GPAPUD.bit.GPIO17 = ENABLE;                // this means NO pullup is enabled
    // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#endif
        GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0x03;              // PWM for DAC
//         GpioDataRegs.GPASET.bit.GPIO30 = ENABLE;
//         GpioCtrlRegs.GPADIR.bit.GPIO30 = ENABLE;
     //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
         GpioCtrlRegs.GPAPUD.bit.GPIO30 = ENABLE;                // this means NO pullup is enabled
     // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

         GpioCtrlRegs.GPBMUX1.bit.GPIO44 = DISABLE;              // sync in enable
//          GpioDataRegs.GPBSET.bit.GPIO44 = DISABLE;             // - SELECT B - Low is B - 1PPS
          GpioDataRegs.GPBSET.bit.GPIO44 = ENABLE;              // sELECT A - High is A - 10MHz (Timepulse2)
          GpioCtrlRegs.GPBDIR.bit.GPIO44 = ENABLE;             // enable
      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
          GpioCtrlRegs.GPBPUD.bit.GPIO44 = ENABLE;                // this means NO pullup is enabled
      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

          GpioCtrlRegs.GPBMUX1.bit.GPIO43 = DISABLE;              // VBUS Det
            GpioDataRegs.GPBSET.bit.GPIO43 = DISABLE;
            GpioCtrlRegs.GPBDIR.bit.GPIO43 = DISABLE;
        //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
            GpioCtrlRegs.GPBPUD.bit.GPIO43 = ENABLE;                // this means NO pullup is enabled
        // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

            GpioCtrlRegs.GPBMUX1.bit.GPIO42 = DISABLE;              // 1PPS in en
               GpioDataRegs.GPBSET.bit.GPIO42 = DISABLE;
               GpioCtrlRegs.GPBDIR.bit.GPIO42 = ENABLE;
//             GpioDataRegs.GPBDAT.bit.GPIO42 = ENABLE;
               GpioCtrlRegs.GPBPUD.bit.GPIO42 = ENABLE;                // this means NO pullup is enabled
           // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

               GpioCtrlRegs.GPBMUX1.bit.GPIO34 = DISABLE;              // gpio34
                    GpioDataRegs.GPBSET.bit.GPIO34 = DISABLE;
                    GpioCtrlRegs.GPBDIR.bit.GPIO34 = DISABLE;
                //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                    GpioCtrlRegs.GPBPUD.bit.GPIO34 = ENABLE;                // this means NO pullup is enabled
                // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0x02;              // i2c for 6t
                       GpioDataRegs.GPASET.bit.GPIO28 = DISABLE;
                       GpioCtrlRegs.GPADIR.bit.GPIO28 = DISABLE;
                   //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                       GpioCtrlRegs.GPAPUD.bit.GPIO28 = ENABLE;                // this means NO pullup is enabled
                   // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                       //    GpioCtrlRegs.GPAMUX2.bit.GPIO11 = DISABLE;                // 1PPS for Atlas
                           GpioCtrlRegs.GPAMUX2.bit.GPIO20 = DISABLE;                  // GPIO function
                           GpioDataRegs.GPASET.bit.GPIO20 = DISABLE;                   // Low for 1PPS from LEA
                           GpioCtrlRegs.GPAPUD.bit.GPIO20 = ENABLE;                    // No Pull-Up
                           GpioDataRegs.GPADAT.bit.GPIO20 = DISABLE;
                           GpioCtrlRegs.GPADIR.bit.GPIO20 = ENABLE;                    // Output
                       //    GpioCtrlRegs.GPAMUX2.bit.


                       GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0x02;              // i2c for 6t
                          GpioDataRegs.GPASET.bit.GPIO29 = DISABLE;
                          GpioCtrlRegs.GPADIR.bit.GPIO29 = DISABLE;
                      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                          GpioCtrlRegs.GPAPUD.bit.GPIO29 = ENABLE;                // this means NO pullup is enabled
                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                          GpioCtrlRegs.GPAMUX2.bit.GPIO25 = ENABLE;              // ocxo out m
                          GpioDataRegs.GPASET.bit.GPIO25 = DISABLE;
                          GpioCtrlRegs.GPADIR.bit.GPIO25 = DISABLE;
                      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                          GpioCtrlRegs.GPAPUD.bit.GPIO25 = ENABLE;                // this means NO pullup is enabled
                       // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                          GpioCtrlRegs.GPAMUX2.bit.GPIO24 = ENABLE;              // 1PPS IN
                                         GpioDataRegs.GPASET.bit.GPIO24 = DISABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO24 = DISABLE;
                                     //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO24 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

// ad spi setup:

                                         GpioCtrlRegs.GPAMUX2.bit.GPIO18 = ENABLE;              // SPI CLK
                                         GpioDataRegs.GPASET.bit.GPIO18 = DISABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO18 = ENABLE;
                                     //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO18 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                                         GpioCtrlRegs.GPAMUX2.bit.GPIO17 = DISABLE;              // SPI CS
                                         GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO17 = ENABLE;
                                         GpioDataRegs.GPADAT.bit.GPIO17 = DISABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO17 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#if 1
                                         GpioCtrlRegs.GPBMUX1.bit.GPIO40 = DISABLE;              // i2c_EN atlas bus now
                                         GpioDataRegs.GPBSET.bit.GPIO40 = DISABLE;                // active high
                                         GpioCtrlRegs.GPBDIR.bit.GPIO40 = ENABLE;
                                         GpioDataRegs.GPBDAT.bit.GPIO40 = DISABLE;
                                         GpioCtrlRegs.GPBPUD.bit.GPIO40 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#endif
                                         GpioCtrlRegs.GPBMUX1.bit.GPIO41 = DISABLE;              // 280_EN SYNTH CLK
                                         GpioDataRegs.GPBSET.bit.GPIO41 = ENABLE;
                                         GpioCtrlRegs.GPBDIR.bit.GPIO41 = ENABLE;
                                         GpioDataRegs.GPBDAT.bit.GPIO41 = ENABLE;
                                         GpioCtrlRegs.GPBPUD.bit.GPIO41 = ENABLE;                // this means NO pullup is enabled
                                         // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;


                                         GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
                                          GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
                                          GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
                                          GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
                                          GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                                          GpioCtrlRegs.GPBMUX2.bit.GPIO50 = DISABLE;              // PGA CS
                                          GpioDataRegs.GPBSET.bit.GPIO50 = ENABLE;
                                          GpioCtrlRegs.GPBDIR.bit.GPIO50 = ENABLE;
                                          GpioDataRegs.GPBDAT.bit.GPIO50 = ENABLE;
                                          GpioCtrlRegs.GPBPUD.bit.GPIO50 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                                          GpioCtrlRegs.GPAMUX1.bit.GPIO13 = DISABLE;              // UBLOX IRQ
                                         //  GpioDataRegs.GPASET.bit.GPIO13 = ENABLE;
                                         //  GpioCtrlRegs.GPADIR.bit.GPIO13 = ENABLE;
                                         //  GpioDataRegs.GPADAT.bit.GPIO13 = ENABLE;

                                          GpioDataRegs.GPASET.bit.GPIO13 = DISABLE;
                                          GpioCtrlRegs.GPADIR.bit.GPIO13 = DISABLE;
                                          GpioDataRegs.GPADAT.bit.GPIO13 = DISABLE;
                                          GpioCtrlRegs.GPAPUD.bit.GPIO13 = ENABLE;                // this means NO pullup is enabled
                                       // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;



                                          GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0x03;              // clkin hrcap - ECAP1
                                        //  GpioDataRegs.GPASET.bit.GPIO11 = DISABLE;
                                        //  GpioCtrlRegs.GPADIR.bit.GPIO11 = DISABLE;
                                       //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                          GpioCtrlRegs.GPAPUD.bit.GPIO11 = ENABLE;                // this means NO pullup is enabled
                                        GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0x03;

                                          GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0x03;              // clkin hrcap - hrcap1
                                      //    GpioDataRegs.GPBSET.bit.GPIO54 = DISABLE;
                                      //    GpioCtrlRegs.GPBDIR.bit.GPIO54 = DISABLE;
                                      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                          GpioCtrlRegs.GPBPUD.bit.GPIO54 = DISABLE;                // this means NO pullup is enabled
                                          GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 0x03;

#ifdef USE_HRCAP2

                                      //    GpioCtrlRegs.GPBMUX1.bit.GPIO55 = 1;
                                          GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0x03;              // clkin hrcap2
                                      //    GpioDataRegs.GPBSET.bit.GPIO55 = DISABLE;
                                      //    GpioCtrlRegs.GPBDIR.bit.GPIO55 = DISABLE;
                                      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                          GpioCtrlRegs.GPBPUD.bit.GPIO55 = DISABLE;                // this means NO pullup is enabled
                                          GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 0x03;
#else
                                              GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;                  // GPIO55
                                          //    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0x03;              // clkin hrcap2
                                              GpioDataRegs.GPBSET.bit.GPIO55 = DISABLE;
                                              GpioCtrlRegs.GPBDIR.bit.GPIO55 = DISABLE;
                                          //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                              GpioCtrlRegs.GPBPUD.bit.GPIO55 = DISABLE;                // this means NO pullup is enabled
                                          //    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 0x03;

#endif


                                          GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0x03;              // clkin hrcap3
                                      //    GpioDataRegs.GPBSET.bit.GPIO56 = DISABLE;
                                      //    GpioCtrlRegs.GPBDIR.bit.GPIO56 = DISABLE;
                                      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                          GpioCtrlRegs.GPBPUD.bit.GPIO56 = DISABLE;                // this means NO pullup is enabled
                                          GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 0x03;

                                          GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0x03;              // clkin hrcap4
                                    //    GpioDataRegs.GPBSET.bit.GPIO56 = DISABLE;
                                    //    GpioCtrlRegs.GPBDIR.bit.GPIO56 = DISABLE;
                                      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                          GpioCtrlRegs.GPBPUD.bit.GPIO57 = DISABLE;                // this means NO pullup is enabled
                                          GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 0x03;

                                         GpioCtrlRegs.GPAMUX2.bit.GPIO16 = ENABLE;              // SPI MOSI
                                         GpioDataRegs.GPASET.bit.GPIO16 = DISABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO16 = ENABLE;
                                     //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO16 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                                         GpioCtrlRegs.GPAMUX1.bit.GPIO15 = DISABLE;              //
                                         GpioDataRegs.GPASET.bit.GPIO15 = DISABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO15 = DISABLE;
                                     //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO15 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;


                                         GpioCtrlRegs.GPAMUX1.bit.GPIO14 = DISABLE;              //
                                         GpioDataRegs.GPASET.bit.GPIO14 = DISABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO14 = DISABLE;
                                     //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO14 = ENABLE;                // this means NO pullup is enabled
                                      // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#if 0
// 10MHz CLK EN
                                         GpioCtrlRegs.GPAMUX1.bit.GPIO10 = DISABLE;              //
                                         GpioDataRegs.GPASET.bit.GPIO10 = ENABLE;
                                         GpioCtrlRegs.GPADIR.bit.GPIO10 = ENABLE;
                                         GpioDataRegs.GPADAT.bit.GPIO10 = ENABLE;
                                         GpioCtrlRegs.GPAPUD.bit.GPIO10 = ENABLE;                // this means NO pullup is enabled
                                        // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#endif

// Prog Status

                                                              GpioCtrlRegs.GPAMUX1.bit.GPIO3 = DISABLE;              //
                                                              GpioDataRegs.GPASET.bit.GPIO3 = DISABLE;
                                                              GpioCtrlRegs.GPADIR.bit.GPIO3 = ENABLE;
                                                         //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                                              GpioCtrlRegs.GPAPUD.bit.GPIO3 = ENABLE;                // this means NO pullup is enabled
                                                           // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

 // 1PPS OUT

                                                             GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
                                                             GpioDataRegs.GPASET.bit.GPIO4 = ENABLE;
                                                             GpioCtrlRegs.GPADIR.bit.GPIO4 = ENABLE;
                                                             GpioDataRegs.GPADAT.bit.GPIO4 = ENABLE;
                                                             GpioCtrlRegs.GPAPUD.bit.GPIO4 = ENABLE;                // this means NO pullup is enabled
                                                          // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
// TX/RX Wireless

                                                              GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0x01;              //  RX Wireless - SCIRXDA(I)
                                                              GpioDataRegs.GPASET.bit.GPIO7 = DISABLE;
                                                              GpioCtrlRegs.GPADIR.bit.GPIO7 = DISABLE;
                                                          //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                                              GpioCtrlRegs.GPAPUD.bit.GPIO7 = DISABLE;                // this means NO pullup is enabled
                                                              GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;


                                                              GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0x1;              //  TX Wireless - SCITXDA(O)
                                                              GpioDataRegs.GPASET.bit.GPIO12 = DISABLE;
                                                              GpioCtrlRegs.GPADIR.bit.GPIO12 = ENABLE;
                                                          //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                                              GpioCtrlRegs.GPAPUD.bit.GPIO12 = DISABLE;                // this means NO pullup is enabled
                                                          //    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = ENABLE;

                                                          //     GpioCtrlRegs.GPAPUD.bit.GPIO6 = ENABLE;
                                                              GpioCtrlRegs.GPAPUD.bit.GPIO6 = DISABLE;              // PULL-UP ENABLED
                                                              //
                                                                // Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)
                                                                //
                                                                GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;
                                                               GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0x02;              //  SYNC INPUT
                                                         //     GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0x00;
                                                          //    GpioDataRegs.GPASET.bit.GPIO6 = DISABLE;
                                                          //    GpioCtrlRegs.GPADIR.bit.GPIO6 = DISABLE;
                                                          //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
                                                                              // this means NO pullup is enabled
                                                           // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#if 0
                                                              GpioCtrlRegs.GPBMUX2.bit.GPIO52 = DISABLE;              // OCXO CLOCK SEL
                                                              GpioDataRegs.GPBSET.bit.GPIO52 = ENABLE;
                                                              GpioCtrlRegs.GPBDIR.bit.GPIO52 = ENABLE;
                                                              GpioDataRegs.GPBDAT.bit.GPIO52 = ENABLE;
                                                              GpioCtrlRegs.GPBPUD.bit.GPIO52 = ENABLE;                // this means NO pullup is enabled
                                                             // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
#endif


    EDIS;

}


void themis_usb_gpio_init(void)
{

	EALLOW;
    GpioCtrlRegs.GPACTRL2.bit.USB0IOEN = 1;

//        GpioCtrlRegs.GPBLOCK.all = 0x00000000;
        //GpioCtrlRegs.GPBAMSEL.bit.GPIO42 = 1; //USBDM GPIO 27
        //GpioCtrlRegs.GPBAMSEL.bit.GPIO43 = 1; //USBDP GPIO 26
//        GpioCtrlRegs.GPBAMSEL.bit.GPIO27 = 1; //USBDM GPIO 27
//        GpioCtrlRegs.GPBAMSEL.bit.GPIO26 = 1; //USBDP GPIO 26
//        GpioCtrlRegs.
        //VBUS
        //GpioCtrlRegs.GPBDIR.bit.GPIO46 = 0; // GPIO 39
        //ID
        //GpioCtrlRegs.GPBDIR.bit.GPIO47 = 0; // GPIO 2

        //GpioCtrlRegs.GPDGMUX2.bit.GPIO120 = 3;
        //GpioCtrlRegs.GPDMUX2.bit.GPIO120 = 3;
        //GpioCtrlRegs.GPDGMUX2.bit.GPIO121 = 3;
        //GpioCtrlRegs.GPDMUX2.bit.GPIO121 = 3;

//	HWREG(USBMODESEL) = USBMODESEL_DEV;
//	HWREG(USB0_BASE + USB_O_GPCS) = USBGPCS_DEV;

    //GpioCtrlRegs.GPAMUX1.bit.GPIO8 = ENABLE;                // ENABLE MEMS OSC - 90 MHz
    //GpioCtrlRegs.GPAPUD.bit.GPIO8 = DISABLE;
    // GpioCtrlRegs.GPADIR.bit.GPIO8 = ENABLE;

//    GpioCtrlRegs.GPAMUX2.bit.GPIO11 = DISABLE;                // ENABLE USB
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = ENABLE;
	GpioDataRegs.GPASET.bit.GPIO21 = ENABLE;
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = ENABLE;
	GpioCtrlRegs.GPADIR.bit.GPIO21 = ENABLE;
//    GpioCtrlRegs.GPAMUX2.bit.
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = DISABLE;                // PFLT
	GpioDataRegs.GPASET.bit.GPIO31 = ENABLE;
	GpioCtrlRegs.GPAPUD.bit.GPIO31 = DISABLE;
	GpioCtrlRegs.GPADIR.bit.GPIO31 = DISABLE;

	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = DISABLE;                // ENABLE u-blox
	GpioDataRegs.GPBSET.bit.GPIO32 = ENABLE;
	GpioCtrlRegs.GPBPUD.bit.GPIO32 = DISABLE;
	GpioCtrlRegs.GPBDIR.bit.GPIO32 = ENABLE;

	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = DISABLE;                // USB_ID
	GpioDataRegs.GPBSET.bit.GPIO33 = DISABLE;
	GpioCtrlRegs.GPBPUD.bit.GPIO33 = DISABLE;
	GpioCtrlRegs.GPBDIR.bit.GPIO33 = DISABLE;





    // GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;

//     HWREG(USBMODESEL) = USBMODESEL_DEV;
//     HWREG(USB0_BASE + USB_O_GPCS) = USBGPCS_DEV;
	EDIS;
}


void themis_uart_gpio_init(void)
{


	EALLOW;
#if 0
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;           // rx - tx from ublox
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = DISABLE;
	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;
//	GpioCtrlRegs.GPADIR.bit.GPIO23 = DISABLE;

GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;           // tx - rx to ublox
GpioCtrlRegs.GPAPUD.bit.GPIO22 = DISABLE;
#endif
// GpioCtrlRegs.GPADIR.bit.GPIO22 = ENABLE;

GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;
GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;
GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;

GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;
GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;

GpioCtrlRegs.GPAMUX1.bit.GPIO0 = DISABLE;                // 1PPS_OUT6T_EN
GpioDataRegs.GPASET.bit.GPIO0 = ENABLE;
GpioCtrlRegs.GPAPUD.bit.GPIO0 = ENABLE;
GpioDataRegs.GPADAT.bit.GPIO0 = DISABLE;
GpioCtrlRegs.GPADIR.bit.GPIO0 = ENABLE;                 // active low

#if 0
GpioCtrlRegs.GPAMUX1.bit.GPIO1 = DISABLE;                // 10MHZ_6TIN_EN
GpioDataRegs.GPASET.bit.GPIO1 = ENABLE;
GpioCtrlRegs.GPADIR.bit.GPIO1 = ENABLE;                 // active low
GpioDataRegs.GPADAT.bit.GPIO1 = ENABLE;
GpioCtrlRegs.GPAPUD.bit.GPIO1 = ENABLE;
#endif

GpioCtrlRegs.GPAMUX1.bit.GPIO2 = DISABLE;                // I2C_ENABLE_280
GpioDataRegs.GPASET.bit.GPIO2 = ENABLE;
GpioCtrlRegs.GPAPUD.bit.GPIO2 = DISABLE;
GpioCtrlRegs.GPADIR.bit.GPIO2 = DISABLE;                 // active high


#if 0
GpioCtrlRegs.GPBMUX2.bit.GPIO58 = DISABLE;                // 10MHZ_OUT6T_EN
GpioDataRegs.GPBSET.bit.GPIO58 = ENABLE;
GpioCtrlRegs.GPBPUD.bit.GPIO58 = ENABLE;
GpioCtrlRegs.GPBDIR.bit.GPIO58 = ENABLE;                // mux - this selects A
GpioDataRegs.GPBDAT.bit.GPIO58 = DISABLE;
#endif
   // GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
EDIS;

}


