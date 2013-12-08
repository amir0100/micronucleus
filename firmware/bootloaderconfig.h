/* Name: bootloaderconfig.h
 * Project: USBaspLoader
 * Author: Christian Starkjohann
 * Creation Date: 2007-12-08
 * Tabsize: 4
 * Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
 * Portions Copyright: (c) 2012 Louis Beaudoin
 * License: GNU GPL v2 (see License.txt)
 * This Revision: $Id: bootloaderconfig.h 729 2009-03-20 09:03:58Z cs $
 */

#ifndef __bootloaderconfig_h_included__
#define __bootloaderconfig_h_included__

// uncomment this to enable the 'jumper from d5 to gnd to enable programming' mode
//#define BUILD_JUMPER_MODE 1

#ifndef BOOTLOADER_ADDRESS
#define BOOTLOADER_ADDRESS 0
#endif

/*
General Description:
This file (together with some settings in Makefile) configures the boot loader
according to the hardware.

This file contains (besides the hardware configuration normally found in
usbconfig.h) two functions or macros: bootLoaderInit() and
bootLoaderCondition(). Whether you implement them as macros or as static
inline functions is up to you, decide based on code size and convenience.

bootLoaderInit() is called as one of the first actions after reset. It should
be a minimum initialization of the hardware so that the boot loader condition
can be read. This will usually consist of activating a pull-up resistor for an
external jumper which selects boot loader mode.

bootLoaderCondition() is called immediately after initialization and in each
main loop iteration. If it returns TRUE, the boot loader will be active. If it
returns FALSE, the boot loader jumps to address 0 (the loaded application)
immediately.

For compatibility with Thomas Fischl's avrusbboot, we also support the macro
names BOOTLOADER_INIT and BOOTLOADER_CONDITION for this functionality. If
these macros are defined, the boot loader uses them.
*/

#define TINY85_HARDWARE_CONFIG_1  1
#define TINY85_HARDWARE_CONFIG_2  2
#define TINY84_HARDWARE_CONFIG_1  3
#define TINY841_HARDWARE_CONFIG_1  3

/* ---------------------------- Hardware Config ---------------------------- */
//#define HARDWARE_CONFIG     TINY85_HARDWARE_CONFIG_2
#define HARDWARE_CONFIG     TINY841_HARDWARE_CONFIG_1

#define USB_CFG_IOPORTNAME      B
/* This is the port where the USB bus is connected. When you configure it to
 * "B", the registers PORTB, PINB and DDRB will be used.
 */


#if HARDWARE_CONFIG == TINY84_HARDWARE_CONFIG_1
# define USB_CFG_DMINUS_BIT      0
/* This is the bit number in USB_CFG_IOPORT where the USB D- line is connected.
 * This may be any bit in the port.
 */
# define USB_CFG_DPLUS_BIT       1
/* This is the bit number in USB_CFG_IOPORT where the USB D+ line is connected.
 * This may be any bit in the port, but must be configured as a pin change interrupt.
 */
 #endif

 
#if HARDWARE_CONFIG == TINY85_HARDWARE_CONFIG_1
# define USB_CFG_DMINUS_BIT      0
/* This is the bit number in USB_CFG_IOPORT where the USB D- line is connected.
 * This may be any bit in the port.
 */
# define USB_CFG_DPLUS_BIT       2
/* This is the bit number in USB_CFG_IOPORT where the USB D+ line is connected.
 * This may be any bit in the port, but must be configured as a pin change interrupt.
 */
#endif
 
#if HARDWARE_CONFIG == TINY85_HARDWARE_CONFIG_2
# define USB_CFG_DMINUS_BIT      3
/* This is the bit number in USB_CFG_IOPORT where the USB D- line is connected.
 * This may be any bit in the port.
 */
# define USB_CFG_DPLUS_BIT       4
/* This is the bit number in USB_CFG_IOPORT where the USB D+ line is connected.
 * This may be any bit in the port, but must be configured as a pin change interrupt.
 */
 #endif


#define USB_CFG_CLOCK_KHZ       (F_CPU/1000)
/* Clock rate of the AVR in MHz. Legal values are 12000, 16000 or 16500.
 * The 16.5 MHz version of the code requires no crystal, it tolerates +/- 1%
 * deviation from the nominal frequency. All other rates require a precision
 * of 2000 ppm and thus a crystal!
 * Default if not specified: 12 MHz
 */

/* ----------------------- Optional Hardware Config ------------------------ */

/* #define USB_CFG_PULLUP_IOPORTNAME   D */
/* If you connect the 1.5k pullup resistor from D- to a port pin instead of
 * V+, you can connect and disconnect the device from firmware by calling
 * the macros usbDeviceConnect() and usbDeviceDisconnect() (see usbdrv.h).
 * This constant defines the port on which the pullup resistor is connected.
 */
/* #define USB_CFG_PULLUP_BIT          4 */
/* This constant defines the bit number in USB_CFG_PULLUP_IOPORT (defined
 * above) where the 1.5k pullup resistor is connected. See description
 * above for details.
 */

/* ------------------------------------------------------------------------- */
/* ---------------------- feature / code size options ---------------------- */
/* ------------------------------------------------------------------------- */

//#define HAVE_EEPROM_PAGED_ACCESS    0
/* If HAVE_EEPROM_PAGED_ACCESS is defined to 1, page mode access to EEPROM is
 * compiled in. Whether page mode or byte mode access is used by AVRDUDE
 * depends on the target device. Page mode is only used if the device supports
 * it, e.g. for the ATMega88, 168 etc. You can save quite a bit of memory by
 * disabling page mode EEPROM access. Costs ~ 138 bytes.
 */
//#define HAVE_EEPROM_BYTE_ACCESS     0
/* If HAVE_EEPROM_BYTE_ACCESS is defined to 1, byte mode access to EEPROM is
 * compiled in. Byte mode is only used if the device (as identified by its
 * signature) does not support page mode for EEPROM. It is required for
 * accessing the EEPROM on the ATMega8. Costs ~54 bytes.
 */
#define BOOTLOADER_CAN_EXIT         1
/* If this macro is defined to 1, the boot loader will exit shortly after the
 * programmer closes the connection to the device. Costs ~36 bytes.
 * Required for TINY85MODE
 */
 
//#define HAVE_CHIP_ERASE             0
/* If this macro is defined to 1, the boot loader implements the Chip Erase
 * ISP command. Otherwise pages are erased on demand before they are written.
 */
//#define SIGNATURE_BYTES             0x1e, 0x93, 0x0b, 0     /* ATtiny85 */
/* This macro defines the signature bytes returned by the emulated USBasp to
 * the programmer software. They should match the actual device at least in
 * memory size and features. If you don't define this, values for ATMega8,
 * ATMega88, ATMega168 and ATMega328 are guessed correctly.
 */

/* The following block guesses feature options so that the resulting code
 * should fit into 2k bytes boot block with the given device and clock rate.
 * Activate by passing "-DUSE_AUTOCONFIG=1" to the compiler.
 * This requires gcc 3.4.6 for small enough code size!
 */
// #if USE_AUTOCONFIG
// #   undef HAVE_EEPROM_PAGED_ACCESS
// #   define HAVE_EEPROM_PAGED_ACCESS     (USB_CFG_CLOCK_KHZ >= 16000)
// #   undef HAVE_EEPROM_BYTE_ACCESS
// #   define HAVE_EEPROM_BYTE_ACCESS      1
// #   undef BOOTLOADER_CAN_EXIT
// #   define BOOTLOADER_CAN_EXIT          1
// #   undef SIGNATURE_BYTES
// #endif /* USE_AUTOCONFIG */

/* ------------------------------------------------------------------------- */

/* Example configuration: Port D bit 3 is connected to a jumper which ties
 * this pin to GND if the boot loader is requested. Initialization allows
 * several clock cycles for the input voltage to stabilize before
 * bootLoaderCondition() samples the value.
 * We use a function for bootLoaderInit() for convenience and a macro for
 * bootLoaderCondition() for efficiency.
 */

#define JUMPER_BIT  0   /* jumper is connected to this bit in port B, active low */

/* tiny85 Architecture Specifics */
/*
#ifndef __AVR_ATtiny85__
#  error "uBoot is only designed for attiny85"
#endif
*/
#define TINYVECTOR_RESET_OFFSET     4
#define TINYVECTOR_USBPLUS_OFFSET   2
#define TINYVECTOR_OSCCAL_OFFSET    6

#ifdef TINY84_HARDWARE_CONFIG_1

	// Attiny 841 redirection
	#ifndef OSCCAL
	#define OSCCAL OSCCAL0
	#endif
// setup interrupt for Pin Change for D+
// Uses Pin Change Interrupt 1

	#define RESET_VECTOR_OFFSET         0
	#define USBPLUS_VECTOR_OFFSET       3

// PCINT1
		
	#define USB_INTR_CFG            PCMSK1
	#define USB_INTR_CFG_SET        (1 << USB_CFG_DPLUS_BIT)
	#define USB_INTR_CFG_CLR        0
	#define USB_INTR_ENABLE         GIMSK
	#define USB_INTR_ENABLE_BIT     PCIE1
	#define USB_INTR_PENDING        GIFR
	#define USB_INTR_PENDING_BIT    PCIF1
	#define USB_INTR_VECTOR         PCINT1_vect
#else
// ATtiny85
#define TINY85MODE
// setup interrupt for Pin Change for D+
// number of bytes before the boot loader vectors to store the tiny application vector table

	#define RESET_VECTOR_OFFSET         0
	#define USBPLUS_VECTOR_OFFSET       2

	#define USB_INTR_CFG            PCMSK
	#define USB_INTR_CFG_SET        (1 << USB_CFG_DPLUS_BIT)
	#define USB_INTR_CFG_CLR        0
	#define USB_INTR_ENABLE         GIMSK
	#define USB_INTR_ENABLE_BIT     PCIE
	#define USB_INTR_PENDING        GIFR
	#define USB_INTR_PENDING_BIT    PCIF
	#define USB_INTR_VECTOR         PCINT0_vect
#endif

 
// uncomment for chips with clkdiv8 enabled in fuses
//#define LOW_POWER_MODE 1

// set clock prescaler to a value before running user program
//#define SET_CLOCK_PRESCALER _BV(CLKPS0) /* divide by 2 for 8mhz */


#ifdef BUILD_JUMPER_MODE
  #define START_JUMPER_PIN 5
  #define digitalRead(pin) (PINB & _BV(pin))
  #define bootLoaderStartCondition() (!digitalRead(START_JUMPER_PIN))
  #define bootLoaderCondition() 1
  
  #ifndef __ASSEMBLER__   /* assembler cannot parse function definitions */
    static inline void  bootLoaderInit(void) {
      // DeuxVis pin-5 pullup
      PORTB |= _BV(START_JUMPER_PIN); // has pullup enabled
      _delay_ms(10);
    }
    static inline void  bootLoaderExit(void) {
      // DeuxVis pin-5 pullup
      PORTB = 0;
    }
  #endif /* __ASSEMBLER__ */
    
#else
  #define bootLoaderInit()
  #define bootLoaderExit()
  #define bootLoaderCondition()   (++idlePolls < (AUTO_EXIT_MS * 10UL))
  #if LOW_POWER_MODE
    // only starts bootloader if USB D- is pulled high on startup - by putting your pullup in to an external connector
    // you can avoid ever entering an out of spec clock speed or waiting on bootloader when that pullup isn't there
    #define bootLoaderStartCondition() \
      (PINB & (_BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DMINUS_BIT))) == _BV(USB_CFG_DMINUS_BIT)
  #else
    #define bootLoaderStartCondition() 1
//    #define bootLoaderStartCondition() (MCUSR&_BV(EXTRF))  // enter only after external reset. Needs pull up on RS pin!

	#endif
#endif

/* ----------------------- Optional MCU Description ------------------------ */

/* The following configurations have working defaults in usbdrv.h. You
 * usually don't need to set them explicitly. Only if you want to run
 * the driver on a device which is not yet supported or with a compiler
 * which is not fully supported (such as IAR C) or if you use a different
 * interrupt than INT0, you may have to define some of these.
 */
/* #define USB_INTR_CFG            MCUCR */
/* #define USB_INTR_CFG_SET        ((1 << ISC00) | (1 << ISC01)) */
/* #define USB_INTR_CFG_CLR        0 */
/* #define USB_INTR_ENABLE         GIMSK */
/* #define USB_INTR_ENABLE_BIT     INT0 */
/* #define USB_INTR_PENDING        GIFR */
/* #define USB_INTR_PENDING_BIT    INTF0 */
/* #define USB_INTR_VECTOR         INT0_vect */

// todo: change to pin 5
//#define DEUXVIS_JUMPER_PIN 5
//#define digitalRead(pin) ((PINB >> pin) & 0b00000001)
//#define bootLoaderStartCondition() (!digitalRead(DEUXVIS_JUMPER_PIN))
//#define bootLoaderCondition()   (1)

#ifndef __ASSEMBLER__   /* assembler cannot parse function definitions */

/*
 * Define bootloader timeout value. 
 * 
 *  These will only be used if is bootLoaderCondition() evaluates idlePolls below!
 * 
 *  AUTO_EXIT_NO_USB_MS        The bootloader will exit after this delay if no USB is connected.
 *                             Set to 0 to disable
 *                             Adds ~6 bytes.
 *                             (This will wait for an USB SE0 reset from the host)
 *  AUTO_EXIT_MS               The bootloader will exit after this delay if no USB communication
 *                             from the host tool was received.
 *  
 *  All values are approx. in milliseconds
 */

#define AUTO_EXIT_NO_USB_MS    0
#define AUTO_EXIT_MS           6000

 /*
 *	Defines the setting of the RC-oscillator calibration after quitting the bootloader. (OSCCAL)
 * 
 *  OSCCAL_RESTORE            Set this to '1' to revert to factory calibration, which is 16.0 MHZ +/-10%
 *                            Adds ~14 bytes.
 *
 *  OSCCAL_16.5MHz            Set this to '1' to use the same calibration as during program upload.
 *                            This value is 16.5Mhz +/-1% as calibrated from the USB timing. Please note
 *                            that only true if the ambient temperature does not change.
 *                            This is the default behaviour of the Digispark.
 *                            Adds ~38 bytes.
 *
 *  If both options are selected, OSCCAL_RESTORE takes precedence.
 *
 *  If no option is selected, OSCCAL will be left untouched and stay at either 16.0Mhz or 16.5Mhz depending
 *  on whether the bootloader was activated. This will take the least memory. You can use this if your program
 *  comes with its own OSCCAL calibration or an external clock source is used.
 */
 
 #define OSCCAL_RESTORE 1
 #define OSCCAL_16_5MHz 0
 
/*  
 *  Defines handling of an indicator LED while the bootloader is active.  
 * 
 *  LED_PRESENT               Set this this to '1' to active all LED related code. If this is 0, all other
 *                            defines are ignored.
 *                            Adds 18 bytes depending on implementation.								
 *
 *  LED_DDR,LED_PORT,LED_PIN  Where is you LED connected?
 *
 *  LED_INIT                  Called once after bootloader entry
 *  LED_EXIT                  Called once during bootloader exit
 *  LED_MACRO                 Called in the main loop with the idle counter as parameter.
 *                            Use to define pattern.
 */ 

#define	LED_PRESENT	    1

#define	LED_DDR			DDRB
#define LED_PORT		PORTB
#define	LED_PIN			PB2

#define LED_INIT(x)		LED_PORT &=~_BV(LED_PIN);
#define LED_EXIT(x)		LED_DDR  &=~_BV(LED_PIN);
#define LED_MACRO(x)	if ( x & 0xd ) {LED_DDR&=~_BV(LED_PIN);} else {LED_DDR|=_BV(LED_PIN);}

#endif /* __ASSEMBLER__ */


/* ------------------------------------------------------------------------- */

#endif /* __bootloader_h_included__ */
