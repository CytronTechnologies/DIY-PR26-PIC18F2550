//==========================================================================
//	Author				: Cytron Technologies		
//	Project				: USB Game Controller
//	Project description             : This firmware implements a standard USB game controller with
//                                        12 inputs (4 directional buttons and 8 general purpose buttons).
//                                        This project use PIC18F255.Competible with MPLAB and MPLABX
//                                        Compiler only with Microchip C18.
//                                        This project modify the program provided by Microchip Technology, Inc.

//==========================================================================

//	include
//==========================================================================
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include "HardwareProfile.h"
#include "usb_function_hid.h"

//	configuration
//==========================================================================
#ifndef USBMOUSE_C
#define USBMOUSE_C

#if defined(PICDEM_FS_USB)                  // Configuration bits for PICDEM FS USB Demo Board (based on PIC18F4550)
        #pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
        #pragma config CPUDIV   = OSC1_PLL2   
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON         //USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = ON
        #pragma config LPT1OSC  = OFF
        #pragma config PBADEN   = OFF
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
        #pragma config CPB      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
        #pragma config EBTRB    = OFF

#elif defined(PIC18F87J50_PIM)              // Configuration bits for PIC18F87J50 FS USB Plug-In Module board
        #pragma config XINST    = OFF       // Extended instruction set
        #pragma config STVREN   = ON        // Stack overflow reset
        #pragma config PLLDIV   = 3         // (12 MHz crystal used on this board)
        #pragma config WDTEN    = OFF       // Watch Dog Timer (WDT)
        #pragma config CP0      = OFF       // Code protect
        #pragma config CPUDIV   = OSC1      // OSC1 = divide by 1 mode
        #pragma config IESO     = OFF       // Internal External (clock) Switchover
        #pragma config FCMEN    = OFF       // Fail Safe Clock Monitor
        #pragma config FOSC     = HSPLL     // Firmware must also set OSCTUNE<PLLEN> to start PLL!
        #pragma config WDTPS    = 32768
        #pragma config MSSPMSK  = MSK5
        #pragma config CCP2MX   = DEFAULT   

#elif defined(PIC18F46J50_PIM)
     #pragma config WDTEN = OFF          //WDT disabled (enabled by SWDTEN bit)
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
     #pragma config STVREN = ON          //stack overflow/underflow reset enabled
     #pragma config XINST = OFF          //Extended instruction set disabled
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
     #pragma config CP0 = OFF            //Program memory is not code-protected
     #pragma config OSC = HSPLL          //HS oscillator, PLL enabled, HSPLL used by USB
     #pragma config T1DIG = ON           //Sec Osc clock source may be selected
     #pragma config LPT1OSC = OFF        //high power Timer1 mode
     #pragma config FCMEN = OFF          //Fail-Safe Clock Monitor disabled
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
     #pragma config WDTPS = 32768        //1:32768
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as clock
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as clock
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep
     #pragma config DSWDTEN = OFF        //Disabled
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
     #pragma config IOL1WAY = OFF        //IOLOCK bit can be set and cleared
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
     #pragma config WPEND = PAGE_0       //Start protection at page 0
     #pragma config WPCFG = OFF          //Write/Erase last page protect Disabled
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored 
#elif defined(LOW_PIN_COUNT_USB_DEVELOPMENT_KIT)

//	PIC18F14K50
        #pragma config CPUDIV = NOCLKDIV
        #pragma config USBDIV = OFF
        #pragma config FOSC   = HS
        #pragma config PLLEN  = ON
        #pragma config FCMEN  = OFF
        #pragma config IESO   = OFF
        #pragma config PWRTEN = OFF
        #pragma config BOREN  = OFF
        #pragma config BORV   = 30
        #pragma config WDTEN  = OFF
        #pragma config WDTPS  = 32768
        #pragma config MCLRE  = OFF
        #pragma config HFOFST = OFF
        #pragma config STVREN = ON
        #pragma config LVP    = OFF
        #pragma config XINST  = OFF
        #pragma config BBSIZ  = OFF
        #pragma config CP0    = OFF
        #pragma config CP1    = OFF
        #pragma config CPB    = OFF
        #pragma config WRT0   = OFF
        #pragma config WRT1   = OFF
        #pragma config WRTB   = OFF
        #pragma config WRTC   = OFF
        #pragma config EBTR0  = OFF
        #pragma config EBTR1  = OFF
        #pragma config EBTRB  = OFF        
        

#elif defined(EXPLORER_16)
    #ifdef __PIC24FJ256GB110__ //Defined by MPLAB when using 24FJ256GB110 device
        _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
        _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV2 & IOL1WAY_ON)
    #elif defined(__32MX460F512L__)
        #pragma config UPLLEN   = ON        // USB PLL Enabled
        #pragma config FPLLMUL  = MUL_15        // PLL Multiplier
        #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
        #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
        #pragma config FPLLODIV = DIV_1         // PLL Output Divider
        #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
        #pragma config FWDTEN   = OFF           // Watchdog Timer
        #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
        #pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
        #pragma config OSCIOFNC = OFF           // CLKO Enable
        #pragma config POSCMOD  = HS            // Primary Oscillator
        #pragma config IESO     = OFF           // Internal/External Switch-over
        #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
        #pragma config FNOSC    = PRIPLL        // Oscillator Selection
        #pragma config CP       = OFF           // Code Protect
        #pragma config BWP      = OFF           // Boot Flash Write Protect
        #pragma config PWP      = OFF           // Program Flash Write Protect
        #pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
        #pragma config DEBUG    = ON            // Background Debugger Enable
    #else
        #error No hardware board defined, see "HardwareProfile.h" and __FILE__
    #endif
#elif defined(PIC24F_STARTER_KIT)
    _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
    _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV3 & IOL1WAY_ON)
#elif defined(PIC32_USB_STARTER_KIT)
    #pragma config UPLLEN   = ON        // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_15        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLODIV = DIV_1         // PLL Output Divider
    #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
    #pragma config FWDTEN   = OFF           // Watchdog Timer
    #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
    #pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF           // CLKO Enable
    #pragma config POSCMOD  = HS            // Primary Oscillator
    #pragma config IESO     = OFF           // Internal/External Switch-over
    #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL        // Oscillator Selection
    #pragma config CP       = OFF           // Code Protect
    #pragma config BWP      = OFF           // Boot Flash Write Protect
    #pragma config PWP      = OFF           // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
    #pragma config DEBUG    = ON            // Background Debugger Enable
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif

//	define
//================================================================================									
//output->LAT, input->PORT
#define LED		LATBbits.LATB7 								//RB7=output
#define SELECT		PORTAbits.RA4 								//RA4=input
#define START		PORTBbits.RB5 								//RB5=input
#define b_1		PORTCbits.RC7 								//RC7=input
#define b_2		PORTCbits.RC1 								//RC1=input
#define b_3		PORTCbits.RC6 								//RC6=input
#define b_4		PORTCbits.RC2 								//RC2=input
#define b_up		PORTBbits.RB2 								//RB2=input
#define b_down		PORTBbits.RB0 								//RB0=input
#define b_right		PORTBbits.RB3 								//RB3=input
#define b_left		PORTBbits.RB1 								//RB1=input
#define analog_sw	PORTBbits.RB4								//RB4=input
#define BUTTON1		PORTCbits.RC0								//RC0=input
#define BUTTON2		PORTAbits.RA5								//RA5=input
#define	CHANNEL0	0b00000001									// AN0
#define	CHANNEL1	0b00000101									// AN1
#define	CHANNEL2	0b00001001									// AN2
#define	CHANNEL3	0b00001101									// AN3
#define ADGO ADCON0bits.GO

//	variable
//================================================================================	
#pragma udata
BYTE old_sw2,old_sw3;
BOOL emulate_mode;
BYTE movement_length;
BYTE vector = 0;
char buffer[3];
USB_HANDLE lastTransmission;
unsigned char analog=0, adc_progress=0, step=0;

//The direction that the mouse will move in
ROM signed char dir_table[]={-4,-4,-4, 0, 4, 4, 4, 0};

//	private prototype
//================================================================================	
//void BlinkUSBStatus(void);
void Emulate_Joystick(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void choose_adc(unsigned char channel);

//	variable remapping
//================================================================================	
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
        #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif
	
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

#elif defined(__C30__)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
    #endif
#endif

//	declaration
//================================================================================	
#pragma code

//	main function					(main fucntion of the program)
//==========================================================================
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.
        #endif
    				  
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


// static void InitializeSystem(void)       
// (initilize system is a centralize initailization, all required USB initialization routines are called from here.)
//==========================================================================
static void InitializeSystem(void)
{
    #if (defined(__18CXX) & !defined(PIC18F87J50_PIM))

	//ADC Configuration
	ADCON0=0x01;				//enable ADON
	ADCON1=0x0B;				//use only 4 analog pins
	ADCON2=0b00110101;			//use left justified for A/D Result Format Select bit 

	//Tris configuration (input or output)	INPUT=1, OUTPUT=0
	TRISA = 0b00111111;			//set RA2, RA3, RA4 and RA5 as input, else as output
	TRISB = 0b00111111;			//set RB0, RB1, RB2 RB3 and RB4 as input, else as output
	TRISC = 0b11001111;			//set PORTC pin RC4 and RC5 as output else as input

    #elif defined(__C30__)
        AD1PCFGL = 0xFFFF;
    #elif defined(__C32__)
        AD1PCFG = 0xFFFF;
    #endif

    #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
	//On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
	//This allows the device to power up at a lower initial operating frequency, which can be
	//advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
	//operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
	//power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif

    #if defined(PIC18F87J50_PIM)
	//Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
	//use the ANCONx registers to control this, which is different from other devices which
	//use the ADCON1 register for this purpose.
    WDTCONbits.ADSHR = 1;           // Select alternate SFR location to access ANCONx registers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    WDTCONbits.ADSHR = 0;           // Select normal SFR locations
    #endif

    #if defined(PIC18F46J50_PIM)
	//Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
	//use the ANCONx registers to control this, which is different from other devices which
	//use the ADCON1 register for this purpose.
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
    
//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.

    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();                //usb_device.c.  Initializes USB module SFRs and firmware
                                    //variables to known states.
}//end InitializeSystem


// Initailization
//==============================================================================
void UserInit(void)
{
    //Initialize all of the mouse data to 0,0,0 (no movement)
    buffer[0]=buffer[1]=buffer[2]=0;

    //enable emulation mode.  This means that the game controller data will be send to 
    //the PC causing the indicator to move. If this is set to FALSE then resulting in 
	//no game controller action
    emulate_mode = TRUE;
    
    //initialize the variable holding the handle for the last transmission
    lastTransmission = 0;
}//end UserInit


//	ProcessIO functions
//=============================================================================
void ProcessIO(void)
{   
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    //Call the function that emulates the joystick
    Emulate_Joystick();
    
}//end ProcessIO


//	Emulate_joystick functions      
//  (The generic function HIDTxPacket() is used to send HID IN packets over USB to the host.     
//=============================================================================
void Emulate_Joystick(void)
{ 
	unsigned short i; 
	unsigned char direction, angle, old_analog_sw;
	static unsigned char x_value, y_value, z_value, Rz_value;

	//switch between activate the analog and deactivated the analog
	if((analog_sw==0)&&(old_analog_sw==1))				
	{
		analog ^= 1;					//analog switched
		LED ^= 1;					//led switched
		old_analog_sw=0;				//reset old_analog_sw as 0
	}
	else old_analog_sw = analog_sw;				//if analog switch is not pressed, do nothing

	//analog was deactivated
	if (analog==0)										
	{	
		LED=0;						//led off
		hid_report_in[2]=127;				//initialized the z_value 
		hid_report_in[3]=127;				//initialized the Rz_value

								//direction:	7	0	1
                                                                //				6		2
		//direction					//				5	4	3
		if(!b_up)direction=0;				//if b_up is pressed, point to direction 0
		else if(!b_right)direction=2;			//if b_right is pressed, point to direction 2
		else if(!b_down)direction=4;			//if b_down is pressed, point to direction 4
		else if(!b_left)direction=6;			//if b_left is pressed, point to direction 6
		else direction=8;				//if button is not pressed, point to center
	
		if((!b_up)&&(!b_right))direction=1;			//if b_up and b_right are pressed, point to direction 1
		else if((!b_right)&&(!b_down))direction=3;		//if b_right and b_down are pressed, point to direction 3
		else if((!b_down)&&(!b_left))direction=5;		//if b_down and b_left are pressed, point to direction 5
		else if((!b_left)&&(!b_up))direction=7;			//if b_left and b_up are pressed, point to direction 7
	
		switch(direction)                                   //x:0-255(from left to right), y:0-255(from up to down)
		{
			case 0: hid_report_in[0]=127;               //send report to x-axis, x=127
					hid_report_in[1]=0;         //send report to y-axis, y=0
					break;
			case 1: hid_report_in[0]=255;               //x=255
					hid_report_in[1]=0;         //y=0
					break;
			case 2: hid_report_in[0]=255;               //x=255
					hid_report_in[1]=127;       //y=127
					break;
			case 3: hid_report_in[0]=255;               //x=255
					hid_report_in[1]=255;       //y=255
					break;
			case 4: hid_report_in[0]=127;               //x=127
					hid_report_in[1]=255;       //=255
					break;
			case 5: hid_report_in[0]=0;                 //x=0
					hid_report_in[1]=255;       //y=255
					break;	
			case 6: hid_report_in[0]=0;                 //x=0
					hid_report_in[1]=127;       //y=127
					break;	
			case 7: hid_report_in[0]=0;                 //x=0
					hid_report_in[1]=0;         //y=0
					break;
			case 8: hid_report_in[0]=127;               //x=127
					hid_report_in[1]=127;       //y=127
					break;		
		}	
	}

	//analog was activated
	else												
	{
		LED=1;							//led on

									//hat switch direction:	7	0	1
                                                                        //						6 		2
		//Hat switch                                            //						5	4	3
		if(!b_up)hid_report_in[4]=0x00;				//if b_up is pressed, send report to hat switch, point to 0
		else if(!b_right)hid_report_in[4]=0x02;			//if b_right is pressed, point to 2
		else if(!b_down)hid_report_in[4]=0x04;			//if b_down is pressed, point to 4
		else if(!b_left)hid_report_in[4]=0x06;			//if b_left is pressed, point to 6
		else hid_report_in[4]=0x08;				//if no button is pressed, point to 8			

		if((!b_up)&&(!b_right))hid_report_in[4]=0x01;			//if b_up and b_right are pressed, send report to hat switch, point to direction 1
		else if((!b_right)&&(!b_down))hid_report_in[4]=0x03;            //if b_right and b_down are pressed, point to direction 3
		else if((!b_down)&&(!b_left))hid_report_in[4]=0x05;		//if b_down and b_left are pressed, point to direction 5
		else if((!b_left)&&(!b_up))hid_report_in[4]=0x07;		//if b_left and b_up are pressed, point to direction 7

		//analog stick
		switch(step)									
		{
			case 0: if(adc_progress==0)				//when adc_progress is 0
					{
						ADCON0=CHANNEL2;		//channel 2 is selected
						ADCON0bits.GO=1;		//ADC start to process 
						adc_progress=1;			//set adc_progress as 1
					}
					if(ADCON0bits.GO==0)			//when adc finish process				
					{
						adc_progress=0;			//set adc_progress as 0
						x_value = 255-ADRESH;		//save x_value as ADRESH (255-ADRESH due to the wrong connection of the hardware)
						step=1;				//jump to step 1
					}
					break;
	
			case 1: if(adc_progress==0)				//when adc_progress is 0
					{
						ADCON0=CHANNEL3;		//channel 3 is selected
						ADCON0bits.GO=1;		//ADC start to process
						adc_progress=1;			//set adc_progress as 1
					}
					if(ADCON0bits.GO==0)                    //when adc finish process
					{
						adc_progress=0;			//set adc_progress as 0
						y_value = ADRESH;		//save y_value as ADRESH
						step=2;				//jump to step 2
					}
					break;	
	
			case 2: if(adc_progress==0)				//when adc_progress is 0
					{
						ADCON0=CHANNEL0;		//channel 0 is selected
						ADCON0bits.GO=1;		//ADC start to process
						adc_progress=1;			//set adc_progress as 1
					}
					if(ADCON0bits.GO==0)                    //when adc process finish
					{
						adc_progress=0;			//set adc_progress as 0
						z_value = ADRESH;		//save z_value as ADRESH
						step=3;				//jump to step 3
					}
					break;	
	
			case 3: if(adc_progress==0)				//when adc_progress is 0
					{
						ADCON0=CHANNEL1;		//channel 1 is selected
						ADCON0bits.GO=1;		//ADC start to process
						adc_progress=1;			//set adc_progress as 1
					}
					if(ADCON0bits.GO==0)			//when adc process finish
					{
						adc_progress=0;			//set adc_progress as 0
						Rz_value = ADRESH;		//save Rz_value as ADRESH
						step=0;				//jump to step 0
					}
					break;	
		}	
		hid_report_in[0]=x_value;				//send x_value to computer
		hid_report_in[1]=y_value;				//send y_value to computer
		hid_report_in[2]=z_value;				//send z_value to computer
		hid_report_in[3]=Rz_value;				//send Rz_value to computer
	}

	//button 1:up1		button 2:right1		button 3:down1		button 4:left1		
	//button	//button 5:start	button 6:select		button 7:button1	button 8:button2
	hid_report_in[5]=0x00;                          //initialized all button as 0
	
	if(!b_1)hid_report_in[5] |=0x01;		//if b_1 is pressed, send report to button and set bit0 as 1
		else hid_report_in[5] &=0xFE;		//if not, clear bit0 to 0
	if(!b_2)hid_report_in[5] |=0x02;		//if b_1 is pressed, set bit1 as 1
		else hid_report_in[5] &=0xFD;		//if not, clear bit1 to 0					
	if(!b_3)hid_report_in[5] |=0x04;		//if b_1 is pressed, set bit2 as 1
		else hid_report_in[5] &=0xFB;		//if not, clear bit2 to 0
	if(!b_4)hid_report_in[5] |=0x08;		//if b_1 is pressed, set bit3 as 1
		else hid_report_in[5] &=0xF7;		//if not, clear bit3 to 0 

	if(!SELECT)hid_report_in[5] |=0x10;		//if select is pressed, set bit4 as 1
		else hid_report_in[5] &=0xEF;		//if not, clear bit4 to 0 
	if(!START)hid_report_in[5] |=0x20;		//if start is pressed, set bit5 as 1
		else hid_report_in[5] &=0xDF;		//if not, clear bit5 to 0
	if(!BUTTON2)hid_report_in[5] |=0x40;            //if button2 is pressed, set bit6 as 1
		else hid_report_in[5] &=0xBF;		//if not, clear bit6 to 0 
	if(!BUTTON1)hid_report_in[5] |=0x80;            //if button1 is pressed, set bit7 as 1
		else hid_report_in[5] &=0x7F;				

    if(HIDTxHandleBusy(lastTransmission) == 0)
    {
        //Send the 6 byte packet over USB to the host.
        lastTransmission = HIDTxPacket(HID_EP, (BYTE*)hid_report_in, 0x06);
    }
}//end Emulate_Joystick

//choose_adc function
void choose_adc(unsigned char channel)
{
	ADCON0bits.CHS0=channel & 0x01;
	ADCON0bits.CHS1=(channel & 0x02)>>1;
	ADCON0bits.CHS2=(channel & 0x04)>>2;
	ADCON0bits.CHS3=(channel & 0x08)>>3;
}		


//USB Callback Functions
//=============================================================================

//	void USBCBSuspend(void)   (Call back that is invoked when a USB suspend is detected)  
//=============================================================================
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        TRISA &= 0xFF3F;
        LATAbits.LATA6 = 1;
        Sleep();
        LATAbits.LATA6 = 0;
    #endif
    #endif
}

//	void _USB1Interrupt(void)   
// (This function is called when the USB interrupt bit is set.In this example the interrupt is only used when the device)  
// goes to sleep when it receives a USB suspend command)
//=============================================================================
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {       
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif


//	void USBCBWakeFromSuspend(void)   (This call back is invoked when a wakeup from USB suspend is detected.)  
//=============================================================================
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *******************************************************************/

//	void USBCBErrorHandler(void)   (This call back is invoked when a wakeup from USB suspend is detected.)  
//=============================================================================
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;               // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}

/** EOF mouse.c *************************************************/
#endif
