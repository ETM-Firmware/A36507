 /*********************************************************************
 *
 *	Hardware specific definitions for:
 *    - Explorer 16
 *    - PIC24F, PIC24H, dsPIC33F
 *    - Ethernet PICtail Plus (ENC28J60)
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    Compiler.h
 * Processor:       PIC24F, PIC24H, dsPIC30F, dsPIC33F
 * Compiler:        Microchip C30 v3.24 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2010 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		09/16/2010	Regenerated for specific boards
 ********************************************************************/
#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include <xc.h>

// Define a macro describing this hardware set up (used in other files)
#define ETM_ENC28J60

// Clock frequency values
#define MAXIMUM_PIC_FREQ		(80000000ul)

// These directly influence timed events using the Tick module.  They also are used for UART and SPI baud rate generation.
#define GetSystemClock()		(MAXIMUM_PIC_FREQ)	// Hz
#define GetInstructionClock()	        (GetSystemClock()/4)	// Normally GetSystemClock()/4 for PIC18, GetSystemClock()/2 for PIC24/dsPIC, and GetSystemClock()/1 for PIC32.  Might need changing if using Doze modes.
//#define GetPeripheralClock()	        (GetSystemClock()/4)	// Normally GetSystemClock()/4 for PIC18, GetSystemClock()/2 for PIC24/dsPIC, and GetSystemClock()/1 for PIC32.  Divisor may be different if using a PIC32 since it's configurable.

// Hardware I/O pin mappings
// LEDs
// DPARKER - Why are these LEDs mapped to this module???  Why does the Ethernet module need access to the LEDs???
//#define LEDA_TRIS			(TRISGbits.TRISG12)	// Ref D3
//#define LEDA_IO				(LATGbits.LATG12)
//#define LEDB_TRIS			(TRISGbits.TRISG13)	// Ref D4
//#define LEDB_IO				(LATGbits.LATG13)
//#define LEDOP_TRIS			(TRISAbits.TRISA7)	// Ref D5
//#define LEDOP_IO		   	(LATAbits.LATA7)



// ENC28J60 I/O pins

#define ENC_CS_TRIS			(TRISDbits.TRISD15)	
#define ENC_CS_IO			(LATDbits.LATD15)

#define ENC_RST_TRIS		        (TRISAbits.TRISA15)
#define ENC_RST_IO		        (LATAbits.LATA15)

// SPI SCK, SDI, SDO pins are automatically controlled by the 
// PIC24/dsPIC SPI module 
#define ENC_SPI_IF			(IFS0bits.SPI1IF)
#define ENC_SSPBUF			(SPI1BUF)
#define ENC_SPISTAT			(SPI1STAT)
#define ENC_SPISTATbits                 (SPI1STATbits)
#define ENC_SPICON1			(SPI1CON)
#define ENC_SPICON1bits                 (SPI1CONbits)
#define ENC_SPICON2			(SPI2CON)


#endif // #ifndef HARDWARE_PROFILE_H
