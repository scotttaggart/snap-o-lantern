/*
snap.c

Written by Windell Oskay, http://www.evilmadscientist.com/ 
Highly nmodified by Scott Taggart, October 2013: 
    - fixed many bugs
    - added 4 more servo levels
    - added mouth "bounce" logic so mouth bounces a bt twice after close to improve animation
    - Improved code flow logic so it's far easier to understand and maintain
    - Added state machine logic so it's possible to do more things, easier
    - Re-coded all hard-coded constants as named constans
    - Made all values easy to tune and, where appropriate, dynamic based on the master clock frequency, etc.
    - fixed code formatting
    - added a bunch more comments
    - removed or replaced dead/bad/misledaing code
    - factored out mouth-close and -open functions

Copyright 2013 Windell H. Oskay
Distributed under the terms of the GNU General Public License, please see below.

An avr-gcc program for the Atmel ATTiny2313 

Date Created:   10/13/07
Last Modified:  10/17/13
 
Purpose: Simple control of an RC servo motor for a robotic pumpkin.
Designed for "snapping" pumpkin servo output on pins B0 or B1.

The servo signal on Port B1 has a  range twice that of the one at B0, 
allowing you to quickly switch (in hardware) the servo output range
even after the chip is programmed.

LED "Eye" outputs: Pins D4, D5

Connect a 1k resistor in series with a Red, orange, or yellow LED on each output D4, D5.

More information about this project is at 
http://wiki.evilmadscientist.com/Snap-O-Lantern
http://shop.evilmadscientist.com/productsmenu/tinykitlist/659

We will run with the 8 MHz internal RC oscillator,
and with the clock prescaler turned off (divide by 1)

Count: System clock.
Available Servo PWM outputs: pins B0 - B5
Servo PWM outputs used: B0, B1 
LED outputs: Pins D4, D5

-------------------------------------------------

 Rev C, October 2013. 
 Updated signal names to reflect changes in AVR libraries.
 Initial delay after turn-on is now 1/4 of the delay, or 5 s from our default of 20s delay.
 
-------------------------------------------------
USAGE: How to compile and install

You will need an AVR programmer and appropriate target board to use this firmware.  

Brand new to AVR?  Start here: http://www.ladyada.net/learn/avrdevtut/index.html
Please also see: http://www.evilmadscientist.com/article.php/avrtargetboards

A makefile is provided to compile and install this program using AVR-GCC and avrdude.

To use it, follow these steps:
1. Update the header of the makefile as needed to reflect the type of AVR programmer that you use.
2. Open a terminal window and move into the directory with this file and the makefile.  
3. At the terminal enter
        make clean   <return>
        make all     <return>
        make program <return>
4. Make sure that avrdude does not report any errors.  If all goes well, the last few lines output by avrdude
should look something like this:

Reading | ################################################## | 100% 0.37s

avrdude: verifying ...
avrdude: 566 bytes of flash verified

avrdude: safemode: lfuse reads as E4
avrdude: safemode: hfuse reads as DF
avrdude: safemode: efuse reads as FF
avrdude: safemode: Fuses OK

avrdude done.  Thank you.

If you do use a different programming environment, make sure that you copy over 
the fuse settings from the makefile.

-------------------------------------------------

This code should be relatively straightforward, so not much documentation is provided.  If you'd like to ask 
questions, suggest improvements, or report success, please use the evilmadscientist forum:
http://www.evilmadscientist.com/forum/

-------------------------------------------------

 This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


*/

#include <avr/io.h>  
#include <avr/interrupt.h> 

#define BIT_MASK(bit_no) (1<<bit_no)

// clock rate is 8MHz = 1024 * 1024 * 8 = 8388608
// clock division from TCCR0B is 5(1024): 8388608 / 1024 = 8192 clocks per "tick"
// clock counter register overflows every 256 "ticks" so,
// 8388608 / 1024 / 256 = 32 times per second is our tick frequency
#define CPU_CLOCK_RATE_MHZ          (8U)                // Rate of MCU clock, in MHz
#define CLOCK_DIVISOR               (5U)
#define TICK_FREQ                   (32U)               // main control loop frequency
#define TICK_PER_SEC                (TICK_FREQ)         // main control loop frequency

#define SECS_BETWEEN_MOUTH_OPENS    (30U)                                       // # seconds between mouth opens
#define TICKS_BETWEEN_MOUTH_OPENS   (SECS_BETWEEN_MOUTH_OPENS * TICK_PER_SEC)	// # of 1/30 s intervals to wait before snapping 
#define SECS_TO_OPEN_MOUTH          (3U)                                        // time it takes to open the mouth
#define TICKS_TO_OPEN_MOUTH         (SECS_TO_OPEN_MOUTH * TICK_PER_SEC)         // amount of time to bounch mouth open
#define TICKS_TO_BOUNCE_MOUTH       (TICKS_TO_OPEN_MOUTH/2)                     // effectively the mouth bounce height as a percentage of opening height, bigger values (smaller divisor) mean bigger bounce
#define TICKS_BOUNCE_MOUTH_CLOSED   (20)                                        // time when mouth is closed between bounces
#define TICKS_PER_BOUNCE_MOUTH_JUMP (TICKS_TO_BOUNCE_MOUTH/5)                   // conrols speed of mouth bounce, smaller values (bigger divisor) mean slower

#define CLOCK_SCALE(f)              (((long)f*(long)1024)/(long)1000)           // the clocks run a little fast which means our servo signals end up a bit short if we don't adjust
#define NUM_SERVOS                  (6U)
#define SERVO_BASE_VALUE_US         (CLOCK_SCALE(1000))         // base value added to each servo pulse-width - 1000 = 1ms
#define SIGNAL_UP_DELAY_US          (50U)                       // short delay between programming timers and servo signal up interrupt... unsure why this is needed


//  PORT D values
#define LED_1               BIT_MASK(PD5)
#define LED_2               BIT_MASK(PD4)
#define LED_3               BIT_MASK(PD3)
#define LED_4               BIT_MASK(PD2)
#define LED_ENABLE_MASK     (LED_1 | LED_2 | LED_3 | LED_4 )

#define EYE_LEDS_ON         (LED_1 | LED_2 )

#define LED3_ON()   PORTD = (PORTD & ~LED_3) | (LED_3)
#define LED3_OFF()  PORTD = (PORTD & ~LED_3)
#define LED4_ON()   PORTD = (PORTD & ~LED_4) | (LED_4)
#define LED4_OFF()  PORTD = (PORTD & ~LED_4)
#define PEIZO_ON()  PORTD = (PORTD & ~LED_3) | (LED_3)
#define PEIZO_OFF() PORTD = (PORTD & ~LED_3)

#define FALSE   (0)
#define TRUE    (1)

// Global variables
volatile unsigned short CUR_SERVO_INDEX;       // Varies over 0-5, which servo output we are using.
volatile unsigned short PulseDone;          // boolean flag

void OpenMouth( unsigned int MounthOpeningTick );
void CloseMouth();

//
// interrupt handlers
// 

// raise signal on port B
ISR(TIMER1_COMPA_vect)
{
    PORTB = (1 << CUR_SERVO_INDEX);
}

// drop signal on port B
ISR(TIMER1_COMPB_vect)
{
    PORTB = 0;
    PulseDone = 1;
}

typedef enum
{
    STATE_WAITING_OPEN,
    STATE_OPENING,
    STATE_WAITING_TO_BOUNCE_FIRST,
    STATE_BOUNCING_FIRST,
    STATE_WAITING_TO_BOUNCE_SECOND,
    STATE_BOUNCING_SECOND,
    STATE_FINAL_CLOSE,
} STATE;

unsigned int servo_position[NUM_SERVOS];        // Normals values should range 200+- thru 1000+-. 
// these are the max postions for each servo line -- set them to any values you like
// they do not have to be in pin-order...  They can even be the same, if needed
// The SERVO_BASE_VALUE_US value is added to each one.
// All values are microseconds.
unsigned int servo_max_position[NUM_SERVOS] =
{
    // each of these times in us is added to the SERVO_BASE_VALUE_US to set the maximum
    // pulse width for each respective servo.
    // So, each servo range is from SERVO_BASE_VALUE_US to SERVO_BASE_VALUE_US + the value below
    // For example, if the first entry below ocntains 300, and SERVO_BASE_VALUE_US=1000, then the
    // range would be 1000-1300us for the first servo.
    CLOCK_SCALE(350),            // PB0
    CLOCK_SCALE(550),            // PB1
    CLOCK_SCALE(750),            // PB2
    CLOCK_SCALE(950),            // PB3
    CLOCK_SCALE(1150),           // PB4
    CLOCK_SCALE(1350),           // PB5
};    // max open position for each servo

int main (void)
{ 

    asm("cli");     // DISABLE global interrupts


    unsigned short i;

    unsigned int CurTicks;

    PORTB = 0;          // All of port B should be off.
    DDRB =  0x3F;       // #0011 1111; PB0 - PB6 are our outputs


    PORTD = 0;
    DDRD =  LED_ENABLE_MASK;    // Pin D4, D5 are our LED outputs


    // Extensible design: can handle up to 6 servo outputs:
    // The position value ranges over 0 - 1000.
    // position[i] PLUS SERVO_BASE_VALUE_US  is the number of microseconds that the
    // control signal is high; it should range from 1-2 ms.

    for( i = 0; i < NUM_SERVOS; ++i )
    {
        servo_position[i] = 10;
    }

    ////////////////////////////////////////////////////
    //
    //Timing controlled by 16-bit timer 1, a free running timer at the CPU clock rate
    //
    ////////////////////////////////////////////////////


    //TCCR1A:
    // Bits 7-4:  0000, Normal port operation, OC1A/B disconnected
    // Bits 1,0: 00, WGM, normal mode
    //TCCR1B:
    // Bit 7: 0: Input noise cancel
    // Bit 6: 0:  Input Capture Edge Select; positive edge trigger
    // Bit 4,3: 0 (WGM)
    // Bits 2,1,0:  001 Clock Select, use main clock w/o division
    // Bits 2,1,0:  000 Clock Select: No clock; clock stopped.
    //TCCR1B =  1;		// Run clock
    TCCR1A = 0;
    TCCR1B = 0;        // No clock
    TCCR1C = 0;        // CYA Only

    ////////////////////////////////////////////////////
    // 8-bit Timer 0: For real-time waiting

    CurTicks = (TICKS_BETWEEN_MOUTH_OPENS) - (TICKS_BETWEEN_MOUTH_OPENS / 6); // Initial wait is 1/6 of full delay
    TCNT0 = 0;
    TCCR0A = 0;             // "Normal operation"
    TCCR0B = CLOCK_DIVISOR; // Start timer, at Clock Rate/1024.

    TIFR |= BIT_MASK(TOV0);  /// Clear bit 1, Ctr 0 overflow flag by writing 1.

    CUR_SERVO_INDEX = 0;
    PulseDone = 0;

    asm("sei");     // ENABLE global interrupts 

    STATE State = STATE_WAITING_OPEN;

    unsigned int MounthOpeningTick = 0;
    unsigned char Sleeping = TRUE;
    for (;;)  // main loop										
    {
        unsigned int CurTimer = CurTicks;
        // wait until next clock tick -- this makes the timing for the servo pulses regular
        while( CurTimer == CurTicks )
        {
            if (TIFR & BIT_MASK(TOV0))          // Overflow every 256th tick (8388608/1024/256) = 32Hz
            {
                CurTicks++;                     // incremented at 32 Hz.
                TIFR |= BIT_MASK(TOV0);         // Clear overflow flag
            }
        }
        // send out the servo pulses
        for( i = 0; i < NUM_SERVOS; ++i )
        {
            CUR_SERVO_INDEX = i;
            TIMSK   = 0;            // Disable timer interrupts
            TCCR1B  = 0;            // Stop Counter -- probably not needed since interrupts are disabled
            PulseDone = 0;
    
            if( Sleeping == FALSE )
            {
                //Set Low & Hi compare values:
                OCR1A = SIGNAL_UP_DELAY_US;                                                                          // signal on -- why the delay?
                OCR1B = ((SERVO_BASE_VALUE_US + servo_position[i])*CPU_CLOCK_RATE_MHZ ) + SIGNAL_UP_DELAY_US;        // signal off after this many microseconds
        
                // Reset counter to zero.
                TCNT1  = 0;
                TIMSK  = BIT_MASK(OCIE1A) | BIT_MASK(OCIE1B);      // enable timer OC interrupts (bit 0: overflow)
                TCCR1B = 1;         // Start Counter
                //
                // we will now get two interrupts --
                // one to turn the pulse on and one to turn the pulse off
                // We will recognize the pulse being turned off since PulseDone will go high on the
                // second interrupt...
                // wait for pulse to complete so we have clean timing and control
                while (PulseDone == 0)
                {
                    ;
                }
            }
        }

        switch( State )
        {
        case STATE_WAITING_OPEN:
            if (CurTicks < TICKS_BETWEEN_MOUTH_OPENS)   
                break;
            MounthOpeningTick = 0;
            Sleeping = FALSE;
            ++State;
            // fall-thorugh OK
        case STATE_OPENING:
            PEIZO_ON();
            if( MounthOpeningTick < TICKS_TO_OPEN_MOUTH )
            {
                OpenMouth( MounthOpeningTick );
                ++MounthOpeningTick;
                break;
            }
            CloseMouth();
            ++State;
            MounthOpeningTick = 0;
            // fall-through OK
        case STATE_WAITING_TO_BOUNCE_FIRST:
            if( ++MounthOpeningTick < TICKS_BOUNCE_MOUTH_CLOSED )
            {
                break;
            }
            ++State;
            MounthOpeningTick = 0;
            // fall-through OK
        case STATE_BOUNCING_FIRST:
            if( MounthOpeningTick < TICKS_TO_BOUNCE_MOUTH )
            {
                OpenMouth( MounthOpeningTick );
                MounthOpeningTick += TICKS_PER_BOUNCE_MOUTH_JUMP;
                break;
            }
            CloseMouth();
            MounthOpeningTick = 0;
            ++State;
            // fall-through OK
        case STATE_WAITING_TO_BOUNCE_SECOND:
            if( ++MounthOpeningTick < TICKS_BOUNCE_MOUTH_CLOSED )
            {
                break;
            }
            ++State;
            MounthOpeningTick = 0;
            // fall-through OK
        case STATE_BOUNCING_SECOND:
            if( MounthOpeningTick < TICKS_TO_BOUNCE_MOUTH )
            {
                OpenMouth( MounthOpeningTick );
                MounthOpeningTick += TICKS_PER_BOUNCE_MOUTH_JUMP;
                break;
            }
            CloseMouth();
            PEIZO_OFF();
            ++State;
            break;
        // we need to wait for mouth to shut before sleeping
        case STATE_FINAL_CLOSE:
            Sleeping = TRUE;
            State = STATE_WAITING_OPEN;
            CurTicks = 0;
        }

    } // End main loop


    return 0;
}

//
// OpenMouth -- set servo positions based on mouth open time
//
void OpenMouth( unsigned int MounthOpeningTick )
{
    PORTD |= EYE_LEDS_ON;
    unsigned int i;
    for( i = 0; i < NUM_SERVOS; ++i )
    {
        // the new postion for this servo is the max setting divided by how many ticks we have to open mouth
        unsigned int NewServoPosition = ((long)((long)servo_max_position[i] * (long)MounthOpeningTick ) / TICKS_TO_OPEN_MOUTH);
        servo_position[i] = NewServoPosition;
    }
}

//
// CloseMouth -- close the mouth
//
void CloseMouth()
{
    unsigned int i;
    PORTD &= ~EYE_LEDS_ON;
    for( i = 0; i < NUM_SERVOS; ++i )
    {
        servo_position[i]  = 10;
    }
}


