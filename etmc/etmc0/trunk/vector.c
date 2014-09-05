/************************************************************
07-14-2011 - Adds UART4,5
...
10/02/2012 - CAN interrupts
*************************************************************/
#include "panic_leds.h"
#include "libopencm3/stm32/f4/scb.h"
#include "libopencm3/stm32/nvic.h"


/*
 * This file is part of the libopenstm32 project.
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 * Modified 9/22/2010: D.E. Haselwood <dhaselwood@ieee.org>
 *  Changing naming to be more CMSIS-like.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#define WEAK __attribute__ ((weak))

void main(void);
void Reset_Handler(void);
void blocking_handler1(void);
void blocking_handler2(void);
void blocking_handler3(void);
void blocking_handler4(void);
void null_handler(void);

//void WEAK  _estack
//void WEAK  Reset_Handler(void);
void WEAK NMI_Handler(void);
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK Reset_Handler(void);
void WEAK NMI_Handler(void);
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK UsageFault_Handler(void);
void WEAK SVC_Handler(void);
void WEAK DebugMon_Handler(void);
void WEAK PendSV_Handler(void);
void WEAK  SysTick_Handler(void);


void WEAK     WWDG_IRQHandler(void);                  /* Window WatchDog              */
void WEAK     PVD_IRQHandler(void);                   /* PVD through EXTI Line detection */
void WEAK     TAMP_STAMP_IRQHandler(void);            /* Tamper and TimeStamps through the EXTI line */
void WEAK     RTC_WKUP_IRQHandler(void);              /* RTC Wakeup through the EXTI line */
void WEAK     FLASH_IRQHandler(void);                 /* FLASH                        */
void WEAK     RCC_IRQHandler(void);                   /* RCC                          */
void WEAK     EXTI0_IRQHandler(void);                 /* EXTI Line0                   */
void WEAK     EXTI1_IRQHandler(void);                 /* EXTI Line1                   */
void WEAK     EXTI2_IRQHandler(void);                 /* EXTI Line2                   */
void WEAK     EXTI3_IRQHandler(void);                 /* EXTI Line3                   */
void WEAK     EXTI4_IRQHandler(void);                 /* EXTI Line4                   */
void WEAK     DMA1_Stream0_IRQHandler(void);          /* DMA1 Stream 0                */
void WEAK     DMA1_Stream1_IRQHandler(void);          /* DMA1 Stream 1                */
void WEAK     DMA1_Stream2_IRQHandler(void);          /* DMA1 Stream 2                */
void WEAK     DMA1_Stream3_IRQHandler(void);          /* DMA1 Stream 3                */
void WEAK     DMA1_Stream4_IRQHandler(void);          /* DMA1 Stream 4                */
void WEAK     DMA1_Stream5_IRQHandler(void);          /* DMA1 Stream 5                */
void WEAK     DMA1_Stream6_IRQHandler(void);          /* DMA1 Stream 6                */
void WEAK     ADC_IRQHandler(void);                   /* ADC1, ADC2 and ADC3s         */
void WEAK     CAN1_TX_IRQHandler(void);               /* CAN1 TX                      */
void WEAK     CAN1_RX0_IRQHandler(void);              /* CAN1 RX0                     */
void WEAK     CAN1_RX1_IRQHandler(void);              /* CAN1 RX1                     */
void WEAK     CAN1_SCE_IRQHandler(void);              /* CAN1 SCE                     */
void WEAK     EXTI9_5_IRQHandler(void);               /* External Line[9:5]s          */
void WEAK     TIM1_BRK_TIM9_IRQHandler(void);         /* TIM1 Break and TIM9          */
void WEAK     TIM1_UP_TIM10_IRQHandler(void);         /* TIM1 Update and TIM10        */
void WEAK     TIM1_TRG_COM_TIM11_IRQHandler(void);    /* TIM1 Trigger and Commutation and TIM11 */
void WEAK     TIM1_CC_IRQHandler(void);               /* TIM1 Capture Compare         */
void WEAK     TIM2_IRQHandler(void);                  /* TIM2                         */
void WEAK     TIM3_IRQHandler(void);                  /* TIM3                         */
void WEAK     TIM4_IRQHandler(void);                  /* TIM4                         */
void WEAK     I2C1_EV_IRQHandler(void);               /* I2C1 Event                   */
void WEAK     I2C1_ER_IRQHandler(void);               /* I2C1 Error                   */
void WEAK     I2C2_EV_IRQHandler(void);               /* I2C2 Event                   */
void WEAK     I2C2_ER_IRQHandler(void);               /* I2C2 Error                   */
void WEAK     SPI1_IRQHandler(void);                  /* SPI1                         */
void WEAK     SPI2_IRQHandler(void);                  /* SPI2                         */
void WEAK     USART1_IRQHandler(void);                /* USART1                       */
void WEAK     USART2_IRQHandler(void);                /* USART2                       */
void WEAK     USART3_IRQHandler(void);                /* USART3                       */
void WEAK     EXTI15_10_IRQHandler(void);             /* External Line[15:10]s        */
void WEAK     RTC_Alarm_IRQHandler(void);             /* RTC Alarm (A and B) through EXTI Line */
void WEAK     OTG_FS_WKUP_IRQHandler(void);           /* USB OTG FS Wakeup through EXTI line */
void WEAK     TIM8_BRK_TIM12_IRQHandler(void);        /* TIM8 Break and TIM12         */
void WEAK     TIM8_UP_TIM13_IRQHandler(void);         /* TIM8 Update and TIM13        */
void WEAK     TIM8_TRG_COM_TIM14_IRQHandler(void);    /* TIM8 Trigger and Commutation and TIM14 */
void WEAK     TIM8_CC_IRQHandler(void);               /* TIM8 Capture Compare         */
void WEAK     DMA1_Stream7_IRQHandler(void);          /* DMA1 Stream7                 */
void WEAK     FSMC_IRQHandler(void);                  /* FSMC                         */
void WEAK     SDIO_IRQHandler(void);                  /* SDIO                         */
void WEAK     TIM5_IRQHandler(void);                  /* TIM5                         */
void WEAK     SPI3_IRQHandler(void);                  /* SPI3                         */
void WEAK     UART4_IRQHandler(void);                 /* UART4                        */
void WEAK     UART5_IRQHandler(void);                 /* UART5                        */
void WEAK     TIM6_DAC_IRQHandler(void);              /* TIM6 and DAC1&2 underrun errors */
void WEAK     TIM7_IRQHandler(void);                  /* TIM7                         */
void WEAK     DMA2_Stream0_IRQHandler(void);          /* DMA2 Stream 0                */
void WEAK     DMA2_Stream1_IRQHandler(void);          /* DMA2 Stream 1                */
void WEAK     DMA2_Stream2_IRQHandler(void);          /* DMA2 Stream 2                */
void WEAK     DMA2_Stream3_IRQHandler(void);          /* DMA2 Stream 3                */
void WEAK     DMA2_Stream4_IRQHandler(void);          /* DMA2 Stream 4                */
void WEAK     ETH_IRQHandler(void);                   /* Ethernet                     */
void WEAK     ETH_WKUP_IRQHandler(void);              /* Ethernet Wakeup through EXTI line */
void WEAK     CAN2_TX_IRQHandler(void);               /* CAN2 TX                      */
void WEAK     CAN2_RX0_IRQHandler(void);              /* CAN2 RX0                     */
void WEAK     CAN2_RX1_IRQHandler(void);              /* CAN2 RX1                     */
void WEAK     CAN2_SCE_IRQHandler(void);              /* CAN2 SCE                     */
void WEAK     OTG_FS_IRQHandler(void);                /* USB OTG FS                   
> I expect with the hw fp enabled there is little benefit to using the
> polynomial approximations but it is also galling to calculate things to
> many decimal digit accuracy when just a few will do.  The double
> precision transcendentals are just obnoxious for most control systems
> applications.   I looked at the atan listing the fp
> multiply-accumulates are sweet.  Is this atan or atan2.  Going through
> 90 degrees it would seem that atan2 would allow faster code for the
> same accuracy.  There is nothing magic about the quarter sine and
> raised cosine functions I use for the tapers.  They are just convenient
> for setting the slope at the end points I want for each application. 
> In fact, because I need to delay a bit after the cable speed peaks
> before beginning to ramp up the tension to the climb tension, I
> switched from a raised cosine to a quarter sine taper to better match
> the slope of the tension curve at the junction.  But the quarter sign
> still sometimes has a noticeable kink in the transition.  The math to
> make a polynomial that matches the initial tension and slope would not
> be that difficult.  Doing it with the sine function would be much more
> difficult.  Again, this is all premature optimization at this point.  
> 
> In my work we generate what we call kernel filter functions as well as
> sine/cosines with segmented polynomials.  For our applications we use
> more segments and lower order polynomials for speed but we often need
> 20 bit accuracy and above.   In FPGAs we have large dual port block
> SRAMs and we tend to use linear fits but maybe 512 segments.  The 18x18
> multipliers are limited  and memory is more available.  In ASICs we can
> tend to use quadratic polynomials but much fewer segments, typically 16
> to 64 depending on the accuracy needed.  There we can tailor the
> multiplier size and this allows us to get high speed with the least
> resources consumed.
> 
> -George
> 
> 
> 
> 
> 
>  
> 
> -----Original Message-----
> From: Donald E Haselwood [mailto:dhaselwood@verizon.net] 
> Sent: Tuesday, September 02, 2014 12:48 PM
> To: MOORE,GEORGE (K-Spokane,ex1)
> Cc: c.wells@ieee.org
> Subject: RE: STM32 Core Coupled Memory
> 
> George,
> 
> On Tue, 2014-09-02 at 18:11 +0000, george_moore@keysight.com wrote:
> > Don,
> > 
> > Got the Java MC/PS working last night for multiple launches before 
> > retiring early - two back-to-back meetings with Europe this morning.
> > Will be ready to start on the F4 MC this evening.
> > 
> > There now seems to be a motivation to keep the memory footprint small. 
> > I had asked about using chars for flags and you had indicated you 
> > thought it was simpler to just always use ints and that chars might 
> > not be optimized.  But now there is a value for conserving memory so I 
> > would ask again, should I use chars for flags and in general be aware 
> > of the size of all the memory being allocated?
> > 
> It depends on what memory is being conserved.  There is plenty of flash, but not RAM.  Using chars may increase the amount of flash used as it may take more bytes for the instructions.  I think the STM32/thumb2 does OK with 1/2 words, i.e. 2 bytes, but I'd have to look up if the instructions are 32b or 16b.  Also, I'm not clear on whether a 32b instruction executes as fast as 16b, as it gets in to how the pipeline works.  Even though the flash has 5 wait states when the clock is at 168MHz the flash is read in 128b chunks (IIRC!).
> 
> To get the advantage of putting code in CCM for speed improvement is a bit of an undertaking.  The STM32F407VGT6 has a 64K CCM, but that is not big enough to hold everything.  There is about 50-60K of "overhead" in subroutines.  Using chars vs ints for flags would be a drop in the ocean.
> 
> Putting the stack on CCM is something that is dog-easy to do and would some advantage since the stack gets used a lot.
> 
> When it comes to speed and the MC the answer is to use the interrupt levels (or RTOS which in a sense does the same thing) and separate out things that are fast (e.g. control law), medium (e.g. polling various switches), and slow (e.g. printfs, LCD display), and not depend on getting a single polling loop uber-fast.
> 
> > Something I thought about during my backpacking trip I tried out this 
> > morning taking a break.  Instead of using trig functions for shaping 
> > the tension during the startup taper up, rotation taper down, climb 
> > ramp up, and end-of-launch (EOL) taper down just use polynomials.  It 
> > looks like this will work very well and would completely avoid all 
> > trig in the MC.  For each of these transition regions I currently use 
> > raised cosine or quarter sine functions.  With a cubic equation I can 
> > match the y values at two x-axis points and the slopes at two points 
> > which is generally what I am trying to do anyway with these trip functions.
> > Attached is a pdf of the startup tension profile comparing a raised 
> > cosine to a  cubic approximation that is 0 with 0 slope at the 
> > beginning and at ground tension level (here 1) and slope 0 at the end 
> > of the taper up.  Blue is the polynomial and red is the raised cosine 
> > - for our purposes effectively the same.  I can similarly match all 
> > the trig functions I currently use equally well and even better for 
> > the quarter sine.  Essentially I am doing a Taylor series for the 
> > desired profiles that matches these trig functions.  This can easily 
> > be done with integer math if I want to avoid the use of fp and the 
> > coefficients
> > (4 sets of 4) can all be calculated by the HC except the cljmb ramp up 
> > where the starting point and slope depends on when the cable speed peak
> > occurs.   My arguments always start at 0 and go to a bounded value so
> > the dynamic range required is not that great - 32 bits would be far 
> > more than sufficient and likely 16 bits would suffice for the 
> > coefficients.  The ones based on time would generally span less than 8 
> > seconds (climb tension ramp up) so 9 bits is the maximum number of 
> > bits for the time (64 ticks per second x 8 seconds.)  Using Horner's 
> > equation you don't explicitly square or cube these values and I 
> > believe
> > 16x16 multiplies would likely be more than sufficient.
> 
> There are only 32x32 multiplies and those execute in one clock cycle.
> 
> >   The other two
> > are based on cable speed and 8 or 9 bits would be more than sufficient.
> > Alas, my penchant for optimization is rearing its ugly head.   For the
> > present I will leave in the trig functions but plan to move to
> > polynomials for the long run.    
> > 
> 
> Yes, as you note, the trig functions are merely polynomial approximations anyway, so it is a case of trading off some accuracy for speed.
> 
> It looks like the hw fp is so fast that there is little advantage to using integers.  I'll forward a note I sent to Charlie with an attached .list file.  They make use of the fp mac and just rip down the list.
> 
> If we were using a PIC, or the classic Arduino, then the efficiencies you have in mind would make a lot of sense.  With the Cortex-M4 the savings are small, and maybe even non-existent when the overhead is thrown in.
> 
> > 
> > Along the same lines, I believe for the accuracy (~0.5 degrees) I 
> > need, the atan function for the cable angle sensor could be easily 
> > replaced with a couple of polynomial approximations - one for angles 
> > less that
> > 45 and one for angles greater than 45.  One won't work because the 
> > argument approaches infinity near 90 degrees, i.e. use y/x up to 45 
> > and x/y for greater than 45. The swap over point occurs when x = y.  
> > Too much time spent thinking about this already.
> > 
> I looked at what was supposed to be the source code for the gnu atan function.  IIRC it used a 5 or 6 term polynomial for each of 5 segments.
> It looks like a lot of thought has been given to making the trig functions fast.
> 
> 
> Don
> 
> 
> > -George
> > 
> > -----Original Message-----
> > From: MOORE,GEORGE (K-Spokane,ex1)
> > Sent: Monday, September 01, 2014 3:26 PM
> > To: 'Charles Wells'; Donald E Haselwood
> > Subject: RE: STM32 Core Coupled Memory
> > 
> > Don,
> > 
> > Sounds like an easy way to significantly increase speed.  When I get the MC coded on the F4 we need to see how big the time sensitive elements are and see if they will fit in the 8K bytes.  Another reason to avoid/minimize the use of printfs.
> > 
> > -George
> > 
> > -----Original Message-----
> > From: c.wells [mailto:c.wells@tampabay.rr.com]
> > Sent: Friday, August 29, 2014 5:13 PM
> > To: Donald E Haselwood
> > Cc: MOORE,GEORGE (K-Spokane,ex1)
> > Subject: Re: STM32 Core Coupled Memory
> > 
> > Don,
> > 
> > Good idea.  Assuming the Chibios stuff is in its own linker section, it should be simply a matter of fiddling with the ".ld" file.
> > 
> > Char1ie
> > 
> > 
> > On Aug 29, 2014, at 12:42 PM, Donald E Haselwood <dhaselwood@verizon.net> wrote:
> > 
> > > Charlie,
> > > 
> > > As I remember you said that some RTOS placed the stack on CCM memory 
> > > because it is faster.  I ran across this on the STM32F3--
> > > 
> > > "Boosted execution of control algorithms.
> > > Easier support for meta-language tools.
> > > Real-time performance:
> > > - CCM-SRAM (core coupled memory): 8 Kbytes of SRAM mapped to the 
> > > instruction bus; critical routines loaded in the 8-Kbyte CCM at 
> > > startup can be completed at full speed with zero wait states, 
> > > achieving 94 Dhrystone MIPS and CoreMark score 155 at 72 MHz."
> > > 
> > > 
> > > 
> > > The STM32F407 (DiscoveryF4) has 64K CCM, and running with the clock 
> > > at
> > > 168 MHz the flash wait states is set to 5, so the speed up by 
> > > running out of the SRAM that is on the instruction bus would make a 
> > > big difference.
> > > 
> > > In addition to putting the stack on CCM it would make sense to 
> > > locate the whole RTOS there as well.  The Chibios RTOS is only 5-7KB 
> > > so there is plenty of room.
> > > 
> > > Don
> > > 
> > > 
> > 
> 
> 
> */
void WEAK     DMA2_Stream5_IRQHandler(void);          /* DMA2 Stream 5                */
void WEAK     DMA2_Stream6_IRQHandler(void);          /* DMA2 Stream 6                */
void WEAK     DMA2_Stream7_IRQHandler(void);          /* DMA2 Stream 7                */
void WEAK     USART6_IRQHandler(void);                /* USART6                       */
void WEAK     I2C3_EV_IRQHandler(void);               /* I2C3 event                   */
void WEAK     I2C3_ER_IRQHandler(void);               /* I2C3 error                   */
void WEAK     OTG_HS_EP1_OUT_IRQHandler(void);        /* USB OTG HS End Point 1 Out   */
void WEAK     OTG_HS_EP1_IN_IRQHandler(void);         /* USB OTG HS End Point 1 In    */
void WEAK     OTG_HS_WKUP_IRQHandler(void);           /* USB OTG HS Wakeup through EXTI */
void WEAK     OTG_HS_IRQHandler(void);                /* USB OTG HS                   */
void WEAK     DCMI_IRQHandler(void);                  /* DCMI                         */
void WEAK     CRYP_IRQHandler(void);                  /* CRYP crypto                  */
void WEAK     HASH_RNG_IRQHandler(void);              /* Hash and Rng                 */
void WEAK     FPU_IRQHandler(void);                   /* FPU                          */

__attribute__ ((section(".isr_vector")))
void (*const vector_table[]) (void) = {
	(void *)(0x20000000+(96*1024)-1),/* Put stack at top */
	Reset_Handler,		/* Either this or main */
 NMI_Handler,
 HardFault_Handler,
 MemManage_Handler,
 BusFault_Handler,
 UsageFault_Handler,
 0,
 0,
 0,
 0,
 SVC_Handler,
 DebugMon_Handler,
 0,
 PendSV_Handler,
 SysTick_Handler,
    WWDG_IRQHandler,                  /* Window WatchDog              */
    PVD_IRQHandler,                   /* PVD through EXTI Line detection */
    TAMP_STAMP_IRQHandler,            /* Tamper and TimeStamps through the EXTI line */
    RTC_WKUP_IRQHandler,              /* RTC Wakeup through the EXTI line */
    FLASH_IRQHandler,                 /* FLASH                        */
    RCC_IRQHandler,                   /* RCC                          */
    EXTI0_IRQHandler,                 /* EXTI Line0                   */
    EXTI1_IRQHandler,                 /* EXTI Line1                   */
    EXTI2_IRQHandler,                 /* EXTI Line2                   */
    EXTI3_IRQHandler,                 /* EXTI Line3                   */
    EXTI4_IRQHandler,                 /* EXTI Line4                   */
    DMA1_Stream0_IRQHandler,          /* DMA1 Stream 0                */
    DMA1_Stream1_IRQHandler,          /* DMA1 Stream 1                */
    DMA1_Stream2_IRQHandler,          /* DMA1 Stream 2                */
    DMA1_Stream3_IRQHandler,          /* DMA1 Stream 3                */
    DMA1_Stream4_IRQHandler,          /* DMA1 Stream 4                */
    DMA1_Stream5_IRQHandler,          /* DMA1 Stream 5                */
    DMA1_Stream6_IRQHandler,          /* DMA1 Stream 6                */
    ADC_IRQHandler,                   /* ADC1, ADC2 and ADC3s         */
    CAN1_TX_IRQHandler,               /* CAN1 TX                      */
    CAN1_RX0_IRQHandler,              /* CAN1 RX0                     */
    CAN1_RX1_IRQHandler,              /* CAN1 RX1                     */
    CAN1_SCE_IRQHandler,              /* CAN1 SCE                     */
    EXTI9_5_IRQHandler,               /* External Line[9:5]s          */
    TIM1_BRK_TIM9_IRQHandler,         /* TIM1 Break and TIM9          */
    TIM1_UP_TIM10_IRQHandler,         /* TIM1 Update and TIM10        */
    TIM1_TRG_COM_TIM11_IRQHandler,    /* TIM1 Trigger and Commutation and TIM11 */
    TIM1_CC_IRQHandler,               /* TIM1 Capture Compare         */
    TIM2_IRQHandler,                  /* TIM2                         */
    TIM3_IRQHandler,                  /* TIM3                         */
    TIM4_IRQHandler,                  /* TIM4                         */
    I2C1_EV_IRQHandler,               /* I2C1 Event                   */
    I2C1_ER_IRQHandler,               /* I2C1 Error                   */
    I2C2_EV_IRQHandler,               /* I2C2 Event                   */
    I2C2_ER_IRQHandler,               /* I2C2 Error                   */
    SPI1_IRQHandler,                  /* SPI1                         */
    SPI2_IRQHandler,                  /* SPI2                         */
    USART1_IRQHandler,                /* USART1                       */
    USART2_IRQHandler,                /* USART2                       */
    USART3_IRQHandler,                /* USART3                       */
    EXTI15_10_IRQHandler,             /* External Line[15:10]s        */
    RTC_Alarm_IRQHandler,             /* RTC Alarm (A and B) through EXTI Line */
    OTG_FS_WKUP_IRQHandler,           /* USB OTG FS Wakeup through EXTI line */
    TIM8_BRK_TIM12_IRQHandler,        /* TIM8 Break and TIM12         */
    TIM8_UP_TIM13_IRQHandler,         /* TIM8 Update and TIM13        */
    TIM8_TRG_COM_TIM14_IRQHandler,    /* TIM8 Trigger and Commutation and TIM14 */
    TIM8_CC_IRQHandler,               /* TIM8 Capture Compare         */
    DMA1_Stream7_IRQHandler,          /* DMA1 Stream7                 */
    FSMC_IRQHandler,                  /* FSMC                         */
    SDIO_IRQHandler,                  /* SDIO                         */
    TIM5_IRQHandler,                  /* TIM5                         */
    SPI3_IRQHandler,                  /* SPI3                         */
    UART4_IRQHandler,                 /* UART4                        */
    UART5_IRQHandler,                 /* UART5                        */
    TIM6_DAC_IRQHandler,              /* TIM6 and DAC1&2 underrun errors */
    TIM7_IRQHandler,                  /* TIM7                         */
    DMA2_Stream0_IRQHandler,          /* DMA2 Stream 0                */
    DMA2_Stream1_IRQHandler,          /* DMA2 Stream 1                */
    DMA2_Stream2_IRQHandler,          /* DMA2 Stream 2                */
    DMA2_Stream3_IRQHandler,          /* DMA2 Stream 3                */
    DMA2_Stream4_IRQHandler,          /* DMA2 Stream 4                */
    ETH_IRQHandler,                   /* Ethernet                     */
    ETH_WKUP_IRQHandler,              /* Ethernet Wakeup through EXTI line */
    CAN2_TX_IRQHandler,               /* CAN2 TX                      */
    CAN2_RX0_IRQHandler,              /* CAN2 RX0                     */
    CAN2_RX1_IRQHandler,              /* CAN2 RX1                     */
    CAN2_SCE_IRQHandler,              /* CAN2 SCE                     */
    OTG_FS_IRQHandler,                /* USB OTG FS                   */
    DMA2_Stream5_IRQHandler,          /* DMA2 Stream 5                */
    DMA2_Stream6_IRQHandler,          /* DMA2 Stream 6                */
    DMA2_Stream7_IRQHandler,          /* DMA2 Stream 7                */
    USART6_IRQHandler,                /* USART6                       */
    I2C3_EV_IRQHandler,               /* I2C3 event                   */
    I2C3_ER_IRQHandler,               /* I2C3 error                   */
    OTG_HS_EP1_OUT_IRQHandler,        /* USB OTG HS End Point 1 Out   */
    OTG_HS_EP1_IN_IRQHandler,         /* USB OTG HS End Point 1 In    */
    OTG_HS_WKUP_IRQHandler,           /* USB OTG HS Wakeup through EXTI */
    OTG_HS_IRQHandler,                /* USB OTG HS                   */
    DCMI_IRQHandler,                  /* DCMI                         */
    CRYP_IRQHandler,                  /* CRYP crypto                  */
    HASH_RNG_IRQHandler,              /* Hash and Rng                 */
    FPU_IRQHandler,                   /* FPU                          */
};

void hard_fault_handler(void)
{
	panic_leds(1);
//	while (1) ;
}
void mem_manage_handler(void)
{
	panic_leds(2);
//	while (1) ;
}
void bus_fault_handler(void)
{
	panic_leds(3);
//	while (1) ;
}
void usage_fault_handler(void)
{
	panic_leds(4);
//	while (1) ;
}

void null_handler(void)
{
	panic_leds(5);
//	while (1) ;
}
/* **************************************************************************************
 * void relocate_vector(void);
 * @brief	: Copy vector to ram and change
 * ************************************************************************************** */
void relocate_vector(void)
{
	/* Copy flash vector to sram */
//	unsigned char *pram   = vectorram;
//	unsigned char *pflash = vector_table;
//	while ( pflash < (unsigned char *)(&hard_fault_handler) )
//	{
//		*pram++ = *pflash++;
//	}
	
	/* Disable all interrupts */
	NVIC_ICER(0) = -1;
	NVIC_ICER(1) = -1;
	NVIC_ICER(2) = -1;

	/* Reset stack pointer */
	
	/* Move vector to beginning of sram*/
//	SCB_VTOR  = (1<<29);
	SCB_VTOR = (unsigned int)vector_table;

	return;
}
/* start & end addresses for the .data section. (see .ld file) */
extern unsigned char __data_section_start; 	/* Beginning of data section in RAM */
extern unsigned char __data_image_end;		/* End of data section in RAM */
extern unsigned char __data_image;		/* Beginning of data section in ROM */

/* start & end addresses for the .bss section. (see .ld file) */
extern unsigned char _start_of_bss;
extern unsigned char _end_of_bss;

/* External function prototypes ----------------------------------------------*/
extern void main(void);                /* Application's main function */
extern void SystemInit(void);         /* STM's system init? */


void Reset_Handler(void)
{
  unsigned char *pX, *pY;

extern unsigned char _sdata; 	// start of initialized data: flash
extern unsigned char _edata; 	// end of initialized data: flash
extern unsigned char _sidata; 	// start of initialized data: ram

extern unsigned char _sbss; 	// start of zero'ed ram
extern unsigned char _ebss; 	// end of zero'ed ram

  /* Copy initial values for static variables (from flash to SRAM) */

  pX = &_sidata;
  pY = &_sdata;

  while ( pY < &_edata ) *pY++ = *pX++;


  /* Zero fill the .bss section. */
  pX = &_sbss;
  while ( pX < &_ebss) *pX++ = 0;

/* Call the clock system intitialization function.*/ 
	SystemInit();

/* Call static constructors */
//	__libc_init_array();

  /* Call the application's entry point.*/
  main();

x: goto x;	// Hang loop in a style for all you FORTRAN programmers

}


/* --------------- For debugging...----------------------------------- */
int Default_HandlerCode = 999;
u32 DH08;
void Default_Handler08(void) {DH08 += 1; return;}

void OTG_FS_IRQHandler(void);
void Default_Handler76(void) {	OTG_FS_IRQHandler(); return; }

void Default_Handler00(void) { Default_HandlerCode =  0; panic_leds(5); }
void Default_Handler01(void) { Default_HandlerCode =  1; panic_leds(5); }
void Default_Handler02(void) { Default_HandlerCode =  2; panic_leds(5); }
void Default_Handler03(void) { Default_HandlerCode =  3; panic_leds(5); }
void Default_Handler04(void) { Default_HandlerCode =  4; panic_leds(5); }
void Default_Handler05(void) { Default_HandlerCode =  5; panic_leds(5); }
void Default_Handler06(void) { Default_HandlerCode =  6; panic_leds(5); }
void Default_Handler07(void) { Default_HandlerCode =  7; panic_leds(5); }
//void Default_Handler08(void) { Default_HandlerCode =  8; panic_leds(5); }
void Default_Handler09(void) { Default_HandlerCode =  9; panic_leds(5); }
void Default_Handler10(void) { Default_HandlerCode = 10; panic_leds(5); }
void Default_Handler11(void) { Default_HandlerCode = 11; panic_leds(5); }
void Default_Handler12(void) { Default_HandlerCode = 12; panic_leds(5); }
void Default_Handler13(void) { Default_HandlerCode = 13; panic_leds(5); }
void Default_Handler14(void) { Default_HandlerCode = 14; panic_leds(5); }
void Default_Handler15(void) { Default_HandlerCode = 15; panic_leds(5); }
void Default_Handler16(void) { Default_HandlerCode = 16; panic_leds(5); }
void Default_Handler17(void) { Default_HandlerCode = 17; panic_leds(5); }
void Default_Handler18(void) { Default_HandlerCode = 18; panic_leds(5); }
void Default_Handler19(void) { Default_HandlerCode = 19; panic_leds(5); }
void Default_Handler20(void) { Default_HandlerCode = 20; panic_leds(5); }
void Default_Handler21(void) { Default_HandlerCode = 21; panic_leds(5); }
void Default_Handler22(void) { Default_HandlerCode = 22; panic_leds(5); }
void Default_Handler23(void) { Default_HandlerCode = 23; panic_leds(5); }
void Default_Handler24(void) { Default_HandlerCode = 24; panic_leds(5); }
void Default_Handler25(void) { Default_HandlerCode = 25; panic_leds(5); }
void Default_Handler26(void) { Default_HandlerCode = 26; panic_leds(5); }
void Default_Handler27(void) { Default_HandlerCode = 27; panic_leds(5); }
void Default_Handler28(void) { Default_HandlerCode = 28; panic_leds(5); }
void Default_Handler29(void) { Default_HandlerCode = 29; panic_leds(5); }
void Default_Handler30(void) { Default_HandlerCode = 30; panic_leds(5); }
void Default_Handler31(void) { Default_HandlerCode = 31; panic_leds(5); }
void Default_Handler32(void) { Default_HandlerCode = 32; panic_leds(5); }
void Default_Handler33(void) { Default_HandlerCode = 33; panic_leds(5); }
void Default_Handler34(void) { Default_HandlerCode = 34; panic_leds(5); }
void Default_Handler35(void) { Default_HandlerCode = 35; panic_leds(5); }
void Default_Handler36(void) { Default_HandlerCode = 36; panic_leds(5); }
void Default_Handler37(void) { Default_HandlerCode = 37; panic_leds(5); }
void Default_Handler38(void) { Default_HandlerCode = 38; panic_leds(5); }
void Default_Handler39(void) { Default_HandlerCode = 39; panic_leds(5); }
void Default_Handler40(void) { Default_HandlerCode = 40; panic_leds(5); }
void Default_Handler41(void) { Default_HandlerCode = 41; panic_leds(5); }
void Default_Handler42(void) { Default_HandlerCode = 42; panic_leds(5); }
void Default_Handler43(void) { Default_HandlerCode = 43; panic_leds(5); }
void Default_Handler44(void) { Default_HandlerCode = 44; panic_leds(5); }
void Default_Handler45(void) { Default_HandlerCode = 45; panic_leds(5); }
void Default_Handler46(void) { Default_HandlerCode = 46; panic_leds(5); }
void Default_Handler47(void) { Default_HandlerCode = 47; panic_leds(5); }
void Default_Handler48(void) { Default_HandlerCode = 48; panic_leds(5); }
void Default_Handler49(void) { Default_HandlerCode = 49; panic_leds(5); }
void Default_Handler50(void) { Default_HandlerCode = 50; panic_leds(5); }
void Default_Handler51(void) { Default_HandlerCode = 51; panic_leds(5); }
void Default_Handler52(void) { Default_HandlerCode = 52; panic_leds(5); }
void Default_Handler53(void) { Default_HandlerCode = 53; panic_leds(5); }
void Default_Handler54(void) { Default_HandlerCode = 54; panic_leds(5); }
void Default_Handler55(void) { Default_HandlerCode = 55; panic_leds(5); }
void Default_Handler56(void) { Default_HandlerCode = 56; panic_leds(5); }
void Default_Handler57(void) { Default_HandlerCode = 57; panic_leds(5); }
void Default_Handler58(void) { Default_HandlerCode = 58; panic_leds(5); }
void Default_Handler59(void) { Default_HandlerCode = 59; panic_leds(5); }
void Default_Handler60(void) { Default_HandlerCode = 60; panic_leds(5); }
void Default_Handler61(void) { Default_HandlerCode = 61; panic_leds(5); }
void Default_Handler62(void) { Default_HandlerCode = 62; panic_leds(5); }
void Default_Handler63(void) { Default_HandlerCode = 63; panic_leds(5); }
void Default_Handler64(void) { Default_HandlerCode = 64; panic_leds(5); }
void Default_Handler65(void) { Default_HandlerCode = 65; panic_leds(5); }
void Default_Handler66(void) { Default_HandlerCode = 66; panic_leds(5); }
void Default_Handler67(void) { Default_HandlerCode = 67; panic_leds(5); }
void Default_Handler68(void) { Default_HandlerCode = 68; panic_leds(5); }
void Default_Handler69(void) { Default_HandlerCode = 69; panic_leds(5); }
void Default_Handler70(void) { Default_HandlerCode = 70; panic_leds(5); }
void Default_Handler71(void) { Default_HandlerCode = 71; panic_leds(5); }
void Default_Handler72(void) { Default_HandlerCode = 72; panic_leds(5); }
void Default_Handler73(void) { Default_HandlerCode = 73; panic_leds(5); }
void Default_Handler74(void) { Default_HandlerCode = 74; panic_leds(5); }
void Default_Handler75(void) { Default_HandlerCode = 75; panic_leds(5); }
//void Default_Handler76(void) { Default_HandlerCode = 76; panic_leds(5); }
void Default_Handler77(void) { Default_HandlerCode = 77; panic_leds(5); }
void Default_Handler78(void) { Default_HandlerCode = 78; panic_leds(5); }
void Default_Handler79(void) { Default_HandlerCode = 79; panic_leds(5); }
void Default_Handler80(void) { Default_HandlerCode = 80; panic_leds(5); }
void Default_Handler81(void) { Default_HandlerCode = 81; panic_leds(5); }
void Default_Handler82(void) { Default_HandlerCode = 82; panic_leds(5); }
void Default_Handler83(void) { Default_HandlerCode = 83; panic_leds(5); }
void Default_Handler84(void) { Default_HandlerCode = 84; panic_leds(5); }
void Default_Handler85(void) { Default_HandlerCode = 85; panic_leds(5); }
void Default_Handler86(void) { Default_HandlerCode = 86; panic_leds(5); }
void Default_Handler87(void) { Default_HandlerCode = 87; panic_leds(5); }
void Default_Handler88(void) { Default_HandlerCode = 88; panic_leds(5); }
void Default_Handler89(void) { Default_HandlerCode = 89; panic_leds(5); }
void Default_Handler90(void) { Default_HandlerCode = 90; panic_leds(5); }


#pragma weak _estack = null_handler
#pragma weak NMI_Handler = Default_Handler00
#pragma weak HardFault_Handler = Default_Handler01
#pragma weak MemManage_Handler = Default_Handler02
#pragma weak BusFault_Handler = Default_Handler03
#pragma weak UsageFault_Handler = Default_Handler04
#pragma weak SVC_Handler = Default_Handler05
#pragma weak DebugMon_Handler = Default_Handler06
#pragma weak PendSV_Handler = Default_Handler07
#pragma weak SysTick_Handler = Default_Handler08
#pragma weak WWDG_IRQHandler = Default_Handler09
#pragma weak PVD_IRQHandler = Default_Handler10
#pragma weak TAMP_STAMP_IRQHandler = Default_Handler11
#pragma weak RTC_WKUP_IRQHandler = Default_Handler12
#pragma weak FLASH_IRQHandler = Default_Handler13
#pragma weak RCC_IRQHandler = Default_Handler14
#pragma weak EXTI0_IRQHandler = Default_Handler15
#pragma weak EXTI1_IRQHandler = Default_Handler16
#pragma weak EXTI2_IRQHandler = Default_Handler17
#pragma weak EXTI3_IRQHandler = Default_Handler18
#pragma weak EXTI4_IRQHandler = Default_Handler19
#pragma weak DMA1_Stream0_IRQHandler = Default_Handler20
#pragma weak DMA1_Stream1_IRQHandler = Default_Handler21
#pragma weak DMA1_Stream2_IRQHandler = Default_Handler22
#pragma weak DMA1_Stream3_IRQHandler = Default_Handler23
#pragma weak DMA1_Stream4_IRQHandler = Default_Handler24
#pragma weak DMA1_Stream5_IRQHandler = Default_Handler25
#pragma weak DMA1_Stream6_IRQHandler = Default_Handler26
#pragma weak ADC_IRQHandler = Default_Handler27
#pragma weak CAN1_TX_IRQHandler = Default_Handler28
#pragma weak CAN1_RX0_IRQHandler = Default_Handler29 
#pragma weak CAN1_RX1_IRQHandler = Default_Handler30
#pragma weak CAN1_SCE_IRQHandler = Default_Handler31
#pragma weak EXTI9_5_IRQHandler = Default_Handler32
#pragma weak TIM1_BRK_TIM9_IRQHandler = Default_Handler33
#pragma weak TIM1_UP_TIM10_IRQHandler = Default_Handler34
#pragma weak TIM1_TRG_COM_TIM11_IRQHandler = Default_Handler35
#pragma weak TIM1_CC_IRQHandler = Default_Handler36
#pragma weak TIM2_IRQHandler = Default_Handler37
#pragma weak TIM3_IRQHandler = Default_Handler38
#pragma weak TIM4_IRQHandler = Default_Handler39
#pragma weak I2C1_EV_IRQHandler = Default_Handler40
#pragma weak I2C1_ER_IRQHandler = Default_Handler41
#pragma weak I2C2_EV_IRQHandler = Default_Handler42
#pragma weak I2C2_ER_IRQHandler = Default_Handler43
#pragma weak SPI1_IRQHandler = Default_Handler44
#pragma weak SPI2_IRQHandler = Default_Handler45
#pragma weak USART1_IRQHandler = Default_Handler46
#pragma weak USART2_IRQHandler = Default_Handler47
#pragma weak USART3_IRQHandler = Default_Handler48
#pragma weak EXTI15_10_IRQHandler = Default_Handler49
#pragma weak RTC_Alarm_IRQHandler = Default_Handler50
#pragma weak OTG_FS_WKUP_IRQHandler = Default_Handler51
#pragma weak TIM8_BRK_TIM12_IRQHandler = Default_Handler52
#pragma weak TIM8_UP_TIM13_IRQHandler = Default_Handler53
#pragma weak TIM8_TRG_COM_TIM14_IRQHandler = Default_Handler54
#pragma weak TIM8_CC_IRQHandler = Default_Handler55
#pragma weak DMA1_Stream7_IRQHandler = Default_Handler56
#pragma weak FSMC_IRQHandler = Default_Handler57
#pragma weak SDIO_IRQHandler = Default_Handler58
#pragma weak TIM5_IRQHandler = Default_Handler59
#pragma weak SPI3_IRQHandler = Default_Handler60
#pragma weak UART4_IRQHandler = Default_Handler61
#pragma weak UART5_IRQHandler = Default_Handler62
#pragma weak TIM6_DAC_IRQHandler = Default_Handler63
#pragma weak TIM7_IRQHandler = Default_Handler64
#pragma weak DMA2_Stream0_IRQHandler = Default_Handler65
#pragma weak DMA2_Stream1_IRQHandler = Default_Handler66
#pragma weak DMA2_Stream2_IRQHandler = Default_Handler67
#pragma weak DMA2_Stream3_IRQHandler = Default_Handler68
#pragma weak DMA2_Stream4_IRQHandler = Default_Handler69
#pragma weak ETH_IRQHandler = Default_Handler70
#pragma weak ETH_WKUP_IRQHandler = Default_Handler71
#pragma weak CAN2_TX_IRQHandler = Default_Handler72
#pragma weak CAN2_RX0_IRQHandler = Default_Handler73
#pragma weak CAN2_RX1_IRQHandler = Default_Handler74
#pragma weak CAN2_SCE_IRQHandler = Default_Handler75
#pragma weak OTG_FS_IRQHandler = Default_Handler76
#pragma weak DMA2_Stream5_IRQHandler = Default_Handler77
#pragma weak DMA2_Stream6_IRQHandler = Default_Handler78
#pragma weak DMA2_Stream7_IRQHandler = Default_Handler79
#pragma weak USART6_IRQHandler = Default_Handler80
#pragma weak I2C3_EV_IRQHandler = Default_Handler81
#pragma weak I2C3_ER_IRQHandler = Default_Handler82
#pragma weak OTG_HS_EP1_OUT_IRQHandler = Default_Handler83
#pragma weak OTG_HS_EP1_IN_IRQHandler = Default_Handler84
#pragma weak OTG_HS_WKUP_IRQHandler = Default_Handler85
#pragma weak OTG_HS_IRQHandler = Default_Handler86
#pragma weak DCMI_IRQHandler = Default_Handler87
#pragma weak CRYP_IRQHandler = Default_Handler88
#pragma weak HASH_RNG_IRQHandler = Default_Handler89
#pragma weak FPU_IRQHandler = Default_Handler90

