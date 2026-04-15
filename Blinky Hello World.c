#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

void delay_ms(uint32_t n) {

    SysTick->LOAD = (4000 - 1);
    SysTick->VAL = 0; // Clear current value
    SysTick->CTRL = 0x5; // Enable SysTick, use processor clock

    for (uint32_t i = 0; i < n; i++) {
        // Wait until the COUNTFLAG is set
        while (!(SysTick->CTRL & 0x10000));
    }

    SysTick->CTRL = 0; // Disable timer
}

void stop_motors(void) {
    // According to table, In1=H, In2=L, PWM=L results in Short Brake
    PTB->PSOR = (1 << 1); // AI1 High
    PTB->PCOR = (1 << 0); // AI2 Low
    PTC->PSOR = (1 << 1); // BI1 High
    PTC->PCOR = (1 << 2); // BI2 Low

    // Kill the PWM signal to trigger the brake
    TPM2->CONTROLS[0].CnV = 0;
    TPM2->CONTROLS[1].CnV = 0;
}

void Init_8MHz_Clock(void) {
    OSC0->CR = OSC_CR_ERCLKEN_MASK;
    MCG->C2 = MCG_C2_RANGE0(1) | MCG_C2_EREFS0_MASK;
    MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
    while (!(MCG->S & MCG_S_OSCINIT0_MASK));
    while (MCG->S & MCG_S_IREFST_MASK);
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));
}

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    // 1. Enable Clocks for Port B, and C
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
    // Setup PWM
    Init_8MHz_Clock();
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK; //Enable clock for the correct TPM register (Page 172)
    SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(2);

    // 2. Configure Motor Pins as GPIO (ALT1)
    // Left Motor: PTB0, PTB1 | Right Motor: PTC1, PTC2
    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    PORTC->PCR[1] = PORT_PCR_MUX(1);
    PORTC->PCR[2] = PORT_PCR_MUX(1);

    //Setup PWMA(Left Motor) and PWMB(Right Motor) for PWM
    PORTB->PCR[2] = PORT_PCR_MUX(3);
    PORTB->PCR[3] = PORT_PCR_MUX(3);
    // Setup TPM
    // MSB:MSA = 10 (Edge-aligned), ELSB:ELSA = 10 (High-true pulses)
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Channel 0 (PTB2)
    TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Channel 1 (PTB3)
    TPM2->MOD = 7999; // calculate the value of MOD using the following formula:
    //time period (1ms for motors) = (MOD + 1)/(f_clk/PS)
    TPM2->CONTROLS[0].CnV = 4000; TPM2->CONTROLS[1].CnV = 4000;// setup cnv value for default
    TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);// set prescale to 1 and start the timer by changing cmod to clock selected

    // Set Motor Pins as Outputs
    PTB->PDDR |= (1 << 0) | (1 << 1);
    PTC->PDDR |= (1 << 1) | (1 << 2);

    // 3. Configure Onboard Switches (SW1: PTC3, SW2: PTC12)
    // Note: MUX(1) for GPIO and PE/PS for Pull-up resistor
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    PTC->PDDR &= ~(1 << 3);
    PTC->PDDR &= ~(1 << 12);

    while(1) {

    	// Check SW1 (Left Turn) - Switch is Active LOW
            if (!(PTC->PDIR & (1 << 3))) {
            	// DEFAULT: Move Straight
            	  PTB->PSOR = (1 << 0); PTB->PCOR = (1 << 1);
            	  PTC->PCOR = (1 << 2); PTC->PSOR = (1 << 1);
            	  TPM2->CONTROLS[0].CnV = 5000;
            	  TPM2->CONTROLS[1].CnV = 7500;
            	  delay_ms(10000);
            	  stop_motors();
            }
            // Check SW2 (Right Turn) - Switch is Active LOW
            else if (!(PTC->PDIR & (1 << 12))) {
            	// DEFAULT: Move Straight
            	PTB->PSOR = (1 << 0); PTB->PCOR = (1 << 1);
            	PTC->PCOR = (1 << 2); PTC->PSOR = (1 << 1);
            	TPM2->CONTROLS[0].CnV = 7500;
            	TPM2->CONTROLS[1].CnV = 5000;
            	delay_ms(10000);
            	stop_motors();

            }
        }
    return 0;
}
