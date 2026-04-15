/*
 * Project3.c - PID Speed Control
 * CSE 325 Spring 2026
 *
 * Hardware:
 *   - Left  Encoder: PTA6 (A), PTA7 (B)
 *   - Right Encoder: PTA14 (A), PTA15 (B)
 *   - Motor Driver:
 *       PWMA -> PTB2, AI1 -> PTB1, AI2 -> PTB0  (Left Motor)
 *       PWMB -> PTB3, BI1 -> PTC1, BI2 -> PTC2  (Right Motor)
 *   - SW1  -> PTC3
 *
 * Interrupts used:
 *   1. PORTA_IRQHandler  - counts encoder ticks (GPIO interrupt on PTA6, PTA14)
 *   2. PIT_IRQHandler    - runs PID every 100ms (timer interrupt)
 *   3. PORTC_PORTD_IRQHandler - detects SW1 button press
 *
 * PID:
 *   Setpoint  = TARGET_TICKS_PER_INTERVAL (encoder ticks per 100ms)
 *   PV        = measured ticks in last 100ms interval
 *   Output    = duty cycle correction applied to CnV
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

/* ──────────────────────────────────────────────
 * System State
 * ────────────────────────────────────────────── */
typedef enum {
    IDLE,       // waiting for SW1 press
    WAITING,    // 2-second delay before moving
    MOVING,     // actively running + PID active
    STOPPED     // 6 seconds elapsed, motors off
} SystemState;

volatile SystemState state = IDLE;

/* ──────────────────────────────────────────────
 * Timing counters (incremented in PIT handler)
 * ────────────────────────────────────────────── */
// PIT fires every 100 ms  →  10 ticks = 1 second
#define PIT_HZ          10          // PIT fires 10 times per second
#define WAIT_TICKS      (2  * PIT_HZ)   // 2 s  = 20 PIT ticks
#define MOVE_TICKS      (6  * PIT_HZ)   // 6 s  = 60 PIT ticks

volatile uint32_t stateTicks = 0;   // counts PIT interrupts in current state

/* ──────────────────────────────────────────────
 * Encoder tick counters
 * ────────────────────────────────────────────── */
// 144 ticks per full wheel rotation (rising edges on Pin A only)
#define TICKS_PER_REV   148

volatile uint32_t leftTicks  = 0;
volatile uint32_t rightTicks = 0;

// Snapshot taken each PIT interval for speed measurement
volatile uint32_t leftTicksLast  = 0;
volatile uint32_t rightTicksLast = 0;

/* ──────────────────────────────────────────────
 * PID Parameters
 * ────────────────────────────────────────────── */
// Target: ~67% duty cycle on smooth surface ≈ 12 tick count per 100ms
// Start with a comfortable speed; tune Kp first, then Ki, then Kd.
#define TARGET_TICKS    12          // desired encoder ticks per 100ms interval

#define KP  3.0f
#define KI  0.5f
#define KD  0.01f

// PID state variables (one set per motor)
static float leftPrevError  = 0.0f,  leftSumError  = 0.0f;
static float rightPrevError = 0.0f,  rightSumError = 0.0f;

/* ──────────────────────────────────────────────
 * PWM duty cycle  (CnV values, stored as float)
 * Clamped to [MIN_CNV, MAX_CNV]
 * ────────────────────────────────────────────── */
#define PWM_MOD     7999            // TPM1/2 MOD value for ~1 kHz PWM
#define MIN_CNV     1000
#define MAX_CNV     7999            // 100% duty – safety cap

static float leftPower  = 0.0f;
static float rightPower = 0.0f;

/* ══════════════════════════════════════════════
 * Helper: clamp a float to [lo, hi]
 * ══════════════════════════════════════════════ */
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ══════════════════════════════════════════════
 * PID update for ONE motor
 *   ticks    – encoder ticks measured this interval
 *   power    – current CnV (modified in-place)
 *   prevErr  – previous error  (modified in-place)
 *   sumErr   – accumulated error (modified in-place)
 * ══════════════════════════════════════════════ */
static void pid_update(uint32_t ticks,
                       float *power,
                       float *prevErr,
                       float *sumErr)
{
    float error     = (float)TARGET_TICKS - (float)ticks;  // e(t) = SP - PV
    *sumErr        += error;                                // integral
    float diffError = error - *prevErr;                    // derivative
    float correction = KP * error + KI * (*sumErr) + KD * diffError;
    *prevErr        = error;
    *power          = clampf(*power + correction, MIN_CNV, MAX_CNV);
}

/* ══════════════════════════════════════════════
 * Motor control helpers
 * ══════════════════════════════════════════════ */
static void motors_forward(void) {
    GPIOB->PSOR = (1 << 0);  // AI2 = HIGH
    GPIOB->PCOR = (1 << 1);  // AI1 = LOW
    GPIOC->PCOR = (1 << 2);  // BI2 = LOW
    GPIOC->PSOR = (1 << 1);  // BI1 = HIGH
}

static void motors_stop(void) {
    // Short brake: AI1=HIGH, AI2=LOW, PWM=0
    GPIOB->PSOR = (1 << 1);  // AI1 = HIGH
    GPIOB->PCOR = (1 << 0);  // AI2 = LOW
    GPIOC->PSOR = (1 << 1);  // BI1 = HIGH
    GPIOC->PCOR = (1 << 2);  // BI2 = LOW
    TPM2->CONTROLS[0].CnV = 0;  // PWM = 0 → brake triggers
    TPM2->CONTROLS[1].CnV = 0;
}
/* ══════════════════════════════════════════════
 * Clock init  (select OSCERCLK = 8 MHz for TPM)
 * ══════════════════════════════════════════════ */
static void clock_init(void) {
    // TPM Source = (OSCERCLK, 8 MHz)

	// Step 1: Clear bits 25:24 of SOPT2
	// ~0x03000000 = 0xFCFFFFFF
    SIM->SOPT2 &= 0xFCFFFFFF;
    // Step 2: Set bits 25:24 to 10 (OSCERCLK = 8MHz)
    // 0x2 << 24 = 0x02000000
    SIM->SOPT2 |= 0x2 << 24;

    // PIT uses bus clock (24MHz) automatically
    // No configuration needed here
}

/* ══════════════════════════════════════════════
 * GPIO init
 * ══════════════════════════════════════════════ */
static void gpio_init(void) {
    // Enable clocks: Port A, B, C, D, E
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK |
                  SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTC_MASK;

    /* ── Motor direction pins (GPIO output) ── */
    PORTB->PCR[0] = PORT_PCR_MUX(1);   // AI2
    PORTB->PCR[1] = PORT_PCR_MUX(1);   // AI1
    PORTC->PCR[1] = PORT_PCR_MUX(1);   // BI1
    PORTC->PCR[2] = PORT_PCR_MUX(1);   // BI2
    GPIOB->PDDR |= (1<<0) | (1<<1);
    GPIOC->PDDR |= (1<<1) | (1<<2);

    /* ── SW1 = PTC3, input, pull-up ── */
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOC->PDDR &= ~(1<<3);  // input

    /* ── Encoder inputs (GPIO input) ── */
    // Left encoder Pin A = PTA6
    PORTA->PCR[6] = PORT_PCR_MUX(1);
    GPIOA->PDDR &= ~(1<<6);

    // Right encoder Pin A = PTA14
    PORTA->PCR[14] = PORT_PCR_MUX(1);
    GPIOA->PDDR  &= ~(1<<14);
}

/* ══════════════════════════════════════════════
 * PWM init  (TPM1 ch0 = Left, TPM2 ch0 = Right)
 *   OSCERCLK = 8 MHz, prescale /1, MOD = 7999
 *   → period = 8000/8MHz = 1 ms  (1 kHz)
 * ══════════════════════════════════════════════ */
static void pwm_init(void) {
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK;

    PORTB->PCR[2] = PORT_PCR_MUX(3);   // TPM2_CH0  (Left PWMA)
    PORTB->PCR[3] = PORT_PCR_MUX(3);   // TPM2_CH1  (Right PWMB)

    // Configure TPM2
    TPM2->SC   = 0;                     // disable while configuring
    TPM2->MOD  = PWM_MOD;
    // Channel 0 – edge-aligned PWM, high-true
    TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[0].CnV  = 0;
    // Channel 1 – same
    TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[1].CnV  = 0;
    TPM2->SC   = TPM_SC_CMOD(1);        // internal clock, PS=1 (/1)
}

/* ══════════════════════════════════════════════
 * Encoder GPIO interrupt init  (PORTA)
 *   Rising-edge interrupt on PTA6 and PTA14
 * ══════════════════════════════════════════════ */
static void encoder_interrupt_init(void) {
    // PTA6: rising-edge interrupt  IRQC = 1001 (0x9)
    PORTA->PCR[6]  |= (0x9 << 16);
    // PTA14: rising-edge interrupt
    PORTA->PCR[14] |= (0x9 << 16);

    // IRQ 30 = PORTA (Table 3-7, KL46 ref manual)
    NVIC->ISER[0] |= (1 << 30);
}

/* ══════════════════════════════════════════════
 * PIT init  (100 ms interval)
 *   Bus clock = 24 MHz
 *   LDVAL = 24,000,000 * 0.1 - 1 = 2,399,999
 * ══════════════════════════════════════════════ */
static void pit_init(void) {
    SIM->SCGC6 |= (1 << 23);          // enable PIT clock
    PIT->MCR    = 0;                             // enable PIT module (FRZ=0, MDIS=0)
    PIT->CHANNEL[0].LDVAL = 2399999U;            // 100 ms at 24 MHz bus clock
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK  // enable interrupt (0x03 for both mask,TIE = Timer Interrupt Enable.TEN = Timer Enable)
                           | PIT_TCTRL_TEN_MASK; // start timer

    // IRQ 22 = PIT (Table 3-7, KL46 ref manual)
    NVIC_EnableIRQ(22);
}

/* ══════════════════════════════════════════════
 * SW1 interrupt init  (PTC3, falling-edge)
 * ══════════════════════════════════════════════ */
static void button_interrupt_init(void) {
    // IRQC = 1010 (0xA) → falling edge
    PORTC->PCR[3] &= ~PORT_PCR_IRQC_MASK; // clear bits 19:16
    PORTC->PCR[3] |=  PORT_PCR_IRQC(0xA); // set bits 19:16

    // IRQ 31 = PORTC/PORTD (Table 3-7)
    NVIC_EnableIRQ(31);
}

/* ══════════════════════════════════════════════
 * INTERRUPT HANDLER 1: PORTA
 *   Counts rising edges on encoder Pin A
 *   (PTA6 = left encoder, PTA14 = right encoder)
 * ══════════════════════════════════════════════ */
void PORTA_IRQHandler(void) {
    if (PORTA->ISFR & (1 << 6)) {
        PORTA->ISFR = (1 << 6);     // clear flag
        leftTicks++;
    }
    if (PORTA->ISFR & (1 << 14)) {
        PORTA->ISFR = (1 << 14);    // clear flag
        rightTicks++;
    }
}

/* ══════════════════════════════════════════════
 * INTERRUPT HANDLER 2: PIT  (every 100 ms)
 *   - Manages WAITING and MOVING timing
 *   - Runs PID and updates PWM duty cycles
 * ══════════════════════════════════════════════ */
void PIT_IRQHandler(void) {
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
    stateTicks++;

    if (state == WAITING) {
        if (stateTicks >= WAIT_TICKS) {
            state = MOVING;
            stateTicks = 0;
            // reset PID state fresh
            leftPower  = 4000.0f;
            rightPower = 4000.0f;
            leftSumError  = 0; leftPrevError  = 0;
            rightSumError = 0; rightPrevError = 0;
            leftTicksLast  = leftTicks;
            rightTicksLast = rightTicks;
            motors_forward();
            TPM2->CONTROLS[0].CnV = (uint32_t)leftPower;
            TPM2->CONTROLS[1].CnV = (uint32_t)rightPower;
        }
    }
    else if (state == MOVING) {
        if (stateTicks >= MOVE_TICKS) {
            // stop FIRST before anything else
            state = STOPPED;
            motors_stop();
            return;  // ← exit immediately, don't run PID
        }

        uint32_t lCount = leftTicks  - leftTicksLast;
        uint32_t rCount = rightTicks - rightTicksLast;
        leftTicksLast  = leftTicks;
        rightTicksLast = rightTicks;

        pid_update(lCount, &leftPower,  &leftPrevError,  &leftSumError);
        pid_update(rCount, &rightPower, &rightPrevError, &rightSumError);

        TPM2->CONTROLS[0].CnV = (uint32_t)leftPower;
        TPM2->CONTROLS[1].CnV = (uint32_t)rightPower;
    }
    // STOPPED state → do nothing, PID never runs
}

/* ══════════════════════════════════════════════
 * INTERRUPT HANDLER 3: PORTC/PORTD
 *   SW1 press → transition IDLE → WAITING
 * ══════════════════════════════════════════════ */
void PORTC_PORTD_IRQHandler(void) {
    if (PORTC->ISFR & (1 << 3)) {
        PORTC->ISFR = (1 << 3);     // clear flag

        if (state == IDLE) {
            state      = WAITING;
            stateTicks = 0;
        }
    }
}

/* ══════════════════════════════════════════════
 * main
 * ══════════════════════════════════════════════ */
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    clock_init();
    gpio_init();
    pwm_init();
    encoder_interrupt_init();
    pit_init();
    button_interrupt_init();

    motors_stop();
    state = IDLE;

    // Main loop does nothing – all logic lives in interrupt handlers
    while (1) {
        __asm volatile ("nop");
    }

    return 0;
}
