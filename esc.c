#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MINIMUM_DUTY_CYCLE 25
#define STARTUP_DUTY_CYCLE 30


/*
Driver Outputs are active low
PWM high side inputs using timer0

Timers
timer0: phase A PWM
        overflow triggers zero crossing detection

timer1: commutation timing
        counts up twice per commutation
            1. counts up until zero crossing is detected and is reset. interrupt B enables zero crossing (compare A disabled)
            2. counts up to OCR1A, interrupt triggers a commutation (compare B disabled)
        commutation time is approximately 2 * OCR1A

timer2: phase B and C PWM

*/

volatile int step = 0;
volatile unsigned char dutyCycle = STARTUP_DUTY_CYCLE;
volatile unsigned char MUXTemp = 0;


void nextStep(void);
void startMotor(void);

/*-----------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------Main--------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------*/

int main(void){

    // all outputs high (inverting inputs on driver)
    //PORTD = 00A0 C000
    //PORTB = 0000 Babc

    DDRD = 0x28;
    DDRB = 0x0F;
    PORTD = 0x28;
    PORTB = 0x0F;

    cli();

    // timer 0; phase correct PWM phase A
    TCCR0A |= (1 << WGM00);
    TCCR0B |= (1 << CS00); // no prescaler
    OCR0B = dutyCycle;

    // timer 2; phase correct PWM phase B and C
    TCCR2A |= (1 << WGM20);
    TCCR2B |= (1 << CS20); // no prescaler
    OCR2A = dutyCycle;
    OCR2B = dutyCycle;

    // timer1 for commutation; normal mode, /8 prescaler
    TCCR1B |= (1 << CS11); // 8 prescaler

    nextStep();
    startMotor();

    // enable adc for speed measurement
    ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIF) | (1 << ADPS2); // enable auto trigger, /16 prescaler
    ADCSRB |= (1 << ADTS2); // auto trigger on timer 0 overflow
    ADMUX = (1 << REFS0) | (1 << ADLAR);

    //watchdog timer for stall detection
    __asm__ __volatile__("wdr");
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE); // start watchdog timer change sequene
    WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP0); // interrupt on time out, 16ms timer and clear WDCE

    sei();

    for(;;){

    }
}

/*-----------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------ISRs--------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------*/

// set to high state on timer overflow
ISR(TIMER0_OVF_vect){
    if (ADMUX != 0x60){
        //          falling                                      ||           Rising
        if ((((ACSR & (1 << ACO)) != 0) && ((step & 0x01) == 1)) || (((ACSR & (1 << ACO)) == 0) && ((step & 0x01) == 0))) {
            OCR1A = (3 * OCR1A + TCNT1)/4; // update commutation time
            OCR1B = OCR1A/2; //delay before checking for zero crossing
            TCNT1 = 40; // reset timer 1 for commutation timing
            TIMSK1 = (1 << OCIE1A); // enable commutation interrupt and disable zero crossing hold off
            TIFR1 = TIFR1; // clear interrupt flags

            // enable adc
            ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIF) | (1 << ADPS2); // enable auto trigger, /16 prescaler
            ADCSRB |= (1 << ADTS2); // auto trigger on timer 0 overflow
            ADMUX = (1 << REFS0) | (1 << ADLAR);
        }
    }
    // read speed reference (Potentiometer)
    else {
        while (!(ADCSRA & (1 << ADIF))); // wait for ADC
        dutyCycle = ADCH;
        dutyCycle = dutyCycle > MINIMUM_DUTY_CYCLE ? dutyCycle : MINIMUM_DUTY_CYCLE;
        OCR0B = dutyCycle;
        OCR2A = dutyCycle;
        OCR2B = dutyCycle;
    }
}

// commutation
ISR(TIMER1_COMPA_vect){
    nextStep();
    // enable adc
    ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIF) | (1 << ADPS2); // enable auto trigger, /16 prescaler
    ADCSRB |= (1 << ADTS2); // auto trigger on timer 0 overflow
    ADMUX = (1 << REFS0) | (1 << ADLAR);

    TIMSK1 = (1 << OCIE1B); // disable commutation interrupt and enable zero crossing hold off
    TIFR1 = TIFR1; // clear interrupt flags
    TIFR0 = TIFR0;
    __asm__ __volatile__("wdr");
}

// enables zero crossing detection
ISR(TIMER1_COMPB_vect){
    //enable comparator
    ADMUX = MUXTemp;
    ADCSRA = 0;
    ADCSRB = (1 << ACME);

    TIFR1 = TIFR1; // clear interrupt flags
    TIFR0 = TIFR0;
}

// stop timers if watchdog is not reset (stall)
ISR(WDT_vect){
    TCCR0B = 0;
    TCCR1B = 0;
    TCCR2B = 0;

}

/*-----------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------Functions------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------*/


// Open loop Startup procedure
// Cycles through commutation steps at
// an increaing rate to get motor moving
// Enables interrupts for b-emf commutation
void startMotor(){
    TCNT1 = 0;
    for (unsigned int i = 30000; i > 5000; i *= 0.99){ //2050
        OCR1A = i;
        // delay until match occurs
        while(!(TIFR1 & (1 << OCF1A))){
        }
        nextStep();
        TIFR1 |= (1 << OCF1A); // clear compare flag
    }

    TIMSK1 |= (1 << OCIE1B); // zero corssing hold off interrupt
    TIMSK0 |= (1 << TOV0); // overflow interrupt
    OCR1A = OCR1A/2;
    OCR1B = OCR1A/2; //delay before checking for zero crossing
}


// move to the next commutation step
//      Updates timers/PWM
//      Updates GPIO
//      Updates ADC Multiplexer

void nextStep(){
    TCNT1 = 0; // reset timer

    //PORTD = 00A0 C000
    //PORTB = 0000 Babc
    switch (step) {
        case 0: //A Top, b Bottom
            TCCR2A = 0x01; // B&C disabled
            PORTD = 0x28; // 0010 1000
            PORTB = 0x0D; // 0000 1101
            TCCR0A = 0x31; // A top
            MUXTemp = 0x03;
            break;
        case 1: //A Top, c Bottom
            TCCR2A = 0x01; // B&C disabled
            PORTD = 0x28; // 0010 1000
            PORTB = 0x0E; // 0000 1110
            TCCR0A = 0x31; // A top
            MUXTemp = 0x02;
            break;
        case 2: //B Top, c Bottom
            TCCR0A = 0x01; // A disabled
            PORTD = 0x28; // 0010 1000
            PORTB = 0x0E; // 0000 1110
            TCCR2A = 0xC1; // B top
            MUXTemp = 0x01;
            break;
        case 3: //B Top, a Bottom
            TCCR0A = 0x01; // A disabled
            PORTD = 0x28; // 0010 1000
            PORTB = 0x0B; //0000 1011
            TCCR2A = 0xC1; // B top
            MUXTemp = 0x03;
            break;
        case 4: //C Top, a Bottom
            TCCR0A = 0x01; // A disabled
            PORTD = 0x28; // 0010 1000
            PORTB = 0x0B; //0000 1011
            TCCR2A = 0x31; // C top
            MUXTemp = 0x02;
            break;
        case 5: //C Top, b Bottom
            TCCR0A = 0x01; // A disabled
            PORTD = 0x28; // 0010 1000
            PORTB = 0x0D; // 0000 1101
            TCCR2A = 0x31; // C top
            MUXTemp = 0x01;
            break;
    }
    step = (step + 1) % 6;
}
