/********************************************
 *
 *  Name: Hannah Nguyen
 *  Email: nnguyen3@usc.edu
 *  Section: 12:30 W Lab
 *  Assignment: Thermostat
 * 
 * NOTE: did not get the last two check offs in time, working now
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "ds18b20.h"
#include <stdint.h>
#include "lcd.h"
#include <avr/io.h>


#define BUZZER PB5 

void timer0_init();
void timer1_init();
void timer2_init();
void temp_init();
void updateTemp(unsigned char newLow, unsigned char newHigh);
void buzzer_init();
void play_note();
void button_pressed();

volatile uint8_t new_state, old_state; // states for rotary
volatile uint8_t changed = 0;  // flag for state change
volatile int16_t count = 0;		// count for temp settings
volatile uint8_t a, b;
volatile uint16_t red, blue;

volatile uint8_t flag = 0; // flag for button and led values
volatile uint8_t old_flag = 0;
volatile uint16_t color = 0;

unsigned char celciusMSB = 0; // temperature variables
unsigned char celciusLSB = 0;
unsigned char high, low;
unsigned char old_high, old_low;
volatile uint16_t currTemp = 0;

volatile uint16_t soundBuzzer = 0; // buzzer variables
volatile uint8_t alertActive = 0;
volatile uint8_t buzzerActive = 0;
volatile uint16_t max = 0; 
volatile uint16_t run = 0;

volatile uint16_t displayState = 0; // servo variables
volatile uint16_t timeRun = 0;
volatile uint16_t timerOn = 0;
volatile uint8_t buttonPressed = 0;
int16_t slope;
int16_t y_intercept;

void initServoMapping() {
	// slope =  (12 - 35) / (100 - 40);
	slope = -383; // scaled slope

    y_intercept = 50;
}

// Function to calculate OCR2A value based on temperature
uint16_t calculateOCR2A(uint8_t temperature) {

	// linear equation
	int32_t result = (int32_t)((temperature * slope)/1000 + y_intercept);

	// accomodate for switching neg
	if (result >= 70){
		result -= 65;
	}

    return result;
}

int main(void) {
	
    unsigned char t[2];

    if (ds_init() == 0) {    // Initialize the DS18B20
        // Sensor not responding
    }

    ds_convert();    // Start first temperature conversion

	// call initialization functions
	lcd_init();
	temp_init();
	timer0_init();
	timer2_init();
	timer1_init();
	initServoMapping();

    // Initialize DDR and PORT registers and LCD 
	PORTC |= (1 << PC1) | (1 << PC2);
	DDRB |= (1 << PB4);

	// buzzer
	DDRB |= (1 << BUZZER);

	// servo
	DDRB |= (1 << PB3);
	
	// pin interrupts
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);

	// red + blue buttons
	PORTD |= (1 << PD3) | (1 << PD2);
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT19) | (1 << PCINT18);

	// LED
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5);
	
	
	sei();

    // Write a spash screen to the LCD
	lcd_writecommand(1);
	lcd_moveto(0, 0);
    lcd_stringout("EE 109 Project");
    lcd_moveto(1, 0);
    lcd_stringout("Hannah Nguyen");
    _delay_ms(1000);
	lcd_writecommand(1);
	lcd_stringout("Temp: ");
	lcd_moveto(1,0);

	// get low and high from eeprom memory
	low = eeprom_read_byte((void *)0);
	high = eeprom_read_byte((void *)100);

	// display on lcd
	lcd_stringout("Low: ");
	char l[4];
	snprintf(l, 4, "%d", low);
	lcd_stringout(l);

	lcd_moveto(1, 8);
	lcd_stringout("High: ");
	char h[4];
	snprintf(h, 4, "%d", high);
	lcd_stringout(h);

	// set holders
	old_low = low;
	old_high = high;
	color = 0;
	
    // Read the A and B inputs to determine the intial state.
    // In the state number, B is the MSB and A is the LSB.
    // Warning: Do NOT read A and B separately.  You should read BOTH inputs
    // at the same time, then determine the A and B values from that value.    
	uint8_t x = PINC;
	a = (x & (1 << PC1)) >> PC1;
	b = (x & (1 << PC2)) >> PC2;

    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else {
	old_state = 3;
	}
	

	new_state = old_state;
	
	

    while (1) {                 // Loop forever

		// update temperatures stored
		updateTemp(low, high);

		if (ds_temp(t)) {    // True if conversion complete
            /*
              Process the values returned in t[0]
              and t[1] to find the temperature.
            */
		   	char MSB = t[1];
			char LSB = t[0];

			unsigned char oldMSB = celciusMSB;
			//combine MSB and LSB into a single 16-bit signed variable
			unsigned char celciusMSB = ((MSB & 0x0F) << 4) | ((LSB & 0xF0) >> 4);
			unsigned char celciusLSB = (LSB & 0x0F);

			// change tenths range from 0-15 to 0-10
			unsigned char tenths = (celciusLSB * 10) / 15;

			if (tenths >= 10){ // works when temp going down
				tenths = 0;
				celciusMSB += 1;
			}

			unsigned char f = (((celciusMSB)) * 9) / 5 + 32;
			// unsigned char f2 = (((celciusLSB)) * 9) / 5 + 32;
			
			// scale temp
			currTemp = (f*10) + tenths;
			lcd_moveto(0, 6);

			// display curr temp
			char buff[15];
			snprintf(buff, 15, "%d.%d", f, tenths);
			lcd_stringout(buff);

			if (displayState == 0) {
				uint16_t ocr2aValue = calculateOCR2A(f);
				OCR2A = ocr2aValue;
			}
			
            ds_convert();   // Start next conversion
		
        }
		   
		// change led colors
		if (color == 1 || (currTemp >= high*10)) { 			// led blue
			PORTC &= ~(1 << PC5); 
			PORTC |= ((1 << PC3) | (1 << PC4));
		} else if (color == 2 || (currTemp <= low*10)) { 	// led red
			PORTC &= ~(1 << PC3); 
			PORTC |= ((1 << PC5) | (1 << PC4));
		} else { 											// led green
			PORTC &= ~(1 << PC4); 
			PORTC |= ((1 << PC3) | (1 << PC5));
		}

		// sounds buzzer if temp goes more than 3 below or higher than low/high bounds
		if ( ((currTemp <= (low - 3)*10)) && !alertActive && !buzzerActive) {
			alertActive = 1; 

		} else if ( ((currTemp >= (high + 3)*10)) && !alertActive && !buzzerActive) {
			alertActive = 1;
		}

		// temperature within the range, reset the alert flag
		if ((currTemp > (low-3)*10) && (currTemp < (high+3)*10)) {
			alertActive = 0;
			buzzerActive = 0;
		}

		if (alertActive && !buzzerActive){
			play_note();
		}
		


		// change highs and lows when buttons are pressed
		if (flag == 2){
			// start 4 second timer
			if (timerOn == 0 && buttonPressed){
				// lcd_stringout("here");
				// _delay_ms(50);
				button_pressed();
			}

			lcd_moveto(1, 8);
			lcd_stringout("High: ");
			lcd_moveto(1,0);
			lcd_stringout("Low? ");

			if (changed) { // low
				
				// reset changed variable
				changed = 0;
 
				// update low if higher than high
				low = count;
				if (low >= high){
					low = high - 1;
				}
				
				// print low
				lcd_moveto(1, 5);
				char buf[4];
				snprintf(buf, 4, "%d", low);
				lcd_stringout(buf);
				
				// servo: display low setting 
				if (displayState == 1) {
					uint16_t ocr2aValueLow = calculateOCR2A(low);
					OCR2A = ocr2aValueLow;
				}
			}
		}
		else if (flag == 1){
			// start 4 second timer
			if (!timerOn && buttonPressed){
				button_pressed();
			}
			

			lcd_moveto(1,0);
			lcd_stringout("Low: ");
			lcd_moveto(1, 8);
			lcd_stringout("High? ");

			if (changed) { // high
				
				// update changed
				changed = 0;

				// update high if less than low
				high = count;
				if (high <= low){
					high = low + 1;
				}
				
				// print high
				lcd_moveto(1, 14);
				char buf[4];
				snprintf(buf, 4, "%d", high);
				lcd_stringout(buf);


				// servo displays high setting
				if (displayState == 2) {
					uint16_t ocr2aValueHigh = calculateOCR2A(high);
					OCR2A = ocr2aValueHigh;
				}
			
			}
		}
		else {
			// update print screen
			lcd_moveto(1,0);
			lcd_stringout("Low: ");
			char l[4];
			snprintf(l, 4, "%d", low);
			lcd_stringout(l);

			lcd_moveto(1, 8);
			lcd_stringout("High: ");
			char h[4];
			snprintf(h, 4, "%d", high);
			lcd_stringout(h);
		}


	}
}


ISR(PCINT1_vect)
{
	// only accounts for rotary if button is pushed
	if (flag != 0) {

		// count value
		uint8_t x = PINC;
		a = (x & (1 << PC1)) >> PC1;
		b = (x & (1 << PC2)) >> PC2;
		
		if (flag == 1) { // red
			count = old_high;
		}
		if (flag == 2) { // red
			count = old_low;
		}

		// update count based on rotary
		if (old_state == 0) {
			if (a) {
				new_state = 1;
				count++;
			}
			else if (b) {
				new_state = 3;
				count--;
			}

		}
		else if (old_state == 1) {
			// Handle A and B inputs for state 1
			if (b) {
				new_state = 2;
				count++;
			}
			else if (!a) {
				new_state = 0;
				count--;
			}
		}
		else if (old_state == 2) {
			// Handle A and B inputs for state 2
			if (!a) {
				new_state = 3;
				count++;
			}
			else if (!b) {
				new_state = 1;
				count--;
			}
		}
		else {   // old_state = 3
			// Handle A and B inputs for state 3
			if (a) {
				new_state = 2;
				count--;
			}
			else if (!b) {
				new_state = 0;
				count++;
			}
		}

		// keeps count within 50-90
		if (count < 50){
			count = 50;
		}
		if (count > 90){
			count = 90;
		}
		
	}
	// red: attempting to set low higher than high
	if (flag == 1 && count <= low + 1) { 
		color = 2;
	}
	// blue: attempting to set high lower than low
	else if (flag == 2 && count >= high - 1){ 
		color = 1;
	}
	// green
	else {
		color = 0;
	}

	// update old states and values
	if (new_state != old_state){
		changed = 1;
		old_state = new_state;
		
		if (flag == 1) { 
			old_high = count;
			high = count;
		}
		if (flag == 2) { 
			old_low = count;
			low = count;
		}
	}
}

ISR(PCINT2_vect) {
	uint8_t x = PIND;
	red = (x & (1 << PD3)) >> PC3;
	blue = (x & (1 << PC2)) >> PC2;

	// red is pushed and not set
	if (!red && flag == 0){
		flag = 1;
		buttonPressed = 1;
	} 
	// blue is pushed and not set
	else if (!blue && flag == 0){
		flag = 2;
		buttonPressed = 1;
	} 
	// set when buttons flip
	else if (!red && flag == 2){
		low = count;
		flag = 1;
		buttonPressed = 1;
	}
	else if (!blue && flag == 1){
		high = count;
		flag = 2;
		buttonPressed = 1;
	}
}

void temp_init() {
	// EEPROM low location : 0
	// EEPROM low location : 100
    unsigned char storedLow = eeprom_read_byte((void *)0);
	unsigned char storedHigh = eeprom_read_byte((void *)100);

    // check range, set default if out of range or not set yet
    if (storedLow == 0xFF || storedLow < 50 || storedLow > 90) {
        eeprom_update_byte((void *)0, 50);
    }
	if (storedHigh == 0xFF || storedHigh < 50 || storedHigh > 90) {
        eeprom_update_byte((void *)100, 90);
    }
}

void updateTemp(unsigned char newLow, unsigned char newHigh) {
    // set to new values if in range
    if (newLow >= 50 && newLow <= 90) {
        eeprom_update_byte((void *)0, newLow);
    }
	if (newHigh >= 50 && newHigh <= 90) {
        eeprom_update_byte((void *)100, newHigh);
    }
}

void timer0_init() {
    // Set prescaler to 64
    TCCR0B |= (1 << CS01) | (1 << CS00);
    // Enable TIMER0 compare match interrupt
    TIMSK0 |= (1 << OCIE0A);
	TCCR0A |= (1 << WGM01);
}

void play_note() {
	buzzerActive = 1;
    max = 256;
    // prescaler 64 -> 16000000 * 0.5 / (256 * 64)
    OCR0A = 488;
    TCCR0B |= (1 << CS01) | (1 << CS00);

    // start the timer 
    TCCR0B |= (1 << WGM02);
}


ISR(TIMER0_COMPA_vect)
{
	// toggle bit to play sound
	PORTB ^= (1 << BUZZER); 

	run++;

	// stop when max reached: 0.5 sec
	if (run >= max) {
		TCCR0B &= ~( (1 << CS01) | (1 << CS00));
		PORTB &= ~(1 << BUZZER); 
		run = 0;
	}
}

void timer2_init(void) {
    OCR2A = 128;
    
    TCCR2A |= (0b11 << WGM20);
    TCCR2A |= (0b10 << COM2A0);
	// TIMSK2 |= (1 << OCIE2A);

	TCCR2B |= (0b111 << CS20); 

}

ISR(TIMER1_COMPA_vect) {

	timeRun++;

	// stop after 4 seconds
	if (timeRun >= 1) {
		// stop timer
		TCCR1B = 0;
		timeRun=0;
		displayState = 0;
		timerOn = 0;
		buttonPressed = 0;
	}
    
}

void timer1_init() {
    TCCR1B |= (1 << WGM12); 
    TIMSK1 |= (1 << OCIE1A);
}

void button_pressed() {
	
	TCNT1 = 0;
	timerOn = 1;

    // set display state based on the button pressed
    if (flag == 1) {
        displayState = 2;
    } else if (flag == 2) {
        displayState = 1;
    }

	// start TIMER1 
	OCR1A = 62500; // 16,000,000 * 4 / 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);  

	// reset counter
	timeRun = 0; 

}