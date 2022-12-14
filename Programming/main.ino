#include <stdio.h>

#define CLKPIN 0
#define DATAPIN 0

// TYPE DECLARATIONS
typedef unsigned char uint8_t, byte;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;
typedef unsigned long long uint128_t;

typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long int64_t;
typedef long long int128_t;

/*
? queries - https://ccrma.stanford.edu/~fgeorg/250a/lab2/arduino-0019/reference/PortManipulation.html
* B - 8:13 (PORTB, DDRB, PINB) (two high bits, 6 & 7, are not usable)
* C - 14:21 (PORTC, DDRC, PINC) (analog pins, accept 0 - 1023)
* D - 0:7 (PORTD, DDRD, PIND) (0 and 1 are tx and rx respectively)
*/

// Kearney 12.7.22
#define GET(STREAM, N) STREAM & (1 << N)
#define SET(STREAM, N) STREAM |= 1 << (N)
#define CLR(STREAM, N) STREAM &= ~(1 << (N))

// SET/CLR without changing 1st or 2nd bits
#define SETEXC1N2(STREAM, N) SET(STREAM, N) & 0b11111100
#define CLREXC1N2(STREAM, N) CLR(STREAM, N) & 0b11111100

// SET/CLR without changing 6th or 7th bits
#define SETEXC6N7(STREAM, N) SET(STREAM, N) & 0b00111111
#define CLREXC6N7(STREAM, N) CLR(STREAM, N) & 0b00111111

// Kearney 12.7.22
#define WRITEPIN(PIN, BSTREAM, DSTREAM, N) {	\
	if (PIN >= 8 && PIN <= 13) { 		\
		if (N & 1) {			\
			SET(BSTREAM, PIN - 8);	\
		} else {			\
			CLR(BSTREAM, PIN - 8);	\
		}				\
	} else if (PIN >= 0 && PIN <= 13) {	\
		if (N & 1) {			\
			SET(DSTREAM, PIN);	\
		} else {			\
			CLR(DSTREAM, PIN);	\
		}				\
	} else {				\
		/* ! error */			\
	}					\
}

// ! The following function is considered slow, use port manipulation directly  when dealing with quick calculations or pulses.
// @param pin: a byte of data which represents which pin number to use (0:7, 8:13)
// @param datum: the bit to set the pin
// Kearney 12.7.22
inline void writePin(uint8_t pin, bool datum) {
	WRITEPIN(pin, PORTB, PORTD, datum);
}

// ! The following function is considered slow, use port manipulation directly when dealing with quick calculations or PWM.
// @param pin: a byte of data which represents which pin number to read (0:7, 8:13)
// Kearney 12.7.22
inline bool readPin(uint8_t pin) {
	if (pin >= 8 && pin <= 13) {
		// read the (n-8)th pin of PORTB
		return GET(PORTB, pin - 8);
	} else if (pin >= 0 && pin <= 7) {
		// read the (n-1)th pin of PORTD
		return GET(PORTD, pin);
	} else {
		// ! error
	}
}

// ! The following function is considered slow, use port manipulation directly when dealing with quick calculations or PWM.
// @param pin: a byte of data which represents which pin number to read (0:7, 8:13)
// @param mode: the bit to set the pin
// Kearney 12.7.22
inline void setPinMode(uint8_t pin, bool mode) {
	WRITEPIN(pin, DDRB, DDRD, mode);
}

uint16_t PWMData;		// The PWM timing data.
uint128_t duePWM[6] = { 	// The delay between PWM pulses
	(uint64_t)micros(), 	// Front Right
	(uint64_t)micros(),	// Front Left
	(uint64_t)micros(), 	// Back Right
	(uint64_t)micros(), 	// Back Left
	(uint64_t)micros(), 	// Launcher Right
	(uint64_t)micros()	// Launcher Left
}

uint8_t outputPin[4] = { 	// The shift register pins to be pulsed
	// >> 1 for Left Wheels to swap them to reverse mode. << 1 for Right wheels to do the same.
	1 << 0, // Front Right
	1 << 3, // Front Left
	1 << 4, // Back Right
	1 << 7	// Back Left
}

// GND/VCC on motor srivers from shift register:
/*
2Q4 - MMRF VCC
2Q3 - MMRF GND
2Q2 - MMLF GND
2Q1 - MMLF VCC

1Q8 - MMRB VCC
1Q7 - MMRB GND
1Q6 - MMLB GND
1Q5 - MMLB VCC

1Q4 - LMR VCC
1Q3 - LMR GND
1Q2 - LML GND
1Q1 - LML VCC
*/

// revData is multiple boolean variables (conglomerated into one) which swaps which pin is being pulsed
// Kearney 12.14.22

inline void shiftOutFast(uint8_t dataPin, uint8_t clkPin, uint16_t data) {
	// latch low

	/*
	for 0 to 15 do
	{
		/ send bit to data pin
		/ clk high
		/ clk low
	}
	*/

	// latch high
}

// calculations for speed
uint8_t launcherRightSpeed = 0,
	launcherLeftSpeed,
	frontRightSpeed,
	frontLeftSpeed,
	backRightSpeed,
	backLeftSpeed;

void pulseDuePWM() {
	// flush
	PWMData ^= PWMData; // Clear PWM data variable (set to zero) with xor. Just incase The compiler doesn't go for maximum optimability
	
	// Launcher Right
	if (micros() >= duePWM[4]) {
		duePWM[4] = micros() + launcherRightSpeed;
		PWMData |= (uint16_t)(1 << 8);
	}

	// Launcher Left
	if (micros() >= duePWM[5]) {
		duePWM[5] = micros() + launcherLeftSpeed;
		PWMData |= (uint16_t)(1 << 11);
	}

	// Instead of looping, loop unrolling is used. This is to improve speed ever so slightly.
	// Front Right
	if (micros() >= duePWM[0]) {
		duePWM[0] = micros() + frontRightSpeed;
		PWMData |= outputPin[0];
	}

	// Front Left
	if (micros() >= duePWM[1]) {
		duePWM[1] = micros() + frontLeftSpeed;
		PWMData |= outputPin[1];
	}

	// Back Right
	if (micros() >= duePWM[2]) {
		duePWM[1] = micros() + backRightSpeed;
		PWMData |= outputPin[2];
	}

	// Back Left
	if (micros() >= duePWM[3]) {
		duePWM[1] = micros() + backLeftSpeed;
		PWMData |= outputPin[3];
	}

	// send datum quickly
	shiftOutFast(DATAPIN, CLKPIN, PWMData);
}

void setup() {
	
}

void loop() {
	pulseDuePWM();
	delayMicroseconds(100); // slow and steady wins the race
}