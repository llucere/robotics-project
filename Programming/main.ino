#include <SoftwareSerial.h>

#define CLKPIN 0
#define DATAPIN 0
#define LATCHPIN 0

#define BTRXPIN 0
#define BTTXPIN 0

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

SoftwareSerial BluetoothModule(BTRXPIN, BTTXPIN); // RX, TX
/*
? queries - https://ccrma.stanford.edu/~fgeorg/250a/lab2/arduino-0019/reference/PortManipulation.html
* B - 8:13 (PORTB, DDRB, PINB) (two high bits, 6 & 7, are not usable)
* C - 14:21 (PORTC, DDRC, PINC) (analog pins, accept 0 - 1023)
* D - 0:7 (PORTD, DDRD, PIND) (0 and 1 are tx and rx respectively)
*/

typedef enum {
	movementOPC 	= 0x0,
	launchOPC	= 0x1,
	modifySpeedOPC	= 0x2
} opcodes;

// Kearney 12.7.22
#define GET(STREAM, N) STREAM & (1 << N)
#define SET(STREAM, N) STREAM |= 1 << (N)
#define CLR(STREAM, N) STREAM &= ~(1 << (N))

// Kearney 12.15.22
#define ABS(x) ((x) < 0 ? (-x) : (x))
#define SQR(x) ((x) * (x))

// Kearney 12.7.22
// SET/CLR without changing 1st or 2nd bits
// #define SETEXC1N2(STREAM, N) SET(STREAM, N) & 0b11111100
// #define CLREXC1N2(STREAM, N) CLR(STREAM, N) & 0b11111100

// SET/CLR without changing 6th or 7th bits
// #define SETEXC6N7(STREAM, N) SET(STREAM, N) & 0b00111111
// #define CLREXC6N7(STREAM, N) CLR(STREAM, N) & 0b00111111

// Kearney 12.7.22
#define WRITE_PIN(PIN, BSTREAM, DSTREAM, N) {	\
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
	WRITE_PIN(pin, PORTB, PORTD, datum);
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
	(uint64_t)millis(), 	// Front Right
	(uint64_t)millis(),	// Front Left
	(uint64_t)millis(), 	// Back Right
	(uint64_t)millis(), 	// Back Left
	(uint64_t)millis(), 	// Launcher Right
	(uint64_t)millis()	// Launcher Left
}

uint8_t outputPin[4] = { 	// The shift register pins to be pulsed
	// >> 1 for Left Wheels to swap them to reverse mode. << 1 for Right wheels to do the same. it's that simple, no additional logic needed
	// e.g. `outputPin[0] <<= 1;` changes the front right wheel to reverse. doing this operation again messes stuff up unless u set it back
	1 << 0, // Front Right
	1 << 3, // Front Left
	1 << 4, // Back Right
	1 << 7	// Back Left
}

typedef enum {
	frontRightRev = 0,
	frontLeftRev,
	backRightRev,
	backLeftRev
} revPins;

inline void setReverse(uint8_t index, bool rev) {
	switch (index) {
		case (frontRightRev):
			outputPin[0] = rev ? 1 << 1 : 1 << 0;
			break;
		case (frontLeftRev):
			outputPin[1] = rev ? 1 << 2 : 1 << 3;
			break;
		case (backRightRev):
			outputPin[2] = rev ? 1 << 5 : 1 << 4;
			break;
		case (backLeftRev):
			outputPin[3] = rev ? 1 << 6 : 1 << 7;
			break;
	}
}

// GND/VCC on motor drivers from shift register:
/*
quick documentation:
- 1 is the first shift register, 2 is the one daisy-chained to it
- Q4, Q8, Q2, Q3, etc, are the output pins of the corresponding shift register
- VCC and GND really doesnt make sense here but its pretty much the two pins connected to the motor from the shift register to the motor driver to the motor.
- MM means the movement motor, R means right, L means left (and launcher for the last 4), F is front, B is back

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

inline void shiftOutFast(uint8_t dataPin, uint8_t clkPin, uint16_t data) {
	// latch low

	/*
	for 0 to 15 do
	{
		/ send lsb to data pin
		/ clk high
		/ clk low
		/ shift data >> 1
	}
	*/

	// latch high
}

// Individual motor speed calculations
// Smaller is faster, larger is significantly slower
uint32_t launcherRightSpeed,
	launcherLeftSpeed,
	frontRightSpeed,
	frontLeftSpeed,
	backRightSpeed,
	backLeftSpeed;

// Kearney 12.14.22
void pulseDuePWM() {
	PWMData ^= PWMData; // Clear PWM data variable (set to zero) with XOR. XOR Just incase The compiler doesn't go for maximum optimability	
	register uint64_t time = millis();

	// Instead of looping, loop unrolling is used. This is to improve speed ever so slightly.
	// Launcher Right
	if (time >= duePWM[4]) {
		duePWM[4] = time + launcherRightSpeed;
		PWMData |= 1 << 8; // 1<<8 is the pin to set
	}

	// Launcher Left
	if (time >= duePWM[5]) {
		duePWM[5] = time + launcherLeftSpeed;
		PWMData |= 1 << 11;  // 1<<11 is the pin to set
	}

	// Front Right
	if (time >= duePWM[0]) {
		duePWM[0] = time + frontRightSpeed;
		PWMData |= outputPin[0];
	}

	// Front Left
	if (time >= duePWM[1]) {
		duePWM[1] = time + frontLeftSpeed;
		PWMData |= outputPin[1];
	}

	// Back Right
	if (time >= duePWM[2]) {
		duePWM[1] = time + backRightSpeed;
		PWMData |= outputPin[2];
	}

	// Back Left
	if (time >= duePWM[3]) {
		duePWM[1] = time + backLeftSpeed;
		PWMData |= outputPin[3];
	}

	// send datum quickly
	shiftOutFast(DATAPIN, CLKPIN, PWMData);
}

// Kearney 12.15.22
inline void setSpeed(uint32_t* speedVariable, uint16_t speed) {
	*speedVariable = (16 * SQR(ABS(speed))) / 1000;
}

inline void config() {
	Serial.begin(9600); // enable serial with data exchange rate of 9600
	BluetoothModule.begin(9600); // enable bluetooth module with data exchange rate of 9600
}

void setup() {
	
}



uint64_t lastRec = millis();
void loop() {
	if (BluetoothModule.available() >= 4) {
		// if 4 bytes are able to be read
		uint8_t instruction = BluetoothModule.read();
		int8_t data[3];
		for (int i = 0; i < 3; i++) {
			data[i] = BluetoothModule.read();
		}

		// reset reverse data
		setReverse(frontRightRev, false);
		setReverse(frontLeftRev, false);
		setReverse(backRightRev, false);
		setReverse(backLeftRev, false);

		switch (instruction) {
			case (movementOPC):
				// change motors to reverse mode, execute speed functions, etc.
				int8_t X = data[0] * 6.35, Z = data[1] * 6.35;
				int8_t midPoint = (X + Z) / 2;

				if (X > 0) {
					if (Z > 0) {
						// move forward right
					} elseif (Z == 0) {
						// turn right
					} elseif (Z < 0) {
						// move backwards right
					}
				} elseif (X == 0) {
					if (Z > 0) {
						// move forward
					} elseif (Z == 0) {
						// dont move
					} elseif (Z < 0) {
						// move backwards	
					}
				} elseif (X < 0) {
					if (Z > 0) {
						// move forward left
					} elseif (Z == 0) {
						// turn left
					} elseif (Z < 0) {
						// move back left
						setSpeed(&frontLeftWheel, (int8_t)(Z / 3));
						setSpeed(&backRightWheel, (int8_t)(Z / 3));
						backLeftWheel = 0;
						frontRightWheel = 0;

						setReverse(frontLeftRev, true);
						setReverse(backRightRev, true);
					}
				}

				break;
			case (launchOPC):
				break;
			case (modifySpeedOPC):
				break
			default:
				break;
		}
	}
	pulseDuePWM();
	delayMicroseconds(100); // slow and steady wins the race
}