#define MOTOR_PIN 8

void setup() {
	pinMode(MOTOR_PIN, OUTPUT);
}

void loop() {
	digitalWrite(MOTOR_PIN, HIGH);
	delay(1000);
	digitalWrite (MOTOR_PIN, LOW);
	delay(1000);
}

// test failed: not enough amperage from Arduino pin (20mA, motor required *more*)