#include <Arduino.h>
#include <DHT.h>
#include <U8g2lib.h>

// Constants for serial
const unsigned long BAUD = 115200;

// Constants for pins
const byte DHT_PIN = 2; // DHT sensor pin
const byte FAN_PIN = 9; // FAN pin

// Constants for temperature and fan control
const float TEMP_LOW = 25.0; // lowest temperature when fan should turn off
const float TEMP_HIGH = 38.0; // highest temperature when fan should be 100%
const float HYSTERESIS = 3.0; // prevent frequent on/off switching at the threshold
const byte FAN_OFF_DUTY = 0; // speed when fan is off
const byte FAN_MIN_DUTY = 10; // fan minimum speed
const byte FAN_MAX_DUTY = 100; // fan maximum speed

// Constants for delays
const int WAIT_INTERVAL = 2000; // pause time in milliseconds between loop runs

// Constants for frequency optimization
const word PWM_FREQ_HZ = 25000; // adjust the fan frequency to 25khz
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ); // sets the counter's max value to the clock frequency over twice the required pwm frequency

// Function prototypes
void setup();
void loop();

// Class that manages the DHT sensor
class DHTSensor {
public:
	DHTSensor(byte pin, byte type) : dht(pin, type) {}

	void begin() {
		dht.begin();
	}

	// read the temperature and humidity from the sensor
	void read(bool t = true, bool h = true) {
		if (t) {
			temp = dht.readTemperature();
		}
		if (h) {
			hum = dht.readHumidity();
		}
	}

	float getTemperature() {
		return temp;
	}

	float getHumidity() {
		return hum;
	}

private:
	DHT dht;
	float temp;
	float hum;
};

// Class that manages the display and u8g2 lib
class Display {
public:
	Display(const u8g2_cb_t *rotation, uint8_t clock, uint8_t data, uint8_t reset) : u8g2(rotation, clock, data, reset) {}

	void begin() {
		u8g2.begin();
		u8g2.enableUTF8Print();
	}

	void clear() {
		u8g2.clearBuffer();
	}

	void printTemperature(float temp) {
		u8g2.setFont(u8g2_font_fur35_tn);
		u8g2.setCursor(20, 37);
		u8g2.print(temp, 1);
		u8g2.setFont(u8g2_font_unifont_tf);
		u8g2.setCursor(112, 37);
		u8g2.print(F("°C"));
	}

	void printHumidity(float hum) {
		u8g2.setCursor(85, 50);
		u8g2.print(F("Hum"));
		u8g2.setCursor(85, 64);
		u8g2.print(hum, 1);
		u8g2.setCursor(120, 64);
		u8g2.print(F("%"));
	}

	void printFanSpeed(byte fanSpeed) {
		u8g2.setCursor(15, 50);
		u8g2.print(F("Fan"));
		u8g2.setCursor(15, 64);
		u8g2.print(fanSpeed, 1);
		u8g2.setCursor(40, 64);
		u8g2.print(F("%"));
	}

	void printFanBar(byte fanSpeed) {
		u8g2.drawFrame(0, 0, 10, 128);
		u8g2.drawBox(0, map(fanSpeed, 0, 100, 60, 0), 10, 128);
	}

	void sendBuffer() {
		u8g2.sendBuffer();
	}

	// update the screen with the info
	void update(float temp, float hum, byte speed) {
		clear();
		printTemperature(temp);
		printHumidity(hum);
		printFanSpeed(speed);
		printFanBar(speed);
		sendBuffer();
	}

private:
	U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2;
};

// Class that manages the fan
class FanController {
public:
	FanController(byte pin) : pin(pin), dutyCycle(FAN_OFF_DUTY), fanState(false), countMax(TCNT1_TOP) {}

	void begin() {
		pinMode(pin, OUTPUT);
		setupTimer1();
	}

	// set fan speed
	void setSpeed(byte speed) {
		// if speed changed
		if (speed != dutyCycle) {
			analogWrite(pin, (word)(speed * countMax) / 100); // set the new fan speed
			dutyCycle = speed;
			fanState = (speed > FAN_OFF_DUTY); // set the new fan state
		}
	}

	// get current fan speed
	byte getSpeed() {
		return dutyCycle;
	}

	// calculate fan speed depending on the temperature
	void controlFan(float temp) {
		// temp below minimum temperature
		if (temp < TEMP_LOW) {
			if (fanState) {
				if (temp < TEMP_LOW - (HYSTERESIS / 2)) {
					// fan is on, temp below threshold minus hysteresis -> switch off
					setSpeed(FAN_OFF_DUTY);
				} else {
					// fan is on, temp not below threshold minus hysteresis -> keep minimum speed
					setSpeed(FAN_MIN_DUTY);
				}
			}
		// temp is in between the minimum and maximum temperature
		} else if (temp < TEMP_HIGH) {
			// distinguish two cases to consider hysteresis
			if (fanState) {
				// fan is on, temp above threshold > control fan speed
				setSpeed(speedFluct(temp));
			} else {
				if (temp > TEMP_LOW + (HYSTERESIS / 2)) {
					// fan is off, temp above threshold plus hysteresis -> switch on
					setSpeed(speedFluct(temp));
				} else {
					// fan is off, temp not above threshold plus hysteresis -> keep off
					setSpeed(FAN_OFF_DUTY);
				}
			}
		// temp above maximum temperature
		} else {
			// fan is on, temp above maximum temperature -> maximum speed
			setSpeed(FAN_MAX_DUTY);
		}
	}

private:
	byte pin;
	byte dutyCycle;
	bool fanState;
	word countMax;

	// calculate the speed fluctuation depending on the temperature
	byte speedFluct(float temp) {
		return map(temp, TEMP_LOW, TEMP_HIGH, FAN_MIN_DUTY, FAN_MAX_DUTY);
	}

	// setup Timer control for fans frequency
	void setupTimer1(){
		// Clear Timer1 control and count registers
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1  = 0;

		// Set Timer1 configuration
		// COM1A(1:0) = 0b10   (Output A clear rising/set falling)
		// COM1B(1:0) = 0b00   (Output B normal operation)
		// WGM(13:10) = 0b1010 (Phase correct PWM)
		// ICNC1      = 0b0    (Input capture noise canceler disabled)
		// ICES1      = 0b0    (Input capture edge select disabled)
		// CS(12:10)  = 0b001  (Input clock select = clock/1)

		// sets the COM1A1 bit and also sets some of the waveform generation bits
		// COM1A1 bit enables toggling the pin every time the timer is full
		TCCR1A |= (1 << COM1A1) | (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << CS10);
		ICR1 = countMax;
	}
};

// Instantiate objects
DHTSensor dhtSensor(DHT_PIN, DHT22);
Display display(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
FanController fan(FAN_PIN);

// Setup and initialize program
void setup() {
	Serial.begin(BAUD);
	dhtSensor.begin();
	display.begin();
	fan.begin();
}

// Run program in loop
void loop() {
	// read the sensor
	dhtSensor.read();

	// check if there is a problem with the sensor
	if (isnan(dhtSensor.getTemperature()) || isnan(dhtSensor.getHumidity())) {
		Serial.println(F("Failed to read from DHT sensor!"));
		delay(WAIT_INTERVAL);
		return;
	}

	// calculate the fan speed depending on the temperature
	fan.controlFan(dhtSensor.getTemperature());
	// update the screen with the new information
	display.update(dhtSensor.getTemperature(), dhtSensor.getHumidity(), fan.getSpeed());

	// We don't really need to implement a ticker or any non-blocking operation here
	// since we are only reading 1 sensor and displaying it without any other operation.
	// Also the DHT library is blocking and uses `delay()`, so no matter what
	// this sensor operation is blocking and we could even get rid of the `delay()` call
	// here, but it's good to leave a 2s pause anyway.
	delay(WAIT_INTERVAL);
}
