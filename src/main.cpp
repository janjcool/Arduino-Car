/* links:
 *      - multi threading: https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
 *      - hobby gear motor: https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
 *      -
 */

#include <Arduino.h>  // remove this line if you are using arduino

class Motor {
public:
    int pwm_channel{};
    int duty_cycle = 200;
    int pin_1{};
    int pin_2{};

    void Init(int MotorPin1, int MotorPin2, int enable1Pin, int freq,
              int MotorPWMChannel, int resolution, int MotorDutyCycle) {
        Serial.println("Initializing motor");
        pwm_channel = MotorPWMChannel;
        duty_cycle = MotorDutyCycle;
        pin_1 = MotorPin1;
        pin_2 = MotorPin2;

        // sets the pins as outputs:
        pinMode(pin_1, OUTPUT);
        pinMode(pin_2, OUTPUT);
        pinMode(enable1Pin, OUTPUT);

        // configure PWM
        ledcSetup(pwm_channel, freq, resolution);

        // attach the channel to the GPIO to be controlled
        ledcAttachPin(enable1Pin, pwm_channel);

        ledcWrite(pwm_channel, duty_cycle);
        Serial.print("Motor initialized at pwm channel ");
        Serial.println(pwm_channel);
    }

    void Stop() const {
        Serial.print("Stop motor at pwm channel ");
        Serial.println(pwm_channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
    }

    void Backwards() const {
        Serial.print("Moving motor Backwards at pwm channel ");
        Serial.println(pwm_channel);

        digitalWrite(pin_1, HIGH);
        digitalWrite(pin_2, LOW);
    }

    void Forward() const {
        Serial.print("Moving motor Forward at pwm channel ");
        Serial.println(pwm_channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, HIGH);
    }

    void UpdateDutyCycle() const {
        Serial.print("Update duty cycle: duty cycle \"");
        Serial.print(duty_cycle);
        Serial.print("\" at pwm channel \"");
        Serial.print(pwm_channel);
        Serial.println("\"");

        ledcWrite(pwm_channel, duty_cycle);
    }
};

class LED {
public:
    int pin;
    int pwm_channel;
    int pwm_frequency = 500;
    int duty_cycle = 0;

    const int PWM_RESOLUTION = 8;

    void Init(int LedPin, int LedPWMChannel) {
        // save parameters in public variables
        pwm_channel = LedPWMChannel;
        pin = LedPin;

        // sets the pin as output
        pinMode(pin, OUTPUT);

        // configure PWM
        ledcSetup(pwm_channel, pwm_frequency, PWM_RESOLUTION);

        // attach the channel to the GPIO to be controlled
        ledcAttachPin(pin, pwm_channel);

        ledcWrite(pwm_channel, duty_cycle);

        Serial.print("Led initialized at pwm channel ");
        Serial.println(pwm_channel);
    }

    void off() {
        duty_cycle = 0;
        UpdateDutyCycle();
    }

    void on() {
        duty_cycle = 255;
        UpdateDutyCycle();
    }

    void fade_on(int step_time) {
        Serial.println("Start of led fading on");
        duty_cycle = 0;

        for( ; duty_cycle <= 255; duty_cycle++){
            UpdateDutyCycle();
            delay(step_time);
        }
    }

    void fade_off(int step_time) {
        Serial.println("Start of led fading off");
        duty_cycle = 255;

        for( ; duty_cycle >= 0; duty_cycle--){
            UpdateDutyCycle();
            delay(step_time);
        }
    }

    void UpdateDutyCycle() const {
        Serial.print("Update duty cycle: duty cycle \"");
        Serial.print(duty_cycle);
        Serial.print("\" at pwm channel \"");
        Serial.print(pwm_channel);
        Serial.println("\"");

        ledcWrite(pwm_channel, duty_cycle);
    }
};

Motor left_motor;
Motor right_motor;

LED red_led1;
LED red_led2;

const int IR_sensor = 7;

void setup() {
    // Begin serial connect
    Serial.begin(115200);
    Serial.println("Begin of setup");

    // Init pins
    pinMode(IR_sensor, INPUT);

    // Init LED's
    red_led1.Init(4, 2);
    red_led2.Init(16, 3);

    // Init motors
    left_motor.Init(27, 26, 14, 30000, 0, 8, 200);
    right_motor.Init(25, 33, 32, 30000, 1, 8, 200);

    // Debug info
    Serial.print("Setup() running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("End of setup");
}

void loop() {
    //Serial.println(digitalRead(IR_sensor));

    /*
    left_motor.Forward();
    right_motor.Forward();
    delay(2000);

    left_motor.Stop();
    right_motor.Stop();
    delay(1000);

    left_motor.duty_cycle = 150;
    left_motor.UpdateDutyCycle();
     */
}