#include <Arduino.h>  // remove this line if you are using arduino

#include "SSD1306Wire.h"  // library for SSD1306
#include "SoundData.h"  // The music file
#include "XT_DAC_Audio.h"  // The music playing library

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
        //Serial.print("Moving motor Backwards at pwm channel ");
        //Serial.println(pwm_channel);

        digitalWrite(pin_1, HIGH);
        digitalWrite(pin_2, LOW);
    }

    void Forward() const {
        //Serial.print("Moving motor Forward at pwm channel ");
        //Serial.println(pwm_channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, HIGH);
    }

    void UpdateDutyCycle() {
        Serial.println("Update duty cycle to " + String(duty_cycle) + " at pwm channel " + String(pwm_channel));
        ledcWrite(pwm_channel, duty_cycle);
    }

    uint32_t GetDutyCycle() const {
        return ledcRead(pwm_channel);
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
        // Save parameters in public variables
        pwm_channel = LedPWMChannel;
        pin = LedPin;

        // Sets the pin as output
        pinMode(pin, OUTPUT);

        // Configure PWM
        ledcSetup(pwm_channel, pwm_frequency, PWM_RESOLUTION);

        // Attach the channel to the GPIO to be controlled
        ledcAttachPin(pin, pwm_channel);
        UpdateDutyCycle();

        Serial.println("Led initialized at pwm channel ");
        Serial.println(pwm_channel);
    }

    void Off() {
        duty_cycle = 0;
        UpdateDutyCycle();
    }

    void On() {
        duty_cycle = 255;
        UpdateDutyCycle();
    }

    void FadeOn(int step_time) {
        Serial.println("Start of led fading on");
        duty_cycle = 0;

        for( ; duty_cycle <= 255; duty_cycle++){
            UpdateDutyCycle();
            delay(step_time);
        }
    }

    void FadeOff(int step_time) {
        Serial.println("Start of led fading off");
        duty_cycle = 255;

        for( ; duty_cycle >= 0; duty_cycle--){
            UpdateDutyCycle();
            delay(step_time);
        }
    }

    void UpdateDutyCycle() const {
        Serial.print("Update duty cycle to " + String(duty_cycle) + " at pwm channel " + String(pwm_channel));
        ledcWrite(pwm_channel, duty_cycle);
    }
};

// INIT AND SET VARIABLES
// Init and set black and white sensor objects
int IR_sensor_pin = 15;  // Pin of the black and white sensor
int circumference = 21221; // circumference of the wheel in micrometer
long read_colors; // for testing reasons so i can perfect the circumference

// Init display
SSD1306Wire display(0x3c, 5, 17);

// Init sound library
XT_Wav_Class Sound(rawData);
XT_DAC_Audio_Class DacAudio(25,0);

// INIT VARIABLES
// Init motor objects
Motor left_motor;
Motor right_motor;

// Init LED objects
LED red_led1;
LED red_led2;

// Init black and white sensor objects
int previous_output_of_IR_sensor;  // output of sensor from previous loop
float sensor_output_time;  // moment in time when the sensor changes output
float car_speed;  // micrometer/milliseconds traveling calculated by radius of wheel and RPS
float total_distance_traversed;

void read_car_speed () {
    if (digitalRead(IR_sensor_pin) != previous_output_of_IR_sensor) {
        read_colors++;
        previous_output_of_IR_sensor = digitalRead(IR_sensor_pin);
        try {
            car_speed = circumference/((millis()-sensor_output_time)*2);  // calculating the speed in micrometer/milliseconds
            total_distance_traversed = read_colors/2*circumference;  // calculate the traveled distance in micrometer
            Serial.println("Speed updated current speed is " + String(car_speed) + " (in micrometer/milliseconds )");
            Serial.println("Total distance traversed is " + String(total_distance_traversed));
        } catch (...) {
            Serial.println("Failed updating speed");
        }
        sensor_output_time = millis();
    }
}

void setup() {
    // Begin serial connect
    Serial.begin(115200);
    Serial.println("Begin of setup");

    // Init LED's
    red_led1.Init(4, 2);
    red_led1.On();
    red_led2.Init(16, 3);
    red_led2.On();

    // Init display
    display.init();
    display.clear();
    display.flipScreenVertically();
    display.display();

    // Init sensor and sensor objects
    pinMode(IR_sensor_pin, INPUT);
    previous_output_of_IR_sensor = digitalRead(IR_sensor_pin);
    sensor_output_time = millis();

    // Init motors
    left_motor.Init(27, 26, 14, 30000, 0, 8, 200);
    right_motor.Init(35, 33, 32, 30000, 1, 8, 200);

    // Debug info
    Serial.print("Setup() running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("End of setup");

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Setup finished");
    display.display();

    left_motor.duty_cycle = 214;
    right_motor.duty_cycle = 214;

    left_motor.UpdateDutyCycle();
    right_motor.UpdateDutyCycle();

    left_motor.Forward();
    right_motor.Forward();
}

void loop() {
    read_car_speed();

    if (millis() <= 7000) {
        left_motor.duty_cycle = map(millis(), 0, 7000, 130, 255);
        right_motor.duty_cycle = map(millis(), 0, 7000, 130, 255);

        left_motor.UpdateDutyCycle();
        right_motor.UpdateDutyCycle();
    } else {
        left_motor.Stop();
        right_motor.Stop();
    }

    //DacAudio.FillBuffer();
    //if (Sound.Playing == false) {
    //    DacAudio.Play(&Sound);
    //}

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    Serial.println("------------------------------------");
    Serial.println("Time: " + String(millis()));
    Serial.println("Speed: " + String(car_speed));
    Serial.println("Left duty cycle: " + String(left_motor.GetDutyCycle()));
    Serial.println("Right duty cycle: " + String(right_motor.GetDutyCycle()));
    Serial.println("Sound: " + String(Sound.Playing));
    Serial.println("Circumference: " + String(circumference));
    Serial.println("Sensor_output_time: " + String(sensor_output_time));
    Serial.println("IR sensor pin: " + String(digitalRead(IR_sensor_pin)));
    display.drawString(0, 15, "t: " + String(millis()) + " v: " + String(car_speed));
    display.drawString(0, 30, "s: " + String(total_distance_traversed));
    display.drawString(0, 45, "Duty cycle: L:" + String(left_motor.GetDutyCycle()) + " R: " + String(right_motor.GetDutyCycle()));
    display.display();

    delay(100);
}