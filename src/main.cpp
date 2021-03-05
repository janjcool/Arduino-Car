/* links:
 *      - multi threading: https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
 *                         and https://savjee.be/2020/01/multitasking-esp32-arduino-freertos/
 *      - hobby gear motor: https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
 * docs:
 *      - multi threading: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
 */

#include <Arduino.h>  // remove this line if you are using arduino

void CoreZeroLoop( void * pvParameters );

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

        Serial.print("Led initialized at pwm channel ");
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
        Serial.print("Update duty cycle: duty cycle \"");
        Serial.print(duty_cycle);
        Serial.print("\" at pwm channel \"");
        Serial.print(pwm_channel);
        Serial.println("\"");

        ledcWrite(pwm_channel, duty_cycle);
    }
};

// Init motor object
Motor left_motor;
Motor right_motor;

// Init LED object
LED red_led1;
LED red_led2;

// Init multi threading task handler object
TaskHandle_t TaskHandler;

// Init variables
const int IR_sensor_pin = 17;
int previous_state_of_sensor;
long sensorStateTime;

void setup() {
    // Begin serial connect
    Serial.begin(115200);
    Serial.println("Begin of setup");

    // Init pins
    pinMode(IR_sensor_pin, INPUT);
    previous_state_of_sensor = digitalRead(IR_sensor_pin);
    sensorStateTime = millis();

    // Init LED's
    red_led1.Init(4, 2);
    red_led1.On();
    red_led2.Init(16, 3);
    red_led2.On();


    // Init motors
    left_motor.Init(27, 26, 14, 30000, 0, 8, 200);
    right_motor.Init(25, 33, 32, 30000, 1, 8, 200);

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
            CoreZeroLoop,   /* Task function. */
            "motor control loop",     /* name of task. */
            10000,       /* Stack size of task */
            NULL,        /* parameter of the task */
            10,           /* priority of the task */
            &TaskHandler,      /* Task handle to keep track of created task */
            0);          /* pin task to core 0 */
    delay(500);

    // Debug info
    Serial.print("Setup() running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("End of setup");
}

void CoreZeroLoop( void * pvParameters ){
    // Setup of second core
    Serial.print("Start of second core");

    for(;;){
        Serial.print("Message form core zero");
    }
}

void loop() {
    if (digitalRead(IR_sensor_pin) != previous_state_of_sensor) {
        previous_state_of_sensor = digitalRead(IR_sensor_pin);
        Serial.println("Sensor changed state, previouse sensor state was " + String(millis()-sensorStateTime) + " milliseconds long");
        sensorStateTime = millis();
    }

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