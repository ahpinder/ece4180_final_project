
#include "mbed.h"
#include "Servo.h"
#include <chrono>
#include <stdint.h>
#include <cstdio>
#include "ICM20948.h"
#include "MadgwickAHRS.h"
#include "uLCD.hpp"



// ICM20948 settings
#define accel_div   1
#define accel_lp    0
#define accel_lp_en    1
#define accel_fs    1
#define gyro_div   1
#define gyro_lp    0
#define gyro_lp_en    1
#define gyro_fs    1
#define accel_conf (accel_lp_en | (accel_fs << 1) | (accel_lp << 3))
#define gyro_conf (gyro_lp_en | (gyro_fs << 1) | (gyro_lp << 3))

// helper function to get current time in ms
inline uint32_t milliseconds() {
    return (uint32_t)((std::chrono::time_point_cast<std::chrono::milliseconds>(Kernel::Clock::now())).time_since_epoch().count());
}

// period of main loop in ms (30ms -> 33Hz)
int period = 30;

// distance to closest obstacle (inches)
volatile int current_distance = 12;

// starting time of HC-SR04 pulse back
volatile uint32_t starttime = 0;
// ending time of HC-SR04 pulse back
volatile uint32_t endtime = 0;
// start time of HC-SR04 pulse sendout (for timeout if no signal comes back)
volatile uint32_t sonarstarttime = 0;

// P parameter (of PID loop) for servo 1
float s1_p = 2.f*((float)period/20.f)/90.f;
// D parameter (of PID loop) for servo 1
float s1_d = 0.01f/90.f;
// I parameter (of PID loop) for servo 1
float s1_i = 0.07f*((float)period/20.f)/90.f;
// integrator of servo 1 PID loop
float s1_integrator = 0.f;
// last pitch measurement
float lastpitch = 0;
// P parameter (of PID loop) for servo 2
float s2_p = 2.f*((float)period/20.f)/90.f;
// D parameter (of PID loop) for servo 2
float s2_d = 0.01f/90.f;
// I parameter (of PID loop) for servo 2
float s2_i = 0.02f*((float)period/20.f)/90.f;
// integrator of servo 2 PID loop
float s2_integrator = 0.f;
// last roll measurement
float lastroll = 0;

uLCD lcd(p28, p27, p13, uLCD::BAUD_115200);

// trigger output for HC-SR04
DigitalOut trig(p17);
// echo input from HC-SR04
InterruptIn echo(p16);

Thread sonarthread;

// trim values and trim increment per loop
float pitchtrim = 0, rolltrim = 0, triminc = 0.3;

// this gets called when the HC-SR04 output goes high
void uptransition() {
    starttime = us_ticker_read();
}

// this gets called when the HC-SR04 output goes low
void downtransition() {
    endtime = us_ticker_read();
    current_distance = (endtime - starttime) / 148;
}

// this thread manages the HC-SR04 module
void sonar_management() {
    // set up ISRs for HC-SR04 outptut pin (echo)
    echo.rise(&uptransition);
    echo.fall(&downtransition);
    while(1) {
        // set endtime to 0 so when it goes nonzero, we know it saw something
        endtime = 0;
        // set start time for later timeout check
        sonarstarttime = milliseconds();
        // send pulse
        trig = 1;
        wait_us(7);
        trig = 0;
        // wait for pulse back from HC-SR04 or timeout interval
        while(endtime == 0 && milliseconds() - sonarstarttime < 500) {
            ThisThread::sleep_for(50);
        }
    }
}

int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    // initialize the motion sensor
    ICM20948 icm(p9, p10);
    // initialize the angle filter
    Madgwick mg;

    // inputs from nav switch (active low)
    DigitalIn up(p5);
    DigitalIn down(p6);
    DigitalIn right(p7);
    DigitalIn left(p8);
    // check if motion sensor is working
    if (icm.whoAmI() == 68)
        led = 1;
    else
        led = 0;
    // initialize motion sensor
    icm.init(accel_conf, accel_div, gyro_conf, gyro_div);
    ThisThread::sleep_for(50ms);

    // set up servos for pitch and roll
    Servo s1(p21);
    Servo s2(p22);
    s1.calibrate(0.001, 90);
    s2.calibrate(0.001, 90);
    s1 = 1;
    s2 = 0.5;

    // start HC-SR04 thread
    sonarthread.start(sonar_management);
    uint32_t before = milliseconds();
    while (true) {
        // get current motion sensor measurements
        icm.getAccGyro();
        // update angle filter
        mg.updateIMU(icm.getGX(), icm.getGY(), icm.getGZ(), icm.getAX(), icm.getAY(), icm.getAZ());

        // errors for PID loops
        float pitcherror = mg.getPitch() - pitchtrim;
        float rollerror = mg.getRoll() - rolltrim;

        if (current_distance >= 4) {
            // obstacle is far away or not present, perform normally
            s1 = (s1 * 0.9) + ((-s1_p * pitcherror) + (-s1_d * (pitcherror - lastpitch)) + (-s1_i * s1_integrator) + 0.5) * 0.1;
            s2 = (s2 * 0.9) + ((-s2_p * rollerror) + (-s2_d * (rollerror - lastroll)) + (-s2_i * s2_integrator) + 0.5) * 0.1;
            s1_integrator += pitcherror;
            s2_integrator += rollerror;
        }
        else {
            // obstacle is too close, avoid by moving servo so the platform moves out of the way
            s2 = (s2 >= 1) ? 1 : s2 + 0.03;
        }
        led = !led;
        // wait for loop time
        while(milliseconds() < before + period);
        // update loop time
        before = milliseconds();
        lastpitch = pitcherror;
        lastroll = rollerror;

        // handle trim inputs
        if (!left) {
            pitchtrim += triminc;
        }
        else if (!right) {
            pitchtrim -= triminc;
        }
        if (!down) {
            rolltrim -= triminc;
        }
        else if (!up) {
            rolltrim += triminc;
        }

        // we don't want trim getting excessively large
        // so we don't let trim values exceed |x|=45
        if (rolltrim > 45)
            rolltrim = 45;
        if (rolltrim < -45)
            rolltrim = -45;
        if (pitchtrim > 45)
            pitchtrim = 45;
        if (pitchtrim < -45)
            pitchtrim = -45;
        
        // print out current status on uLCD
        lcd.locate(0, 0);
        lcd.printf("trim: %d,%d    \r\nposition:%d,%d    ",(int)rolltrim,(int)pitchtrim,(int)lastroll,(int)lastpitch);
    }
}
