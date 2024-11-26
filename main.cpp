/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "Servo.h"
#include <chrono>
#include <stdint.h>
#include <cstdio>
#include "ICM20948.h"
#include "MadgwickAHRS.h"
#include "uLCD.hpp"


// Blinking rate in milliseconds
#define BLINKING_RATE     50ms
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

inline uint32_t milliseconds() {
    return (uint32_t)((std::chrono::time_point_cast<std::chrono::milliseconds>(Kernel::Clock::now())).time_since_epoch().count());
}

float s1_p = 2.f/90.f;
float s1_d = 0.01f/90.f;
float s1_i = 0.07f/90.f;
float s1_integrator = 0.f;
float lastpitch = 0;
float s2_p = 2.f/90.f;
float s2_d = 0.01f/90.f;
float s2_i = 0.02f/90.f;
float s2_integrator = 0.f;
float lastroll = 0;

uLCD lcd(p28, p27, p13, uLCD::BAUD_115200);

int pitchtrim = 0, rolltrim = 0;


int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    ICM20948 icm(p9, p10);
    Madgwick mg;
    DigitalIn up(p5);
    DigitalIn down(p6);
    DigitalIn right(p7);
    DigitalIn left(p8);
    // char accel_div = ACC_RATE_16g;
    // char accel_conf = ACC_LPF_136HZ;
    // char gyro_div = GYRO_RATE_2000;
    // char gyro_conf = GYRO_LPF_230Hz;
    if (icm.whoAmI() == 68)
        led = 1;
    else
        led = 0;
    icm.init(accel_conf, accel_div, gyro_conf, gyro_div);
    ThisThread::sleep_for(50ms);
    Servo s1(p21);
    Servo s2(p22);
    s1.calibrate(0.001, 90);
    s2.calibrate(0.001, 90);
    s1 = 1;
    s2 = 0.5;
    uint32_t before = milliseconds();
    while (true) {
        icm.getAccGyro();
        mg.updateIMU(icm.getGX(), icm.getGY(), icm.getGZ(), icm.getAX(), icm.getAY(), icm.getAZ());
        //printf("%x\r\n",icm.whoAmI());
        //printf("pitch: %d. roll: %d\r\n",(int)mg.getPitch(),(int)mg.getRoll());
        s1 = (s1 * 0.9) + ((-s1_p * mg.getPitch()) + (-s1_d * (mg.getPitch() - lastpitch)) + (-s1_i * s1_integrator) + 0.5) * 0.1;
        s2 = (s2 * 0.9) + ((-s2_p * mg.getRoll()) + (-s2_d * (mg.getRoll() - lastroll)) + (-s2_i * s2_integrator) + 0.5) * 0.1;
        printf("p: %d.ival: %d\r\n",(int)(mg.getPitch()),(int)(s1_integrator));
        //printf("ax: %d. ay: %d. az: %d. gx: %d. gy: %d. gz: %d.\r\n", (int)(icm.getAX() * 100), (int)(icm.getAY() * 100), (int)(icm.getAZ() * 100), (int)(icm.getGX()), (int)(icm.getGY()), (int)(icm.getGZ()));
        led = !led;
        //s1 = 1 - s1;
        while(milliseconds() < before + 20);
        before += 20;
        lastpitch = mg.getPitch();
        s1_integrator += lastpitch;
        lastroll = mg.getRoll();
        s2_integrator += lastroll;
        if (!up) {
            pitchtrim += 1;
        }
        else if (!down) {
            pitchtrim -= 1;
        }
        if (!right) {
            rolltrim += 1;
        }
        else if (!left) {
            rolltrim -= 1;
        }
        
        //s1_integrator = (s1_integrator * s1_i > 1) ? 1/s1_i : s1_integrator;
        //s1_integrator = (s1_integrator * s1_i < -1) ? -1/s1_i : s1_integrator;
    }
}
