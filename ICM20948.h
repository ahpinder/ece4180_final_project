#ifndef ICM20948_H
#define ICM20948_H

#include "mbed.h"
 
// USER BANK 0
#define ICM20948_WHO_AM_I         0x00 // Should return 0xEA
#define ICM20948_PWR_MGMT_1       0x06  // Device defaults to the SLEEP mode
#define ICM20948_PWR_MGMT_2       0x07
#define ICM20948_ACCEL_XOUT_H     0x2D
#define ICM20948_ACCEL_XOUT_L     0x2E
#define ICM20948_ACCEL_YOUT_H     0x2F
#define ICM20948_ACCEL_YOUT_L     0x30
#define ICM20948_ACCEL_ZOUT_H     0x31
#define ICM20948_ACCEL_ZOUT_L     0x32
#define ICM20948_GYRO_XOUT_H      0x33
#define ICM20948_GYRO_XOUT_L      0x34
#define ICM20948_GYRO_YOUT_H      0x35
#define ICM20948_GYRO_YOUT_L      0x36
#define ICM20948_GYRO_ZOUT_H      0x37
#define ICM20948_GYRO_ZOUT_L      0x38
#define ICM20948_TEMP_OUT_H       0x39
#define ICM20948_TEMP_OUT_L       0x3A
 
// USER BANK 2
#define ICM20948_GYRO_SMPLRT_DIV  0x00
#define ICM20948_GYRO_CONFIG_1    0x01
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG     0x14
 
// COMMON
#define ICM20948_REG_BANK_SEL     0x7F
 
// OTHER
#define USER_BANK_0               0b00000000
#define USER_BANK_1               0b00010000
#define USER_BANK_2               0b00100000
#define USER_BANK_3               0b00110000
 

#define GYRO_RATE_250             0x00
#define GYRO_RATE_500             0x02
#define GYRO_RATE_1000            0x04
#define GYRO_RATE_2000            0x06
#define GYRO_LPF_OFF              0x00
#define GYRO_LPF_230Hz            0x01
#define GYRO_LPF_17Hz             0x29
#define GYRO_SMPLRT_100Hz         0x0A
 
// 加速度のレンジは2:1bitの値に相当
// 加速度のモジュール内ローパスフィルタは5:3,0bitに相当
#define ACC_RATE_2g               0x00
#define ACC_RATE_4g               0x02
#define ACC_RATE_8g               0x04
#define ACC_RATE_16g              0x06
#define ACC_LPF_OFF               0x00
#define ACC_LPF_136HZ             0x11
#define ACC_SMPLRT_100Hz          0x0A
 
 
 
 
// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define ICM20948_slave_addr 0x69<<1  // Device address when ADO = 1
#else
#define ICM20948_slave_addr 0x68<<1  // Device address when ADO = 0
#endif
 
#define IMU_ONE_G 9.80665
 
class ICM20948 {
    public:
        ICM20948(PinName sda, PinName sck);
        ICM20948(I2C &i2c);  
        void ICM_WriteByte(char ICM20948_reg, char ICM20948_data);
        char ICM_ReadByte(char ICM20948_reg);
        int  whoAmI();
        void powerOn();
        void init(char accel_conf, char accel_div, char gyro_conf, char gyro_div);
        void gyroCalib();
        void getAccGyro();
        int  getIMUTemp();
        float getAX();
        float getAY();
        float getAZ();
        float getGX();
        float getGY();
        float getGZ();
    private:
        I2C _i2c;
        float aRes, gRes;
        float gyroBias[3], acc[3], gyro[3];
        float ax, ay, az, gx, gy, gz;
};
 
#endif
            