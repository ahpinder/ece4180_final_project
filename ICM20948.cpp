#include "mbed.h"
#include "ICM20948.h"
 
ICM20948::ICM20948(PinName sda, PinName sck) : _i2c(sda, sck)
{
    _i2c.frequency(400000);
    gyroBias[0] = 0;
    gyroBias[1] = 0;
    gyroBias[2] = 0;
}
 
void ICM20948::ICM_WriteByte(char ICM20948_reg, char ICM20948_data)
{
    char data_out[2];
    data_out[0]=ICM20948_reg;
    data_out[1]=ICM20948_data;
    _i2c.write(ICM20948_slave_addr, data_out, 2, 0);
}
 
char ICM20948::ICM_ReadByte(char ICM20948_reg)
{
    char data_out[1], data_in[1];
    data_out[0] = ICM20948_reg;
    _i2c.write(ICM20948_slave_addr, data_out, 1, 1);
    _i2c.read(ICM20948_slave_addr, data_in, 1, 0);
    return (data_in[0]);
}
 
// Communication test: WHO_AM_I register reading
int ICM20948::whoAmI()
{
    char whoAmI = ICM_ReadByte(ICM20948_WHO_AM_I);   // Should return 0x68

    if(whoAmI==0xea) {
        return 68;
    } else {
        return -1;
    }
 
}
 
void ICM20948::powerOn()
{
    // USER_BANK_0
    ICM_WriteByte(ICM20948_REG_BANK_SEL, USER_BANK_0);
    ICM_WriteByte(ICM20948_PWR_MGMT_1, 0x01);
    //ICM_WriteOneByte(ICM20948_PWR_MGMT_2, (0x38 | 0x07));
    ICM_WriteByte(ICM20948_PWR_MGMT_2, (0x38 | 0x07));
    ThisThread::sleep_for(10ms);
    //ICM_WriteOneByte(ICM20948_PWR_MGMT_2, (0x00 | 0x00));
    ICM_WriteByte(ICM20948_PWR_MGMT_2, (0x00 | 0x00));
}
 
void ICM20948::init(char accel_conf, char accel_div, char gyro_conf, char gyro_div)
{
    powerOn();

    switch((accel_conf >> 1) & 0x03)
    {
        case 0:
            aRes = 2.0/32768.0;
            break;
        case 1:
            aRes = 4.0/32768.0;
            break;
        case 2:
            aRes = 8.0/32768.0;
            break;
        case 3:
            aRes = 16.0/32768.0;
            break;
    }
    switch((gyro_conf >> 1) & 0x03)
    {
        case 0:
            gRes = 250.0/32768.0;
            break;
        case 1:
            gRes = 500.0/32768.0;
            break;
        case 2:
            gRes = 1000.0/32768.0;
            break;
        case 3:
            gRes = 2000.0/32768.0;
            break;
    }
    // USER_BANK_0 to access data
    ICM_WriteByte(ICM20948_REG_BANK_SEL, USER_BANK_2);
    ICM_WriteByte(ICM20948_GYRO_CONFIG_1, gyro_conf);
    ICM_WriteByte(ICM20948_GYRO_SMPLRT_DIV, gyro_div);
    ICM_WriteByte(ICM20948_ACCEL_CONFIG, accel_conf);
    ICM_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_2, accel_div);
 
    // USER_BANK_0 to access data
    ICM_WriteByte(ICM20948_REG_BANK_SEL, USER_BANK_0);
    ThisThread::sleep_for(10ms);

    gyroCalib();
}
 
void ICM20948::gyroCalib(){
    int16_t gxtmp, gytmp, gztmp;
    float gx, gy, gz;
    gx = 0.0; gy = 0.0; gz = 0.0;
    uint8_t LoByte, HiByte;
    for(int i = 0; i < 20; i++){
        LoByte = ICM_ReadByte(ICM20948_GYRO_XOUT_L); // read Gyrometer X_Low  value
        HiByte = ICM_ReadByte(ICM20948_GYRO_XOUT_H); // read Gyrometer X_High value
        gxtmp = ((HiByte<<8) | LoByte);
        gx += (float)(gxtmp) * gRes;
        
        LoByte = ICM_ReadByte(ICM20948_GYRO_YOUT_L);
        HiByte = ICM_ReadByte(ICM20948_GYRO_YOUT_H);
        gytmp = ((HiByte<<8) | LoByte);
        gy += (float)(gytmp) * gRes;
        
        LoByte = ICM_ReadByte(ICM20948_GYRO_ZOUT_L);
        HiByte = ICM_ReadByte(ICM20948_GYRO_ZOUT_H);
        gztmp = ((HiByte<<8) | LoByte);
        gz += (float)(gztmp) * gRes;
        //pc.printf("%d %f, %d %f, %d %f\n\r", gxtmp, gx, gytmp, gy, gztmp, gz);
        ThisThread::sleep_for(10ms);
    }
    
    gyroBias[0] = gx / 20.0;
    gyroBias[1] = gy / 20.0;
    gyroBias[2] = gz / 20.0;

}
 
void ICM20948::getAccGyro(){
    int16_t _ax, _ay, _az, _gx, _gy, _gz;
    uint8_t LoByte, HiByte;
    
    LoByte = ICM_ReadByte(ICM20948_ACCEL_XOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM_ReadByte(ICM20948_ACCEL_XOUT_H); // read Accelerometer X_High value
    _ax = (HiByte<<8) | LoByte;
    ax = ((float)(_ax)) * aRes;
    
    LoByte = ICM_ReadByte(ICM20948_ACCEL_YOUT_L);
    HiByte = ICM_ReadByte(ICM20948_ACCEL_YOUT_H);
    _ay = (HiByte<<8) | LoByte;
    ay = ((float)(_ay)) * aRes;
    
    LoByte = ICM_ReadByte(ICM20948_ACCEL_ZOUT_L);
    HiByte = ICM_ReadByte(ICM20948_ACCEL_ZOUT_H);
    _az = (HiByte<<8) | LoByte;
    az = ((float)(_az)) * aRes;
    
    LoByte = ICM_ReadByte(ICM20948_GYRO_XOUT_L); // read Gyrometer X_Low  value
    HiByte = ICM_ReadByte(ICM20948_GYRO_XOUT_H); // read Gyrometer X_High value
    _gx = (HiByte<<8) | LoByte;
    gx = ((float)(_gx)) * gRes - gyroBias[0];
    
    LoByte = ICM_ReadByte(ICM20948_GYRO_YOUT_L);
    HiByte = ICM_ReadByte(ICM20948_GYRO_YOUT_H);
    _gy = (HiByte<<8) | LoByte;
    gy = ((float)(_gy)) * gRes - gyroBias[1];
    
    LoByte = ICM_ReadByte(ICM20948_GYRO_ZOUT_L);
    HiByte = ICM_ReadByte(ICM20948_GYRO_ZOUT_H);
    _gz = (HiByte<<8) | LoByte;
    gz = ((float)(_gz)) * gRes - gyroBias[2];
    
}

float ICM20948::getAX()
{
    return ax;
}

float ICM20948::getAY()
{
    return ay;
}

float ICM20948::getAZ()
{
    return az;
}

float ICM20948::getGX()
{
    return gx;
}

float ICM20948::getGY()
{
    return gy;
}

float ICM20948::getGZ()
{
    return gz;
}

int ICM20948::getIMUTemp()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM_ReadByte(ICM20948_TEMP_OUT_L); // read Accelerometer X_Low  value
    HiByte = ICM_ReadByte(ICM20948_TEMP_OUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}
            