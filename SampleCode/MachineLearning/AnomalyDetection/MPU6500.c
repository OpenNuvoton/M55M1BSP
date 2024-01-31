/**************************************************************************//**
 * @file     MPU6500.c
 * @version  V3.00
 * @brief
 *           Show how to use I2C Signle byte API Read and Write data to Slave
 *           Needs to work with I2C_Slave sample code.
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "MPU6500.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile    uint8_t     g_u8DeviceAddr;
unsigned    char        BUF[10];
short T_X, T_Y, T_Z, T_T;

uint8_t buff[512];
#define IMU_I2C  LPI2C0
/*-----------------------------------------------*/
// Init MPU6500
//
/*-----------------------------------------------*/
void Init_MPU6500(void)
{
    LPI2C_WriteByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, PWR_MGMT_1,   0x00);//IMU use internal oscillator
    LPI2C_WriteByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, GYRO_CONFIG,  0x18); //Gyro FS +-2000dps
    LPI2C_WriteByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_CONFIG, 0x00);//Accel FS +-2g
    LPI2C_WriteByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, USER_CTRL,    0x04);//Enable  FIFO_RST
    LPI2C_WriteByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, FIFO_EN,      0x08);//Keep Accel in FIFO only
    LPI2C_WriteByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, SMPLRT_DIV,   0x64);//FIFO Sample Rate 125Hz

}

void MPU6500_low_levle_init(void)
{
    SET_LPI2C0_SCL_PC12();

    SET_LPI2C0_SDA_PC11();

    /* Open LPI2C0 module and set bus clock */
    LPI2C_Open(IMU_I2C, 100000);

    //LPI2C_EnableInt(LPI2C0);
    //NVIC_EnableIRQ(LPI2C0_IRQn);
}

int8_t MPU6500_test(void)
{
    /* Slave Address */
    g_u8DeviceAddr = MPU6500_DEVICE_ID;

    printf("Who am I = %02x\n", LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, WHO_AM_I));

    if (LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, WHO_AM_I) == 0x70)
    {
        printf("MPU6500 found.\n");
        /* Init MPU6500 sensor */
        Init_MPU6500();
    }
    else
    {
        printf("MPU6500 not found!!!\n");
        return -1;
    }

    /* X-axis */
    BUF[0] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_XOUT_L);
    BUF[1] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_XOUT_H);
    T_X = (BUF[1] << 8) | BUF[0];


    /* Y-axis */
    BUF[2] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_YOUT_L);
    BUF[3] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_YOUT_H);
    T_Y = (BUF[3] << 8) | BUF[2];


    /* Z-axis */
    BUF[4] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_ZOUT_L);
    BUF[5] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_ZOUT_H);
    T_Z = (BUF[5] << 8) | BUF[4];


    if (((T_Z + T_Y + T_X) < 0) || ((T_Z + T_Y + T_X) > 40000))
    {
        printf("GYRO Sensor Error\n");
        return -1;
    }

    printf("<done>\n");

    return 0;
}


int8_t MPU6500_readXYZ_mg(float *pftr)
{
    float acc_sensitivity = 16384 / 1000;

    /* X-axis */
    BUF[0] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_XOUT_L);
    BUF[1] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_XOUT_H);
    T_X = (BUF[1] << 8) | BUF[0];


    /* Y-axis */
    BUF[2] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_YOUT_L);
    BUF[3] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_YOUT_H);
    T_Y = (BUF[3] << 8) | BUF[2];


    /* Z-axis */
    BUF[4] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_ZOUT_L);
    BUF[5] = LPI2C_ReadByteOneReg(IMU_I2C, MPU6500_DEVICE_ID, ACCEL_ZOUT_H);
    T_Z = (BUF[5] << 8) | BUF[4];


    *pftr++ = (float)(T_X) / (acc_sensitivity);
    *pftr++ = (float)(T_Y) / (acc_sensitivity);
    *pftr++ = (float)(T_Z) / (acc_sensitivity);

    return 0;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
