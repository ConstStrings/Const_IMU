#include "icm42688.h"

/* 
* 功能  icm42688写寄存器 
* 参数  addr	寄存器地址
*		dat		需要写入的数据
* 返回  无
*/
static void icm42688_writeReg(uint8_t addr, uint8_t data)
{
    uint8_t txData[2];
    txData[0] = addr; 
    txData[1] = data;

    CS_ENABLE;
    
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);

    CS_DISABLE;
}

/* 
* 功能  icm42688读寄存器 
* 参数  addr	寄存器地址
*		dat		读取得到的数据
* 返回  无
*/
uint8_t icm42688_readReg(uint8_t addr)
{
    uint8_t dat = 0;
    uint8_t txData[1];
    txData[0] = addr|0x80;

    CS_ENABLE;

    HAL_SPI_Transmit(&hspi1, txData, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&hspi1, &dat, 1, HAL_MAX_DELAY);

    CS_DISABLE;

    return dat;
}

void icm42688_readMem(uint8_t addr, uint8_t * dat, uint16_t len)
{
    uint8_t txData[1];
    txData[0] = addr|0x80;

    CS_ENABLE;

    HAL_SPI_Transmit(&hspi1, txData, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&hspi1, dat, len, HAL_MAX_DELAY);

    CS_DISABLE;
}



uint8_t icm42688_test()
{
    uint8_t id = icm42688_readReg(ICM42688_WHO_AM_I);
    return id;
}

void icm42688_Init()
{
    icm42688_writeReg(ICM42688_REG_BANK_SEL,  0); 	 //设置bank 0区域寄存器
    icm42688_writeReg(ICM42688_DEVICE_CONFIG, 0x01); //软复位传感器（此后需要至少等待1ms）
    HAL_Delay(3);

    uint8_t reg_val = 0;

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 1); 	//设置bank 1区域寄存器
    icm42688_writeReg(ICM42688_INTF_CONFIG4, 0x02); //设置为4线SPI通信

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0); 	//设置bank 0区域寄存器
    icm42688_writeReg(ICM42688_FIFO_CONFIG, 0x40);  //Stream-to-FIFO Mode(page63)


    reg_val = icm42688_readReg(ICM42688_INT_SOURCE0);
    icm42688_writeReg(ICM42688_INT_SOURCE0, 0x00);
    icm42688_writeReg(ICM42688_FIFO_CONFIG2, 0x00); // watermark
    icm42688_writeReg(ICM42688_FIFO_CONFIG3, 0x02); // watermark
    icm42688_writeReg(ICM42688_INT_SOURCE0, reg_val);
    icm42688_writeReg(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);
    icm42688_writeReg(ICM42688_INT_CONFIG, 0x36);

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = icm42688_readReg(ICM42688_INT_SOURCE0);
    reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
    icm42688_writeReg(ICM42688_INT_SOURCE0, reg_val);

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = icm42688_readReg(ICM42688_ACCEL_CONFIG0);
    reg_val |= (AFS_4G << 5);   						//量程 ±2g
    reg_val |= (AODR_1000Hz);     						//输出速率 1000HZ
    icm42688_writeReg(ICM42688_ACCEL_CONFIG0, reg_val);

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = icm42688_readReg(ICM42688_GYRO_CONFIG0);
    reg_val |= (GFS_1000DPS << 5);   					//量程 ±1000dps
    reg_val |= (GODR_1000Hz);     						//输出速率 1000HZ
    icm42688_writeReg(ICM42688_GYRO_CONFIG0, reg_val);

    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = icm42688_readReg(ICM42688_PWR_MGMT0);
    reg_val &= ~(1 << 5);								//使能温度测量
    reg_val |= ((3) << 2);								//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);										//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    icm42688_writeReg(ICM42688_PWR_MGMT0, reg_val);
    HAL_Delay(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作
}

icm42688_data icm42688_getAccel()
{
    icm42688_Raw_data acc_raw;
    icm42688_readMem(ICM42688_ACCEL_DATA_X1, (uint8_t*)&acc_raw, 6);
    return Merge_Raw_Data(acc_raw);
}

icm42688_data icm42688_getGYRO()
{
    icm42688_Raw_data gyro_raw;
    icm42688_readMem(ICM42688_GYRO_DATA_X1, (uint8_t*)&gyro_raw, 6);
    return Merge_Raw_Data(gyro_raw);
}

uint16_t icm42688_getFifoCnt()
{
    uint16_t cnt_hi = icm42688_readReg(ICM42688_FIFO_COUNTH);
    uint16_t cnt_lo = icm42688_readReg(ICM42688_FIFO_COUNTL);

    return (cnt_hi << 8) | cnt_lo;
}


float current_yaw = 0.0; 
uint32_t last_update_time = 0;

void Update_Yaw_Angle(void)
{
    icm42688_data gyro = icm42688_getGYRO();
    uint32_t now = HAL_GetTick(); 
    float dt = (now - last_update_time) / 1000.0;  
    last_update_time = now;

    float gyro_z = gyro.z / GYRO_SENSITIVITY;

    current_yaw += gyro_z * dt;
}

void Read_FIFO_Raw_Data(FIFO_Raw_Data * FIFO_List, uint16_t len)
{
    for (int i = 0; i < len; i++)
    {
        icm42688_readMem(ICM42688_FIFO_DATA, (uint8_t*)(FIFO_List + i), sizeof(FIFO_Raw_Data));
    }
}

icm42688_data Merge_Raw_Data(icm42688_Raw_data Raw_Data)
{
    icm42688_data data;
    data.x = (int16_t)(((uint16_t)Raw_Data.x_hi << 8) | Raw_Data.x_lo);
    data.y = (int16_t)(((uint16_t)Raw_Data.y_hi << 8) | Raw_Data.y_lo);
    data.z = (int16_t)(((uint16_t)Raw_Data.z_hi << 8) | Raw_Data.z_lo);
    return data;
}

FIFO_Data Process_FIFO_Raw_Data(FIFO_Raw_Data Raw_Data)
{
    FIFO_Data Data;
    Data.header = Raw_Data.header;
    Data.acc_data = Merge_Raw_Data(Raw_Data.acc_data);
    Data.gyro_data = Merge_Raw_Data(Raw_Data.gyro_data);
    Data.temperature = Raw_Data.temperature;
    Data.TimeStamp = ((uint16_t)Raw_Data.TimeStamp.time_hi << 8) | Raw_Data.TimeStamp.time_lo;
    Data.TimeStamp = Data.TimeStamp * 32 / 30;
    return Data;
}

void Print_FIFO_Data(FIFO_Data data)
{
    myprintf("Header: 0x%x\n",data.header);
    myprintf("Accelerate:\nX:%d\nY:%d\nZ:%d\n", data.acc_data.x, data.acc_data.y, data.acc_data.z);
    myprintf("gyroscope:\nX:%d\nY:%d\nZ:%d\n", data.gyro_data.x, data.gyro_data.y, data.gyro_data.z);
    myprintf("Temperature:%d\n",data.temperature);
    myprintf("TimeStamp:%d\n", data.TimeStamp);
}

Angle_Data Calculate_Angle_ByAcc(icm42688_data acc)  
{
    // 加速度计量程为 +-2g
    double acc_x = (double)acc.x / 16384 * 9.81;
    double acc_y = (double)acc.y / 16384 * 9.81;
    double acc_z = (double)acc.z / 16384 * 9.81;


    Angle_Data angle;
    angle.Pitch = atan2(acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * (180.0 / PI);
    angle.Roll = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * (180.0 / PI);
    angle.Yaw = 0;

    return angle;
}

icm42688_data_float icm42688_getAcc_float()
{
    icm42688_data acc = icm42688_getAccel();

    icm42688_data_float acc_flaot;

    acc_flaot.x = (double)acc.x / 16384 * 9.81;
    acc_flaot.y = (double)acc.y / 16384 * 9.81;
    acc_flaot.z = (double)acc.z / 16384 * 9.81;

    return acc_flaot;
}

icm42688_data_float icm42688_getGYRO_float()
{
    icm42688_data gyro = icm42688_getGYRO();

    icm42688_data_float current_gyro;
    current_gyro.x = (double)gyro.x / 32768 * 500;
    current_gyro.y = (double)gyro.y / 32768 * 500;
    current_gyro.z = (double)gyro.z / 32768 * 500;

    return current_gyro;
}

Angle_Data Calculate_Angle_ByGyro(icm42688_data gyro, double delta_time)  
{
    static icm42688_data_float current_gyro;
    current_gyro.x += (double)gyro.x / 32768 * 500 * delta_time;
    current_gyro.y += (double)gyro.y / 32768 * 500 * delta_time;
    current_gyro.z += (double)gyro.z / 32768 * 500 * delta_time;

    Angle_Data angle;
    angle.Pitch = current_gyro.x;
    angle.Roll = current_gyro.y;
    angle.Yaw = current_gyro.z;

    return angle;
}



