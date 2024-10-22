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

    //bsp_Icm42688GetAres(AFS_8G);						//计算加速度计分辨率
    icm42688_writeReg(ICM42688_REG_BANK_SEL, 0x00);
    reg_val = icm42688_readReg(ICM42688_ACCEL_CONFIG0);
    reg_val |= (AFS_2G << 5);   						//量程 ±2g
    reg_val |= (AODR_1000Hz);     						//输出速率 1000HZ
    icm42688_writeReg(ICM42688_ACCEL_CONFIG0, reg_val);

    //bsp_Icm42688GetGres(GFS_1000DPS);					//计算陀螺仪分辨率
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
    icm42688_data acc;
    uint8_t data[6];
    //icm42688_readMem(ICM42688_ACCEL_DATA_X1, data, 6);

    data[0] = icm42688_readReg(ICM42688_ACCEL_DATA_X0);
    data[1] = icm42688_readReg(ICM42688_ACCEL_DATA_X1);   
    data[2] = icm42688_readReg(ICM42688_ACCEL_DATA_Y0);
    data[3] = icm42688_readReg(ICM42688_ACCEL_DATA_Y1);
    data[4] = icm42688_readReg(ICM42688_ACCEL_DATA_Z0);
    data[5] = icm42688_readReg(ICM42688_ACCEL_DATA_Z1);

    memcpy(&acc, data, 6);

    return acc;
}

icm42688_data icm42688_getGYRO()
{
    icm42688_data gyro;
    uint8_t data[2];
    data[0] = icm42688_readReg(ICM42688_GYRO_DATA_X0);
    data[1] = icm42688_readReg(ICM42688_GYRO_DATA_X1);
    gyro.x = (data[1] << 8) | data[0];
    data[0] = icm42688_readReg(ICM42688_GYRO_DATA_Y0);
    data[1] = icm42688_readReg(ICM42688_GYRO_DATA_Y1);
    gyro.y = (data[1] << 8) | data[0];
    data[0] = icm42688_readReg(ICM42688_GYRO_DATA_Z0);
    data[1] = icm42688_readReg(ICM42688_GYRO_DATA_Z1);
    gyro.z = (data[1] << 8) | data[0];
    return gyro;
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
    data.x = ((uint16_t)Raw_Data.x_hi << 8) | Raw_Data.x_lo;
    data.y = ((uint16_t)Raw_Data.y_hi << 8) | Raw_Data.y_lo;
    data.z = ((uint16_t)Raw_Data.z_hi << 8) | Raw_Data.z_lo;
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