#include "bsp_data_process.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "afe4404.h"
#include "app_uart.h"
#include "nrf_delay.h"
#include "ble_nus.h"
#include "ble_hci.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "calender.h"
#include "mpu6500.h"
#include "ble_data_process.h"
#include "flash.h"
#include "esp8266.h"
#include <string.h>
#include "system_mode_process.h"
#include "nrf_drv_saadc.h"
#include "as7000.h"
#include "mlx90615.h"
#include "sht30.h"
#include "oled.h"
#include "ble_advertising.h"
#include "wb_alg.h"
#include "ble_data_process.h"
#include "SEGGER_RTT.h"

/********************用户自定义变量声明区域**********************/
#if (CALENDER_FUNCTION_EN==1)
extern ble_date_time_t calender;
extern uint32_t calender_count, calender_number;//用于nRF51822日历功能
#endif
#if (FLASH_FUNCTION_EN==1)
flash_date_type flash;
#endif
extern const nrf_drv_spi_t spi0;
extern system_data_type system_data;
extern system_status_type system_status;
extern system_operator_type system_operator;
extern algorithm_data_type algorithm_data;
extern ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
extern uint16_t                         m_conn_handle;
extern uint8_t as7000_init_count;  //indicate enter HANDSHAKE interrupt time
extern hrm_data_tpye hrm_data;
extern mpu6500_buff mpu6500_temp;//mpu6500加速度缓存数组
extern uint8_t device_name[BLE_NAME_LEN];
extern void advertising_init (void);
extern void advertising_para_init(void);


#if (ALGORITHM_FUNCTION_EN==1)
APP_TIMER_DEF (algorithm_timer);//算法定时器
#endif
#if (CALENDER_FUNCTION_EN==1)
APP_TIMER_DEF (calender_timer); //日历功能定时器
#endif
#if (FLASH_FUNCTION_EN==1)
APP_TIMER_DEF (flash_timer);    //flash数据处理定时器
#endif
#if (KEY_FUNCTION_EN==1)
APP_TIMER_DEF (key_timer);			 //按键扫描定时器
#endif
//#if (AS7000_FUNCTION_EN==1)
#if (MPU6500_FUNCTION_EN==1)
APP_TIMER_DEF (mpu6500_timer);			 //按键扫描定时器
#endif
//#endif
APP_TIMER_DEF (motor_timer);	 //马达振动定时器
#if (ADC_FUNCTION_EN==1)
APP_TIMER_DEF (charge_timer);	 //充电定时检测定时器
#endif
/*****************************************************************/

#if (ALGORITHM_FUNCTION_EN==1)
/*****************************************************************
*功  能：定时器进入中断，用于算法数据处理
*参  数: 无
*返回值: 无
*****************************************************************/
void algorithm_interrupt_process (void)
{
    if (system_status.algorithm)
    {
        system_status.algorithm = false;

        switch (system_data.system_mode)
        {
        case NORMAL_MODE:
            normal_mode_algorithm_interrupt_process();
            break;

        case SELF_TEST_MODE:
            self_test_mode_algorithm_interrupt_process();
            break;

        case BLOOD_PRESSURE_MODE:
            blood_pressure_mode_algorithm_interrupt_process();
            break;

        default:
            break;
        }
    }
}
#endif
#if (AFE4404_FUNCTION_EN==1)
/*****************************************************************
*功  能：afe4404中断处理函数，读取4404和6500的缓存数组
*参  数: 无
*返回值: 无
*****************************************************************/
void afe4404_interrupt_process (void)
{
    if (system_status.afe4404)
    {
        system_status.afe4404 = false;

        switch (system_data.system_mode)
        {
#if (ALG_ONCHIP_EN == 1)

        case NORMAL_MODE:
            normal_mode_afe4404_interrupt_process();
            break;
#endif

        case SELF_TEST_MODE:
            self_test_mode_afe4404_interrupt_process();
            break;

        case BLOOD_PRESSURE_MODE:
            blood_pressure_mode_afe4404_interrupt_process();
            break;

        case AFE4404_MODE:
            afe4404_mode_afe4404_interrupt_process();
            break;

        default:
            break;
        }
    }
}
#endif

#if (MPU6500_FUNCTION_EN==1)
/*****************************************************************
*功  能：常规模式下，mpu6500中断处理函数，读取6500最新数据
*参  数: 无
*返回值: 无
*****************************************************************/
void mpu6500_interrupt_process (void)
{
    if (system_status.mpu6500)
    {
        system_status.mpu6500 = false;
        short a_x = mpu6500_read (MPU_ACCEL_X_REG);
        short a_y = mpu6500_read (MPU_ACCEL_Y_REG);
        short a_z = mpu6500_read (MPU_ACCEL_Z_REG);
        short g_x = mpu6500_read (MPU_GYRO_X_REG);
        short g_y = mpu6500_read (MPU_GYRO_Y_REG);
        short g_z = mpu6500_read (MPU_GYRO_Z_REG);
        mpu6500_temp.ax[mpu6500_temp.accel_count] = a_x;
        mpu6500_temp.ay[mpu6500_temp.accel_count] = a_y;
        mpu6500_temp.az[mpu6500_temp.accel_count] = a_z;
        mpu6500_temp.gx[mpu6500_temp.accel_count] = g_x;
        mpu6500_temp.gy[mpu6500_temp.accel_count] = g_y;
        mpu6500_temp.gz[mpu6500_temp.accel_count] = g_z;
        mpu6500_temp.accel_count++;

        if (mpu6500_temp.accel_count == MPU6500_BUFF_SIZE)
        {
            mpu6500_temp.accel_count = 0;
        }
    }
}
#endif

#if (FLASH_FUNCTION_EN==1)
/*****************************************************************
*功  能：定时器进入中断，用于flash数据处理
*参  数: 无
*返回值: 无
*****************************************************************/
void flash_upload_process (void)
{

    if (system_status.flash)
    {
        SEGGER_RTT_printf(0,"flash_upload_process\n"); 
        system_status.flash = false;

        switch (system_operator.flash)
        {
        case FLASH_ERASE_STATUS:
            flash_timer_stop();

            //防止在flash数据上传的过程中擦除flash芯片造成4404和算法定时器不正常工作
            if (flash.backup_status)
            {
#if (WIFI_FUNCTION_EN==1)
                wifi_power_operator (CLOSE_WIFI_POWER);
#endif
#if (AS7000_FUNCTION_EN==1)
                as7000_measure_start();
#if (MPU6500_FUNCTION_EN==1)
                mpu6500_timer_start();
#endif
#endif
#if (ALG_ONCHIP_EN == 1)
#if (AFE4404_FUNCTION_EN==1)
                afe4404_interrupt_enable();
#endif
#endif
#if (ALGORITHM_FUNCTION_EN==1)
                algorithm_timer_start();
#endif
            }

            memset (&flash, 0, sizeof (flash));
            send_ack_to_app (FLASH_ERASE_ACK);
            break;

        case FLASH_STOP_STATUS://停止上传
            flash_timer_stop();

            //停止前看看是否已经备份过了，有则恢复，无则跳过
            if (flash.backup_status)
            {
                memcpy (flash.data_temp, flash.data_backup, FLASH_BUFF_SIZE);
                memset (flash.data_backup, 0, FLASH_BUFF_SIZE);
                flash.backup_status = false;
                flash.temp_count = flash.temp_count_backup;
                flash.temp_count_backup = 0;
                flash.time_out = 0;
#if (WIFI_FUNCTION_EN==1)
                wifi_power_operator (CLOSE_WIFI_POWER);
#endif
#if (AS7000_FUNCTION_EN==1)
                as7000_measure_start();
#if (MPU6500_FUNCTION_EN==1)
                mpu6500_timer_start();
#endif
#endif
#if (ALG_ONCHIP_EN == 1)
#if (AFE4404_FUNCTION_EN==1)
                afe4404_interrupt_enable();
#endif
#endif
#if (ALGORITHM_FUNCTION_EN==1)
                algorithm_timer_start();
#endif
            }

            send_ack_to_app (FLASH_STOP_ACK);
            break;

        case FLASH_UPLOADING_STATUS:
        {
            uint8_t tx_buff[BLE_MAX_LENGTH] = {0};
            //是否已经连接上TCP服务器并且进入透传模式
//            if (system_status.wifi_tcp_en)
            {
//                flash.time_out = 0;

                //蓝牙是否连接
                if (!system_status.ble_connect)//如果蓝牙没有连接，则停止flash上传
                {
                    system_operator.flash = FLASH_STOP_STATUS;
                    return;
                }
                
                //buffer中没数据或者有数据但是已经上传完成
                if (flash.temp_count == 0)
                {
                   //flash数据出现覆盖情况
                    if (flash.write_cover_status)
                    {
                        if (flash.read_pointer < 0)
                        {
                            flash.write_cover_status = false;
                            flash.read_cover_status = true;
                            flash.read_pointer = FLASH_MAX_POINTER - 1;
                        }
                    }

                    else
                    {
                        //读指针在最高物理地址回读的情况
                        if (flash.read_cover_status)
                        {
                            if (flash.read_pointer < flash.logic_pointer)
                            {
                                flash.read_pointer = 0;
                                flash.read_cover_status = false;
                                system_operator.flash = FLASH_COMPLETE_STATUS;
                                return;
                            }
                        }

                        else
                        {
                            if (flash.read_pointer < 0)
                            {
                                flash.read_pointer = 0;
                                system_operator.flash = FLASH_COMPLETE_STATUS;
                                return;
                            }
                        }
                    }
                    memset (flash.data_temp, 0, FLASH_BUFF_SIZE);
                    flash_page_read (FLASH_PAGE_ADDR * flash.read_pointer, flash.data_temp);
                    flash.temp_count = FLASH_BUFF_SIZE;
                    //读出之后对数据进行备份
                    if ((flash.write_cover_status) && (flash.write_pointer > flash.logic_pointer))
                    {
                        flash.logic_pointer = flash.write_pointer;
                    }
                    flash.read_pointer = flash.write_pointer - 1;
                    memcpy (flash.data_backup, flash.data_temp, FLASH_BUFF_SIZE);
                    flash.temp_count = flash.temp_count_backup;
                    flash.backup_status = true;
                    
                    flash.read_pointer--;//改变读指针位置
                }
                //上传一包数据
                tx_buff[0] = FIRST_HEADER;
                tx_buff[1] = SECONED_HEADER;
                tx_buff[2] = DEVICE_ID;
                tx_buff[3] = SET_FLASH_OPERATING;//0x05
                memcpy(&tx_buff[4],&flash.data_temp[flash.temp_count-FLASH_DATA_SIZE],FLASH_DATA_SIZE);
                ble_send(tx_buff,BLE_MAX_LENGTH);
                flash.temp_count -= FLASH_DATA_SIZE;            
            }
        }
/*
            else
            {
                //避免重复打开WIFI模块开关
                if (flash.time_out == 0)
                {
#if (WIFI_FUNCTION_EN==1)
                    wifi_power_operator (OPEN_WIFI_POWER);
#endif
                }

                flash.time_out++;

                //flash数据上传等待超时
                if (flash.time_out >= FLASH_TIME_OUT)
                {
                    system_operator.flash = FLASH_STOP_STATUS;
                    flash.time_out = 0;
                    return;
                }
            }
*/
            break;
        case FLASH_INTERMITTENT_UPLOADING_STATUS://间歇上传
        {
            static uint8_t packet_count = 0;    
            uint8_t tx_buff[BLE_MAX_LENGTH] = {0};
            //是否已经连接上TCP服务器并且进入透传模式
//            if (system_status.wifi_tcp_en)
            {
//                flash.time_out = 0;
                SEGGER_RTT_printf(0,"FLASH_INTERMITTENT_UPLOADING_STATUS\n");
                //蓝牙是否连接
                if (!system_status.ble_connect)//如果蓝牙没有连接，则停止flash上传
                {
                    system_operator.flash = FLASH_STOP_STATUS;
                    return;
                }
                
                //buffer中没数据或者有数据但是已经上传完成
                if (flash.temp_count == 0)
                {
                    if(packet_count >= 1)
                    {
                        system_operator.flash = FLASH_STOP_STATUS;
                        flash.backup_status = false;//数据已经传两页，清除备份标志
                        packet_count = 0;
                        return;
                    }
                   //flash数据出现覆盖情况
                    if (flash.write_cover_status)
                    {
                        if (flash.read_pointer < 0)
                        {
                            flash.write_cover_status = false;
                            flash.read_cover_status = true;
                            flash.read_pointer = FLASH_MAX_POINTER - 1;
                        }
                    }

                    else
                    {
                        //读指针在最高物理地址回读的情况
                        if (flash.read_cover_status)
                        {
                            if (flash.read_pointer < flash.logic_pointer)
                            {
                                flash.read_pointer = 0;
                                flash.read_cover_status = false;
                                system_operator.flash = FLASH_COMPLETE_STATUS;
                                return;
                            }
                        }

                        else
                        {
                            if (flash.read_pointer < 0)
                            {
                                flash.read_pointer = 0;
                                system_operator.flash = FLASH_COMPLETE_STATUS;
                                return;
                            }
                        }
                    }
                    memset (flash.data_temp, 0, FLASH_BUFF_SIZE);
                    flash_page_read (FLASH_PAGE_ADDR * flash.read_pointer, flash.data_temp);
                    flash.temp_count = FLASH_BUFF_SIZE;
                    //读出之后对数据进行备份
                    if ((flash.write_cover_status) && (flash.write_pointer > flash.logic_pointer))
                    {
                        flash.logic_pointer = flash.write_pointer;
                    }
                    flash.read_pointer = flash.write_pointer - 1;
                    memcpy (flash.data_backup, flash.data_temp, FLASH_BUFF_SIZE);
                    flash.temp_count = flash.temp_count_backup;
                    flash.backup_status = true;
                    
                    flash.read_pointer--;//改变读指针位置
                }
                //缓存中有数据，上传一包数据
                tx_buff[0] = FIRST_HEADER;
                tx_buff[1] = SECONED_HEADER;
                tx_buff[2] = DEVICE_ID;
                tx_buff[3] = SET_FLASH_OPERATING;//0x05
                memcpy(&tx_buff[4],&flash.data_temp[flash.temp_count-FLASH_DATA_SIZE],FLASH_DATA_SIZE);
                ble_send(tx_buff,BLE_MAX_LENGTH);
                flash.temp_count -= FLASH_DATA_SIZE;
                if(flash.temp_count == 0)//说明初始的半包或者后续的一包发送完成
                {
                    packet_count++;
                }               
            }
        }
        break;
        case FLASH_COMPLETE_STATUS:
            flash_timer_stop();
            flash.write_pointer = flash.read_pointer;
            memset (flash.data_backup, 0, FLASH_BUFF_SIZE);
            flash.backup_status = false;
#if (AS7000_FUNCTION_EN==1)
            as7000_measure_start();
#if (MPU6500_FUNCTION_EN==1)
            mpu6500_timer_start();
#endif
#endif
#if (ALG_ONCHIP_EN == 1)
#if (AFE4404_FUNCTION_EN==1)
            afe4404_interrupt_enable();
#endif
#endif
#if (ALGORITHM_FUNCTION_EN==1)
            algorithm_timer_start();
#endif
#if (WIFI_FUNCTION_EN==1)
            wifi_power_operator (CLOSE_WIFI_POWER);
#endif
            send_ack_to_app (FLASH_TX_COMPLETE_ACK);
            break;

        default:
            break;
        }
    }
}
#endif
/*****************************************************************
*功  能：用于查询和配置afe4404.WIFI和mpu6500数据处理
*参  数: 无
*返回值: 无
*****************************************************************/
void device_operator_process (void)
{
    if (system_status.device)
    {
        system_status.device = false;
        uint8_t tx_buff[BLE_MAX_LENGTH];
        uint16_t crc_data = 0;
        memset (tx_buff, 0, BLE_MAX_LENGTH);

        switch (system_operator.device)
        {
#if (MPU6500_FUNCTION_EN==1)

        case GET_MPU_DATA:
        {
            short a_x = mpu6500_read (MPU_ACCEL_X_REG);//读取加速度传感器数据
            short a_y = mpu6500_read (MPU_ACCEL_Y_REG);
            short a_z = mpu6500_read (MPU_ACCEL_Z_REG);
            short g_x = mpu6500_read (MPU_GYRO_X_REG);//读取陀螺仪传感器数据
            short g_y = mpu6500_read (MPU_GYRO_Y_REG);
            short g_z = mpu6500_read (MPU_GYRO_Z_REG);
            tx_buff[0] = FIRST_HEADER;
            tx_buff[1] = SECONED_HEADER;
            tx_buff[2] = DEVICE_ID;
            tx_buff[3] = GET_MPU6500_DATA;
            tx_buff[4] = a_x >> 8;
            tx_buff[5] = a_x;
            tx_buff[6] = a_y >> 8;
            tx_buff[7] = a_y;
            tx_buff[8] = a_z >> 8;
            tx_buff[9] = a_z;
            tx_buff[10] = g_x >> 8;
            tx_buff[11] = g_x;
            tx_buff[12] = g_y >> 8;
            tx_buff[13] = g_y;
            tx_buff[14] = g_z >> 8;
            tx_buff[15] = g_z;
            crc_data = get_check_sum (tx_buff, STATR_BYTE, BLE_CHECK_LENGTH);
            tx_buff[18] = crc_data >> 8;
            tx_buff[19] = crc_data;
            ble_send (tx_buff, BLE_MAX_LENGTH);
        }
        break;
#endif
#if (MLX90615_FUNCTION_EN==1)

        case GET_MLX_SHT_DATA:
        {
            float mlx90615_to = get_mlx90615_to();
            float mlx90615_ta = get_mlx90615_ta();
            int16_t mlx90615_ir = get_mlx90615_ir_value();
            float sht30_temp = get_sht30_temperature_value (PEPEATABILITY_HIGH);
            float sht30_humidity = get_sht30_humidity_value (PEPEATABILITY_HIGH);
            tx_buff[0] = FIRST_HEADER;
            tx_buff[1] = SECONED_HEADER;
            tx_buff[2] = DEVICE_ID;
            tx_buff[3] = GET_MLX90615_SHT30_DATA;
            tx_buff[4] = (int)mlx90615_to;
            tx_buff[5] = ((int) (mlx90615_to * 100)) % 100;
            tx_buff[6] = (int)mlx90615_ta;
            tx_buff[7] = ((int) (mlx90615_ta * 100)) % 100;
            tx_buff[8] = mlx90615_ir >> 8;
            tx_buff[9] = mlx90615_ir;
            tx_buff[10] = (int)sht30_temp;
            tx_buff[11] = ((int) (sht30_temp * 100)) % 100;
            tx_buff[12] = (int)sht30_humidity;
            tx_buff[13] = ((int) (sht30_humidity * 100)) % 100;
            crc_data = get_check_sum (tx_buff, STATR_BYTE, BLE_CHECK_LENGTH);
            tx_buff[18] = crc_data >> 8;
            tx_buff[19] = crc_data;
            ble_send (tx_buff, BLE_MAX_LENGTH);
        }
        break;
#endif
#if (AFE4404_FUNCTION_EN==1)

        case GET_AFE_REG:
        {
            afe4404_write (AFE_ADDR, 0x00, AFE_READ_MODE);
            uint32_t temp = afe4404_read (AFE_ADDR, system_data.afe.reg);
            afe4404_write (AFE_ADDR, 0x00, AFE_WRITE_MODE);
            tx_buff[0] = FIRST_HEADER;
            tx_buff[1] = SECONED_HEADER;
            tx_buff[2] = DEVICE_ID;
            tx_buff[3] = GET_AFE4404_REG;
            tx_buff[4] = system_data.afe.reg;
            tx_buff[5] = temp >> 24;
            tx_buff[6] = temp >> 16;
            tx_buff[7] = temp >> 8;
            tx_buff[8] = temp;
            crc_data = get_check_sum (tx_buff, STATR_BYTE, BLE_CHECK_LENGTH);
            tx_buff[18] = crc_data >> 8;
            tx_buff[19] = crc_data;
            ble_send (tx_buff, BLE_MAX_LENGTH);
        }
        break;

        case GET_AFE_CONFIG:
            tx_buff[0] = FIRST_HEADER;
            tx_buff[1] = SECONED_HEADER;
            tx_buff[2] = DEVICE_ID;
            tx_buff[3] = GET_AFE4404_CONFIG;
            tx_buff[4] = get_afe4404_frq();
            tx_buff[5] = get_afe4404_iled (GREEN);
            tx_buff[6] = get_afe4404_iled (RED);
            tx_buff[7] = get_afe4404_iled (IR);
            tx_buff[8] = get_afe4404_offdac (GREEN);
            tx_buff[9] = get_afe4404_offdac (RED);
            tx_buff[10] = get_afe4404_offdac (IR);
            tx_buff[11] = get_afe4404_offdac (AMBIENT);
            tx_buff[12] = get_TIA_GAIN (GREEN);
            tx_buff[13] = get_TIA_GAIN (RED);
            tx_buff[14] = get_afe4404_numav();
            tx_buff[15] = system_data.afe.duty;
            tx_buff[16] = system_data.afe.ambient_en;
            tx_buff[17]	= system_data.afe.auto_adjust;
            crc_data = get_check_sum (tx_buff, STATR_BYTE, BLE_CHECK_LENGTH);
            tx_buff[18] = crc_data >> 8;
            tx_buff[19] = crc_data;
            ble_send (tx_buff, BLE_MAX_LENGTH);
            break;

        case SET_AFE_CONFIG:
        {
            /*下面判断语句请看协议4404设置部分*/
            if (system_data.afe.set_params[0]&AFE_FRQ_SET_ENABLE)
            {
                set_afe4404_frq (system_data.afe.frq, system_data.afe.numav, system_data.afe.duty);
            }

            if (system_data.afe.set_params[0]&AFE_GREEN_ILED_SET_ENABLE)
            {
                config_LED_Current (system_data.afe.iled[GREEN], GREEN);
            }

            if (system_data.afe.set_params[0]&AFE_RED_ILED_SET_ENABLE)
            {
                config_LED_Current (system_data.afe.iled[RED], RED);
            }

            if (system_data.afe.set_params[0]&AFE_IR_ILED_SET_ENABLE)
            {
                config_LED_Current (system_data.afe.iled[IR], IR);
            }

            if (system_data.afe.set_params[0]&AFE_GREEN_OFFDAC_SET_ENABLE)
            {
                set_OFFDAC (system_data.afe.offdac[GREEN] & 0x0F, system_data.afe.offdac[GREEN] >> 4, GREEN);
            }

            if (system_data.afe.set_params[0]&AFE_RED_OFFDAC_SET_ENABLE)
            {
                set_OFFDAC (system_data.afe.offdac[RED] & 0x0F, system_data.afe.offdac[RED] >>	4, RED);
            }

            if (system_data.afe.set_params[0]&AFE_IR_OFFDAC_SET_ENABLE)
            {
                set_OFFDAC (system_data.afe.offdac[IR] & 0x0F, system_data.afe.offdac[IR] >> 4, IR);
            }

            if (system_data.afe.set_params[0]&AFE_AMBIENT_OFFDAC_SET_ENABLE)
            {
                set_OFFDAC (system_data.afe.offdac[AMBIENT] & 0x0F, system_data.afe.offdac[AMBIENT] >> 4, AMBIENT);
            }

            if (system_data.afe.set_params[1]&AFE_GREEN_GAIN_SET_ENABLE)
            {
                set_TIA_GAIN (system_data.afe.gain[GREEN], GREEN);
            }

            if (system_data.afe.set_params[1]&AFE_RED_GAIN_SET_ENABLE)
            {
                set_TIA_GAIN (system_data.afe.gain[RED], RED);
            }

            send_ack_to_app (AFE4404_SET_ACK);
        }
        break;
#endif
#if (MPU6500_FUNCTION_EN == 1)

        case GET_MPU_CONFIG:
            tx_buff[0] = FIRST_HEADER;
            tx_buff[1] = SECONED_HEADER;
            tx_buff[2] = DEVICE_ID;
            tx_buff[3] = GET_MPU6500_CONFIG;
            tx_buff[4] = get_mpu6500_status (MPU_ACCEL);
            tx_buff[5] = get_mpu6500_status (MPU_GYRO);
            tx_buff[6] = get_mpu6500_range (MPU_ACCEL);
            tx_buff[7] = get_mpu6500_range (MPU_GYRO);
            crc_data = get_check_sum (tx_buff, STATR_BYTE, BLE_CHECK_LENGTH);
            tx_buff[18] = crc_data >> 8;
            tx_buff[19] = crc_data;
            ble_send (tx_buff, BLE_MAX_LENGTH);
            break;

        case SET_MPU_CONFIG:

            /*下面判断语句请看协议6500设置部分*/
            if (system_data.mpu.set_params[0]&MPU_ACCEL_STATUS_ENABLE)
            {
                set_mpu6500_status (MPU_ACCEL, system_data.mpu.mpu_accel_status);
            }

            if (system_data.mpu.set_params[1]&MPU_GYRO_STATUS_ENABLE)
            {
                set_mpu6500_status (MPU_GYRO, system_data.mpu.mpu_gyro_status);
            }

            if (system_data.mpu.set_params[2]&MPU_ACCEL_RANGE_ENABLE)
            {
                set_mpu6500_range (MPU_ACCEL, system_data.mpu.mpu_accel_range);
            }

            if (system_data.mpu.set_params[3]&MPU_GYRO_RANGE_ENABLE)
            {
                set_mpu6500_range (MPU_GYRO, system_data.mpu.mpu_gyro_range);
            }

            send_ack_to_app (MPU6500_SET_ACK);
            break;
#endif
#if (WIFI_FUNCTION_EN==1)

        case SET_WIFI_TCP:
        {
            nrf_delay_ms (500);
            enable_wifi_tx_mode();
            nrf_delay_ms (25);
            enable_wifi_tx_send();
        }
        break;

        case WIFI_VCC_ENABLE:
        {
            wifi_power_operator (OPEN_WIFI_POWER);
        }
        break;

        case WIFI_VCC_DISABLE:
        {
            wifi_power_operator (CLOSE_WIFI_POWER);
        }
        break;
#endif

        case SET_BLE_NAME://对设备名称的相关操作（设置前缀、后缀、恢复默认）后更新广播名称
        {
            uint32_t err_code;

            //断开连接
            //如果并不希望立即断开本次连接，而是用户主动断开本次连接后更新名称，屏蔽此段即可。
//            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
//            {
//                // Disconnect from peer.
//                err_code = sd_ble_gap_disconnect (m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//                APP_ERROR_CHECK (err_code);
//            }

            ble_gap_conn_sec_mode_t sec_mode;
            BLE_GAP_CONN_SEC_MODE_SET_OPEN (&sec_mode);
            err_code = sd_ble_gap_device_name_set (&sec_mode,
                                                   (const uint8_t *)device_name,
                                                   sizeof (device_name));
            APP_ERROR_CHECK (err_code);
            advertising_init();
        }
        break;

        case SET_WIFI_NAME:
            set_8266_name();
            break;

        default:
            break;
        }
    }
}
/*****************************************************************
*功  能：根据按键的长按开关机对系统电源进行操作管理
*参  数: 无
*返回值: 无
*****************************************************************/
void power_operator_process (void)
{
    if (system_status.power)
    {
        system_status.power = false;
        switch (system_operator.power)
        {
        case OPEN_SYSTEM_POWER:
        {
            if(!system_status.spi_en)//如果是第一次上电，则spi和flash在main函数已经初始化过了，这里不再初始化
            {
#if ((FLASH_FUNCTION_EN==1) || (OLED_FUNCTION_EN==1)||(WIFI_FUNCTION_EN==1))
                spi_init();
                nrf_delay_ms(10);
#endif
#if (FLASH_FUNCTION_EN==1)
                system_parameter_init();
#else
                system_data.algorithm_interval = 5;
#endif
            }
            gpiote_init();
            switch (system_data.system_mode)
            {
            case NORMAL_MODE:
#if (UART_FUNCTION_EN==1)
                printf("NORMAL_MODE\n");
#endif
                normal_mode_init();
                break;

            case SELF_TEST_MODE:
#if (UART_FUNCTION_EN==1)
                printf("SELF_TEST_MODE\n");
#endif
                self_test_mode_init();
                break;

            case BLOOD_PRESSURE_MODE:
#if (UART_FUNCTION_EN==1)
                printf("BLOOD_PRESSURE_MODE\n");
#endif
                blood_pressure_mode_init();
                break;

            case AFE4404_MODE:
#if (UART_FUNCTION_EN==1)
                printf("AFE4404_MODE\n");
#endif
                afe4404_mode_init();
                break;

            default:
                break;
            }
        }
        break;

        case CLOSE_SYSTEM_POWER:
#if (UART_FUNCTION_EN==1)
            printf("CLOSE_SYSTEM_POWER\n");
#endif
            device_close_init();
            break;
        case POWER_OFF_CHARGE_STOP://关机状态下充电后充电器拔出
        {
            SEGGER_RTT_printf(0, "POWER_OFF_CHARGE_STOP\n");
#if (OLED_FUNCTION_EN==1)
            system_status.oled_switch2 = false;
            oled_power_onoff (false); //关闭oled电源
            OLED_VPP_DISABLE;
            OLED_VCC_DISABLE;
            OLED_CMD_DAT_OUTPUT;
            OLED_WRITE_CMD_EN;

            OLED_CS_OUTPUT;
            OLED_CS_L;

            OLED_RES_OUTPUT;
            OLED_RES_L;
#endif
            if(system_status.spi_en)
            {
                SEGGER_RTT_printf(0, "BEFORE_uninit_spi\n");
                system_status.spi_en = false;
                nrf_delay_ms(100);
                nrf_drv_spi_uninit(&spi0);

                nrf_gpio_cfg_output(SPI_SCK_PIN);
                nrf_gpio_pin_clear (SPI_SCK_PIN);

                nrf_gpio_cfg_output(SPI_MOSI_PIN);
                nrf_gpio_pin_clear (SPI_MOSI_PIN);

                nrf_gpio_cfg_output(SPI_MISO_PIN);
                nrf_gpio_pin_clear (SPI_MISO_PIN);
            }
        }
            break;
        default:
            break;
        }
    }
}
#if (AFE4404_FUNCTION_EN==1)
/*****************************************************************
*功  能：判断4404的RDY管脚是否有中断，有中断表示有数据更新
*参  数: 无
*返回值: 无
*****************************************************************/
void afe4404_event_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    system_status.afe4404 = true;
//    SEGGER_RTT_printf(0,"afe4404_event_handler\n");
}
#endif

#if (KEY_FUNCTION_EN==1)
uint16_t key_holdon_ms = 0;//按下计数
uint16_t key_up_count = 0;//按键被按下并弹起之后计数
bool long_key_flag = false;//长按标志
bool key_press_flag = false;
bool short_key_flag = false;//短按标志
bool key_up_flag = false;//按键曾被按下并释放标志
bool double_click_flag = false;//双击事件标志
/*****************************************************************
*功  能：判断4404的RDY管脚是否有中断，有中断表示有数据更新
*参  数: 无
*返回值: 无
*****************************************************************/
void key_event_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    /*开关机按键一下就启动按键扫描定时器*/
    if(!system_status.chrg_en)//如果没有在充电状态下
    {
#if (UART_FUNCTION_EN==1)
        printf("key_gpiote\n");
#endif
        key_press_flag = true;
        key_timer_start();
    }
}
#endif


/*****************************************************************
*功  能：充电状态检测及处理
*参  数: 无
*返回值: 无
*****************************************************************/
void charge_status_detection(void)
{
    if((system_data.power_switch) && (!system_status.chrg_en)) //如果当前没在充电状态,关且是在开机状态
    {
        if (CHRG_DECT_STATUS == 1)
        {
            if (CHRG_STAT_STATUS == 0) //检测到充电指示引脚电平
            {
                system_status.chrg_en = true;
//                SEGGER_RTT_printf(0, "charge_en\n");
            }
        }
        else
        {
//            SEGGER_RTT_printf(0, "charge_disable\n");
            system_status.chrg_en = false;
        }
    }
}

/*****************************************************************
*功  能：判断充电器是否断开，然后设备自动开机
*参  数: 无
*返回值: 无
*****************************************************************/
void chrg_dect_event_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_delay_ms(100);

    if (CHRG_DECT_STATUS == 0) //充电器拔除
    {
        if(system_data.power_switch)//如果是在开机状态
        {
            SEGGER_RTT_printf(0, "power_on_stop_charge\n");
            system_status.chrg_en = false;
        }
        else//关机状态
        {
#if (UART_FUNCTION_EN==1)
            printf("power_close_stop_charge\n");
#endif
            system_status.chrg_en = false;
            system_status.oled_switch3 = false;
#if (ADC_FUNCTION_EN==1)
            charge_timer_stop();
#endif
            system_data.power_switch = false;
            system_status.power = true;
            system_operator.power = POWER_OFF_CHARGE_STOP;
        }
    }

    else//充电器插入
    {
        SEGGER_RTT_printf(0, "charger_in\n");
        if (CHRG_STAT_STATUS == 0) //检测到充电指示引脚电平
        {
            SEGGER_RTT_printf(0, "CHRG_STAT_STATUS=0\n");
            if(system_data.power_switch)//如果是在开机状态
            {
                SEGGER_RTT_printf(0, "charge_in_power_on\n");
                system_status.chrg_en = true;
            }
            else
            {
                SEGGER_RTT_printf(0, "power_off_prepare_charge\n");
                system_status.chrg_en = true;

                if(!system_status.spi_en)
                {
                    spi_init();
                }
                nrf_delay_ms(5);//不延时会死机
                oled_init();
                oled_dis_big_battery();
#if (ADC_FUNCTION_EN==1)
                charge_timer_start (1);
#endif
            }
        }
    }
}
#if (ALGORITHM_FUNCTION_EN==1)
/*****************************************************************
*功  能：用于算法定时器累积计时，等待进入定时器中断处理函数
*参  数: 无
*返回值: 无
*****************************************************************/
void algorithm_processing_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);
    system_data.algorithm_count++;

    if (system_data.algorithm_count == system_data.algorithm_interval)
    {
        system_status.algorithm = true;
        system_data.algorithm_count = 0;
    }
}
#endif
#if (CALENDER_FUNCTION_EN==1)
/*****************************************************************
*功  能：每5s钟更新电量
*参  数: 无
*返回值: 无
*****************************************************************/
void update_vbat_level_display(void)
{
    static uint8_t temp_count = 0;
    if(system_data.power_switch)//如果是在开机状态
    {
        temp_count++;
        if (temp_count >= 20) //20秒更新一次电量
        {
            temp_count = 0;
            if(system_status.oled_switch2)
            {
                oled_dis_vbat_level();
            }
        }
    }
}


/*****************************************************************
*功  能：用于nRF51822的日历功能时间计时
*参  数: 无
*返回值: 无
*****************************************************************/
void calender_processing_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);
    calender_count++;
    sec_to_date (&calender, calender_number + calender_count);
    charge_status_detection();//充电状态检测
//    SEGGER_RTT_printf(0, "calender_count=%d\n", calender_count);
    if(system_status.oled_switch2)//如果oled处于开启状态
    {
//        SEGGER_RTT_printf(0, "oled_switch2\n");
        if(system_data.oled_page != DISPLAY_MAC_ADDR)//如果不是要显示mac地址
        {
//            SEGGER_RTT_printf(0,"DISPLAY_MAC_ADDR\n");
            if(system_data.oled_display_time != 0x00)//如果不是永不熄屏，就要比较熄屏时间
            {
//                SEGGER_RTT_printf(0,"oled_display_time\n");
                if(system_data.oled_page != system_data.oled_page_backup)//切换了页面，将熄屏计时清0
                {
                    system_data.oled_display_count = 0;
                }
                system_data.oled_display_count++;
            }
            else//永不熄屏
            {
                system_data.oled_display_count = 0;
            }
            if(system_data.oled_display_count <= system_data.oled_display_time)//显示没有超时
            {
//                SEGGER_RTT_printf(0,"no_over_time\n");
                switch(system_data.system_mode)
                {
                case NORMAL_MODE:
                {
//                        SEGGER_RTT_printf(0,"NORMAL_MODE_oled_display\n");
                    system_status.oled = true;
                    system_operator.oled = system_data.oled_page;
                }
                break;
                case SELF_TEST_MODE:
//                       SEGGER_RTT_printf(0,"self_test_mode_oled_display\n");
                    if(system_status.self_test_mode_display)
                    {
                        system_status.self_test_mode_display = false;
                        system_status.oled = true;
                        system_operator.oled = SELF_TEST_MODE_DISPLAY;
                    }
                    break;

                case BLOOD_PRESSURE_MODE:
//                        SEGGER_RTT_printf(0,"BLOOD_PRESSURE_MODE_oled_display\n");
                    system_status.oled = true;
                    system_operator.oled = BLOOD_PRESSURE_MODE_DISPLAY;
                    break;
                default:
                    break;
                }
            }
            else//显示超时，熄屏
            {
                system_status.oled = true;
                system_operator.oled = CLOSE_OLED_DISPLAY;
                system_data.oled_display_count = 0;
            }

//            else//永不熄屏，则一直显示
//            {
//                system_data.oled_display_count = 0;
//                system_status.oled = true;
//                system_operator.oled = system_data.oled_page;
//            }
        }
        else//要显示mac地址
        {
            system_data.oled_display_count = 0;
            system_status.oled = true;
            system_operator.oled = DISPLAY_MAC_ADDR;//显示mac地址
        }
    }
#if 0

    /*未充电状态*/
    if (CHRG_DECT_STATUS == NOT_CHARGED)
    {
        /*自检模式需要测试R/G灯，所以不自动关闭*/
        if (system_data.system_mode != SELF_TEST_MODE)
        {
        }
    }

    /*充电状态*/
    else
    {
        /*充电时，会自动关机进入低功耗状态*/
        if (system_data.power_switch)
        {
            system_data.power_switch = !system_data.power_switch;
            system_status.power = true;
            system_operator.power = CLOSE_SYSTEM_POWER;
        }

        if (CHRG_STAT_STATUS == CHARGE_COMPLETE)
        {
        }
        else
        {
        }
    }

#endif
}
#endif
#if (FLASH_FUNCTION_EN==1)
/*****************************************************************
*功  能：用于操作flash功能时间计时
*参  数: 无
*返回值: 无
*****************************************************************/
void flash_processing_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);
    system_status.flash = true;
}
#endif
#if (KEY_FUNCTION_EN==1)

/*****************************************************************
*功  能：用于扫描按键长按功能定时器处理
*参  数: 无
*返回值: 无
*****************************************************************/
void key_processing_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);

    if(key_press_flag)
    {
        if (KEY_IN_STATUS == KEY_PRESS)
        {
            if(key_holdon_ms <= KEY_LONG_PRESS_TIME)//一直按下，则一直计数
            {
                key_holdon_ms++;
            }
            else //长按事件触发
            {
#if (UART_FUNCTION_EN==1)
                printf("long_key_flag\n");
#endif
                SEGGER_RTT_printf(0, "long_key_flag\n");
                key_timer_stop();//停止定时器

                key_holdon_ms = 0;
                short_key_flag = 0;
                long_key_flag = 1;
                key_press_flag = 0;
                //长按事件处理
                if (!system_data.power_switch)//如果目前是关机状态
                {
                    SEGGER_RTT_printf(0, "prepare_open\n");
                    MOTOR_ENABLE;//马达振动
                    motor_timer_start(200);
                    system_status.display_bat_logo = true;//立即进行一次电池图标的显示
                    system_data.power_switch = true;
                    system_status.power = true;
                    system_operator.power = OPEN_SYSTEM_POWER;
                }
                else
                {
                    SEGGER_RTT_printf(0, "prepare_close\n");
                    MOTOR_ENABLE;//马达振动
                    motor_timer_start(200);
                    system_data.power_switch = false;
                    system_status.power = true;
                    system_operator.power = CLOSE_SYSTEM_POWER;
                }
            }
        }
        else //按键松开或消抖未通过
        {
            if(!system_data.power_switch)//如果是在关机状态，直接结束
            {
                key_holdon_ms = 0;
                MOTOR_DISABLE;//停止马达振动
                long_key_flag = false;
                key_press_flag = false;
                key_up_flag = false;
                key_timer_stop();
            }
            else//是在开机状态
            {
                if(key_holdon_ms > 50)//短按事件触发
                {
                    key_holdon_ms = 0;
                    MOTOR_DISABLE;//停止马达振动
                    short_key_flag = true;
                    long_key_flag = false;
                    key_press_flag = false;
                    key_up_flag = true;
                    if((key_up_count > 100) && (key_up_count < 500))//双击事件触发
                    {
#if (UART_FUNCTION_EN==1)
                        printf("double_click_flag\n");
#endif
                        SEGGER_RTT_printf(0, "double_click_flag\n");
                        key_timer_stop();
                        short_key_flag = false;
                        double_click_flag = true;
                        short_key_flag = 0;
                        key_up_flag = 0;
                        key_up_count = 0;
                        //双击事件处理
                        if(system_data.power_switch)//如果是在开机状态，双击才有效
                        {
                            if(system_data.system_mode == NORMAL_MODE)
                            {
                                if(system_data.oled_page == DISPLAY_MAC_ADDR)//如果当前是在显示mac地址页面
                                {
#if (UART_FUNCTION_EN==1)
                                    printf("exit_DISPLAY_MAC_ADDR\n");
#endif
                                    system_data.oled_page = system_data.oled_mode_page_backup;
#if (UART_FUNCTION_EN==1)
                                    printf("system_data.oled_page=%d\n", system_data.oled_page);
#endif
                                }
                                else
                                {
#if (UART_FUNCTION_EN==1)
                                    printf("enter_DISPLAY_MAC_ADDR\n");
#endif
                                    system_data.oled_mode_page_backup = system_data.oled_page;
#if (UART_FUNCTION_EN==1)
                                    printf("mode_page_backup=%d\n", system_data.oled_mode_page_backup);
#endif
                                    system_data.oled_page = DISPLAY_MAC_ADDR;
                                }
                            }
                        }
                    }
                }
                else  //判断为抖动
                {
                    SEGGER_RTT_printf(0, "shake\n");
                    MOTOR_DISABLE;//停止马达振动
                    key_holdon_ms = 0;
                    short_key_flag = 0;
                    long_key_flag = 0;
                    key_press_flag = 0;
                    key_timer_stop();
                }
            }//else//是在开机状态
        }
    }
    if(key_up_flag)//按键曾被按下并释放标志
    {

        key_up_count++;
        if(key_up_count > 500)//两次单击之间的时间超时.单击事件触发
        {
            SEGGER_RTT_printf(0, "short_key_flag\n");
            MOTOR_DISABLE;//停止马达振动
            key_timer_stop();
            key_up_count = 0;
            key_up_flag = false;
            //短按事件处理
            if(!system_status.oled_switch2)//如果oled处于关闭状态
            {
                system_status.oled = true;
                system_operator.oled = OPEN_OLED_DISPLAY;
            }
            else
            {
                if(system_data.oled_page != DISPLAY_MAC_ADDR)//如果当前不是在显示mac地址页面
                {
                    if(system_data.system_mode == NORMAL_MODE)//切换页面显示
                    {
                        if(system_data.oled_page == NORMAL_MODE_HOME_PAGE)
                        {
                            system_data.oled_page = NOMAL_MODE_PAGE2;
                        }
                        else
                        {
                            system_data.oled_page = NORMAL_MODE_HOME_PAGE;
                        }
                    }
                }
            }
        }
    }

}
#endif

#if (AS7000_FUNCTION_EN==1)

#if (MPU6500_FUNCTION_EN==1)
/*****************************************************************
*功  能：用于常规模式下采样mpu6500数据的功能定时器处理
*参  数: 无
*返回值: 无
*****************************************************************/
void mpu6500_processing_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);
    system_status.mpu6500 = true;
}
#endif

/****************************************************************************
 *函数功能: as7000心率数据准备好时会通过handshake引脚触发这个gpio中断处理函数
 *输入参数:
 *返回值  :
****************************************************************************/
void as7000_event_handler (nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (as7000_init_count == 0) //if fisrt time enter this function
    {
        as7000_init_count++;
        uint8_t app_id = as7000_read (REG_ADDR_APPLICATION_ID);
        //you can add write application id failure judgement.
    }

    else if (as7000_init_count == 1) //if second time enter this function
    {
        as7000_init_count++;
        as7000_read (REG_ADDR_SYNC);
    }
    else
    {
        system_status.as7000_ready = true;
//        get_hrm_value();
    }
}
#endif
void mpu6500_timer_stop (void);
/****************************************************************************
 *函数功能: 蓝牙图标闪烁定时器
 *输入参数:
 *返回值  :
****************************************************************************/
void motor_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);
    MOTOR_DISABLE;//停止马达振动
    motor_timer_stop();  
}
#if (ADC_FUNCTION_EN==1)
/****************************************************************************
 *函数功能: 充电电量采集定时器超时处理函数
 *输入参数:
 *返回值  :
****************************************************************************/
void charge_timeout_handler (void * p_context)
{
    UNUSED_PARAMETER (p_context);
    if(system_status.chrg_en)
    {
        uint8_t temp_vbat_power = get_vbat_power();
        temp_vbat_power = temp_vbat_power / 16 * 10 + temp_vbat_power % 16;//转换成hex
        if(system_status.oled_switch3)
        {
            oled_dis_charge_anime(temp_vbat_power);
        }
        charge_timer_start(CHARGE_TIME_INTERVAL);//再次开启定时器
    }
}
#endif
/*****************************************************************
*功  能：为算法和日历功能创建定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void timers_init (void)
{
    uint32_t err_code;
    // Create timers.
#if (ALGORITHM_FUNCTION_EN==1)
    err_code = app_timer_create (&algorithm_timer, APP_TIMER_MODE_REPEATED, algorithm_processing_timeout_handler);
    APP_ERROR_CHECK (err_code);
#endif
#if (CALENDER_FUNCTION_EN==1)
    err_code = app_timer_create (&calender_timer, APP_TIMER_MODE_REPEATED, calender_processing_timeout_handler);
    APP_ERROR_CHECK (err_code);
#endif
#if (FLASH_FUNCTION_EN==1)
    err_code = app_timer_create (&flash_timer, APP_TIMER_MODE_REPEATED, flash_processing_timeout_handler);
    APP_ERROR_CHECK (err_code);
#endif
#if (KEY_FUNCTION_EN==1)
    err_code = app_timer_create (&key_timer, APP_TIMER_MODE_REPEATED, key_processing_timeout_handler);
    APP_ERROR_CHECK (err_code);
#endif
#if (AS7000_FUNCTION_EN==1)
#if (MPU6500_FUNCTION_EN==1)
    err_code = app_timer_create (&mpu6500_timer, APP_TIMER_MODE_REPEATED, mpu6500_processing_timeout_handler);
    APP_ERROR_CHECK (err_code);
#endif
#endif
    //用于蓝牙图标闪烁的定时器
    err_code = app_timer_create (&motor_timer, APP_TIMER_MODE_REPEATED, motor_timeout_handler);
    APP_ERROR_CHECK (err_code);
#if (ADC_FUNCTION_EN==1)
    //用于充电电量检测的定时器
    err_code = app_timer_create (&charge_timer, APP_TIMER_MODE_SINGLE_SHOT, charge_timeout_handler);
    APP_ERROR_CHECK (err_code);
#endif

}
#if (CALENDER_FUNCTION_EN==1)
/*****************************************************************
*功  能：使能日历功能定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void calender_timer_start (void)
{
    uint32_t err_code;
    err_code = app_timer_start (calender_timer, APP_TIMER_TICKS (CALENDER_TIME_INTERVAL, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止日历功能定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void calender_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (calender_timer);
    APP_ERROR_CHECK (err_code);
}
#endif
#if (ALGORITHM_FUNCTION_EN==1)
/*****************************************************************
*功  能：使能算法定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void algorithm_timer_start (void)
{
    uint32_t err_code;
    err_code = app_timer_start (algorithm_timer, APP_TIMER_TICKS (ALGORITHM_TIME_INTERVAL, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止算法定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void algorithm_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (algorithm_timer);
    APP_ERROR_CHECK (err_code);
}
#endif
#if (FLASH_FUNCTION_EN==1)
/*****************************************************************
*功  能：使能flash操作定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void flash_timer_start (void)
{
    uint32_t err_code;
    err_code = app_timer_start (flash_timer, APP_TIMER_TICKS (FLASH_STATUS_TIME_INTERVAL, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止flash操作定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void flash_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (flash_timer);
    APP_ERROR_CHECK (err_code);
}
#endif
#if (KEY_FUNCTION_EN==1)
/*****************************************************************
*功  能：使能按键扫描操作定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void key_timer_start (void)
{
    uint32_t err_code;
    err_code = app_timer_start (key_timer, APP_TIMER_TICKS (KEY_TIME_INTERVAL, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止按键扫描操作定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void key_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (key_timer);
    APP_ERROR_CHECK (err_code);
}
#endif

//#if (AS7000_FUNCTION_EN==1)

#if (MPU6500_FUNCTION_EN==1)
/*****************************************************************
*功  能：使能mpu6500操作定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void mpu6500_timer_start (void)
{
    uint32_t err_code;
    err_code = app_timer_start (mpu6500_timer, APP_TIMER_TICKS (MPU6500_TIME_INTERVAL, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止mpu6500操作定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void mpu6500_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (mpu6500_timer);
    APP_ERROR_CHECK (err_code);
}
#endif
//#endif
/*****************************************************************
*功  能：启动马达定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void motor_timer_start (uint16_t time)
{
    uint32_t err_code;
    err_code = app_timer_start (motor_timer, APP_TIMER_TICKS (time, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止马达定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void motor_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (motor_timer);
    APP_ERROR_CHECK (err_code);
}

#if (ADC_FUNCTION_EN==1)
/*****************************************************************
*功  能：开启充电定时检测定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void charge_timer_start (uint16_t timer_interval)
{
    uint32_t err_code;
    err_code = app_timer_start (charge_timer, APP_TIMER_TICKS (timer_interval, 0), NULL);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：停止充电定时检测定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void charge_timer_stop (void)
{
    uint32_t err_code;
    err_code = app_timer_stop (charge_timer);
    APP_ERROR_CHECK (err_code);
}
#endif

/*****************************************************************
*功  能：串口发送函数
*参  数: *tx_buff 需要发送数组的首地址
*		 length 需要发送数组的长度
*返回值: 无
*****************************************************************/
void uart_send (uint8_t *tx_buff, uint16_t length)
{
    for (int i = 0; i < length; i++)
    {
        app_uart_put (tx_buff[i]);
    }
}

/*****************************************************************
*功  能：获取当前电池电量百分比
*参  数: 无
*返回值: 电池电量百分比的BCD码
*****************************************************************/
uint8_t get_vbat_power (void)
{
    uint8_t vbat_power = 0;
    int16_t adc_value = 0;
    VBAT_ADC_ON_ENABLE;
    nrf_drv_saadc_sample_convert (1, &adc_value);
    VBAT_ADC_ON_DISABLE;

    if (adc_value > VBAT_ADC_MAX)
    {
        adc_value = VBAT_ADC_MAX;
    }

    else if (adc_value < VBAT_ADC_MIN)
    {
        adc_value = VBAT_ADC_MIN;
    }

    vbat_power = (adc_value - VBAT_ADC_MIN) * 100 / (VBAT_ADC_MAX - VBAT_ADC_MIN);
    return (vbat_power / 10 << 4) | (vbat_power % 10);//返回电量百分比,BCD码形式
}



void saadc_callback (nrf_drv_saadc_evt_t const * p_event)
{
#if (UART_FUNCTION_EN==1)
    printf ("saadc_callback\n");
#endif

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
#if (UART_FUNCTION_EN==1)
        printf ("adc sample size=%d\n", p_event->data.done.size);

        for (uint8_t i = 0; i < p_event->data.done.size; i++)
        {
            printf ("sample_buff[%d]=%d ", i, p_event->data.done.p_buffer[i]);
        }

#endif
        return;
    }
}

/*****************************************************************
*功  能：配置电池管理的初始化函数
*参  数: 无
*返回值: 无
*****************************************************************/
void vbat_power_init (void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config;
    nrf_drv_saadc_config_t adc_config;
    adc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;//ADC分辨率为12bit
    adc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;//关闭过采样功能
    adc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    err_code = nrf_drv_saadc_init (&adc_config, saadc_callback);
    APP_ERROR_CHECK (err_code);
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;
    channel_config.pin_p = (nrf_saadc_input_t) (NRF_SAADC_INPUT_AIN1); //设置ADC接口
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US; //设置采样转换时间
    channel_config.gain = NRF_SAADC_GAIN1_4; //设置参考增益
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; //选择参考电压
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; //设置采样模式为常规采样
    err_code = nrf_drv_saadc_channel_init (1, &channel_config);
    APP_ERROR_CHECK (err_code);
    VBAT_ADC_ON_CFG_OUTPUT;
    VBAT_ADC_ON_DISABLE;
    system_status.adc_en = true;
}
/*****************************************************************
*功  能：配置afe4404的RDY管脚的中断设置和其他常用GPIO口设置
*参  数: 无
*返回值: 无
*****************************************************************/
void gpiote_init (void)
{
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t pin_config;
    pin_config.hi_accuracy = true;
    pin_config.is_watcher = false;//这里如果设置为true会影响功耗
    pin_config.pull = NRF_GPIO_PIN_PULLUP;//按键配置成内部上拉
//    pin_config.pull = NRF_GPIO_PIN_NOPULL;
#if (KEY_FUNCTION_EN==1)
    /*配置按键中断*/
    pin_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    nrf_drv_gpiote_in_init (KEY_IN, &pin_config, key_event_handler);
    nrf_drv_gpiote_in_event_enable (KEY_IN, true);
#endif
    pin_config.pull = NRF_GPIO_PIN_NOPULL;//其它配置成无上拉

#if (ADC_FUNCTION_EN==1)
    /*配置充电器电平中断*/
    pin_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    nrf_drv_gpiote_in_init (CHRG_DECT, &pin_config, chrg_dect_event_handler);
    nrf_drv_gpiote_in_event_enable (CHRG_DECT, true);
#endif

#if (WIFI_FUNCTION_EN==1)
    /*WIFI模块GPIO设置*/
    ESP_VCC_CFG_OUTPUT;
    ESP_VCC_DISABLE;
    WIFI_CS_CFG_OUTPUT;
    WIFI_CS_L;
#endif

     nrf_gpio_cfg_output(TX_PIN_NUMBER);
     nrf_gpio_cfg_output(RX_PIN_NUMBER);
     nrf_gpio_pin_clear(TX_PIN_NUMBER);
     nrf_gpio_pin_clear(RX_PIN_NUMBER);

#if (AFE4404_FUNCTION_EN==1)
    /*配置4404管脚中断*/
    pin_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    nrf_drv_gpiote_in_init (AFE4404_RDY_PIN, &pin_config, afe4404_event_handler);
    nrf_drv_gpiote_in_event_enable (AFE4404_RDY_PIN, true);
#endif

#if (AS7000_FUNCTION_EN==1)
    pin_config.pull = NRF_GPIO_PIN_PULLUP;
    pin_config.sense = NRF_GPIOTE_POLARITY_HITOLO;  //trigger polarity
    nrf_drv_gpiote_in_init (AS7000_SYN, &pin_config, as7000_event_handler);
    nrf_drv_gpiote_in_event_enable (AS7000_SYN, true);
#endif

#if (AFE4404_FUNCTION_EN==1)
    /*AFE4404电源开关管脚设置*/
    AFE_VCC_CFG_OUTPUT;
    AFE_RESETZ_CFG_OUTPUT;
    AFE_VCC_ENABLE;
#endif

#if (MPU6500_FUNCTION_EN==1)
    MPU_VCC_CFG_OUTPUT;
    MPU_VCC_ENABLE;
#endif
#if (AS7000_FUNCTION_EN==1)
    AS7000_EN_CFG_OUTPUT;
#endif
    CHRG_STAT_CFG_INPUT;

#if (UART_FUNCTION_EN==1)
    printf("general_power_on\n");
#endif
#if((MPU6500_FUNCTION_EN==1)||(MLX90615_FUNCTION_EN==1)||(AS7000_FUNCTION_EN==1)||(SHT30_FUNCTION_EN==1)||(AFE4404_FUNCTION_EN==1))//如果要使用IIC器件
    twi_init();
#endif
    nrf_delay_ms(5);//如果这里不延时，有时候不能正常启动
#if (MLX90615_FUNCTION_EN==1)
    set_mlx90615_sample_times (0x07);
#endif


#if (OLED_FUNCTION_EN==1)
    oled_init();
#endif
}

/*****************************************************************
*功  能：启动日历定时器
*参  数: 无
*返回值: 无
*****************************************************************/
void calender_start(void)
{
#if (CALENDER_FUNCTION_EN==1)
    calender_number = date_to_sec (&calender);
    calender_timer_start();
#endif
}




#if (FLASH_FUNCTION_EN==1)
/*****************************************************************
*功  能：初始化系统参数
*参  数: 无
*返回值: 无
*****************************************************************/
void system_parameter_init (void)
{
    uint8_t flash_data[FLASH_BUFF_SIZE];
    flash_power_init();
    memset (flash_data, 0, FLASH_BUFF_SIZE);
    flash_page_read (SYSTEM_DATA_ADDR, flash_data);
    uint8_t temp[13];
    memset (temp, 0, 13);
    uint8_t i, j;
#if (UART_FUNCTION_EN==1)
    printf("ALGORITHM_TIME=%d\n", flash_data[ALGORITHM_TIME]);
#endif
//    for(uint8_t n = 0; n < 15; n++)
//    {
//        SEGGER_RTT_printf(0, "flash_data[%d]=%d ", n, flash_data[n]);
//    }
//    SEGGER_RTT_printf(0, "\n");
    /*读取flash数据为FF表示里没有正确或者还没写入有效数据*/
    if ((flash_data[ALGORITHM_TIME] == 0xFF) || (flash_data[ALGORITHM_TIME] == 0))
    {
        system_data.algorithm_interval = 8;
    }

    else
    {
        system_data.algorithm_interval = flash_data[ALGORITHM_TIME];
    }

    if (flash_data[BLE_TX_DBM] >= 0x07)
    {
        system_data.ble_tx_dbm = 0;
    }

    else
    {
        switch (flash_data[BLE_TX_DBM])
        {
        case TX_POWER_POS_4DBM:
            system_data.ble_tx_dbm = 4;
            break;

        case TX_POWER_POS_0DBM:
            system_data.ble_tx_dbm = 0;
            break;

        case TX_POWER_NEG_4DBM:
            system_data.ble_tx_dbm = -4;
            break;

        case TX_POWER_NEG_8DBM:
            system_data.ble_tx_dbm = -8;
            break;

        case TX_POWER_NEG_12DBM:
            system_data.ble_tx_dbm = -12;
            break;

        case TX_POWER_NEG_16DBM:
            system_data.ble_tx_dbm = -16;
            break;

        case TX_POWER_NEG_20DBM:
            system_data.ble_tx_dbm = -20;
            break;

        default:
            break;
        }
    }

    if (flash_data[SYSTEM_MODE] >= 0x04)
    {
        system_data.system_mode = NORMAL_MODE;
    }

    else
    {
        system_data.system_mode = flash_data[SYSTEM_MODE];
    }

    system_data.system_mode_backup = system_data.system_mode;

    if (flash_data[AFE_NUMAV] >= 0x10)
    {
        system_data.afe.numav = 3;
    }

    else
    {
        system_data.afe.numav = flash_data[AFE_NUMAV];
    }

    if (flash_data[AFE_DUTY] >= 0x0B)
    {
        system_data.afe.duty = 1;
    }

    else
    {
        system_data.afe.duty = flash_data[AFE_DUTY];
    }

    if (flash_data[AFE_AMBIENT_EN] == 0xFF)
    {
        system_data.afe.ambient_en = true;
    }

    else
    {
        system_data.afe.ambient_en = flash_data[AFE_AMBIENT_EN];
    }

    if (flash_data[AFE_AUTO_ADJUST] == 0xFF)
    {
        system_data.afe.auto_adjust = true;
    }

    else
    {
        system_data.afe.auto_adjust = flash_data[AFE_AUTO_ADJUST];
    }

    for ( i = ALGORITHM_DATA, j = 0; i < ALGORITHM_DATA+ALGORITHM_DATA_LEN; i++, j++)
    {
        temp[j] = flash_data[i];
    }
    memcpy(system_data.algorithm_reserved1_control,temp,sizeof(system_data.algorithm_reserved1_control));
    if((temp[5]==0xFF)&&(temp[6]==0xFF)&&(temp[7]==0xFF)&&(temp[8]==0xFF))//如果没存过修正值，则全部置为0
    {
        memset(&temp[5],0,sizeof(system_data.algorithm_modified_value));
    }
    memcpy(system_data.algorithm_modified_value,&temp[5],sizeof(system_data.algorithm_modified_value));

    if (flash_data[OLED_DISPLAY_TIME] != 0xFF)//获取oled显示熄屏间隔
    {
        set_oled_display_time(flash_data[OLED_DISPLAY_TIME]);
    }
    else
    {
        set_oled_display_time(0x00);//如果之前没有设置，就设置为永不熄屏
    }


    //判断是否加载设备名称前缀
    if (flash_data[BLE_PRE_NAME] == STORE_FLAG) //存储有广播名称前缀
    {
        system_data.ble_name_pre_len = flash_data[BLE_PRE_NAME + 1];
        memset (device_name, 0, BLE_NAME_LEN);
        memcpy (device_name, &flash_data[BLE_PRE_NAME + 2], system_data.ble_name_pre_len);
    }

    else //未存储广播名称前缀
    {
        system_data.ble_name_pre_len = DEFAULT_BLE_NAME_LEN;
    }

    //判断是否加载设备名称后缀
    if (flash_data[BLE_SUF_NAME] == STORE_FLAG) //存储有广播名称
    {
        memset (&device_name[system_data.ble_name_pre_len], 0, 31 - system_data.ble_name_pre_len);
        memcpy (&device_name[system_data.ble_name_pre_len], &flash_data[BLE_SUF_NAME + 1], BLE_SUF_NAME_LEN - 1);
        system_status.ble_name_en = true;
    }
}
#endif
#if (AFE4404_FUNCTION_EN==1)
/*****************************************************************
*功  能：使能afe4404管脚中断
*参  数: 无
*返回值: 无
*****************************************************************/
void afe4404_interrupt_enable (void)
{
    nrf_drv_gpiote_in_event_enable (AFE4404_RDY_PIN, true);//开启afe4404中断
}
/*****************************************************************
*功  能：关闭afe4404管脚中断
*参  数: 无
*返回值: 无
*****************************************************************/
void afe4404_interrupt_disable (void)
{
    nrf_drv_gpiote_in_event_disable (AFE4404_RDY_PIN);//关闭afe4404中断
}
#endif
/*****************************************************************
*功  能：串口中断事件处理函数
*参  数: 无
*返回值: 无
*****************************************************************/
void uart_event_handle (app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint8_t crc_ack = 0;

    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
        UNUSED_VARIABLE (app_uart_get (&data_array[index]));
        index++;

        if (index == BLE_NUS_MAX_DATA_LEN)
        {
            crc_ack = ble_data_check (data_array, index);

            if (crc_ack == CRC_OK && data_array[3] == DEV_ACK_COMMAND)
            {
                switch (data_array[4])
                {
                case WIFI_TCP_CONNECT_ACK:
                    system_status.wifi_tcp_en = true;
                    break;

                case WIFI_TCP_DISCONNECT_ACK:
                    system_status.wifi_tcp_en = false;
                    break;

                default:
                    break;
                }
            }

            index = 0;
        }

        break;

    case APP_UART_COMMUNICATION_ERROR:
        APP_ERROR_HANDLER (p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER (p_event->data.error_code);
        break;

    default:
        break;
    }
}
/*****************************************************************
*功  能：串口初始化
*参  数: 无
*返回值: 无
*****************************************************************/
void uart_init (void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    APP_UART_FIFO_INIT ( &comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_event_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);
    APP_ERROR_CHECK (err_code);
}
/*****************************************************************
*功  能：将十六进制数转换成字符串
*参  数: 无
*返回值: 无
*****************************************************************/
void hex_to_str (uint8_t *pbDest, uint8_t *pbSrc, int nLen)
{
    char ddl, ddh;
    int i;

    for (i = 0; i < nLen; i++)
    {
        ddh = 48 + pbSrc[i] / 16;
        ddl = 48 + pbSrc[i] % 16;

        if (ddh > 57)
        {
            ddh = ddh + 7;
        }

        if (ddl > 57)
        {
            ddl = ddl + 7;
        }

        pbDest[i * 2] = ddh;
        pbDest[i * 2 + 1] = ddl;
    }

    pbDest[nLen * 2] = '\0';
}



