#ifndef BSP_DATA_PROCESS_H
#define BSP_DATA_PROCESS_H

#include <stdint.h>
#include <stdbool.h>
//外设开关宏
#define AFE4404_FUNCTION_EN			1	//AFE4404功能
#define MPU6500_FUNCTION_EN			1	//MPU6500功能
#define WIFI_FUNCTION_EN				0 	//WIFI功能
#define FLASH_FUNCTION_EN				1	//FLASH存储功能
#define CALENDER_FUNCTION_EN		1	//日历功能
#define ADC_FUNCTION_EN 				1	//检测电池电量功能	
#define KEY_FUNCTION_EN					1	//按键功能
#define ALGORITHM_FUNCTION_EN 	1	//算法定时器功能
#define AS7000_FUNCTION_EN 			1	//AS7000做心率功能
#define SHT30_FUNCTION_EN				1	//环境的温湿度功能
#define	OLED_FUNCTION_EN				1	//OLED显示功能
#define MLX90615_FUNCTION_EN		1 //人体体温检测
#define ENCRYPTION_FUNCTION_EN	0 //通信数据加解密功能
#define UART_FUNCTION_EN 				0

//#if (AS7000_FUNCTION_EN + SFH7050_FUNCTION_EN) > 1
//#error "AS7000 and SFH7050 can not be turned on at the same time."
//#endif

#if (UART_FUNCTION_EN + WIFI_FUNCTION_EN) > 1
#error "UART TEST and WIFI can not be turned on at the same time."
#endif

#define PREPARE_SLEEP   0x01
#define PREPARE_OPEN    0x03

#define ENTER_SLEEP     0x02

#define ALGORITHM_TIME_INTERVAL 		1000 	//用于算法定时器操作时间间隔设置
#define CALENDER_TIME_INTERVAL			1000	//用于日历功能定时器操作时间间隔设置，单位为ms
#define FLASH_STATUS_TIME_INTERVAL 	30 		//用于算法定时器操作时间间隔设置
#define DEVICE_STATUS_TIME_INTERVAL 100 	//用于设置和查询设备参数定时器操作时间间隔设置
#define KEY_TIME_INTERVAL 					1//50 		//检测按键动作定时器
#define MPU6500_TIME_INTERVAL			200		//常规模式下，mpu6500采集数据定时间隔
#define ADV_LED_TIME_INTERVAL 			1000 	//广播指示灯LED定时器闪烁间隔
#define BT_LOGO_FLASH_TIME_INTERVAL		500		//常规模式下，mpu6500采集数据定时间隔
#define CHARGE_TIME_INTERVAL			300//1000		//充电时定时检测电量的间隔
#define EMERGENCY_TIME_INTERVAL 		1			//紧急事件定时器间隔，单位1ms
#define FLASH_TIME_OUT							4000 	//单位为30ms

#define KEY_IN			24				//按键对应的GPIO
#define CHRG_DECT		9
#define CHRG_STAT		6
#define	VBAT_ADC_ON	    8
#define	MOTOR_PIN	    23
#define VBAT_ADC_PIN    3//电池电量采样脚，在ADC初始化时直接定义了输入通道，并没有使用该宏定义。
//该宏定义在关机引脚初始化时使用。


#define CHRG_DECT_CFG_INPUT			nrf_gpio_cfg_input(CHRG_DECT, NRF_GPIO_PIN_NOPULL)
#define CHRG_STAT_CFG_INPUT			nrf_gpio_cfg_input(CHRG_STAT, NRF_GPIO_PIN_NOPULL)
#define VBAT_ADC_ON_CFG_OUTPUT	nrf_gpio_cfg_output(VBAT_ADC_ON)
#define	VBAT_ADC_ON_ENABLE			nrf_gpio_pin_set(VBAT_ADC_ON)
#define	VBAT_ADC_ON_DISABLE			nrf_gpio_pin_clear(VBAT_ADC_ON)
#define KEY_IN_STATUS				nrf_gpio_pin_read(KEY_IN)
#define CHRG_DECT_STATUS			nrf_gpio_pin_read(CHRG_DECT)
#define CHRG_STAT_STATUS			nrf_gpio_pin_read(CHRG_STAT)
#define MOTOR_PIN_CFG_OUTPUT	    nrf_gpio_cfg_output(MOTOR_PIN)
#define MOTOR_ENABLE	            nrf_gpio_pin_clear(MOTOR_PIN)
#define MOTOR_DISABLE	            nrf_gpio_pin_set(MOTOR_PIN)
#define KEY_PRESS					0
#define KEY_LONG_PRESS_TIME		2000/KEY_TIME_INTERVAL//40
#define CHARGE_COMPLETE		1
#define NOT_CHARGED 			1
#define ADC_SAMPLE_COUNT	10

/*串口的管脚定义*/
#define RX_PIN_NUMBER  25
#define TX_PIN_NUMBER  26
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           false
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//ADC电池采样
#define VBAT_ADC_MAX (int32_t)(4.1/2/4/0.6*4096) //4.1V认为满电，除2是因为两个等值分压电阻，除4是因为输入增益为1/4，除0.6是参考电压为0.6V，4096是12位分辨率下的最大值，对应0.6V
#define VBAT_ADC_MIN (int32_t)(3.4/2/4/0.6*4096) //3.4V认为完全没电


enum SYSTEM_POWER_STATUS
{
    OPEN_SYSTEM_POWER,
    CLOSE_SYSTEM_POWER,
    POWER_OFF_CHARGE_STOP,
};
//按键类型
enum KEY_PRESS_TYPE
{
    KEY_NO_PRESS,
    KEY_LONG_PRESS,
    KEY_SIGNAL_PRESS,
    KEY_DOUBLE_PRESS,
};

typedef struct
{
    uint8_t data_temp[256];
    uint8_t data_backup[256];
    uint16_t temp_count;
    uint16_t temp_count_backup;
    uint16_t time_out;//flash数据上传超时
    bool backup_status;
    bool write_cover_status;//写数据覆盖
    bool read_cover_status;	//回读数据覆盖
    int16_t read_pointer;
    int16_t write_pointer;
    uint16_t logic_pointer;//逻辑地址指针
    //uint16_t send_count;
} flash_date_type;

void gpiote_init (void);
void timers_init (void);
void afe4404_interrupt_process (void);
void algorithm_interrupt_process (void);
void mpu6500_interrupt_process (void);
void flash_upload_process (void);
void device_operator_process (void);
void power_operator_process (void);
void algorithm_timer_start (void);
void calender_timer_start (void);
void calender_timer_stop (void);
void algorithm_timer_stop(void);
void flash_timer_start (void);
void flash_timer_stop (void);
void key_timer_start (void);
void key_timer_stop (void);
void mpu6500_timer_start (void);
void mpu6500_timer_stop (void);
void uart_send(uint8_t *tx_buff, uint16_t length);
void afe4404_interrupt_enable (void);
void afe4404_interrupt_disable (void);
void system_parameter_init(void);
void vbat_power_init (void);
uint8_t get_vbat_power (void);
void uart_init (void);
void hex_to_str (uint8_t *pbDest, uint8_t *pbSrc, int nLen);
void oled_dis_vbat_level (void);
void motor_timer_start (uint16_t time);
void motor_timer_stop (void);
void charge_timer_start (uint16_t timer_interval);
void charge_timer_stop (void);
void charge_status_detection(void);
void calender_start(void);
void power_handle(void);
#endif
