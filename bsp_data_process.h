#ifndef BSP_DATA_PROCESS_H
#define BSP_DATA_PROCESS_H

#include <stdint.h>
#include <stdbool.h>
//���迪�غ�
#define AFE4404_FUNCTION_EN			1	//AFE4404����
#define MPU6500_FUNCTION_EN			1	//MPU6500����
#define WIFI_FUNCTION_EN				0 	//WIFI����
#define FLASH_FUNCTION_EN				1	//FLASH�洢����
#define CALENDER_FUNCTION_EN		1	//��������
#define ADC_FUNCTION_EN 				1	//����ص�������	
#define KEY_FUNCTION_EN					1	//��������
#define ALGORITHM_FUNCTION_EN 	1	//�㷨��ʱ������
#define AS7000_FUNCTION_EN 			1	//AS7000�����ʹ���
#define SHT30_FUNCTION_EN				1	//��������ʪ�ȹ���
#define	OLED_FUNCTION_EN				1	//OLED��ʾ����
#define MLX90615_FUNCTION_EN		1 //�������¼��
#define ENCRYPTION_FUNCTION_EN	0 //ͨ�����ݼӽ��ܹ���
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

#define ALGORITHM_TIME_INTERVAL 		1000 	//�����㷨��ʱ������ʱ��������
#define CALENDER_TIME_INTERVAL			1000	//�����������ܶ�ʱ������ʱ�������ã���λΪms
#define FLASH_STATUS_TIME_INTERVAL 	30 		//�����㷨��ʱ������ʱ��������
#define DEVICE_STATUS_TIME_INTERVAL 100 	//�������úͲ�ѯ�豸������ʱ������ʱ��������
#define KEY_TIME_INTERVAL 					1//50 		//��ⰴ��������ʱ��
#define MPU6500_TIME_INTERVAL			200		//����ģʽ�£�mpu6500�ɼ����ݶ�ʱ���
#define ADV_LED_TIME_INTERVAL 			1000 	//�㲥ָʾ��LED��ʱ����˸���
#define BT_LOGO_FLASH_TIME_INTERVAL		500		//����ģʽ�£�mpu6500�ɼ����ݶ�ʱ���
#define CHARGE_TIME_INTERVAL			300//1000		//���ʱ��ʱ�������ļ��
#define EMERGENCY_TIME_INTERVAL 		1			//�����¼���ʱ���������λ1ms
#define FLASH_TIME_OUT							4000 	//��λΪ30ms

#define KEY_IN			24				//������Ӧ��GPIO
#define CHRG_DECT		9
#define CHRG_STAT		6
#define	VBAT_ADC_ON	    8
#define	MOTOR_PIN	    23
#define VBAT_ADC_PIN    3//��ص��������ţ���ADC��ʼ��ʱֱ�Ӷ���������ͨ������û��ʹ�øú궨�塣
//�ú궨���ڹػ����ų�ʼ��ʱʹ�á�


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

/*���ڵĹܽŶ���*/
#define RX_PIN_NUMBER  25
#define TX_PIN_NUMBER  26
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           false
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//ADC��ز���
#define VBAT_ADC_MAX (int32_t)(4.1/2/4/0.6*4096) //4.1V��Ϊ���磬��2����Ϊ������ֵ��ѹ���裬��4����Ϊ��������Ϊ1/4����0.6�ǲο���ѹΪ0.6V��4096��12λ�ֱ����µ����ֵ����Ӧ0.6V
#define VBAT_ADC_MIN (int32_t)(3.4/2/4/0.6*4096) //3.4V��Ϊ��ȫû��


enum SYSTEM_POWER_STATUS
{
    OPEN_SYSTEM_POWER,
    CLOSE_SYSTEM_POWER,
    POWER_OFF_CHARGE_STOP,
};
//��������
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
    uint16_t time_out;//flash�����ϴ���ʱ
    bool backup_status;
    bool write_cover_status;//д���ݸ���
    bool read_cover_status;	//�ض����ݸ���
    int16_t read_pointer;
    int16_t write_pointer;
    uint16_t logic_pointer;//�߼���ַָ��
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
