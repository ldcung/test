/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "device_manager.h"
#include "pstorage.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "nrf_drv_twi.h"
#include "afe4404.h"
#include "mpu6500.h"
#include "flash.h"
#include "mlx90615.h"
#include "sht30.h"
#include "oled.h"
#include "calender.h"
#include "bsp_data_process.h"
#include "ble_data_process.h"
#include "as7000.h"
#include "parameter_pstorage.h"
#include "SEGGER_RTT.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                160                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT (IS_SRVC_CHANGED_CHARACT_PRESENT);                                    /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
			.rc_ctiv       = 0,                                \
											 .rc_temp_ctiv  = 0,                                \
																				.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT


/***********************用户自定义变量声明区域*********************/
//#if (CALENDER_FUNCTION_EN==1)
ble_date_time_t calender = {2015, 7, 8, 19, 47, 30};//2015-7-7 19:47:30
uint32_t calender_count, calender_number;//用于nRF51822日历功能
//#endif
system_data_type system_data;
system_status_type	system_status;
system_operator_type system_operator;
algorithm_data_type algorithm_data = {0};
//hrm_data_tpye hrm_data;
uint8_t device_name[BLE_NAME_LEN] = {'C', 'I', 'M', '-', 'W', 'B', '1', '-'};
/*****************************************************************/
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback (uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler (DEAD_BEEF, line_num, p_file_name);
}

/**********************************************
*功  能：设置52832的广播名称（CIM-AB1-蓝牙mac地址后4个字符）
*参  数: 无
*返回值: 无
**********************************************/
/**********************************************
*功  能：设置52832的广播名称（CIM-AB1-蓝牙mac地址后4个字符）
*参  数: 无
*返回值: 无
**********************************************/
void set_52832_ble_name (void)
{
    if (!system_status.ble_name_en)
    {
        uint32_t err_code;
        ble_gap_addr_t mac_addr;
        uint8_t temp_mac_addr[6];
        uint8_t mac_addr_char[12];
        err_code = sd_ble_gap_address_get (&mac_addr);

        if (err_code == NRF_SUCCESS)
        {
            for (uint8_t i = 0; i < 6; i++)
            {
                temp_mac_addr[i] = mac_addr.addr[5 - i];
            }

            hex_to_str (mac_addr_char, temp_mac_addr, sizeof (temp_mac_addr));

            for (uint8_t i = 0; i < 4; i++)
            {
                device_name[system_data.ble_name_pre_len + i] = mac_addr_char[8 + i];
            }
        }
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init (void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    set_52832_ble_name();
    BLE_GAP_CONN_SEC_MODE_SET_OPEN (&sec_mode);
    err_code = sd_ble_gap_device_name_set (&sec_mode, device_name, sizeof (device_name));
    APP_ERROR_CHECK (err_code);
    memset (&gap_conn_params, 0, sizeof (gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    err_code = sd_ble_gap_ppcp_set (&gap_conn_params);
    if(err_code != NRF_SUCCESS)
    {
#if (UART_FUNCTION_EN==1)
        printf ("err_code1=%d\n", err_code);
#endif
    }
    APP_ERROR_CHECK (err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler (ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    ble_data_process (p_data, length);
}



#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop (void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK (err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load (dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;
    context.len    = sizeof (context_data);
    context.p_data = (uint8_t *)&context_data;
    err_code = dm_application_context_get (p_handle, &context);

    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed (m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);

            if ((err_code != NRF_SUCCESS) &&
                    (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                    (err_code != NRF_ERROR_INVALID_STATE) &&
                    (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                    (err_code != NRF_ERROR_BUSY) &&
                    (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER (err_code);
            }
        }

        err_code = dm_application_context_delete (p_handle);
        APP_ERROR_CHECK (err_code);
    }

    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER (err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
void reset_prepare (void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect (m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK (err_code);
    }

    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK (err_code);
    nrf_delay_ms (500);
}
#endif // BLE_DFU_APP_SUPPORT



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init (void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    memset (&nus_init, 0, sizeof (nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init (&m_nus, &nus_init);
    if(err_code != NRF_SUCCESS)
    {
#if (UART_FUNCTION_EN==1)
        printf ("err_code1=%d\n", err_code);
#endif
    }
    APP_ERROR_CHECK (err_code);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;
    // Initialize the Device Firmware Update Service.
    memset (&dfus_init, 0, sizeof (dfus_init));
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;
    err_code = ble_dfu_init (&m_dfus, &dfus_init);
    if(err_code != NRF_SUCCESS)
    {
#if (UART_FUNCTION_EN==1)
        printf ("err_code=%d\n", err_code);
#endif
    }
    APP_ERROR_CHECK (err_code);
    dfu_app_reset_prepare_set (reset_prepare);
    dfu_app_dm_appl_instance_set (m_app_handle);
#endif // BLE_DFU_APP_SUPPORT
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt (ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect (m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK (err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler (uint32_t nrf_error)
{
    APP_ERROR_HANDLER (nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init (void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    memset (&cp_init, 0, sizeof (cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    err_code = ble_conn_params_init (&cp_init);
    if(err_code != NRF_SUCCESS)
    {
#if (UART_FUNCTION_EN==1)
        printf ("err_code1=%d\n", err_code);
#endif
    }
    APP_ERROR_CHECK (err_code);
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt (ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        system_status.ble_connect = true;
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        system_status.ble_connect = false;
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply (m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK (err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set (m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK (err_code);
        break;

    case BLE_GAP_EVT_TIMEOUT:
        err_code = ble_advertising_start (BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK (err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch (ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler (p_ble_evt);
    ble_nus_on_ble_evt (&m_nus, p_ble_evt);
    ble_conn_params_on_ble_evt (p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    ble_dfu_on_ble_evt (&m_dfus, p_ble_evt);
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt (p_ble_evt);
    ble_advertising_on_ble_evt (p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch (uint32_t sys_evt)
{
    pstorage_sys_event_handler (sys_evt);
    ble_advertising_on_sys_evt (sys_evt);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init (void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT (&clock_lf_cfg, NULL);
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config (CENTRAL_LINK_COUNT,
               PERIPHERAL_LINK_COUNT,
               &ble_enable_params);
    APP_ERROR_CHECK (err_code);
#ifdef BLE_DFU_APP_SUPPORT
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif // BLE_DFU_APP_SUPPORT
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR (CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable (&ble_enable_params);
    APP_ERROR_CHECK (err_code);
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set (ble_evt_dispatch);
    APP_ERROR_CHECK (err_code);
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set (sys_evt_dispatch);
    APP_ERROR_CHECK (err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler (dm_handle_t const * p_handle,
        dm_event_t const  * p_event,
        ret_code_t        event_result)
{
    APP_ERROR_CHECK (event_result);
#ifdef BLE_DFU_APP_SUPPORT

    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load (p_handle);
    }

#endif // BLE_DFU_APP_SUPPORT
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init (bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK (err_code);
    err_code = dm_init (&init_param);
    APP_ERROR_CHECK (err_code);
    memset (&register_param.sec_param, 0, sizeof (ble_gap_sec_params_t));
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
    err_code = dm_register (&m_app_handle, &register_param);

    APP_ERROR_CHECK (err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init (void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    ble_advdata_manuf_data_t	company_data;
    ble_gap_addr_t	mac_addr;
    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset (&advdata, 0, sizeof (advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    memset (&mac_addr, 0, sizeof (mac_addr));
    sd_ble_gap_address_get (&mac_addr);
    memset (&company_data, 0, sizeof (company_data));
    company_data.company_identifier = 0x7480;
    company_data.data.p_data = (uint8_t *)mac_addr.addr;
    company_data.data.size = 6;
    advdata.p_manuf_specific_data = &company_data;
    memset (&scanrsp, 0, sizeof (scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof (m_adv_uuids) / sizeof (m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
    err_code = ble_advertising_init (&advdata, &scanrsp, &options, NULL, NULL);
    if(err_code != NRF_SUCCESS)
    {
#if (UART_FUNCTION_EN==1)
        printf ("err_code1=%d\n", err_code);
#endif
    }
    APP_ERROR_CHECK (err_code);
}

/*****************************************************************
 *功  能：motor初始化
 *参  数：无
 *返回值：
 *****************************************************************/
void motor_init(void)
{
    MOTOR_PIN_CFG_OUTPUT;
    MOTOR_DISABLE;
}
/*****************************************************************
 *功  能：广播数据初始化
 *参  数：无
 *返回值：
 *****************************************************************/
void advertising_para_init(void)
{
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
	aaaaaaaa
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage (void)
{
    //释放FPU资源
    #define FPU_EXCEPTION_MASK 0x0000009F
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));      
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK (err_code);
}

/**@brief Application main function.
 */
int main (void)
{
    bool erase_bonds = false;
    // Initialize.
    APP_TIMER_INIT (APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
   
    timers_init();
#if (UART_FUNCTION_EN==1)
    uart_init();
    printf("start..\n");
#endif
    motor_init();
#if ((FLASH_FUNCTION_EN==1) || (OLED_FUNCTION_EN==1)||(WIFI_FUNCTION_EN==1))
    spi_init();
    nrf_delay_ms(10);
#endif
#if (FLASH_FUNCTION_EN==1)
    system_parameter_init();
#else
    system_data.algorithm_interval = 5;
#endif
    SEGGER_RTT_printf(0,"system_data.algorithm_interval=%d\n",system_data.algorithm_interval);
    ble_stack_init();
    device_manager_init (erase_bonds);
    advertising_para_init();
    calender_start();
#if (ADC_FUNCTION_EN==1)
    vbat_power_init();//如果ADC初始化之后，再次进行初始化，则会造成系统重启
#endif
    system_status.display_bat_logo = true;//立即进行一次电池图标的显示
    system_data.power_switch = true;
    system_status.power = true;
    system_operator.power = OPEN_SYSTEM_POWER;
    SEGGER_RTT_printf(0, "start..\n");
    for (;;)
    {
        /*设备开关机处理*/
        power_operator_process();
#if (AFE4404_FUNCTION_EN==1)
        /*4404中断处理*/
        afe4404_interrupt_process();
#endif
#if (AS7000_FUNCTION_EN==1)
#if (MPU6500_FUNCTION_EN==1)
        mpu6500_interrupt_process();
#endif
#endif
#if (ALGORITHM_FUNCTION_EN==1)
        /*算法定时器中断处理*/
        algorithm_interrupt_process();
#endif
#if (FLASH_FUNCTION_EN==1)
        /*flash数据定时器中断处理*/
        flash_upload_process();
#endif
#if (OLED_FUNCTION_EN==1)
        /*flash数据定时器中断处理*/
        oled_display_process();
#endif

        /*设备参数查询和设置处理*/
        device_operator_process();
        power_manage();
    }
}


/**
 * @}
 */
