/* generated HAL source file - do not edit */
#include "hal_data.h"
dtc_instance_ctrl_t bt_transfer1_ctrl;

#if (1 == 1)
transfer_info_t bt_transfer1_info DTC_TRANSFER_INFO_ALIGNMENT =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_DESTINATION,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_1_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = (uint16_t) 0,
    .length                                  = (uint16_t) 0,
};

#elif (1 > 1)
/* User is responsible to initialize the array. */
transfer_info_t bt_transfer1_info[1] DTC_TRANSFER_INFO_ALIGNMENT;
#else
/* User must call api::reconfigure before enable DTC transfer. */
#endif

const dtc_extended_cfg_t bt_transfer1_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SCI4_RXI,
};

const transfer_cfg_t bt_transfer1_cfg =
{
#if (1 == 1)
    .p_info              = &bt_transfer1_info,
#elif (1 > 1)
    .p_info              = bt_transfer1_info,
#else
    .p_info = NULL,
#endif
    .p_extend            = &bt_transfer1_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t bt_transfer1 =
{
    .p_ctrl        = &bt_transfer1_ctrl,
    .p_cfg         = &bt_transfer1_cfg,
    .p_api         = &g_transfer_on_dtc
};
dtc_instance_ctrl_t bt_transfer0_ctrl;

#if (1 == 1)
transfer_info_t bt_transfer0_info DTC_TRANSFER_INFO_ALIGNMENT =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_SOURCE,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_1_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = (uint16_t) 0,
    .length                                  = (uint16_t) 0,
};

#elif (1 > 1)
/* User is responsible to initialize the array. */
transfer_info_t bt_transfer0_info[1] DTC_TRANSFER_INFO_ALIGNMENT;
#else
/* User must call api::reconfigure before enable DTC transfer. */
#endif

const dtc_extended_cfg_t bt_transfer0_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SCI4_TXI,
};

const transfer_cfg_t bt_transfer0_cfg =
{
#if (1 == 1)
    .p_info              = &bt_transfer0_info,
#elif (1 > 1)
    .p_info              = bt_transfer0_info,
#else
    .p_info = NULL,
#endif
    .p_extend            = &bt_transfer0_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t bt_transfer0 =
{
    .p_ctrl        = &bt_transfer0_ctrl,
    .p_cfg         = &bt_transfer0_cfg,
    .p_api         = &g_transfer_on_dtc
};
sci_uart_instance_ctrl_t     bt_uart_ctrl;

            baud_setting_t               bt_uart_baud_setting =
            {
                /* Baud rate calculated with 0.469% error. */ .semr_baudrate_bits_b.abcse = 0, .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme = false
            };

            /** UART extended configuration for UARTonSCI HAL driver */
            const sci_uart_extended_cfg_t bt_uart_cfg_extend =
            {
                .clock                = SCI_UART_CLOCK_INT,
                .rx_edge_start          = SCI_UART_START_BIT_FALLING_EDGE,
                .noise_cancel         = SCI_UART_NOISE_CANCELLATION_DISABLE,
                .rx_fifo_trigger        = SCI_UART_RX_FIFO_TRIGGER_MAX,
                .p_baud_setting         = &bt_uart_baud_setting,
                .flow_control           = SCI_UART_FLOW_CONTROL_RTS,
                #if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                .flow_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                .rs485_setting = {
                    .enable = SCI_UART_RS485_DISABLE,
                    .polarity = SCI_UART_RS485_DE_POLARITY_HIGH,
                #if 0xFF != 0xFF
                    .de_control_pin = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                    .de_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                },
                .irda_setting = {
                    .ircr_bits_b.ire = 0,
                    .ircr_bits_b.irrxinv = 0,
                    .ircr_bits_b.irtxinv = 0,
                },
            };

            /** UART interface configuration */
            const uart_cfg_t bt_uart_cfg =
            {
                .channel             = 4,
                .data_bits           = UART_DATA_BITS_8,
                .parity              = UART_PARITY_OFF,
                .stop_bits           = UART_STOP_BITS_1,
                .p_callback          = UART_Global_Callback,
                .p_context           = NULL,
                .p_extend            = &bt_uart_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == bt_transfer0)
                .p_transfer_tx       = NULL,
#else
                .p_transfer_tx       = &bt_transfer0,
#endif
#if (RA_NOT_DEFINED == bt_transfer1)
                .p_transfer_rx       = NULL,
#else
                .p_transfer_rx       = &bt_transfer1,
#endif
#undef RA_NOT_DEFINED
                .rxi_ipl             = (5),
                .txi_ipl             = (5),
                .tei_ipl             = (12),
                .eri_ipl             = (12),
#if defined(VECTOR_NUMBER_SCI4_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI4_RXI,
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI4_TXI,
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI4_TEI,
#else
                .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI4_ERI,
#else
                .eri_irq             = FSP_INVALID_VECTOR,
#endif
            };

/* Instance structure to use this module. */
const uart_instance_t bt_uart =
{
    .p_ctrl        = &bt_uart_ctrl,
    .p_cfg         = &bt_uart_cfg,
    .p_api         = &g_uart_on_sci
};
usb_instance_ctrl_t usb_pcdc_basic_ctrl;

#if !defined(g_usb_descriptor)
extern usb_descriptor_t g_usb_descriptor;
#endif
#define RA_NOT_DEFINED (1)
            const usb_cfg_t usb_pcdc_basic_cfg =
            {
                .usb_mode  = USB_MODE_PERI,
                .usb_speed = USB_SPEED_FS,
                .module_number = 0,
                .type = USB_CLASS_PCDC,
#if defined(g_usb_descriptor)
                .p_usb_reg = g_usb_descriptor,
#else
                .p_usb_reg = &g_usb_descriptor,
#endif
                .usb_complience_cb = NULL,
#if defined(VECTOR_NUMBER_USBFS_INT)
                .irq       = VECTOR_NUMBER_USBFS_INT,
#else
                .irq       = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBFS_RESUME)
                .irq_r     = VECTOR_NUMBER_USBFS_RESUME,
#else
                .irq_r     = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBFS_FIFO_0)
                .irq_d0    = VECTOR_NUMBER_USBFS_FIFO_0,
#else
                .irq_d0    = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBFS_FIFO_1)
                .irq_d1    = VECTOR_NUMBER_USBFS_FIFO_1,
#else
                .irq_d1    = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBHS_USB_INT_RESUME)
                .hsirq     = VECTOR_NUMBER_USBHS_USB_INT_RESUME,
#else
                .hsirq     = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBHS_FIFO_0)
                .hsirq_d0  = VECTOR_NUMBER_USBHS_FIFO_0,
#else
                .hsirq_d0  = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_USBHS_FIFO_1)
                .hsirq_d1  = VECTOR_NUMBER_USBHS_FIFO_1,
#else
                .hsirq_d1  = FSP_INVALID_VECTOR,
#endif
                .ipl       = (6),
                .ipl_r     = (6),
                .ipl_d0    = (6),
                .ipl_d1    = (6),
                .hsipl     = (BSP_IRQ_DISABLED),
                .hsipl_d0  = (BSP_IRQ_DISABLED),
                .hsipl_d1  = (BSP_IRQ_DISABLED),
#if (BSP_CFG_RTOS == 0) && defined(USB_CFG_HMSC_USE)
                .p_usb_apl_callback = NULL,
#else
                .p_usb_apl_callback = usb_cdc_callback,
#endif
#if defined(NULL)
                .p_context = NULL,
#else
                .p_context = (void *) &NULL,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
#else
                .p_transfer_tx = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
#else
                .p_transfer_rx = &RA_NOT_DEFINED,
#endif
            };
#undef RA_NOT_DEFINED

/* Instance structure to use this module. */
const usb_instance_t usb_pcdc_basic =
{
    .p_ctrl        = &usb_pcdc_basic_ctrl,
    .p_cfg         = &usb_pcdc_basic_cfg,
    .p_api         = &g_usb_on_usb,
};

dtc_instance_ctrl_t wifi_dtc_transfer0_ctrl;

#if (1 == 1)
transfer_info_t wifi_dtc_transfer0_info DTC_TRANSFER_INFO_ALIGNMENT =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_SOURCE,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_1_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = (uint16_t) 0,
    .length                                  = (uint16_t) 0,
};

#elif (1 > 1)
/* User is responsible to initialize the array. */
transfer_info_t wifi_dtc_transfer0_info[1] DTC_TRANSFER_INFO_ALIGNMENT;
#else
/* User must call api::reconfigure before enable DTC transfer. */
#endif

const dtc_extended_cfg_t wifi_dtc_transfer0_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SCI6_TXI,
};

const transfer_cfg_t wifi_dtc_transfer0_cfg =
{
#if (1 == 1)
    .p_info              = &wifi_dtc_transfer0_info,
#elif (1 > 1)
    .p_info              = wifi_dtc_transfer0_info,
#else
    .p_info = NULL,
#endif
    .p_extend            = &wifi_dtc_transfer0_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t wifi_dtc_transfer0 =
{
    .p_ctrl        = &wifi_dtc_transfer0_ctrl,
    .p_cfg         = &wifi_dtc_transfer0_cfg,
    .p_api         = &g_transfer_on_dtc
};
sci_uart_instance_ctrl_t     wifi_uart_ctrl;

            baud_setting_t               wifi_uart_baud_setting =
            {
                /* Baud rate calculated with 0.469% error. */ .semr_baudrate_bits_b.abcse = 0, .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme = false
            };

            /** UART extended configuration for UARTonSCI HAL driver */
            const sci_uart_extended_cfg_t wifi_uart_cfg_extend =
            {
                .clock                = SCI_UART_CLOCK_INT,
                .rx_edge_start          = SCI_UART_START_BIT_FALLING_EDGE,
                .noise_cancel         = SCI_UART_NOISE_CANCELLATION_DISABLE,
                .rx_fifo_trigger        = SCI_UART_RX_FIFO_TRIGGER_MAX,
                .p_baud_setting         = &wifi_uart_baud_setting,
                .flow_control           = SCI_UART_FLOW_CONTROL_RTS,
                #if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                .flow_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                .rs485_setting = {
                    .enable = SCI_UART_RS485_DISABLE,
                    .polarity = SCI_UART_RS485_DE_POLARITY_HIGH,
                #if 0xFF != 0xFF
                    .de_control_pin = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                    .de_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                },
                .irda_setting = {
                    .ircr_bits_b.ire = 0,
                    .ircr_bits_b.irrxinv = 0,
                    .ircr_bits_b.irtxinv = 0,
                },
            };

            /** UART interface configuration */
            const uart_cfg_t wifi_uart_cfg =
            {
                .channel             = 6,
                .data_bits           = UART_DATA_BITS_8,
                .parity              = UART_PARITY_OFF,
                .stop_bits           = UART_STOP_BITS_1,
                .p_callback          = UART_Global_Callback,
                .p_context           = NULL,
                .p_extend            = &wifi_uart_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == wifi_dtc_transfer0)
                .p_transfer_tx       = NULL,
#else
                .p_transfer_tx       = &wifi_dtc_transfer0,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
                .p_transfer_rx       = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
                .rxi_ipl             = (5),
                .txi_ipl             = (5),
                .tei_ipl             = (12),
                .eri_ipl             = (12),
#if defined(VECTOR_NUMBER_SCI6_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI6_RXI,
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI6_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI6_TXI,
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI6_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI6_TEI,
#else
                .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI6_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI6_ERI,
#else
                .eri_irq             = FSP_INVALID_VECTOR,
#endif
            };

/* Instance structure to use this module. */
const uart_instance_t wifi_uart =
{
    .p_ctrl        = &wifi_uart_ctrl,
    .p_cfg         = &wifi_uart_cfg,
    .p_api         = &g_uart_on_sci
};
dtc_instance_ctrl_t pc_transfer0_ctrl;

#if (1 == 1)
transfer_info_t pc_transfer0_info DTC_TRANSFER_INFO_ALIGNMENT =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_DESTINATION,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_1_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = (uint16_t) 0,
    .length                                  = (uint16_t) 0,
};

#elif (1 > 1)
/* User is responsible to initialize the array. */
transfer_info_t pc_transfer0_info[1] DTC_TRANSFER_INFO_ALIGNMENT;
#else
/* User must call api::reconfigure before enable DTC transfer. */
#endif

const dtc_extended_cfg_t pc_transfer0_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SCI5_RXI,
};

const transfer_cfg_t pc_transfer0_cfg =
{
#if (1 == 1)
    .p_info              = &pc_transfer0_info,
#elif (1 > 1)
    .p_info              = pc_transfer0_info,
#else
    .p_info = NULL,
#endif
    .p_extend            = &pc_transfer0_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t pc_transfer0 =
{
    .p_ctrl        = &pc_transfer0_ctrl,
    .p_cfg         = &pc_transfer0_cfg,
    .p_api         = &g_transfer_on_dtc
};
dtc_instance_ctrl_t pc_transfer1_ctrl;

#if (1 == 1)
transfer_info_t pc_transfer1_info DTC_TRANSFER_INFO_ALIGNMENT =
{
    .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
    .transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_SOURCE,
    .transfer_settings_word_b.irq            = TRANSFER_IRQ_END,
    .transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED,
    .transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_INCREMENTED,
    .transfer_settings_word_b.size           = TRANSFER_SIZE_1_BYTE,
    .transfer_settings_word_b.mode           = TRANSFER_MODE_NORMAL,
    .p_dest                                  = (void *) NULL,
    .p_src                                   = (void const *) NULL,
    .num_blocks                              = (uint16_t) 0,
    .length                                  = (uint16_t) 0,
};

#elif (1 > 1)
/* User is responsible to initialize the array. */
transfer_info_t pc_transfer1_info[1] DTC_TRANSFER_INFO_ALIGNMENT;
#else
/* User must call api::reconfigure before enable DTC transfer. */
#endif

const dtc_extended_cfg_t pc_transfer1_cfg_extend =
{
    .activation_source   = VECTOR_NUMBER_SCI5_TXI,
};

const transfer_cfg_t pc_transfer1_cfg =
{
#if (1 == 1)
    .p_info              = &pc_transfer1_info,
#elif (1 > 1)
    .p_info              = pc_transfer1_info,
#else
    .p_info = NULL,
#endif
    .p_extend            = &pc_transfer1_cfg_extend,
};

/* Instance structure to use this module. */
const transfer_instance_t pc_transfer1 =
{
    .p_ctrl        = &pc_transfer1_ctrl,
    .p_cfg         = &pc_transfer1_cfg,
    .p_api         = &g_transfer_on_dtc
};
sci_uart_instance_ctrl_t     pc_uart_ctrl;

            baud_setting_t               pc_uart_baud_setting =
            {
                /* Baud rate calculated with 0.469% error. */ .semr_baudrate_bits_b.abcse = 0, .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme = false
            };

            /** UART extended configuration for UARTonSCI HAL driver */
            const sci_uart_extended_cfg_t pc_uart_cfg_extend =
            {
                .clock                = SCI_UART_CLOCK_INT,
                .rx_edge_start          = SCI_UART_START_BIT_FALLING_EDGE,
                .noise_cancel         = SCI_UART_NOISE_CANCELLATION_DISABLE,
                .rx_fifo_trigger        = SCI_UART_RX_FIFO_TRIGGER_MAX,
                .p_baud_setting         = &pc_uart_baud_setting,
                .flow_control           = SCI_UART_FLOW_CONTROL_RTS,
                #if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                .flow_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                .rs485_setting = {
                    .enable = SCI_UART_RS485_DISABLE,
                    .polarity = SCI_UART_RS485_DE_POLARITY_HIGH,
                #if 0xFF != 0xFF
                    .de_control_pin = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                    .de_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                },
                .irda_setting = {
                    .ircr_bits_b.ire = 0,
                    .ircr_bits_b.irrxinv = 0,
                    .ircr_bits_b.irtxinv = 0,
                },
            };

            /** UART interface configuration */
            const uart_cfg_t pc_uart_cfg =
            {
                .channel             = 5,
                .data_bits           = UART_DATA_BITS_8,
                .parity              = UART_PARITY_OFF,
                .stop_bits           = UART_STOP_BITS_1,
                .p_callback          = UART_Global_Callback,
                .p_context           = NULL,
                .p_extend            = &pc_uart_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == pc_transfer1)
                .p_transfer_tx       = NULL,
#else
                .p_transfer_tx       = &pc_transfer1,
#endif
#if (RA_NOT_DEFINED == pc_transfer0)
                .p_transfer_rx       = NULL,
#else
                .p_transfer_rx       = &pc_transfer0,
#endif
#undef RA_NOT_DEFINED
                .rxi_ipl             = (5),
                .txi_ipl             = (5),
                .tei_ipl             = (5),
                .eri_ipl             = (5),
#if defined(VECTOR_NUMBER_SCI5_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI5_RXI,
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI5_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI5_TXI,
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI5_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI5_TEI,
#else
                .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI5_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI5_ERI,
#else
                .eri_irq             = FSP_INVALID_VECTOR,
#endif
            };

/* Instance structure to use this module. */
const uart_instance_t pc_uart =
{
    .p_ctrl        = &pc_uart_ctrl,
    .p_cfg         = &pc_uart_cfg,
    .p_api         = &g_uart_on_sci
};
/* Nominal and Data bit timing configuration */

can_bit_timing_cfg_t motor_ctrl_can_bit_timing_cfg =
{
    /* Actual bitrate: 1000000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 29,
    .time_segment_2 = 10,
    .synchronization_jump_width = 4
};

#if BSP_FEATURE_CANFD_FD_SUPPORT
can_bit_timing_cfg_t motor_ctrl_can_data_timing_cfg =
{
    /* Actual bitrate: 2000000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 14,
    .time_segment_2 = 5,
    .synchronization_jump_width = 1
};
#endif


extern const canfd_afl_entry_t can_afl[CANFD_CFG_AFL_CH0_RULE_NUM];
#ifndef CANFD_PRV_GLOBAL_CFG
#define CANFD_PRV_GLOBAL_CFG

#ifdef RA_NOT_DEFINED
#undef RA_NOT_DEFINED
#endif
#define RA_NOT_DEFINED (0)

/* Buffer RAM used: RA_NOT_DEFINED bytes */
canfd_global_cfg_t g_canfd_global_cfg =
{
    .global_interrupts = CANFD_CFG_GLOBAL_ERR_SOURCES,
    .global_config     = (CANFD_CFG_TX_PRIORITY | CANFD_CFG_DLC_CHECK | (BSP_CFG_CANFDCLK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC ? R_CANFD_CFDGCFG_DCS_Msk : 0U) | CANFD_CFG_FD_OVERFLOW |
                          ((RA_NOT_DEFINED) << R_CANFD_CFDGCFG_ITRCP_Pos)),
    .rx_mb_config      = (CANFD_CFG_RXMB_NUMBER | (CANFD_CFG_RXMB_SIZE << R_CANFD_CFDRMNB_RMPLS_Pos)),
    .global_err_ipl = CANFD_CFG_GLOBAL_ERR_IPL,
    .rx_fifo_ipl    = CANFD_CFG_RX_FIFO_IPL,
    .rx_fifo_config    =
    {
        ((CANFD_CFG_RXFIFO0_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO0_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO0_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO0_INT_MODE) | (CANFD_CFG_RXFIFO0_ENABLE)),
        ((CANFD_CFG_RXFIFO1_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO1_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO1_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO1_INT_MODE) | (CANFD_CFG_RXFIFO1_ENABLE)),
#if !BSP_FEATURE_CANFD_LITE
        ((CANFD_CFG_RXFIFO2_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO2_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO2_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO2_INT_MODE) | (CANFD_CFG_RXFIFO2_ENABLE)),
        ((CANFD_CFG_RXFIFO3_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO3_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO3_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO3_INT_MODE) | (CANFD_CFG_RXFIFO3_ENABLE)),
        ((CANFD_CFG_RXFIFO4_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO4_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO4_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO4_INT_MODE) | (CANFD_CFG_RXFIFO4_ENABLE)),
        ((CANFD_CFG_RXFIFO5_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO5_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO5_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO5_INT_MODE) | (CANFD_CFG_RXFIFO5_ENABLE)),
        ((CANFD_CFG_RXFIFO6_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO6_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO6_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO6_INT_MODE) | (CANFD_CFG_RXFIFO6_ENABLE)),
        ((CANFD_CFG_RXFIFO7_INT_THRESHOLD << R_CANFD_CFDRFCC_RFIGCV_Pos) | (CANFD_CFG_RXFIFO7_DEPTH << R_CANFD_CFDRFCC_RFDC_Pos) | (CANFD_CFG_RXFIFO7_PAYLOAD << R_CANFD_CFDRFCC_RFPLS_Pos) | (CANFD_CFG_RXFIFO7_INT_MODE) | (CANFD_CFG_RXFIFO7_ENABLE)),
#endif
    },
    .common_fifo_config =
    {
        CANFD_CFG_COMMONFIFO0,
#if !BSP_FEATURE_CANFD_LITE
        CANFD_CFG_COMMONFIFO1,
        CANFD_CFG_COMMONFIFO2,
        CANFD_CFG_COMMONFIFO3,
        CANFD_CFG_COMMONFIFO4,
        CANFD_CFG_COMMONFIFO5,
#endif
    }
};
#undef RA_NOT_DEFINED

#endif

canfd_extended_cfg_t motor_ctrl_can_extended_cfg =
{
    .p_afl              = can_afl,
    .txmb_txi_enable    = ((1ULL << 0) |  0ULL),
    .error_interrupts   = (R_CANFD_CFDC_CTR_EWIE_Msk | R_CANFD_CFDC_CTR_EPIE_Msk | R_CANFD_CFDC_CTR_BOEIE_Msk | R_CANFD_CFDC_CTR_BORIE_Msk | R_CANFD_CFDC_CTR_OLIE_Msk |  0U),
#if BSP_FEATURE_CANFD_FD_SUPPORT
    .p_data_timing      = &motor_ctrl_can_data_timing_cfg,
#else
    .p_data_timing      = NULL,
#endif
    .delay_compensation = (1),
    .p_global_cfg       = &g_canfd_global_cfg,
};

canfd_instance_ctrl_t motor_ctrl_can_ctrl;
const can_cfg_t motor_ctrl_can_cfg =
{
    .channel                = 0,
    .p_bit_timing           = &motor_ctrl_can_bit_timing_cfg,
    .p_callback             = can_callback,
    .p_extend               = &motor_ctrl_can_extended_cfg,
    .p_context              = NULL,
    .ipl                    = (1),
#if defined(VECTOR_NUMBER_CAN0_COMFRX)
    .rx_irq             = VECTOR_NUMBER_CAN0_COMFRX,
#else
    .rx_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_TX)
    .tx_irq             = VECTOR_NUMBER_CAN0_TX,
#else
    .tx_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_CHERR)
    .error_irq             = VECTOR_NUMBER_CAN0_CHERR,
#else
    .error_irq             = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const can_instance_t motor_ctrl_can =
{
    .p_ctrl        = &motor_ctrl_can_ctrl,
    .p_cfg         = &motor_ctrl_can_cfg,
    .p_api         = &g_canfd_on_canfd
};
void g_hal_init(void) {
g_common_init();
}
