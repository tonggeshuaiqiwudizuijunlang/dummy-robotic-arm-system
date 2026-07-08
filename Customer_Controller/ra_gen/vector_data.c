/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = canfd_error_isr, /* CAN0 CHERR (Channel  error) */
            [1] = canfd_channel_tx_isr, /* CAN0 TX (Transmit interrupt) */
            [2] = canfd_common_fifo_rx_isr, /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [3] = canfd_error_isr, /* CAN GLERR (Global error) */
            [4] = canfd_rx_fifo_isr, /* CAN RXF (Global receive FIFO interrupt) */
            [5] = sci_uart_rxi_isr, /* SCI5 RXI (Receive data full) */
            [6] = sci_uart_txi_isr, /* SCI5 TXI (Transmit data empty) */
            [7] = sci_uart_tei_isr, /* SCI5 TEI (Transmit end) */
            [8] = sci_uart_eri_isr, /* SCI5 ERI (Receive error) */
            [9] = sci_uart_rxi_isr, /* SCI6 RXI (Receive data full) */
            [10] = sci_uart_txi_isr, /* SCI6 TXI (Transmit data empty) */
            [11] = sci_uart_tei_isr, /* SCI6 TEI (Transmit end) */
            [12] = sci_uart_eri_isr, /* SCI6 ERI (Receive error) */
            [13] = usbfs_interrupt_handler, /* USBFS INT (USBFS interrupt) */
            [14] = usbfs_resume_handler, /* USBFS RESUME (USBFS resume interrupt) */
            [15] = usbfs_d0fifo_handler, /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [16] = usbfs_d1fifo_handler, /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
            [17] = sci_uart_rxi_isr, /* SCI4 RXI (Receive data full) */
            [18] = sci_uart_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [19] = sci_uart_tei_isr, /* SCI4 TEI (Transmit end) */
            [20] = sci_uart_eri_isr, /* SCI4 ERI (Receive error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_CAN0_CHERR,GROUP0), /* CAN0 CHERR (Channel  error) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_CAN0_TX,GROUP1), /* CAN0 TX (Transmit interrupt) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_CAN0_COMFRX,GROUP2), /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_CAN_GLERR,GROUP3), /* CAN GLERR (Global error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_CAN_RXF,GROUP4), /* CAN RXF (Global receive FIFO interrupt) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI5_RXI,GROUP5), /* SCI5 RXI (Receive data full) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TXI,GROUP6), /* SCI5 TXI (Transmit data empty) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TEI,GROUP7), /* SCI5 TEI (Transmit end) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI5_ERI,GROUP0), /* SCI5 ERI (Receive error) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI6_RXI,GROUP1), /* SCI6 RXI (Receive data full) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TXI,GROUP2), /* SCI6 TXI (Transmit data empty) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TEI,GROUP3), /* SCI6 TEI (Transmit end) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI6_ERI,GROUP4), /* SCI6 ERI (Receive error) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_USBFS_INT,GROUP5), /* USBFS INT (USBFS interrupt) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_USBFS_RESUME,GROUP6), /* USBFS RESUME (USBFS resume interrupt) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_0,GROUP7), /* USBFS FIFO 0 (DMA/DTC transfer request 0) */
            [16] = BSP_PRV_VECT_ENUM(EVENT_USBFS_FIFO_1,GROUP0), /* USBFS FIFO 1 (DMA/DTC transfer request 1) */
            [17] = BSP_PRV_VECT_ENUM(EVENT_SCI4_RXI,GROUP1), /* SCI4 RXI (Receive data full) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TXI,GROUP2), /* SCI4 TXI (Transmit data empty) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TEI,GROUP3), /* SCI4 TEI (Transmit end) */
            [20] = BSP_PRV_VECT_ENUM(EVENT_SCI4_ERI,GROUP4), /* SCI4 ERI (Receive error) */
        };
        #endif
        #endif