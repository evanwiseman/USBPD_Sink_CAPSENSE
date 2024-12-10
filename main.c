/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the USBPD Sink application.
*
* Related Document: See README.md
*
*******************************************************************************/

/* Include Files */
#include "bsps/TARGET_APP_PMG1-CY7113/config/GeneratedSource/cycfg_peripherals.h"
#include "cy_scb_uart.h"
#include "cy_sysclk.h"
#include "cy_sysint.h"
#include "cy_syslib.h"
#include "cy_usbpd_defines.h"
#include "cybsp.h"
#include "cycfg.h"
#include "config.h"
#include "cy_gpio.h"
#include "cy_pdl.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_phy.h"
#include "cycfg_pins.h"
#include "cyip_gpio.h"
#include "cyip_scb_v2.h"
#include "cypm1311_48lqxi.h"
#include "gpio_pmg1s3_48_qfn_cypm1311.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "psink.h"
#include "swap.h"
#include "vdm.h"
#include "mtbcfg_ezpd.h"


/* Macro Definitions */
#define LED_DELAY_MS (500u)
#define CY_ASSERT_FAILED (0u)
#define DEBUG_PRINT (1u)


/* Global Variables */
static uint16_t gl_LedBlinkRate = LED_TIMER_PERIOD_DETACHED;
cy_stc_pdutils_sw_timer_t gl_TimerCtx;
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */
#if DEBUG_PRINT
cy_stc_scb_uart_context_t uart_context;
#endif /* DEBUG_PRINT */
volatile bool P6_0_Flag = false; // 5VDC(SPR)
volatile bool P6_1_Flag = false; // 9VDC(SPR)
volatile bool P6_2_Flag = false; // 15VDC(SPR)
volatile bool P6_3_Flag = false; // 20VDC(SPR)
volatile bool P2_1_Flag = false; // 24.5VDC(EPR)
volatile bool P2_2_Flag = false; // 28VDC(EPR)
volatile bool P0_0_Flag = false; // Auxillary Pin
#if CY_PD_EPR_ENABLE
volatile bool gl_epr_exit = false;
#endif /* CY_PD_EPR_ENABLE */
volatile GPIO_PRT_Type* curr_port = NULL;
volatile uint32_t curr_pin = 0xFFFFFFFF;


/* Structs and Enums */
enum SPR {
    SPR_5VDC = 5000,
    SPR_9VDC = 9000,
    SPR_15VDC = 15000,
    SPR_20VDC = 20000,
    SPR_3A = 3000,
    SPR_5A = 5000
};

enum EPR {
    EPR_24_5VDC = 24500,
    EPR_28VDC = 28000,
    EPR_5A = 5000
};

/* Parameters and Configurations */
const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params = {
    .dpmSnkWaitCapPeriod = 335,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

cy_stc_pdstack_context_t* gl_PdStackContext = &gl_PdStackPort0Ctx;

const cy_stc_sysint_t wdt_interrupt_config = {
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0u
};

const cy_stc_sysint_t usbpd_port0_intr0_config = {
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0u
};

const cy_stc_sysint_t usbpd_port0_intr1_config = {
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0u
};

const cy_stc_pdstack_app_cbk_t app_callback = {
    app_event_handler,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
    vbus_get_value
};

const cy_stc_sysint_t gpio_interrupt_config = {
    .intrSrc = (IRQn_Type)ioss_interrupt_gpio_IRQn,
    .intrPriority = 0u
};


/* Interrupt Prototypes */
static void wdt_interrupt_handler(void);
static void cy_usbpd0_intr0_handler(void);
static void cy_usbpd0_intr1_handler(void);
#if PMG1_PD_DUALPORT_ENABLE
static void cy_usbpd1_intr0_handler(void);
static void cy_usbpd1_intr1_handler(void);
#endif /* PMG1_PD_DUALPORT_ENABLE */
static void gpio_interrupt_handler(void);


/* Interrupt Definitions */
/*******************************************************************************
* Function Name: wdt_interrupt_handler
*
* Description: This function is called when the WDT interrupt is triggered.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
static void wdt_interrupt_handler(void) {
    Cy_WDT_ClearInterrupt();

    #if (CY_PDUTILS_TIMER_TICKLESS_ENABLE)
    /* Load the timer match register */
    Cy_WDT_SetMatch((Cy_WDT_GetCount()));
    #endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE) */

    /* Invoke the timer handler */
    Cy_PdUtils_SwTimer_InterruptHandler(&gl_TimerCtx);
}

/*******************************************************************************
* Function Name: cy_usbpd_intr0_handler
*
* Description: This function is called when the USBPD interrupt 0 is triggered.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
static void cy_usbpd0_intr0_handler(void) {
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd_intr1_handler
*
* Description: This function is called when the USBPD interrupt 1 is triggered.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
static void cy_usbpd0_intr1_handler(void) {
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
*
* Description: This function is called when the GPIO interrupt is triggered.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
static void gpio_interrupt_handler(void) {
    /* Only one flag can be active at a time */
    if (Cy_GPIO_GetInterruptStatus(P6_0_PORT, P6_0_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P6_0_PORT, P6_0_NUM);
        P6_0_Flag = true;
    }
    else if (Cy_GPIO_GetInterruptStatus(P6_1_PORT, P6_1_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P6_1_PORT, P6_1_NUM);
        P6_1_Flag = true;
    }
    else if (Cy_GPIO_GetInterruptStatus(P6_2_PORT, P6_2_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P6_2_PORT, P6_2_NUM);
        P6_2_Flag = true;
    }
    else if (Cy_GPIO_GetInterruptStatus(P6_3_PORT, P6_3_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P6_3_PORT, P6_3_NUM);
        P6_3_Flag = true;
    }
    else if (Cy_GPIO_GetInterruptStatus(P2_1_PORT, P2_1_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P2_1_PORT, P2_1_NUM);
        P2_1_Flag = true;
    }
    else if (Cy_GPIO_GetInterruptStatus(P2_2_PORT, P2_2_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P2_2_PORT, P2_2_NUM);
        P2_2_Flag = true;
    }
    else if (Cy_GPIO_GetInterruptStatus(P0_0_PORT, P0_0_NUM) == CY_GPIO_INTR_STATUS_MASK) {
        Cy_GPIO_ClearInterrupt(P0_0_PORT, P0_0_NUM);
        P0_0_Flag = true;
    }
    /* hold interrupt */
    NVIC_DisableIRQ(gpio_interrupt_config.intrSrc);
    Cy_SysLib_DelayUs(100);
    NVIC_EnableIRQ(gpio_interrupt_config.intrSrc);
}


/* Function Prototypes */
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void* data);
void instrumentation_cb(uint8_t port, inst_evt_t evt);
cy_stc_pdstack_context_t* get_pdstack_context(void);
cy_stc_pd_dpm_config_t* get_dpm_connect_port0_stat(void);
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t *context);
#if APP_FW_LED_ENABLE
void led_timer_cb(cy_timer_id_t id, void* callbackContext);
#endif /* APP_FW_LED_ENABLE */

void init_device(void);
void init_uart(void);
void init_sw_timer(void);
void init_gpio_interrupts(void);
void init_wdt_interrupt(void);
void init_usbpd_interrupts(void);
void init_usbpd(void);
void init_instrumentation(void);
void init_pdstack_dpm(void);
void init_app(void);
void init_fault_handler(void);

cy_en_pdstack_status_t send_avs_request(cy_stc_pdstack_context_t *context, uint32_t avs_op_voltage, uint32_t avs_op_current);
void set_spr(uint32_t voltage, uint32_t current);
void set_epr(uint32_t voltage, uint32_t current);

void clear_flags(void);
void process_flags(void);

/* Function Definitions */
/*******************************************************************************
* Function Name: sln_pd_event_handler
*
* Description: This function is called by the PD stack to handle the events.
*
* Parameters:
*  ctx: The context of the PD stack.
*  evt: The event to be handled.
*  data: The data associated with the event.
*
* Return:
*  None
*
****************************************************************************/
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void* data) {
    (void)ctx;

    if (evt == APP_EVT_HANDLE_EXTENDED_MSG) {
        cy_stc_pd_packet_extd_t* extd = (cy_stc_pd_packet_extd_t*)data;
        if (extd->msg != CY_PDSTACK_EXTD_MSG_SECURITY_RESP && extd->msg != CY_PDSTACK_EXTD_MSG_FW_UPDATE_RESP) {
            Cy_PdStack_Dpm_SendPdCommand(ctx, CY_PDSTACK_DPM_CMD_SEND_NOT_SUPPORTED, NULL, true, NULL);
        }
    }
}

/*******************************************************************************
* Function Name: instrumentation_cb
*
* Description: This function is called by the instrumentation module to handle the events.
*
* Parameters:
*  port: The port number.
*  evt: The event to be handled.
*
* Return:
*  None
*
****************************************************************************/
void instrumentation_cb(uint8_t port, inst_evt_t evt) {
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t) evt, NULL);
}

/*******************************************************************************
* Function Name: get_pdstack_context
*
* Description: This function is called to get the PD stack context.
*
* Parameters:
*  port: The port number.
*
* Return:
*  The PD stack context.
*
****************************************************************************/
cy_stc_pdstack_context_t* get_pdstack_context(void) {
    return gl_PdStackContext;
}

/*******************************************************************************
* Function Name: get_dpm_connect_port0_stat
*
* Description: This function is called to get the DPM connection status of port 0.
*
* Parameters:
*  None
*
* Return:
*  The DPM connection status of port 0.
*
****************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_connect_port0_stat(void) {
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

/*******************************************************************************
* Function Name: app_get_callback_ptr
*
* Description: This function is called to get the application callback pointer.
*
* Parameters:
*  context: The context of the PD stack.
*
* Return:
*  The application callback pointer.
*
****************************************************************************/
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t *context) {
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t*)&app_callback);
}

#if APP_FW_LED_ENABLE
/*******************************************************************************
* Function Name: led_timer_cb
*
* Description: This function is called by the timer to handle the LED events.
*
* Parameters:
*  id: The timer ID.
*  callbackContext: The context of the callback.
*
* Return:
*  None
*
****************************************************************************/
void led_timer_cb(cy_timer_id_t id, void* callbackContext) {
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
    Cy_PdUtils_SwTimer_Start(
        &gl_TimerCtx,
        callbackContext,
        id,
        gl_LedBlinkRate,
        led_timer_cb
    );
}
#endif /* APP_FW_LED_ENABLE */

/*******************************************************************************
* Function Name: init_device
*
* Description: This function initializes the device.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_device(void) {
    /* Initialize the device */
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*******************************************************************************
* Function Name: init_uart
*
* Description: This function initializes the UART.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_uart(void) {
    /* Initialize and enable the UART */
    cy_en_scb_uart_status_t uart_status = Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &uart_context);
    if (uart_status != CY_SCB_UART_SUCCESS) {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "**********************************\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "UART Debug logs \r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "**********************************\r\n\n");
}

/*******************************************************************************
* Function Name: init_sw_timer
*
* Description: This function initializes the software timer.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_sw_timer(void) {
    cy_stc_pdutils_timer_config_t timer_config;
    timer_config.sys_clk_freq = Cy_SysClk_ClkHfGetFrequency();
    timer_config.hw_timer_ctx = NULL;
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timer_config);
}

/*******************************************************************************
* Function Name: init_gpio_interrupts
*
* Description: This function initializes the GPIO interrupts.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_gpio_interrupts(void) {
    cy_en_sysint_status_t gpio_status = Cy_SysInt_Init(&gpio_interrupt_config, &gpio_interrupt_handler);
    if (gpio_status != CY_SYSINT_SUCCESS) {
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "GPIO Interrupt initialization failed\r\n");
        #endif /* DEBUG_PRINT */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_ClearPendingIRQ(gpio_interrupt_config.intrSrc);
    NVIC_EnableIRQ(gpio_interrupt_config.intrSrc);
}

/*******************************************************************************
* Function Name: init_wdt_interrupt
*
* Description: This function initializes the WDT interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_wdt_interrupt(void) {
    cy_en_sysint_status_t wdt_status = Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    if (wdt_status != CY_SYSINT_SUCCESS) {
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Watchdog Timer Interrupt initialization failed\r\n");
        #endif /* DEBUG_PRINT */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_ClearPendingIRQ(wdt_interrupt_config.intrSrc);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);
}

/*******************************************************************************
* Function Name: init_usbpd_interrupts
*
* Description: This function initializes the USBPD interrupts.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_usbpd_interrupts(void) {
    /* Initialize USBPD interrupts for Port0 */
    cy_en_sysint_status_t usbpd_port0_intr0_stat = Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    if (usbpd_port0_intr0_stat != CY_SYSINT_SUCCESS) {
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "USBPD Port0 Interrupt 0 initialization failed\r\n");
        #endif /* DEBUG_PRINT */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_ClearPendingIRQ(usbpd_port0_intr0_config.intrSrc);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    cy_en_sysint_status_t usbpd_port0_intr1_stat = Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    if (usbpd_port0_intr1_stat != CY_SYSINT_SUCCESS) {
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "USBPD Port0 Interrupt 1 initialization failed\r\n");
        #endif /* DEBUG_PRINT */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_ClearPendingIRQ(usbpd_port0_intr1_config.intrSrc);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);
}

/*******************************************************************************
* Function Name: init_usbpd
*
* Description: This function initializes the USBPD.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_usbpd(void) {
    /* Initialize USBPD for Port0 */
    cy_en_usbpd_status_t usbpd_port0_status = Cy_USBPD_Init(
        &gl_UsbPdPort0Ctx,
        0,
        mtb_usbpd_port0_HW,
        mtb_usbpd_port0_HW_TRIM,
        (cy_stc_usbpd_config_t*)&mtb_usbpd_port0_config,
        get_dpm_connect_port0_stat
    );
    if (usbpd_port0_status != CY_USBPD_STAT_SUCCESS) {
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "USBPD initialization failed for Port0\r\n");
        #endif /* DEBUG_PRINT */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*******************************************************************************
* Function Name: init_instrumentation
*
* Description: This function initializes the instrumentation.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_instrumentation(void) {
    instrumentation_init();
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);
}

/*******************************************************************************
* Function Name: init_pd_dpm
*
* Description: This function initializes the PD DPM.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_pdstack_dpm(void) {
    /* Initialize PD DPM for Port0 */
    cy_en_pdstack_status_t pd_stack_port0_status = Cy_PdStack_Dpm_Init(
        &gl_PdStackPort0Ctx,
        &gl_UsbPdPort0Ctx,
        &mtb_usbpd_port0_pdstack_config,
        app_get_callback_ptr(&gl_PdStackPort0Ctx),
        &pdstack_port0_dpm_params,
        &gl_TimerCtx
    );
    if (pd_stack_port0_status != CY_PDSTACK_STAT_SUCCESS) {
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "PD DPM initialization failed for Port0\r\n");
        #endif /* DEBUG_PRINT */

        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*******************************************************************************
* Function Name: init_app
*
* Description: This function initializes the application.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_app(void) {
    /* Initialize the application */
    app_init(get_pdstack_context());
}

/*******************************************************************************
* Function Name: init_fault_handler
*
* Description: This function initializes the fault handler.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void init_fault_handler(void) {
    fault_handler_init_vars(&gl_PdStackPort0Ctx);
}

/*******************************************************************************
* Function Name: send_avs_request
*
* Description: This function sends the AVS request.
*
* Parameters:
*  context: The context of the PD stack.
*  avs_op_voltage: The AVS operation voltage.
*  avs_op_current: The AVS operation current.
*
* Return:
*  The status of the AVS request.
*
****************************************************************************/
cy_en_pdstack_status_t send_avs_request(cy_stc_pdstack_context_t *pdstack_context, uint32_t avs_op_voltage, uint32_t avs_op_current) {
    uint32_t rdo_op_current = avs_op_current / 50u;
	uint32_t rdo_out_volt = avs_op_voltage / 25u;
	cy_en_pdstack_status_t pd_status = CY_PDSTACK_STAT_CMD_FAILURE;

	/* Check if the source AVS PDO can support request */
	for(uint32_t i = 0; i < (CY_PD_MAX_NO_OF_DO +CY_PD_MAX_NO_OF_EPR_PDO); i++)
	{
		if(pdstack_context->dpmStat.srcCapP->dat[i].epr_avs_src.apdoType == 0x01) {
			cy_stc_pdstack_dpm_pd_cmd_buf_t avs_request_buffer;

			/* Set the AVS Request */
			avs_request_buffer.cmdSop = CY_PD_SOP;
			avs_request_buffer.noOfCmdDo = 2;
			avs_request_buffer.timeout = 0;

			/* Set the AVS RDO */
			avs_request_buffer.cmdDo[0].rdo_epr_avs.opCur = rdo_op_current;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.rsvd1 = 0x0;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.outVolt = rdo_out_volt;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.rsvd2 = 0x0;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.eprModeCapable = 0x1;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.unchunkSup = 0x1;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.noUsbSuspend = 0x0;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.capMismatch = 0x0;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.rsvd3 = 0x0;
			avs_request_buffer.cmdDo[0].rdo_epr_avs.objPos = i + 1 ;
			avs_request_buffer.cmdDo[1].val = pdstack_context->dpmStat.srcCapP->dat[i].val;

            /* Send the AVS Request */
			pd_status = Cy_PdStack_Dpm_SendPdCommand(
				pdstack_context,
				CY_PDSTACK_DPM_CMD_SEND_EPR_REQUEST,
				&avs_request_buffer,
				false,
				NULL
			);

            break;
		}
	}
	return pd_status;
}

/*******************************************************************************
* Function Name: get_pdstack_status_string
*
* Description: This function gets the PD stack status string.
*
* Parameters:
*  status: The PD stack status.
*
* Return:
*  The PD stack status string.
*
****************************************************************************/
char* get_pdstack_status_string(cy_en_pdstack_status_t status) {
    switch (status) {
        case CY_PDSTACK_STAT_SUCCESS:
            return "CY_PDSTACK_STAT_SUCCESS";
        case CY_PDSTACK_STAT_FAILURE:
            return "CY_PDSTACK_STAT_FAILURE";
        case CY_PDSTACK_STAT_BUSY:
            return "CY_PDSTACK_STAT_BUSY";
        case CY_PDSTACK_STAT_BAD_PARAM:
            return "CY_PDSTACK_STAT_BAD_PARAM";
        case CY_PDSTACK_STAT_CMD_FAILURE:
            return "CY_PDSTACK_STAT_CMD_FAILURE";
        case CY_PDSTACK_STAT_NOT_SUPPORTED:
            return "CY_PDSTACK_STAT_NOT_SUPPORTED";
        case CY_PDSTACK_STAT_TIMEOUT:
            return "CY_PDSTACK_STAT_TIMEOUT";
        case CY_PDSTACK_STAT_NOT_READY:
            return "CY_PDSTACK_STAT_NOT_CONNECTED";
        case CY_PDSTACK_STAT_INVALID_PORT:
            return "CY_PDSTACK_STAT_INVALID_PORT";
        case CY_PDSTACK_STAT_NO_RESPONSE:
            return "CY_PDSTACK_STAT_NO_RESPONSE";
        case CY_PDSTACK_STAT_FLASH_DATA_AVAILABLE:
            return "CY_PDSTACK_STAT_FLASH_DATA_AVAILABLE";
        case CY_PDSTACK_STAT_INVALID_COMMAND:
            return "CY_PDSTACK_STAT_INVALID_COMMAND";
        case CY_PDSTACK_STAT_FLASH_UPDATE_FAILED:
            return "CY_PDSTACK_STAT_FLASH_UPDATE_FAILED";
        case CY_PDSTACK_STAT_INVALID_FW:
            return "CY_PDSTACK_STAT_INVALID_FW";
        case CY_PDSTACK_STAT_INVALID_ARGUMENT:
            return "CY_PDSTACK_STAT_INVALID_ARGUMENT";
        case CY_PDSTACK_STAT_INVALID_SIGNATURE:
            return "CY_PDSTACK_STAT_INVALID_SIGNATURE";
        case CY_PDSTACK_STAT_TRANS_FAILURE:
            return "CY_PDSTACK_STAT";
        case CY_PDSTACK_STAT_READ_DATA:
            return "CY_PDSTACK_STAT_READ_DATA";
        case CY_PDSTACK_STAT_INVALID_ID:
            return "CY_PDSTACK_STAT_INVALID_ID";
        case CY_PDSTACK_STAT_INVALID_GUID:
            return "CY_PDSTACK_STAT_INVALID_GUID";
        case CY_PDSTACK_STAT_INVALID_VER:
            return "CY_PDSTACK_STAT_INVALID_VERSION";
        case CY_PDSTACK_STAT_OUT_OF_SEQ_CMD:
            return "CY_PDSTACK_STAT_OUT_OF_SEQ_CMD";
        case CY_PDSTACK_STAT_INVALID_FWCT:
            return "CY_PDSTACK_STAT_INVALID_FWWT";
        case CY_PDSTACK_STAT_HASH_CMP_FAILED:
            return "CY_PDSTACK_STAT_HASH_CMP_FAILED";
        default:
            return "Unknown Status";
    }
}

/*******************************************************************************
* Function Name: set_spr
*
* Description: This function sets the SPR.
*
* Parameters:
*  voltage: The voltage (mV).
*  current: The current (mA).
*
* Return:
*  None
*
****************************************************************************/
void set_spr(uint32_t voltage, uint32_t current) {
    cy_stc_pdstack_context_t* pdstack_context = get_pdstack_context();
    cy_stc_pdstack_dpm_ext_status_t* dpm_ext_status = &(pdstack_context->dpmExtStat);
    uint8_t snk_pdo_mask = 0x00;
    bool is_epr_active = dpm_ext_status->eprActive;
    char* snk_cap_mask_message = "";
    char* pd_message = "";

    switch (voltage) {
        case SPR_5VDC:
            snk_pdo_mask = 0x01u;
            break;
        case SPR_9VDC:
            snk_pdo_mask = 0x03u;
            break;
        case SPR_15VDC:
            snk_pdo_mask = 0x09u;
            break;
        case SPR_20VDC:
            snk_pdo_mask = 0x11u;
            break;
        default:
            snk_pdo_mask = 0x01u;
            break;
    }

    if (!is_epr_active) {
        cy_en_pdstack_status_t snk_cap_mask_status = Cy_PdStack_Dpm_UpdateSnkCapMask(pdstack_context, snk_pdo_mask);
        cy_en_pdstack_status_t pd_status = Cy_PdStack_Dpm_SendPdCommand(
            pdstack_context, 
            CY_PDSTACK_DPM_CMD_SNK_CAP_CHNG, 
            NULL, 
            false, 
            NULL
        );

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "EPR Not Active\r\n");

        snk_cap_mask_message = get_pdstack_status_string(snk_cap_mask_status);
        pd_message = get_pdstack_status_string(pd_status);
        #endif /* DEBUG_PRINT */
    } else if (is_epr_active) {
        cy_en_pdstack_status_t snk_cap_mask_status = Cy_PdStack_Dpm_UpdateSnkCapMask(pdstack_context, snk_pdo_mask);

        // cy_stc_pdstack_dpm_pd_cmd_buf_t cmd_buf;
        // cmd_buf.cmdSop = CY_PD_SOP;
        // cmd_buf.extdType = CY_PDSTACK_EXTD_MSG_EXTD_CTRL_MSG;
        // cmd_buf.extdHdr.extd.dataSize = 0x02u;
        // cmd_buf.extdHdr.extd.chunked = !pdstack_context->dpmStat.unchunkSupLive;
        // cmd_buf.datPtr = (uint8_t*)CY_PDSTACK_EPR_GET_SRC_CAP;
        // cmd_buf.cmdDo[0].val = CY_PDSTACK_EPR_GET_SRC_CAP;

        cy_stc_pdstack_dpm_pd_cmd_buf_t cmd_buf;
        cmd_buf.noOfCmdDo = 1u;
        cmd_buf.cmdDo[0].eprmdo.action = CY_PDSTACK_EPR_MODE_EXIT;
        cmd_buf.cmdDo[0].eprmdo.data = 0u;
        cmd_buf.cmdDo[0].eprmdo.rsvd = 0u;
    
        cy_en_pdstack_status_t pd_status = Cy_PdStack_Dpm_SendPdCommand(
            pdstack_context, 
            CY_PDSTACK_DPM_CMD_SEND_EXTENDED, 
            &cmd_buf, 
            false, 
            NULL
        );

        if (pd_status == CY_PDSTACK_STAT_SUCCESS) {
            gl_epr_exit = false;
        }

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "EPR Active\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "EPR Exit\r\n");

        snk_cap_mask_message = get_pdstack_status_string(snk_cap_mask_status);
        pd_message = get_pdstack_status_string(pd_status);
        #endif /* DEBUG_PRINT */
    }

    #if DEBUG_PRINT
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "SNK Cap Mask Status: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, snk_cap_mask_message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PD Status: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, pd_message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    #endif /* DEBUG_PRINT */

}

/*******************************************************************************
* Function Name: set_epr
*
* Description: This function sets the EPR.
*
* Parameters:
*  voltage: The voltage (mV).
*  current: The current (mA).
*
* Return:
*  None
*
****************************************************************************/
void set_epr(uint32_t voltage, uint32_t current) {
    cy_stc_pdstack_context_t* pdstack_context = get_pdstack_context();
    cy_stc_pdstack_dpm_ext_status_t* dpm_ext_status = &(pdstack_context->dpmExtStat);
    uint8_t snk_pdo_mask = 0x03;
    char* snk_cap_mask_message = "";
    char* pd_message = "";
    char* avs_message = "";

    #if (CY_PD_EPR_ENABLE)
    if (!dpm_ext_status->eprActive &&
        pdstack_context->dpmStat.srcCapP->dat[0].fixed_src.eprModeCapable &&
        pdstack_context->dpmExtStat.epr.snkEnable) {
        /* Update the sink cap mask */
        cy_en_pdstack_status_t snk_cap_mask_status = Cy_PdStack_Dpm_UpdateSnkCapMask(pdstack_context, snk_pdo_mask);
        cy_en_pdstack_status_t pd_status = Cy_PdStack_Dpm_SendPdCommand(
            pdstack_context, 
            CY_PDSTACK_DPM_CMD_SNK_EPR_MODE_ENTRY, 
            NULL, 
            false, 
            NULL
        );

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "EPR Not Active\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "EPR Mode Capable\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "SNK Enable\r\n");

        snk_cap_mask_message = get_pdstack_status_string(snk_cap_mask_status);
        pd_message = get_pdstack_status_string(pd_status);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "SNK Cap Mask Status: ");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, snk_cap_mask_message);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "PD Status: ");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, pd_message);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
        #endif /* DEBUG_PRINT */

    } else if (dpm_ext_status->eprActive) {
        /* Update the sink cap mask */
        cy_en_pdstack_status_t avs_status = send_avs_request(pdstack_context, voltage, current);
        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "EPR Active\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "AVS Request\r\n");
        avs_message = get_pdstack_status_string(avs_status);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "AVS Request Status: ");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, avs_message);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
        #endif /* DEBUG_PRINT */
    }
    #endif /* (CY_PD_EPR_ENABLE) */

    #if DEBUG_PRINT
    
    #endif /* DEBUG_PRINT */
}


/*******************************************************************************
* Function Name: clear_flags
*
* Description: This function clears the flags.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void clear_flags(void) {
    P6_0_Flag = false;
    P6_1_Flag = false;
    P6_2_Flag = false;
    P6_3_Flag = false;
    P2_1_Flag = false;
    P2_2_Flag = false;
    P0_0_Flag = false;
}

/*******************************************************************************
* Function Name: process_flags
*
* Description: This function processes the flags.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
void process_flags(void) {
    if (P6_0_Flag && (curr_port != P6_0_PORT || curr_pin != P6_0_NUM)) {
        P6_0_Flag = false;
        curr_port = P6_0_PORT;
        curr_pin = P6_0_NUM;

        set_spr(SPR_5VDC, SPR_3A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "5VDC - SPR\r\n");
        #endif /* DEBUG_PRINT */
    } else if (P6_1_Flag && (curr_port != P6_1_PORT || curr_pin != P6_1_NUM)) {
        P6_1_Flag = false;
        curr_port = P6_1_PORT;
        curr_pin = P6_1_NUM;

        set_spr(SPR_9VDC, SPR_3A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "9VDC - SPR\r\n");
        #endif /* DEBUG_PRINT */
    } else if (P6_2_Flag && (curr_port != P6_2_PORT || curr_pin != P6_2_NUM)) {
        P6_2_Flag = false;
        curr_port = P6_2_PORT;
        curr_pin = P6_2_NUM;

        set_spr(SPR_15VDC, SPR_3A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "15VDC - SPR\r\n");
        #endif /* DEBUG_PRINT */
    } else if (P6_3_Flag && (curr_port != P6_3_PORT || curr_pin != P6_3_NUM)) {
        P6_3_Flag = false;
        curr_port = P6_3_PORT;
        curr_pin = P6_3_NUM;

        set_spr(SPR_20VDC, SPR_5A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "20VDC - SPR\r\n");
        #endif /* DEBUG_PRINT */
    } else if (P2_1_Flag && (curr_port != P2_1_PORT || curr_pin != P2_1_NUM)) {
        P2_1_Flag = false;
        curr_port = P2_1_PORT;
        curr_pin = P2_1_NUM;

        set_epr(EPR_24_5VDC, EPR_5A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "24.5VDC - EPR\r\n");
        #endif /* DEBUG_PRINT */
    } else if (P2_2_Flag && (curr_port != P2_2_PORT || curr_pin != P2_2_NUM)) {
        P2_2_Flag = false;
        curr_port = P2_2_PORT;
        curr_pin = P2_2_NUM;

        set_epr(EPR_28VDC, EPR_5A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "28VDC - EPR\r\n");
        #endif /* DEBUG_PRINT */
    } else if (P0_0_Flag && (curr_port != P0_0_PORT || curr_pin != P0_0_NUM)) {
        P0_0_Flag = false;
        curr_port = P0_0_PORT;
        curr_pin = P0_0_NUM;
        
        set_spr(SPR_5VDC, SPR_3A);

        #if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Auxillary\r\n");
        #endif /* DEBUG_PRINT */
    }
    clear_flags();
}


/*******************************************************************************
* Function Name: main
*
* Description: The main function of the application.
*
* Parameters:
*  None
*
* Return:
*  None
*
****************************************************************************/
int main(void) {
    init_device();
    #if DEBUG_PRINT
    init_uart();
    #endif /* DEBUG_PRINT */
    init_sw_timer();

    __enable_irq();
    init_gpio_interrupts();
    init_wdt_interrupt();
    init_instrumentation();
    init_usbpd_interrupts();
    init_usbpd();
    init_pdstack_dpm();
    init_app();
    init_fault_handler();    

    instrumentation_start();
    Cy_PdStack_Dpm_Start(get_pdstack_context());
    Cy_PdUtils_SwTimer_Start(
        &gl_TimerCtx,
        NULL,
        0,
        gl_LedBlinkRate,
        led_timer_cb
    );

    /* Main Loop */
    for (;;) {
        Cy_PdStack_Dpm_Task(get_pdstack_context());
        app_task(get_pdstack_context());

        process_flags();

        #if DEBUG_PRINT
        while (Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW) == false) {
            // Wait for the transmission to complete
        }
        #endif /* DEBUG_PRINT */

        #if SYS_DEEPSLEEP_ENABLE
        /* Enter Deep Sleep */
        system_sleep(get_pdstack_context(), NULL);
        #endif /* SYS_DEEPSLEEP_ENABLE */
    }
}