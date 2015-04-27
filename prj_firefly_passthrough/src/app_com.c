/**
 ****************************************************************************************
 *
 * @file app_com.c
 *
 * @brief Pass through project process
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_env.h"
#include "app_com.h"
#include "uart.h"
#include "lib.h"
#include "sleep.h"
#include "at_command.h"
#include "led.h"
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
 
 struct com_env_tag  com_env;
 extern uint32_t get_bit_num(uint32_t val);



void com_gpio_init(void)
{
	
//		gpio_set_direction(COM_TX_ENABLE,GPIO_OUTPUT);
//		gpio_write_pin(COM_TX_ENABLE,GPIO_HIGH);

		gpio_wakeup_config(COM_RX_ENABLE,GPIO_WKUP_BY_LOW);
		gpio_enable_interrupt(COM_RX_ENABLE);

//    gpio_pull_set(COM_AT_ENABLE, GPIO_PULL_UP);
//    gpio_set_direction(COM_AT_ENABLE, GPIO_INPUT);
//    gpio_set_interrupt(COM_AT_ENABLE, GPIO_INT_LOW_LEVEL);	
		gpio_wakeup_config(COM_AT_ENABLE,GPIO_WKUP_BY_LOW);
		gpio_enable_interrupt(COM_AT_ENABLE);	
	
		if (gpio_read_pin(COM_AT_ENABLE) == GPIO_LOW || gpio_read_pin(COM_RX_ENABLE) == GPIO_LOW)
		{
			com_wakeup_handler();
		}
//		else
//		{
//			QPRINTF("AT_RX_ENABLE_IS_HIGH\r\n");
//		}

		if(KE_EVENT_OK != ke_evt_callback_set(EVENT_COM_RX_WAKEUP_ID, 
                                            app_event_com_rx_wakeup_handler))
    {
        ASSERT_ERR(0);
    }

		if(KE_EVENT_OK != ke_evt_callback_set(EVENT_AT_ENABLE_PRESS_ID, 
                                            app_event_at_enable_press_handler))
    {
        ASSERT_ERR(0);
    }

		if(KE_EVENT_OK != ke_evt_callback_set(EVENT_AT_COMMAND_PROC_ID, 
                                            app_com_at_command_handler))
    {
        ASSERT_ERR(0);
    }		
		
}
 
void com_init(void)
{
    //for com uart tx  
    com_env.tx_state = COM_UART_TX_IDLE;	//initialize tx state
		com_env.com_conn = COM_DISCONN;
		com_env.com_mode = COM_MODE_IDLE;
    co_list_init(&com_env.queue_tx);			//init TX queue

		com_gpio_init();
     
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_TX_ID, com_tx_done))
			ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_RX_FRAME_ID, com_event_uart_rx_frame_handler))
			ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_RX_TIMEOUT_ID, com_event_uart_rx_timeout_handler))
			ASSERT_ERR(0);

}
 
void app_event_com_tx_handler(void)
{
	ke_evt_set(1UL<<EVENT_UART_TX_ID);
}
 
 
void com_uart_write(struct ke_msg *msg)
{
    //go to start tx state
    com_env.tx_state = COM_UART_TX_ONGOING;
    uart_write(QN_COM_UART, ((uint8_t *)&msg->param), msg->param_len, app_event_com_tx_handler);
}



void com_wakeup_handler(void)
{
//	if(com_env.com_conn == COM_DISCONN)
//	{
		switch(com_env.com_mode)
		{
			case COM_MODE_IDLE:
			case COM_MODE_TRAN_IDLE:
				if (gpio_read_pin(COM_AT_ENABLE) == GPIO_LOW)
				{
//					QPRINTF("IDLE_TO_AT\r\n");
					com_env.com_mode = COM_MODE_AT;
					led_set(2, LED_ON);
					com_uart_at_rx_start();			
				}
				else if(gpio_read_pin(COM_RX_ENABLE) == GPIO_LOW && com_env.com_conn == COM_CONN)
				{
//					QPRINTF("CONN_IDLE_TO_TRAN\r\n");
					com_env.com_mode = COM_MODE_TRAN;
					led_set(2, LED_OFF);
					uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
					if (bit_num >= QPPS_VAL_CHAR_NUM)  
					{
						com_uart_rx_start();					
					}
				}
				else if(gpio_read_pin(COM_RX_ENABLE) == GPIO_LOW && com_env.com_conn == COM_DISCONN)
				{
//					QPRINTF("DISCONN_IDLE_TO_AT\r\n");
					com_env.com_mode = COM_MODE_AT;
					led_set(2, LED_ON);
					com_uart_at_rx_start();	
				}
				break;
//			case COM_MODE_TRAN_IDLE:

//				break;
			case COM_MODE_TRAN:
				if (gpio_read_pin(COM_AT_ENABLE) == GPIO_LOW)
				{
//					QPRINTF("TRAN_TO_AT\r\n");
					com_env.com_mode = COM_MODE_AT;
					led_set(2, LED_ON);
					com_uart_at_rx_start();
				}
				break;
			case COM_MODE_AT:
//				QPRINTF("AT-1\r\n");
				break;
			default:
				break;
		}
//	}
	ke_timer_set(APP_COM_AT_RX_ENABLE_TIMER, TASK_APP, 2);
}


int app_com_at_rx_enable_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	switch(com_env.com_mode)
	{
		case COM_MODE_IDLE:
			break;
		case COM_MODE_TRAN:
			if (gpio_read_pin(COM_RX_ENABLE) == GPIO_HIGH )
			{
//				QPRINTF("TRAN_TO_IDLE\r\n");
				led_set(2, LED_OFF);
				com_env.com_mode = COM_MODE_IDLE;
				uart_rx_int_enable(QN_COM_UART, MASK_DISABLE);  //disable uart rx interrupt  ?sleep
			}
			break;
		case COM_MODE_AT:
			if(gpio_read_pin(COM_AT_ENABLE) == GPIO_HIGH)
			{
				if (gpio_read_pin(COM_RX_ENABLE) == GPIO_LOW)
				{
					if(com_env.com_conn == COM_CONN)
					{
						led_set(2, LED_OFF);
						uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
						if (bit_num >= QPPS_VAL_CHAR_NUM)
						{		
//							QPRINTF("AT_TO_TRAN\r\n");
							com_env.com_mode = COM_MODE_TRAN;						
							com_uart_rx_start();
						}
						else
						{
//							QPRINTF("AT_TO_TRAN_IDLE\r\n");
							com_env.com_mode = COM_MODE_TRAN_IDLE;
							uart_rx_int_enable(QN_COM_UART, MASK_DISABLE);  //disable uart rx interrupt ?sleep		
						}
					}
//					else
//					{
//						QPRINTF("AT_TO_AT\r\n");
//						com_env.com_mode = COM_MODE_AT;
//						led_set(2, LED_ON);
//						com_uart_at_rx_start();											
//					}
				}
				else
				{
//					QPRINTF("AT_TO_IDLE\r\n");
					led_set(2, LED_OFF);
					com_env.com_mode = COM_MODE_IDLE;
					uart_rx_int_enable(QN_COM_UART, MASK_DISABLE);  //disable uart rx interrupt ?sleep		
				}
//				led_set(2, LED_OFF);
			}
			break;
		default:
			break;
	}
	
	if (gpio_read_pin(COM_AT_ENABLE) == GPIO_LOW || gpio_read_pin(COM_RX_ENABLE) == GPIO_LOW)
	{
		ke_timer_set(APP_COM_AT_RX_ENABLE_TIMER, TASK_APP, 2);
	}
	else
	{
//		QPRINTF("STOP_AT_RX_ENABLE_TIMER\r\n");
	}
	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_com_rx_wakeup_handler(void)
{
	// delay 20ms to debounce
	ke_evt_clear(1UL << EVENT_COM_RX_WAKEUP_ID);
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep)
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif	

		com_wakeup_handler();
}

/**
 ****************************************************************************************
 * @brief Handles at_enable press.
 ****************************************************************************************
 */
void app_event_at_enable_press_handler(void)
{	
	ke_evt_clear(1UL << EVENT_AT_ENABLE_PRESS_ID);
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep)
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif
	com_wakeup_handler();
}


void com_uart_rx()
{
		com_env.com_rx_len++;
		//set pt gpio state 
		if(com_env.com_rx_len == QPPS_VAL_CHAR_NUM*QPP_DATA_MAX_LEN)  //receive data buf is full, should sent them to ble
		{
			ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
		}
		else
		{
			
			uart_read(QN_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
			ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
		}	
}

void	com_uart_rx_start(void)
{
//		QPRINTF("com_uart_rx_start\r\n");
		uart_uart_ClrIntFlag(CFG_COM_UART,0x1ff);
		uart_uart_GetRXD(CFG_COM_UART);
    com_env.com_rx_len = 0;
    uart_read(QN_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
}

#define AT_COMMAN_LEN_MAX	255 
uint8_t at_command_return[AT_COMMAN_LEN_MAX];
void com_uart_at()
{
		if(com_env.com_at_buf[com_env.com_at_len] == '\n' )  //receive data buf is full, should sent them to ble
		{
			ke_evt_set(1UL << EVENT_AT_COMMAND_PROC_ID);
		} 
		else
		{
			com_env.com_at_len++;
			if(com_env.com_at_len < AT_COMMAN_LEN_MAX)
			{
				//ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
				uart_read(QN_COM_UART, &com_env.com_at_buf[com_env.com_at_len], 1, com_uart_at);
			}
			else
			{
				com_env.com_at_buf[AT_COMMAN_LEN_MAX -1] = '\0';
				ke_evt_set(1UL << EVENT_AT_COMMAND_PROC_ID);
			}
		}	
}

void app_com_at_command_handler(void)
{
 			
//		QPRINTF("\r\n at_process_command:\r\n");

//		for(uint8_t i = 0;i < com_env.com_at_len+1;i++)
//			QPRINTF("0x%02x ",com_env.com_at_buf[i]);
	
		if(com_env.com_at_len >= 3)
		{
				if(com_env.com_at_buf[0] == 'A' && com_env.com_at_buf[1] == 'T')
				{
					if(com_env.com_at_len == 3)
					{
						com_pdu_send(sprintf((char *)at_command_return,"OK\r\n"),at_command_return);
					}
					else
					{
						if(com_env.com_at_buf[2] == '+')
						{
							com_env.com_at_buf[com_env.com_at_len-1] = '\0';
							com_pdu_send(at_process_command(com_env.com_at_buf + 2,at_command_return),at_command_return);
						}
						else
						{
							com_pdu_send(sprintf((char *)at_command_return,"AT ERR\r\n"),at_command_return);
//							QPRINTF("AT ERR_2!\r\n");
						}
					}					
				}
				else
				{
					com_pdu_send(sprintf((char *)at_command_return,"AT ERR\r\n"),at_command_return);
//					QPRINTF("AT ERR!\r\n");
				}
		}
		else
		{
			com_pdu_send(sprintf((char *)at_command_return,"AT ERR\r\n"),at_command_return);
//			QPRINTF("AT ERR_1!\r\n");
		}
		com_uart_at_rx_start();
		ke_evt_clear(1UL << EVENT_AT_COMMAND_PROC_ID);
}

void com_uart_at_rx_start(void)
{
//		QPRINTF("com_uart_at_rx_start\r\n");
		uart_uart_ClrIntFlag(CFG_COM_UART,0x1ff);
		uart_uart_GetRXD(CFG_COM_UART);
    com_env.com_at_len = 0;
    uart_read(QN_COM_UART, &com_env.com_at_buf[com_env.com_at_len], 1, com_uart_at);
}

void com_event_uart_rx_timeout_handler(void)
{
	ke_timer_set(APP_COM_RX_TIMEOUT_TIMER, TASK_APP, COM_FRAME_TIMEOUT);
	ke_evt_clear(1UL << EVENT_UART_RX_TIMEOUT_ID);
}

int app_com_rx_timeout_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uart_rx_int_enable(QN_COM_UART, MASK_DISABLE);  //disable uart rx interrupt 
    struct app_uart_data_ind *com_data = ke_msg_alloc(APP_COM_UART_RX_DONE_IND,
                                 TASK_APP,
                                 TASK_APP,
                                 com_env.com_rx_len+1);
    com_data->len=com_env.com_rx_len;
    memcpy(com_data->data,com_env.com_rx_buf,com_env.com_rx_len);
    ke_msg_send(com_data);
    
    return (KE_MSG_CONSUMED);
}


uint8_t baudrate = 3;
struct uart_divider_cfg
{
    uint8_t integer_h;
    uint8_t integer_l;
    uint8_t fractional;
};
extern const struct uart_divider_cfg uart_divider[UART_BAUD_MAX];
int app_com_at_baudrate_change_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	uint32_t reg;
	
//	QPRINTF("app_com_at_baudrate_change_handler:%d\r\n",baudrate);
	reg = (uart_divider[baudrate].integer_h << (UART_POS_DIVIDER_INT + 8))
			| (uart_divider[baudrate].integer_l << UART_POS_DIVIDER_INT)
			| uart_divider[baudrate].fractional;
	uart_uart_SetBaudDivider(QN_COM_UART, reg);	    
	
	return (KE_MSG_CONSUMED);
}

void com_event_uart_rx_frame_handler(void)
{
	uart_rx_int_enable(QN_COM_UART, MASK_DISABLE);  //disable uart rx interrupt
	struct app_uart_data_ind *com_data = ke_msg_alloc(APP_COM_UART_RX_DONE_IND,
															 TASK_APP,
															 TASK_APP,
															 com_env.com_rx_len+1);
	com_data->len=com_env.com_rx_len;
	memcpy(com_data->data,com_env.com_rx_buf,com_env.com_rx_len);
	ke_msg_send(com_data);

	ke_timer_clear(APP_COM_RX_TIMEOUT_TIMER, TASK_APP);
	ke_evt_clear(1UL << EVENT_UART_RX_FRAME_ID);
}

int app_com_uart_rx_done_ind_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
		switch(msgid)
    {
        case APP_COM_UART_RX_DONE_IND:
        {
            struct app_uart_data_ind* frame = (struct app_uart_data_ind*)param;
            
//					  QPRINTF("tx_data:\r\n");
//            for (uint8_t i=0;i<frame->len;i++)
//							QPRINTF("0x%02x ", *(frame->data+i));            
//						QPRINTF("\r\n");
            ///leo test
//              pt_pdu_send(frame->len, &(frame->data[0]));
						///leo test end
            
            if(frame->len) //have data
            {    
                //calculate page num;
                 uint8_t pagket_res = frame->len%QPP_DATA_MAX_LEN;
                 uint8_t pagket_num;
                 if(pagket_res)
                 pagket_num = frame->len/QPP_DATA_MAX_LEN + 1;
                 else
                 pagket_num = frame->len/QPP_DATA_MAX_LEN;
                 
                 uint8_t cnt=0,sent_pagket=0; 

                for (cnt = 0; (sent_pagket<pagket_num) && cnt < QPPS_VAL_CHAR_NUM; cnt++)
                {
                     if ((app_qpps_env->char_status >> cnt) & QPPS_VALUE_NTF_CFG)
                     {
												app_qpps_env->char_status &= ~(QPPS_VALUE_NTF_CFG << cnt);
											 
                         if((pagket_res)&&(pagket_num-sent_pagket==1))
                         app_qpps_data_send(app_qpps_env->conhdl, cnt, pagket_res, (frame->data+sent_pagket*20));
                         else
                         app_qpps_data_send(app_qpps_env->conhdl, cnt, QPP_DATA_MAX_LEN, (frame->data+sent_pagket*20)); 
                         
                         sent_pagket++;
                     }
                }
            }
        }break;
        default :break;
    }
       
    return (KE_MSG_CONSUMED);
}

 
 
 /**
 ****************************************************************************************
 * @brief After-process when one PDU has been sent.
 *
 ****************************************************************************************
 */
void com_tx_done(void)
{
    struct ke_msg * msg;
    // Clear the event
    ke_evt_clear(1<<EVENT_UART_TX_ID);
    // Go back to IDLE state
    com_env.tx_state = COM_UART_TX_IDLE;
    //release current message (which was just sent)
    msg = (struct ke_msg *)co_list_pop_front(&com_env.queue_tx);
    // Free the kernel message space
    ke_msg_free(msg);
    // Check if there is a new message pending for transmission
    if ((msg = (struct ke_msg *)co_list_pick(&com_env.queue_tx)) != NULL)
    {
        // Forward the message to the HCI UART for immediate transmission
        com_uart_write(msg);
    }
}

// Push msg into eaci tx queue
static void com_push(struct ke_msg *msg)
{
    // Push the message into the list of messages pending for transmission
    co_list_push_back(&com_env.queue_tx, &msg->hdr);

    // Check if there is no transmission ongoing
    if (com_env.tx_state == COM_UART_TX_IDLE)
        // Forward the message to the HCI UART for immediate transmission
        com_uart_write(msg);
}

/**
 ****************************************************************************************
 * @brief EACI send PDU
 *
 ****************************************************************************************
 */
void com_pdu_send(uint8_t len, uint8_t *par)
{
    // Allocate one msg for EACI tx
    uint8_t *msg_param = (uint8_t*)ke_msg_alloc(0, 0, 0, len);

    // Save the PDU in the MSG
    memcpy(msg_param, par, len);

     //extract the ke_msg pointer from the param passed and push it in HCI queue
    com_push(ke_param2msg(msg_param));
}

//end
