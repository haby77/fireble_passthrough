#include "at_command.h"
#include "app_env.h"
#include "uart.h"
#include "lib.h"
#include "sleep.h"
#include "nvds.h"
#include "pwm.h"
#include "i2c.h"
void put_char(const int8_t *pcom,uint32_t len);

extern uint8_t baudrate;
int set_baudrate( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
	const int8_t *pcom;
	uint32_t pxParameterStringLength;
	int len;
	
	if(pcCommandString[commpare_length+1] == '?' || pcCommandString[commpare_length+1] == '\0')
	{
		len = commpare_length+1;
		memcpy(pcWriteBuffer, pcCommandString, len);
		len += sprintf((char *)pcWriteBuffer + len,":%d\r\nOK\r\n",baudrate);	
	}
	else if(pcCommandString[commpare_length+1] == '=')
	{
		pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 0, &pxParameterStringLength);
		if(pxParameterStringLength != 0)
		{
			baudrate = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
			if(baudrate > 16)
			{
				baudrate = 3;
			}		
			ke_timer_set(APP_COM_AT_BAUDRATE_CHANGE, TASK_APP, 1);	
			len = sprintf((char *)pcWriteBuffer,"OK\r\n");	
		}
		else
		{
			len = sprintf((char *)pcWriteBuffer,"ERR\r\n");
		}
	}
	else
	{
		len = sprintf((char *)pcWriteBuffer,"ERR\r\n");		
	}
	return len;
}

int get_version( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
	int len;
	len = commpare_length+1;
	memcpy(pcWriteBuffer, pcCommandString, len);
	len += sprintf((char *)pcWriteBuffer + len,":%s %s\r\nOK\r\n",SOFTWARE_VERSION,RELEASE_DATE);
	return len;
}

int get_name( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
		nvds_tag_len_t name_length = 31 - 5; 
		int len;
		len = commpare_length+1;
		memcpy(pcWriteBuffer, pcCommandString, len);
		pcWriteBuffer[len++] = ':';

    if (NVDS_OK != nvds_get(NVDS_TAG_DEVICE_NAME, &name_length, pcWriteBuffer+len))
    {
        // NVDS is empty, use default name
        name_length = strlen(QN_LOCAL_NAME);
        strcpy((char *)pcWriteBuffer+len, QN_LOCAL_NAME);			
    }
    else
    {
        name_length--; // Discard '\0'
    }
		len+=name_length;
		len+=sprintf((char *)pcWriteBuffer+len,"\r\nOK\r\n");
	return len;
}

int ble_discon( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
	app_gap_discon_req(0);
	return sprintf((char*)pcWriteBuffer,"OK\r\n");
}

int ble_adv( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
	const int8_t *pcom;
	int32_t arg1;
	uint32_t pxParameterStringLength;

	pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 1, 0, &pxParameterStringLength);
	arg1 = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
	if(arg1 == 0)
	{
		app_gap_adv_stop_req();
	}
	else if(arg1 == 1)
	{
		app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
						app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
						app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
						GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);		
	}
	else
	{
		return sprintf((char*)pcWriteBuffer,"ERR\r\n");
	}
	return sprintf((char*)pcWriteBuffer,"OK\r\n");
}

void pwm_io_config(enum PWM_CH ch)
{
	if(ch == PWM_CH0)
	{
    syscon_SetPMCR1(QN_SYSCON, P27_PWM0_PIN_CTRL);//P2.7 pwm0
		gpio_pull_set(GPIO_P27, GPIO_PULL_UP);
	}
	else if(ch == PWM_CH1)
	{
		syscon_SetPMCR1(QN_SYSCON, P26_PWM1_PIN_CTRL);//P2.6 pwm1
		gpio_pull_set(GPIO_P26, GPIO_PULL_UP);
	}
}

int ble_pwm( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
	const int8_t *pcom;
	uint32_t pxParameterStringLength;

	enum PWM_CH ch;
	uint16_t pscal;
	uint16_t periodcount;
	uint16_t pulsecount;
	
  uint8_t parm_num = at_get_parameters_numbers((const uint8_t*)pcCommandString + commpare_length + 2);

	if(pcCommandString[commpare_length+1] == '=')
	{
		if(parm_num == 4)
		{
			// pwm_ch?
			pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 0, &pxParameterStringLength);
			if(pxParameterStringLength == 1 )
			{ 
				if(pcom[0] == '0')
				{
					ch = PWM_CH0;
				}
				else if(pcom[0] == '1')
				{
					ch = PWM_CH1;
				}else
				{
					goto pram_err;
				}
			}
			else
			{
				goto pram_err;
			}	
			
			pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 1, &pxParameterStringLength);
			if(pxParameterStringLength > 0)
			{
				pscal = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
				if(pscal > 0x3FF)
				{
					goto pram_err;
				}
			}
			else
			{
				goto pram_err;
			}

			pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 2, &pxParameterStringLength);
			if(pxParameterStringLength > 0)
			{
				periodcount = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
				if(periodcount > 0xFF)
				{
					goto pram_err;
				}
			}
			else
			{
				goto pram_err;
			}			
			
			pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 3, &pxParameterStringLength);
			if(pxParameterStringLength > 0)
			{
				pulsecount = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
				if(pulsecount > 0xFF)
				{
					goto pram_err;
				}
			}
			else
			{
				goto pram_err;
			}

			//Parameters without error
			pwm_init(ch);
			pwm_io_config(ch);	
			pwm_config(ch, pscal, periodcount,pulsecount);
			pwm_enable(ch, MASK_ENABLE);			
	
		}
		//pwm disable
		else if(parm_num == 1)
		{
				if(pcCommandString[commpare_length + 2] == '0')
			{
				pwm_enable(PWM_CH0, MASK_DISABLE);
			}
			else if((pcCommandString[commpare_length + 2] == '1'))
			{
				pwm_enable(PWM_CH1, MASK_DISABLE);
			}
			else
			{
				goto pram_err;
			}
		}
		else
		{
			goto pram_err;
		}
	}
	else
	{
		goto pram_err;
	}
	
	return sprintf((char*)pcWriteBuffer,"OK\r\n");
	
pram_err:	
	return sprintf((char*)pcWriteBuffer,"ERR\r\n");
}


uint8_t i2c_buff[4];
extern bool i2c_is_finish(void);
int ble_i2c( const uint8_t * const pcCommandString,uint8_t* pcWriteBuffer,uint32_t commpare_length)
{
	const int8_t *pcom;
	uint32_t pxParameterStringLength;
	
	uint8_t len;
	uint8_t i2c_addr ;
	uint8_t i2c_reg  ;
	uint8_t i2c_data;
	
//  uint8_t parm_num = at_get_parameters_numbers((const uint8_t*)pcCommandString + commpare_length + 2);
//	QPRINTF("parm_num:%d\r\n",parm_num);

	if(pcCommandString[commpare_length+1] == '=')
	{
//		for(uint8_t a=0;a < parm_num ;a++)
//		{
//			QPRINTF("a=%d:\r\n",a);
//			pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, a, &pxParameterStringLength);
//			QPRINTF("ParameterLength:%d \r\n",pxParameterStringLength);
//			put_char((const int8_t *)pcom,pxParameterStringLength);
//			QPRINTF("\r\n");
//		}
		
		//i2c_addr
		pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 1, &pxParameterStringLength);
		if(pxParameterStringLength > 0)
		{
			i2c_addr = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
		}
		else
		{
			goto pram_err;
		}
		
		//i2c_reg
		pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 2, &pxParameterStringLength);
		if(pxParameterStringLength > 0)
		{
			i2c_reg  = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 
		}
		else
		{
			goto pram_err;
		}
		
		// i2c R&W?
		pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 0, &pxParameterStringLength);
		if(pxParameterStringLength == 1)
		{ 
			syscon_SetPMCR1WithMask(QN_SYSCON,P23_MASK_PIN_CTRL | P24_MASK_PIN_CTRL,P23_I2C_SDA_PIN_CTRL | P24_I2C_SCL_PIN_CTRL);	
			i2c_init(I2C_SCL_RATIO(100000), i2c_buff, 4);
			if(pcom[0] == 'R')
			{
				i2c_data = I2C_BYTE_READ(i2c_addr,i2c_reg);
				if(i2c_is_finish())
				{
					len = commpare_length+1;
					memcpy(pcWriteBuffer, pcCommandString, len);
					len += sprintf((char *)pcWriteBuffer + len,":0x%02x\r\nOK\r\n",i2c_data);	
					return len;
				}
				goto pram_err; 
			}
			else if(pcom[0] == 'W')
			{
				//i2c_data
				pcom = at_get_parameter((const int8_t*)pcCommandString + commpare_length + 2, 3, &pxParameterStringLength);
				if(pxParameterStringLength > 0)
				{
					i2c_data = at_HEXstringToNum((const uint8_t *)pcom, pxParameterStringLength); 	
				}
				else
				{
					goto pram_err;
				}
				
//				//???
//				QPRINTF("i2c_addr:%x \r\n",i2c_addr);
//				QPRINTF("i2c_reg :%x \r\n",i2c_reg );
//				QPRINTF("i2c_data:%x \r\n",i2c_data);
				I2C_BYTE_WRITE(i2c_addr, i2c_reg, i2c_data);
				if(!i2c_is_finish())
				{
					goto pram_err;
				}

			}
		}
		else
		{
			goto pram_err;
		}	
	}
	else
	{
		goto pram_err;
	}
	return sprintf((char*)pcWriteBuffer,"OK\r\n");
	
pram_err:	
	return sprintf((char*)pcWriteBuffer,"ERR\r\n");
}

const At_CommandInput command[] =
{	
	{
		"BAUD",
		 set_baudrate
	},
	{
		"VERSION",
		 get_version
	},
	{
		"NAME",
		 get_name
	},
	{
		"DISCON",
		 ble_discon
	},	
	{
		"ADV",
		 ble_adv
	},
	{
		"PWM",
		 ble_pwm
	}
	,
	{
		"I2C",
		 ble_i2c
	}		
};


uint8_t at_command_length_get(const uint8_t *command)
{

	const uint8_t *sc;

	for (sc = command; (*sc != '\0') && (*sc != ' ') && (*sc != '?') && (*sc != '='); ++sc)
		/* nothing */;
	return sc - command;

}

uint8_t at_get_parameters_numbers( const uint8_t * pcCommandString )
{
	uint8_t cParameters = 0;
	const uint8_t * b_pcCommandString = pcCommandString;
	while( *pcCommandString != 0x00 )
	{
		if( ( *pcCommandString ) == ',' )
		{
				cParameters++;
		}
		pcCommandString++;
	}
	if(b_pcCommandString != pcCommandString)
		cParameters++;
	return cParameters;
}

uint32_t at_HEXstringToNum(const uint8_t *str, uint32_t length)
{  
 uint8_t  revstr[16]={0}; 
 uint32_t   num[16]={0};  
 uint32_t   count=1;  
 uint32_t   result=0; 
 int 	i;
 memcpy(revstr,str,16);  
 for(i=length-1;i>=0;i--)  
 {  
	
	if ((revstr[i]>='0') && (revstr[i]<='9'))  
	   num[i]=revstr[i]-48;
	else if ((revstr[i]>='a') && (revstr[i]<='f'))  
	   num[i]=revstr[i]-'a'+10;  
	else if ((revstr[i]>='A') && (revstr[i]<='F'))  
	   num[i]=revstr[i]-'A'+10;  
	else  
	   num[i]=0;


	if('x' == revstr[1] || 'X' == revstr[1]) {

		result=result+num[i]*count;  
		count=count*16;

	 }
	 
	 else {
		 
		result=result+num[i]*count;  
		count=count*10;
	}
	
 }  
 	
 return result;  
}

const int8_t *at_get_parameter( const int8_t* pcCommandString, int32_t uxWantedParameter, uint32_t *pxParameterStringLength )
{
	int uxParametersFound = 0;
	const int8_t *pcReturn = pcCommandString;

	*pxParameterStringLength = 0;
	
	while(uxParametersFound <= uxWantedParameter)
	{
		if( *pcCommandString != 0x00 )
		{
			while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) != ',' ) )
			{
				pcCommandString++;
			}
			if(uxParametersFound  ==  uxWantedParameter)
			{
				*pxParameterStringLength = pcCommandString - pcReturn;
				break;
			}
			uxParametersFound++;
			if(( *pcCommandString ) != 0x00)
			{
				pcCommandString++;
				pcReturn = pcCommandString;
			}
		}
		else
		{
			break;
		}
	}
	
	return pcReturn;
}
void put_char(const int8_t *pcom,uint32_t len)
{

	for(uint32_t i =0;i<len;i++)
	{
		#ifdef CFG_DBG_PRINT
		UartPutc(pcom[i]);
		#endif
	}
}


int at_process_command(const uint8_t* const pcCommandInput,uint8_t* pcWriteBuffer)
{
	uint8_t i;
	uint8_t s_input_length;
	uint8_t s_command_length;
	uint8_t s_commpare_length;
	const uint8_t *pcCommandString;
	int return_count = 0;
	
	s_input_length   = at_command_length_get((const uint8_t *)pcCommandInput+1);
	for(i = 0;i < sizeof(command)/sizeof(At_CommandInput);i++)
	{
		pcCommandString  = command[i].pcCommand;
		s_command_length = at_command_length_get((const uint8_t *)pcCommandString);

		s_commpare_length = s_input_length > s_command_length?s_input_length:s_input_length;
		
		if (strncmp((const char *)pcCommandInput+1, (const char *)pcCommandString, s_commpare_length) == 0 ) 
		{
			return_count = command[i].pxCommandInterpreter(pcCommandInput,pcWriteBuffer,s_commpare_length);

			break;
		}		
	}
	if(i == sizeof(command)/sizeof(At_CommandInput))
	{
		return_count = sprintf((char *)pcWriteBuffer,"not find comamnd\r\n");
	}
	return return_count;
}



