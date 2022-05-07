#include "RC_DT7.h"
#include "stm32f4xx.h"
#include "rc.h"
#include "Detect_Task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "arm_math.h"
//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700
u16 USART2_RX_STA=0; 
//ȡ������
static int16_t RC_abs(int16_t value);
//ң����������
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_s *rc_ctrl);
//ң�������Ʊ���
static RC_ctrl_s rc_ctrl;
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

//ȡ������
inline static int16_t RC_abs(int16_t value)
{
	if (value > 0)
		return value;
	else
		return -value;
}
//��ʼ��DMA������1
void remote_control_init(void)
{
    RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}
//����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ  ����ң����Ϣ��ָ��ṹ��
const RC_ctrl_s *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//�ж�ң���������Ƿ����
bool RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = 2;
    rc_ctrl.rc.s[1] = 2;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.key_code = 0;
    return 1;
}
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//�����ж�
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
        {
            //��������DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR |= DMA_SxCR_CT;
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
                //��¼���ݽ���ʱ��
                DetectHook(DBUSTOE);
            }
        }
        else
        {
            //��������DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            //��DMA�жϱ�־
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
                //��¼���ݽ���ʱ��
                DetectHook(DBUSTOE);
            }

        }
    }
}

void key_scan(RC_ctrl_s *rc_ctrl)
{
	uint16_t index,i;
	for(index = 1,i = 0; index; index<<=1, i++)
	{
		if(rc_ctrl->key.key_code & index)		//����
		{
			rc_ctrl->key_data.key_press_time[i]++;
			if(rc_ctrl->key_data.key_press_time[i] > 100)  //�̰�
			{
				rc_ctrl->key_data.key_short_press.key_code &= index;	//�̰���־λ��1
			}
			if(rc_ctrl->key_data.key_press_time[i] > 200)	//����
			{
				rc_ctrl->key_data.key_short_press.key_code &= ~index;	//�̰���־λ��0
				rc_ctrl->key_data.key_long_press.key_code |= index;		//������־λ��1
			}
		}
		else		//�ɿ� δ����
		{
			if(rc_ctrl->key_data.key_press_time[i] > 50)
			{
				rc_ctrl->key_data.key_click.key_code |= index;		//������־λ��1
			}
			else
			{
				rc_ctrl->key_data.key_click.key_code &= ~index;		//������־λ��0
			}
		}
	}
}

static void key_dealwith(RC_ctrl_s *rc_ctrl)
{
	static portTickType ulCurrentTime;
	static uint32_t delay_time;
	
	ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��
	if(ulCurrentTime >= delay_time)
	{
		delay_time = ulCurrentTime + 10;
		key_scan(rc_ctrl);
	}
}
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_s *rc_ctrl)//ң�ؽ�������
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);						//Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;					//Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //Mouse Right Is Press ?
    rc_ctrl->key.key_code = sbus_buf[14] | (sbus_buf[15] << 8);				//KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //Channel ����

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
	//�������ݴ���
	key_dealwith(rc_ctrl);
}

