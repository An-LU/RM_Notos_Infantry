#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "main.h"

#define COMMUNICATION_CONTROL_TIME_MS 1
#define COMMUNICATION_LENGTH	6

typedef __packed struct
{
	fp32 send_buff[COMMUNICATION_LENGTH];
	uint8_t tail[4];
}VOFA_UsartSend_s;	//vofa´®¿ÚÐ­Òé

void communication_task(void *pvParameters);

#endif	/* __COMMUNICATION_H */

