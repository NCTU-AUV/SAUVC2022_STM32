#include "read_data.h"
#include "stm32f4xx_hal.h"
#include "usart.h"


Read_data::Read_data()
{
    index = 0;
	accessible = false;
	ch = '\n';
	size_of_data = 40; // yaw + v + velocity + joint = 4+12+12+12 = 40
}

float Read_data::get_single_num()
{
	int i;
	float n;
	char *ch = (char *) &n;
	for(i=0;i<4;i++)
	{
		ch[i] = temp[index];
		index++;
	}
	return n;
}

void Read_data::assign_num()
{
	index = 0;
	yaw = get_single_num();
	v.x = get_single_num();
	v.y = get_single_num();
	v.z = get_single_num();
	velocity[0] = get_single_num();
	velocity[1] = get_single_num();
	velocity[2] = get_single_num();
	joint[0] = get_single_num();
	joint[1] = get_single_num();
	joint[2] = get_single_num();
	index = 0;
	accessible = true;
}

void Read_data::receieve()
{
	if(index == size_of_data )
			assign_num();

	if(index < size_of_data ) //index max 26
	{	
		HAL_UART_Receive_IT(&huart5, &ch, 1);
		if(ch != '\n')
		{
			temp[index] = ch;
			index++;
		}
	}
}