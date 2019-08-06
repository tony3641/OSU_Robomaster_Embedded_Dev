#ifndef USER_TASK_H
#define USER_TASK_H
#include "main.h"

typedef struct
{
	uint16_t f_pressed;
	uint16_t v_pressed;
	uint16_t c_pressed;
	uint16_t b_pressed;
	uint16_t r_pressed;
	uint16_t z_pressed;
	uint16_t x_pressed;
	uint16_t last_f_pressed;
	uint16_t last_v_pressed;
	uint16_t last_c_pressed;
	uint16_t last_b_pressed;
	uint16_t last_r_pressed;
	uint16_t last_z_pressed;
	uint16_t last_x_pressed;
}Keyboard_t;

extern void UserTask(void *pvParameters);
void SoftReset(void);//Èí¼ş¸´Î»

#endif
