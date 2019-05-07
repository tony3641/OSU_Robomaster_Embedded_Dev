/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       filter.c
  * @brief      Filter程序实现，包括Kalman，IIR，FIR和Group Delay
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-01-2019     OSU-RM          1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 OSU****************************
  */

#include "filter.h"
#include "CAN_Receive.h"
#include "buzzer.h"
#include "led.h"

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	//将I中数据导入F中
  mat_init(&F->xhat,MATRIX_ORDER,1,(float *)I->xhat_data);
	mat_init(&F->xhatminus,MATRIX_ORDER,1,(float *)I->xhatminus_data);
	mat_init(&F->z,MATRIX_ORDER,1,(float *)I->z_data);
	mat_init(&F->Q,MATRIX_ORDER,MATRIX_ORDER,(float *)I->Q_data);
	mat_init(&F->R,MATRIX_ORDER,MATRIX_ORDER,(float *)I->R_data);
	mat_init(&F->K,MATRIX_ORDER,MATRIX_ORDER,(float *)I->K_data);
	mat_init(&F->P,MATRIX_ORDER,MATRIX_ORDER,(float *)I->P_data);
	mat_init(&F->Pminus,MATRIX_ORDER,MATRIX_ORDER,(float *)I->Pminus_data);
	mat_init(&F->A,MATRIX_ORDER,MATRIX_ORDER,(float *)I->A_data);
	mat_init(&F->H,MATRIX_ORDER,MATRIX_ORDER,(float *)I->H_data);
	mat_init(&F->AT,MATRIX_ORDER,MATRIX_ORDER,(float *)I->AT_data);
	mat_init(&F->HT,MATRIX_ORDER,MATRIX_ORDER,(float *)I->HT_data);
	
  mat_trans(&F->A, &F->AT);
	mat_trans(&F->H, &F->HT);
}

float *kalman_filter_calc(kalman_filter_t *F, float x, float y, float vx, float vy)
{
  float TEMP_data[MATRIX_ORDER*MATRIX_ORDER] = {0, 0, 0, 0,
																								0, 0, 0, 0,
																								0, 0, 0, 0,
																								0, 0, 0, 0};
  float TEMP_data41[MATRIX_ORDER] = {0,
																		 0,
																		 0,
																		 0};
  mat TEMP,TEMP41;

  mat_init(&TEMP,MATRIX_ORDER,MATRIX_ORDER,(float *)TEMP_data);
  mat_init(&TEMP41,MATRIX_ORDER,1,(float *)TEMP_data41);

  F->z.pData[0] = x;
  F->z.pData[1] = y;
  F->z.pData[2] = vx;
  F->z.pData[3] = vy;
	
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP41);
  mat_sub(&F->z, &TEMP41, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP41);
  mat_add(&F->xhatminus, &TEMP41, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
	F->filtered_value[2] = F->xhat.pData[2];
  F->filtered_value[3] = F->xhat.pData[3];

  return F->filtered_value;
}


/* Butterworth LPF IIR Filter */
const real64_T NUM[N] = {
  4.937227088621e-11,3.949781670897e-10,1.382423584814e-09,2.764847169628e-09,
  3.456058962035e-09,2.764847169628e-09,1.382423584814e-09,3.949781670897e-10,
  4.937227088621e-11
};

const real64_T DEN[N] = {
                   1,   -7.454409929462,    24.32861936777,   -45.40334929689,
      52.99495134458,   -39.61407677781,    18.51935008066,   -4.950374763522,
     0.5792899873104
};

double Butterworth_Filter(IIR_Filter_t *F)
{	
	int i;
	for(i=N-1; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for(i=1;i<N;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
	return F->filtered_value;
}

////////////////////////////////////////////////////////
double Group_Delay(Group_Delay_t *GD)
{
	int i;
	for(i=DELAY_MS-1; i>0; i--)
	{

		GD->group_delay_buffer[i] = GD->group_delay_buffer[i-1];
	}

	GD->group_delay_buffer[0] = GD->group_delay_raw_value;

	return GD->group_delay_buffer[DELAY_MS-1];
}
double Group_Delay_Chassis(Group_Delay_Chassis_t *GD)
{
	int i;
	for(i=CHASSIS_DELAY_MS-1; i>0; i--)
	{

		GD->group_delay_chassis_buffer[i] = GD->group_delay_chassis_buffer[i-1];
	}

	GD->group_delay_chassis_buffer[0] = GD->group_delay_chassis_raw_value;

	return GD->group_delay_chassis_buffer[CHASSIS_DELAY_MS-1];
}
///////////////////////////////////////////////////////


const real64_T BM[BLACKMAN_LENGTH] = {
                  0,2.287467294661e-06,9.032329013555e-06,1.958108015785e-05,
  3.260673212716e-05,4.607285525375e-05,5.721741948241e-05,  6.2566778217e-05,
  5.799167855871e-05,3.881752205103e-05,-7.853376743051e-20,-6.362543989365e-05,
  -0.0001570160621217,-0.0002845176340242,-0.0004493916383499,-0.0006532893776334,
  -0.0008956959266684,-0.001173373882163,-0.001479842722982,-0.001804933718952,
  -0.002134462173269,-0.002450057936512,-0.002729191334113,-0.002945424842725,
  -0.003068911184632,-0.003067146350401,-0.002905971980494, -0.00255080628714,
  -0.001968067166846,-0.001126736314607,9.955895242016e-19, 0.001433108342737,
   0.003186145578245, 0.005263861111236, 0.007661046871207,  0.01036171144677,
    0.01333864351571,  0.01655341415264,   0.0199568484062,  0.02348997464949,
    0.02708543675112,  0.03066933035676,  0.03416340184248,  0.03748752811702,
    0.04056237862518,  0.04331214868228,  0.04566724644916,  0.04756681494391,
    0.04896097565381,  0.04981269138559,  0.05009916245967,  0.04981269138559,
    0.04896097565381,  0.04756681494391,  0.04566724644916,  0.04331214868228,
    0.04056237862518,  0.03748752811702,  0.03416340184248,  0.03066933035676,
    0.02708543675112,  0.02348997464949,   0.0199568484062,  0.01655341415264,
    0.01333864351571,  0.01036171144677, 0.007661046871207, 0.005263861111236,
   0.003186145578245, 0.001433108342737,9.955895242016e-19,-0.001126736314607,
  -0.001968067166846, -0.00255080628714,-0.002905971980494,-0.003067146350401,
  -0.003068911184632,-0.002945424842725,-0.002729191334113,-0.002450057936512,
  -0.002134462173269,-0.001804933718952,-0.001479842722982,-0.001173373882163,
  -0.0008956959266684,-0.0006532893776334,-0.0004493916383499,-0.0002845176340242,
  -0.0001570160621217,-6.362543989365e-05,-7.853376743051e-20,3.881752205103e-05,
  5.799167855871e-05,  6.2566778217e-05,5.721741948241e-05,4.607285525375e-05,
  3.260673212716e-05,1.958108015785e-05,9.032329013555e-06,2.287467294661e-06,
                   0
};

double Blackman_Filter(Blackman_Filter_t *F)
{
	int i;
	for(i=BLACKMAN_LENGTH-1; i>0; i--)
	{

		F->blm_xbuf[i] = F->blm_xbuf[i-1];
	}

	F->blm_xbuf[0] = F->blm_raw_value;
	F->blm_ybuf[0] = BM[0] * F->blm_xbuf[0];
	for(i=1;i<BLACKMAN_LENGTH;i++)
	{
		F->blm_ybuf[0] = F->blm_ybuf[0] + BM[i] * F->blm_xbuf[i];
	}
	F->blm_filtered_value = F->blm_ybuf[0];
	return F->blm_filtered_value;
}
