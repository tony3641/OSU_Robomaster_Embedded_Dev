/* second-order kalman filter on stm32 */

#include "arm_math.h"
#include "filter.h"
#include "CAN_Receive.h"





#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);

  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
}

float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

  F->z.pData[0] = signal1;
  F->z.pData[1] = signal2;

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
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];

  return F->filtered_value;
}


/* Chebyshev Type II LPF IIR Filter */

#define N 10
const real64_T NUM[N] = {
  4.728303125142e-05,-0.0003212344796949,0.0008974755504678,-0.001237838604899,
  0.0006143145830995,0.0006143145830995,-0.001237838604899,0.0008974755504678,
  -0.0003212344796949,4.728303125142e-05
};

const real64_T DEN[N] = {
                   1,   -8.535326876791,    32.39009699572,   -71.72534454754,
      102.1400805577,   -97.00060811724,    61.43343646769,   -25.02018206012,
      5.946087737604,  -0.6282401568797
};

double Chebyshev_Type_II_IIR_LPF(IIR_Filter_t *F)
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
