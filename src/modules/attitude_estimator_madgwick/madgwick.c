//
//  madgwick.c
//
//  Created by Johannes van Niekerk on 6/1/12. Based on gradient descent algorithm in Madgwick publication
//

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "quaternion.h"
#include "madgwick.h"


#define BETA 0.055f
#define ZETA 0.005f


static float q[4]={1.0,0.0,0.0,0.0};
static uint8_t initFlag=0;

static float w_bx,w_by,w_bz;
static float counter;

void AttitudeInit(float ax,float ay,float az)
{
    float eulers[3];//roll_init,pitch_init;
    counter = 0;
    w_bx=w_by=w_bz = 0.0f;
    // Initial conditions
    eulers[0] = atan2(-ay, -az);
    eulers[1] = asin( ax / sqrt(ax*ax + ay*ay + az*az));
    eulers[2] = 0;
    
    QuatOfEulers(eulers, q);
    if(q[0]==q[0])
        initFlag = 0;
    else 
    {q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;}
    
    
}
void getBiases(float *data)
{
	data[0] = w_bx;
	data[1] = w_by;
	data[2] = w_bz;
}

float *AttitudeUpdate(float dt,float _beta, float _zeta, float a_x,float a_y, float a_z,float w_x, float w_y, float w_z,float m_x, float m_y, float m_z)
{

	float qConj[4];
	float unbiasedGyro[4];
	float qDot[4];
	float beta,zeta;
	float F[6];
	float J[6*4];
	float step[4];
	float North,East,Down;
	float b2,b4;

    
    float q1 = q[0]; float q2 = q[1]; float q3 = q[2]; float q4 = q[3];
    float _error[4];
    
    float accel[] = {0,-a_x,-a_y,-a_z};
    float mag[] = {0,m_x,m_y,m_z};
    

    
    // Direction cosine matrix from quaternion
    float qdcm13 = 2.0f*(q2*q4 - q1*q3);
    float qdcm23 = 2.0f*(q3*q4 + q1*q2);
    float qdcm33 = 2.0f*(0.5f - q2*q2 - q3*q3);
    
    float qdcm12 = 2.0f*(q2*q3 + q1*q4);
    float qdcm22 = 2.0f*(0.5f - q2*q2 - q4*q4);
    float qdcm32 = 2.0f*(q3*q4 - q1*q2);
    
    float qdcm11 = 2.0f*(0.5f - q3*q3 - q4*q4);
    float qdcm21 = 2.0f*(q2*q3 - q1*q4);
    float qdcm31 = 2.0f*(q2*q4 + q1*q3);
    
    counter += dt;
       QuatNormalize(accel);
       QuatNormalize(mag);
    // Earth's field
    North = qdcm11*mag[1] + qdcm21*mag[2] + qdcm31*mag[3];
    East =  qdcm12*mag[1] + qdcm22*mag[2] + qdcm32*mag[3];
    Down =  qdcm13*mag[1] + qdcm23*mag[2] + qdcm33*mag[3];

    b2 = sqrtf(North*North + East*East);
    b4 = Down;
    
    
    
    // 6 x 1
    F[0] =        qdcm13 - accel[1];
    F[1] =        qdcm23 - accel[2];
    F[2] =        qdcm33 - accel[3];
    F[3] =        b2*qdcm11 + b4*qdcm13 - mag[1];
    F[4] =        b2*qdcm21 + b4*qdcm23 - mag[2];
    F[5] =        b2*qdcm31 + b4*qdcm33 - mag[3];

    // 6 x 4

    J[0] = -2*q3; 			J[1] =  2*q4; 			J[2] = -2*q1; 			J[3] =  2*q2;
    J[4] =  2*q2;        	J[5] =	2*q1;        	J[6] =	2*q4;        	J[7] =	2*q3;
    J[8] =     0;       	J[9] =	-4*q2;       	J[10]=	-4*q3;          J[11] = 0;
    J[12]=-2*b4*q3;         J[13]=  2*b4*q4;        J[14]=-4*b2*q3-2*b4*q1; J[15]= -4*b2*q4+2*b4*q2;
    J[16]=-2*b2*q4+2*b4*q2; J[17]=  2*b2*q3+2*b4*q1;J[18]= 2*b2*q2+2*b4*q4; J[19]=-2*b2*q1+2*b4*q3;
    J[20]=2*b2*q3;          J[21]=  2*b2*q4-4*b4*q2;J[22]= 2*b2*q1-4*b4*q3; J[23] =2*b2*q2;

    
    //float Jt[4*6];
    //vDSP_mtrans(J,1,Jt,1,4,6);
    
    // step = J'*F
    //       4 x 6 * 6 x 1
    //vDSP_mmul( Jt,1,F,1,step,1,4,1,6);
    step[0] = J[0]*F[0] + J[4]*F[1] + J[8]*F[2] + J[12]*F[3] + J[16]*F[4] + J[20]*F[5];
    step[1] = J[1]*F[0] + J[5]*F[1] + J[9]*F[2] + J[13]*F[3] + J[17]*F[4] + J[21]*F[5];
    step[2] = J[2]*F[0] + J[6]*F[1] + J[10]*F[2] + J[14]*F[3] + J[18]*F[4] + J[22]*F[5];
    step[3] = J[3]*F[0] + J[7]*F[1] + J[11]*F[2] + J[15]*F[3] + J[19]*F[4] + J[23]*F[5];
    
    // normalize step 
    QuatNormalize(step);
    
    qConj[0] = q[0]; qConj[1] = -q[1]; qConj[2]=-q[2]; qConj[3]=-q[3];
    // Error
    QuatMultiply(qConj,step,_error);
    QuatScale(_error,(2.0f*dt));

    // update gyro biases
    w_bx += zeta*_error[1]; w_by += zeta*_error[2]; w_bz += zeta*_error[3];

    // quaternion dynamics
    unbiasedGyro[0] = 0; unbiasedGyro[1] = (w_x-w_bx); unbiasedGyro[2]= (w_y-w_by); unbiasedGyro[3] = (w_z-w_bz);
    QuatMultiply(q,unbiasedGyro,qDot);
    
    
    // higher gain during startup
    if(counter < 5.0f)
    {
        beta = 1.0f;
        zeta = 1.0f;
    }
    else
    {
        beta = _beta;
        zeta = _zeta;
    }
    
    qDot[0] = 0.5f*qDot[0] - beta*step[0];
    qDot[1] = 0.5f*qDot[1] - beta*step[1];
    qDot[2] = 0.5f*qDot[2] - beta*step[2];
    qDot[3] = 0.5f*qDot[3] - beta*step[3];
    
    q[0] += qDot[0]*dt;
    q[1] += qDot[1]*dt;
    q[2] += qDot[2]*dt;
    q[3] += qDot[3]*dt;

    QuatNormalize(q);
return q;



}


