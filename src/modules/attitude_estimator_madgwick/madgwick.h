//
//  madgwick.h
//  Created by Johannes van Niekerk on 6/1/12.
//

#ifndef Ridecaster_madgwick_h
#define Ridecaster_madgwick_h


void AttitudeInit(float a_x, float a_y, float a_z);
float *AttitudeUpdate(float dt,float _beta, float _zeta, float a_x,float a_y, float a_z,float w_x, float w_y, float w_z,float m_x, float m_y, float m_z);
void getBiases(float *data);

#endif
