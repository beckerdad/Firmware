//
//  quaternion.h
//
//  Created by Johannes van Niekerk on 6/1/12.
//

#ifndef Ridecaster_quaternion_h
#define Ridecaster_quaternion_h

void QuatOfEulers(float *eulers,float *result);
void QuatMultiply(float *q, float *r,float *result);
void QuatScale(float *q, float scale);
void QuatNormalize(float *q);
void QuatConjugate(float *q);
void QuatAdd(float *q, float *r);

#endif
