//
//  quaternion.c
//  Ridecaster
//
//  Created by Johannes van Niekerk on 6/1/12.
//  Copyright (c) 2012 EpicSessions. All rights reserved.
//

#include <stdio.h>

#include <math.h>

void QuatMultiply(float *q, float *r,float *result)
{
    
    // Calculate vector portion of quaternion product
    // vec = s1*v2 + s2*v1 + cross(v1,v2)
    result[1] = q[0]*r[1] + r[0]*q[1] + q[2]*r[3]-q[3]*r[2];
    result[2] = q[0]*r[2] + r[0]*q[2] + q[3]*r[1]-q[1]*r[3];
    result[3] = q[0]*r[3] + r[0]*q[3] + q[1]*r[2]-q[2]*r[1];
    
    // Calculate scalar portion of quaternion product
    // scalar = s1*s2 - dot(v1,v2)
    result[0] = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];
    
    
}

void VectorCross(float *a, float *b, float *c)
{
    c[0] =  a[1]*b[2]-a[2]*b[1];
    c[1] =  a[2]*b[0]-a[0]*b[2];
    c[2] =  a[0]*b[1]-a[1]*b[0];
}
void QuatScale(float *q, float scale)
{
	q[0] = q[0]*scale;
	q[1] = q[1]*scale;
	q[2] = q[2]*scale;
	q[3] = q[3]*scale;
}

void QuatNormalize(float *q)
{
	float mag = 1.0f/(sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]) + 1e-12f);
	q[0] = q[0]*mag;
	q[1] = q[1]*mag;
	q[2] = q[2]*mag;
	q[3] = q[3]*mag;
}
void QuatConjugate(float *q)
{

	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}

void QuatAdd(float *q, float *r)
{
	q[0]+=r[0];
	q[1]+=r[1];
	q[2]+=r[2];
	q[3]+=r[3];
    
}
void QuatOfEulers(float *eulers,float *result)
{

  float phi2     = eulers[0] / 2.0;
  float theta2   = eulers[1] / 2.0;
  float psi2     = eulers[2] / 2.0;

  float sinphi2   = sin( phi2 );
  float cosphi2   = cos( phi2 );
  float sintheta2 = sin( theta2 );
  float costheta2 = cos( theta2 );
  float sinpsi2   = sin( psi2 );
  float cospsi2   = cos( psi2 );

  result[0] = cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
  result[1] = sinphi2 * costheta2 * cospsi2 - cosphi2 * sintheta2 * sinpsi2;
  result[2] = cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
  result[3] = cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;

}
