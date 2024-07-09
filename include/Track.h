#ifndef _Track_H_
#define _Track_H_

float rotation_matrix(float axis[3], float theta);
void Circular_track(float p_start[3], float p_mid[3], float p_final[3], int step, float *center, float *rotation_axis, float *theta_per);

#endif

