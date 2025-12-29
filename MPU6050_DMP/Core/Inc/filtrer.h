/*
 * filtrer.h
 *
 *  Created on: Dec 16, 2025
 *      Author: julie
 */

#ifndef INC_FILTRER_H_
#define INC_FILTRER_H_

#ifndef __FILTER_H
#define __FILTER_H
float Kalman_Filter_x(float Accel,float Gyro);
float Complementary_Filter_x(float angle_m, float gyro_m);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_y(float angle_m, float gyro_m);

#endif


#endif /* INC_FILTRER_H_ */
