

#ifndef INC_APP_LQR_H_
#define INC_APP_LQR_H_

#include "AllHeader.h"
#include <math.h>

// Variables d'état LQR (exportées)
extern float x_pose;
extern float x_speed;
extern float angle_x;
extern float gyro_x;
extern float angle_z;
extern float gyro_z;

// Coefficients LQR
extern float K1, K2, K3, K4, K5, K6;
extern float K5OLD, K6OLD;

// Valeurs cibles
extern float Target_x_speed;
extern float Target_angle_x;
extern float Target_gyro_z;

// Constantes
extern float Ratio_accel;
extern float Control_Frequency;
extern float Diameter_67;
extern float Wheel_spacing;

// Fonctions
void LQR_Control(void);
void LQR_Balance_Only(void);

#endif /* INC_APP_LQR_H_ */
