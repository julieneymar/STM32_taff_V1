/*
 * app_LQR.c
 *
 *  Created on: Feb 26, 2026
 *      Author: julie
 */


/*
 * app_lqr.c
 *
 *  Created on: Feb 27, 2026
 *      Author: julie
 */

#include "app_LQR.h"


// État du système
float x_pose = 0;           // Position (m)
float x_speed = 0;          // Vitesse linéaire (m/s)
float angle_x = 0;          // Angle d'inclinaison (rad)
float gyro_x = 0;           // Vitesse angulaire inclinaison (rad/s)
float angle_z = 0;          // Angle de rotation (rad)
float gyro_z = 0;           // Vitesse angulaire rotation (rad/s)
float last_angle = 0;       // Angle précédent

// Accélérations calculées
float L_accel = 0;          // Accélération roue gauche
float R_accel = 0;          // Accélération roue droite
float velocity_L = 0;       // Vitesse roue gauche
float velocity_R = 0;       // Vitesse roue droite

// Coefficients LQR (gains de retour d'état)
float K1 = -62.0484;        // Gain position
float K2 = -73.3232;        // Gain vitesse
float K3 =-300;    //-361.4617;       // Gain angle
float K4 = -30;  //-35.9024;        // Gain gyro
float K5 = 15.8114;         // Gain rotation
float K6 = 15.8114;         // Gain vitesse rotation
float K5OLD = 15.8114;      // Sauvegarde K5
float K6OLD = 15.8114;      // Sauvegarde K6

// Valeurs cibles
float Target_x_speed = 0;         // Vitesse cible (m/s)
float Target_angle_x = 0.0349;    // Angle cible (~2°)
float Target_gyro_z = 0;          // Vitesse rotation cible (rad/s)

// Constantes
float Ratio_accel = 2400;         // Ratio vitesse -> PWM
float Control_Frequency = 200;    // Fréquence contrôle (Hz)
float Diameter_67 = 67;           // Diamètre roue (mm)
float Wheel_spacing = 150;        // Espacement roues (mm)

// Variables externes
extern float Angle_Balance;
extern float Gyro_Balance;
extern float Gyro_Turn;
extern float battery;
extern uint8_t Stop_Flag;
extern float Car_Target_Velocity;
extern float Car_Turn_Amplitude_speed;




void LQR_Balance_Only(void)
{
	int Encoder_Left, Encoder_Right;
	int Motor_Left, Motor_Right;

	//  Lire angle et gyroscope
	Get_Angle(GET_Angle_Way);

	// Lire encodeurs
	Encoder_Left = Read_Encoder(MOTOR_ID_ML);
	Encoder_Right = -Read_Encoder(MOTOR_ID_MR);
	Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);

	// Calculer vitesse linéaire (m/s)
	x_speed = (Encoder_Left + Encoder_Right) / 2.0f * PI * Diameter_67 / 1000.0f / 1560.0f * Control_Frequency;
	x_pose += x_speed / Control_Frequency;

	// Obtenir angle d'inclinaison (rad) et vitesse angulaire (rad/s)
	angle_x = Angle_Balance / 180.0f * PI;
	gyro_x = (angle_x - last_angle) * Control_Frequency;
	last_angle = angle_x;

	// Obtenir vitesse de rotation (rad/s) et angle de rotation (rad)
	gyro_z = (Encoder_Right - Encoder_Left) / Wheel_spacing / 1000.0f * PI * Diameter_67 / 1000.0f / 1560.0f * Control_Frequency;
	angle_z += gyro_z / Control_Frequency;

	// ÉQUILIBRE UNIQUEMENT - Pas de mouvement
	//Target_x_speed = 6/10.0f;      // Vitesse nulle
	//Target_gyro_z = 0;       // Pas de rotation
	//K5 = K5OLD;
	//K6 = K6OLD;
	Target_x_speed =  0; // Car_Target_Velocity / 10.0f;  // Conversion
	x_pose = 0;  // Reset position pour vitesse constante
	Target_gyro_z = -( Car_Turn_Amplitude_speed / 3.0f);  // Rotation
	angle_z = 0;

	//  Calculer les accélérations (contrôleur LQR)
	L_accel = -(K1 * x_pose +
				K2 * (x_speed - Target_x_speed) +
				K3 * (angle_x - Target_angle_x) +
				K4 * gyro_x +
				K5 * angle_z +
				K6 * (gyro_z - Target_gyro_z));

	R_accel = -(K1 * x_pose +
				K2 * (x_speed - Target_x_speed) +
				K3 * (angle_x - Target_angle_x) +
				K4 * gyro_x -
				K5 * angle_z -
				K6 * (gyro_z - Target_gyro_z));

	// 8 Convertir vitesse en PWM
	velocity_L = (int)(Ratio_accel * (x_speed + L_accel / Control_Frequency));
	velocity_R = (int)(Ratio_accel * (x_speed + R_accel / Control_Frequency));

	//  Limiter PWM
	Motor_Left = PWM_Limit(velocity_L, 2600, -2600);
	Motor_Right = PWM_Limit(velocity_R, 2600, -2600);

	Motor_Left = PWM_Ignore(Motor_Left);
	Motor_Right = PWM_Ignore(Motor_Right);

	if(Turn_Off(Angle_Balance, battery) == 0)
	{
		Set_Pwm(Motor_Left, Motor_Right);
	}
}



void Dif_mouvement (){
	 // ---------------Avant-------------

	        Target_x_speed = Car_Target_Velocity / 10.0f;  // Conversion
	        x_pose = 0;  // Reset position pour vitesse constante
	        Target_gyro_z = Car_Turn_Amplitude_speed / 3.0f;  // Rotation
	        angle_z = 0;

	     //-------------- Arrière-----------

	        Target_x_speed = Car_Target_Velocity / 10.0f;
	        x_pose = 0;
	        Target_gyro_z = Car_Turn_Amplitude_speed / 3.0f;
	        angle_z = 0;

	    //-------- Rotation sur place------------------

	        Target_x_speed = 0;
	        Target_gyro_z = Car_Turn_Amplitude_speed / 3.0f;
	        angle_z = 0;
	        K5 = 22.3607;  // Gains augmentés pour rotation
	        K6 = 22.3607;

	      // -------------Arrêt-----------

	        Target_x_speed = 0;
	        Target_gyro_z = 0;
	        K5 = K5OLD;
	        K6 = K6OLD;

}
