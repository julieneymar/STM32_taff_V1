/*
 * app_control.c
 *
 *  Created on: Dec 15, 2025
 *      Author: julie
 */

#include "AllHeader.h"
#include "app_control.h"
extern UART_HandleTypeDef huart1;


//static u16 intstop_time =0 ;
float battery = 12;// 12v The initial state is fully charged 12v
uint8_t Start_Flag = 0; // Valeur initiale
//int ps2_conut = 0;  // Variable globale initialisée à 0


extern uint8_t mpu_data_ready ;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == MPU6050_Int_Pin && Stop_Flag == 0)
    {
    	 LQR_Balance_Only();

/*
	// 1️⃣ Lire l'angle et le gyro
		Get_Angle(GET_Angle_Way);

		// 2️⃣ Lire les encodeurs
		int Encoder_Left = Read_Encoder(MOTOR_ID_ML);
		int Encoder_Right = -Read_Encoder(MOTOR_ID_MR);

		// 3️⃣ Calculer vitesse
		Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);

		// 4️⃣ PID équilibre
		int Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);

		// 5️⃣ PID vitesse
		int Velocity_Pwm = Velocity_PI(Encoder_Left, Encoder_Right);

		// 6️⃣ PID direction
		int Turn_Pwm = Turn_PD(Gyro_Turn);

		// 7️⃣ Calculer PWM finaux
		int Motor_Left = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
		int Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm;

		// 8️⃣ Filtrer zone morte
		Motor_Left = PWM_Ignore(Motor_Left);
		Motor_Right = PWM_Ignore(Motor_Right);

		// 9️⃣ Limiter PWM
		Motor_Left = PWM_Limit(Motor_Left, 2600, -2600);
		Motor_Right = PWM_Limit(Motor_Right, 2600, -2600);

		// 🔟 Application PWM (si pas d'anomalie)
		if(Stop_Flag == 0 && Turn_Off(Angle_Balance, battery) == 0) {
			Set_Pwm(Motor_Left, Motor_Right);
		} else {
			Set_Pwm(0, 0);
		}


*/
    }
}


/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none

**************************************************************************/


void Get_Angle(u8 way)
{
	float gyro_x,gyro_y,accel_x,accel_y,accel_z,gyro_z;
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;



	//Temperature=Read_Temperature();        //Read the data from the MPU6050 built-in temperature sensor, which ap temperature.
	if(way==1)                           //The reading of DMP is interrupted during data collection, string requirements
	{
		Read_DMP();                      	 //Read acceleration, angular velocity, and tilt angle

  // ✅ UTILISATION DIRECTE DES DONNÉES DMP (pas de filtre)
		Angle_Balance = Pitch;      // Angle d'inclinaison
		Gyro_Balance = gyro[0];     // Vitesse angulaire
		Gyro_Turn = gyro[2];        // Gyro de rotation
		Acceleration_Z = accel[2];  // Accélération Z

	}
	else
	{

		 Read_MPU6050_Burst(&gyro_x, &gyro_y, &gyro_z, &accel_x, &accel_y, &accel_z);

		if(GET_Angle_Way==2)
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//Kalman filtering
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;


		}
		else if(GET_Angle_Way==3)
		{

			 Accel_Angle_x = atan2(accel_y, accel_z) * 180.0f / PI;
			 Accel_Angle_y = atan2(accel_x, accel_z) * 180.0f / PI;

			 Pitch = -Complementary_Filter_x(Accel_Angle_x, gyro_x / (16.4f / 939.8f));
			 Roll = -Complementary_Filter_y(Accel_Angle_y, gyro_y / (16.4f / 939.8f));
		}
		Angle_Balance=Pitch;                              //    Update the balance tilt angle

	}

}

// ========================================
// FONCTION OPTIMISÉE : Lecture I2C Burst
// ========================================
void Read_MPU6050_Burst(float *gyro_x, float *gyro_y, float *gyro_z,
                        float *accel_x, float *accel_y, float *accel_z)
{
    uint8_t buffer[14];
    int16_t Gyro_X, Gyro_Y, Gyro_Z;
    int16_t Accel_X, Accel_Y, Accel_Z;

    // ✅ LECTURE BURST : 14 octets en UNE SEULE transaction I2C
    // Au lieu de 12 appels I2C_ReadOneByte() → gain de ~1000µs !
    IICreadBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);

    // Parsing des données (même méthode qu'avant)
    Accel_X = (buffer[0] << 8) | buffer[1];
    Accel_Y = (buffer[2] << 8) | buffer[3];
    Accel_Z = (buffer[4] << 8) | buffer[5];
    // buffer[6-7] = température (ignorée)
    Gyro_X = (buffer[8] << 8) | buffer[9];
    Gyro_Y = (buffer[10] << 8) | buffer[11];
    Gyro_Z = (buffer[12] << 8) | buffer[13];

    // Conversion signée (exactement comme avant)
    if(Gyro_X > 32768)  Gyro_X -= 65536;
    if(Gyro_Y > 32768)  Gyro_Y -= 65536;
    if(Gyro_Z > 32768)  Gyro_Z -= 65536;
    if(Accel_X > 32768) Accel_X -= 65536;
    if(Accel_Y > 32768) Accel_Y -= 65536;
    if(Accel_Z > 32768) Accel_Z -= 65536;

    // Conversion en unités physiques (mêmes facteurs qu'avant)
    *accel_x = Accel_X / 1671.84f;
    *accel_y = Accel_Y / 1671.84f;
    *accel_z = Accel_Z / 1671.84f;
    *gyro_x = Gyro_X / 939.8f;
    *gyro_y = Gyro_Y / 939.8f;
    *gyro_z = Gyro_Z / 939.8f;

    // Mise à jour gyro brut pour Gyro_Balance
    Gyro_Balance = -Gyro_X;
    Gyro_Turn = Gyro_Z;
    Acceleration_Z = Accel_Z;
}


int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //  Step 1
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<50)               // Condition 1: The car is approaching a standstill
			count0++;
			else
			count0=0;
			if(count0>10)
			flag=1,count0=0;
	 }
	 if(flag==1)                                                      // Go to step 2
	 {
			if(++count1>200)       count1=0,flag=0;                       // No more waiting for 2000ms after timeout, return to the first step
			if(Acceleration>22000&&(Angle>(-20+Mid_Angle))&&(Angle<(20+Mid_Angle)))   // Condition 2, the car is picked up near 0 degrees
			flag=2;
	 }
	 if(flag==2)                                                       // Step 3
	 {
		  if(++count2>100)       count2=0,flag=0;                        // Timeout no longer waits 1000ms
	    if(myabs(encoder_left+encoder_right)>50)                       //    Condition 3: The tires of the car reao positive feedback
      {
				flag=0;
				return 1;                                                    // Detected the car being picked up
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action

**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag;//,count;
	 if(Stop_Flag==0)                     //    Prevent false positives
			return 0;
	 if(flag==0)
	 {
			if(Angle>(-10+Mid_Angle)&&Angle<(10+Mid_Angle)&&encoder_left==0&&encoder_right==0) // Condition 1, the car is around 0 degrees
			flag=1;
	 }
	 if(flag==1)
	 {
//		  if(++count>50)                     // 500ms  Timeout no longer waits 500ms
//		  {
//				count=0;flag=0;
//		  }
		 // Increase sensitivity
	    if((encoder_left>3&&encoder_left<40)||(encoder_right>3&&encoder_right<40))
      {
				flag=0;
				return 1;                         // Detected that the car has been lowered
			}
	 }
	return 0;
}


uint8_t received;
void Serial_Controle(void){

	if(HAL_UART_Receive(&huart1, &received, 1, 10) == HAL_OK) {
	  switch(received) {

		  case 'A': // devant
			  Car_Target_Velocity = 6.0f;
			  Car_Turn_Amplitude_speed = 0;
			  printf(">>> avancer <<<\r\n");
			  break;

		  case 'Z': //arrire
			  Car_Target_Velocity = -6.0f;
			  Car_Turn_Amplitude_speed = 0;
			  printf(">>> reculer <<<\r\n");
			  break;

		  case 'E': // gauche
			  Car_Target_Velocity = 0;
			  Car_Turn_Amplitude_speed = -6.0f;
			  printf(">>>  gauche <<<\r\n");
			  break;

		  case 'R': // droite
			  Car_Target_Velocity = 0;
			  Car_Turn_Amplitude_speed = 6.0f;
			  printf(">>> droite  <<<\r\n");
			  break;

		  case 'T':  // Avant-Gauche
			  Car_Target_Velocity = 6.0f;
			  Car_Turn_Amplitude_speed = -6.0f;
			  printf(">>> avant-gauche <<<\r\n");
			  break;

		  case 'Q':  // Avant-Droite
			  Car_Target_Velocity = 6.0f;
			  Car_Turn_Amplitude_speed = 6.0f;
			  printf(">>> avant-droite <<<\r\n");
			  break;

		  case 'S':  // Arrière-Gauche
			  Car_Target_Velocity = -6.0f;
			  Car_Turn_Amplitude_speed = -6.0f;
			  printf(">>> arriere-gauche <<<\r\n");
			  break;

		  case 'D':  // Arrière-Droite
			  Car_Target_Velocity = -6.0f;
			  Car_Turn_Amplitude_speed = 6.0f;
			  printf(">>> arriere-droite <<<\r\n");
			  break;

		  case 'F':  // Stop
			  Car_Target_Velocity = 0;
			  Car_Turn_Amplitude_speed = 0;
			  printf(">>> STOP <<<\r\n");
			  break;

		  case 'I':  // Info
			  printf("==== STATUS ====\r\n");
			  printf("Velocity: %.1f\r\n", Car_Target_Velocity);
			  printf("Turn: %.1f\r\n", Car_Turn_Amplitude_speed);
			  printf("Angle: %.1f°\r\n", Angle_Balance);
			  printf("================\r\n");
			  break;
	  }
	}

}

/*
void Ultrasonic(void){
	Get_Distane();
    float distance = g_distance;

	 if(distance < 8) {
		// ⚠ TROP PROCHE → RECULER + TOURNER
		Car_Target_Velocity = -6.0f;  // Reculer
		Car_Turn_Amplitude_speed = 6.0f;  // Tourner droite
		printf("⚠️ OBSTACLE %.0fcm → BACK+TURN\r\n", distance);
	} else {
		Car_Target_Velocity = 6.0f;
	    Car_Turn_Amplitude_speed = 0;
	    printf(">>> avancer dis=%lf<<<\r\n",distance);
	}

}
*/

void Ultrasonic(void)
{
    static uint32_t last_trigger = 0;
    static uint8_t waiting_for_measurement = 0;
    static uint32_t last_valid_distance = 0;  // Garder dernière mesure valide

    uint32_t now = HAL_GetTick();

    // 1️⃣ Déclencher une nouvelle mesure toutes les 100ms
    if (!waiting_for_measurement && (now - last_trigger) > 100)
    {
        // Reset et déclencher
        TIM2CH2_CAPTURE_STA = 0;
        TIM2CH2_CAPTURE_VAL = 0;

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        delay_us(15);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

        last_trigger = now;
        waiting_for_measurement = 1;
    }

    // 2️⃣ Vérifier si mesure terminée (sans bloquer)
    if (waiting_for_measurement)
    {
        // Timeout après 60ms
        if ((now - last_trigger) > 60)
        {
            waiting_for_measurement = 0;
            // Garder l'ancienne valeur en cas de timeout
        }
        // Mesure prête
        else if (TIM2CH2_CAPTURE_STA & 0x80)
        {
            uint32_t capture_time = (TIM2CH2_CAPTURE_STA & 0x3F) * 65536 + TIM2CH2_CAPTURE_VAL;
            uint32_t distance_mm = (capture_time * 170) / 1000;

            if (distance_mm > 0 && distance_mm < 4000)
            {
                last_valid_distance = distance_mm;  // Mettre à jour seulement si valide
            }

            waiting_for_measurement = 0;
        }
    }

    // 3️⃣ Utiliser la dernière distance valide pour le contrôle
    float distance_cm = last_valid_distance / 10.0f;

    if (distance_cm > 0 && distance_cm < 15.0f)
    {
        // ⚠️ OBSTACLE PROCHE → Reculer + Tourner
        Car_Target_Velocity = -4.0f;
        Car_Turn_Amplitude_speed = 5.0f;

        // Afficher seulement de temps en temps pour ne pas surcharger UART
        static uint32_t last_print = 0;
        if ((now - last_print) > 200)
        {
            printf("⚠️ OBSTACLE %.1fcm → ÉVITEMENT\r\n", distance_cm);
            last_print = now;
        }
    }
    else if (distance_cm >= 15.0f && distance_cm < 30.0f)
    {
        // Zone intermédiaire → Ralentir
        Car_Target_Velocity = 2.0f;
        Car_Turn_Amplitude_speed = 0;
    }
    else
    {
        // Voie libre → Avancer normalement
        Car_Target_Velocity = 6.0f;
        Car_Turn_Amplitude_speed = 0;
    }
}





