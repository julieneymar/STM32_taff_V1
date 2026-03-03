/*
 * ultrasonic.c
 *
 *  Created on: Feb 9, 2026
 *      Author: julie
 */
#include "ultrasonic.h"

volatile uint16_t TIM2CH2_CAPTURE_STA = 0;
volatile uint16_t TIM2CH2_CAPTURE_VAL = 0;
volatile uint32_t g_distance = 0;

extern TIM_HandleTypeDef htim2;

/**
 * @brief Initialisation du capteur ultrasonique
 */
void Ultrasonic_Init(void)
{
    // Initialiser les variables
    TIM2CH2_CAPTURE_STA = 0;
    TIM2CH2_CAPTURE_VAL = 0;
    g_distance = 0;

    // Démarrer TIM2 avec interruptions
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

    printf("[INFO] Ultrasonic sensor initialized (TIM2 CH2)\r\n");
}



void APP_avoid()
{
	if(g_distance < 20)
	{
		/*
			Move_X = -6;
			delay_us(1);

			Move_X = 0;
			Move_Z = 6;
			delay_us(2); */
		 Car_Target_Velocity = 0;
		 Car_Turn_Amplitude_speed = 2.0f;
		 //printf("tourner dis = %ld mm \n", g_distance);

	}
	else
	{

			//Move_X = 6;
			//Move_Z = 0;
		Car_Target_Velocity = 2.0f;    // Avancer
		Car_Turn_Amplitude_speed = 0;
		//printf(" avancer dis = %ld mm \n", g_distance);

	}


}

void Get_Distane(void)
{
	 TRIG_SIG = 1;
	 delay_us(15);
	 TRIG_SIG = 0;
	 if(TIM2CH2_CAPTURE_STA&0X80) //Successfully captured a high level once
	 {
		 g_distance=TIM2CH2_CAPTURE_STA&0X3F;
		 g_distance*=65536;					        // Overflow time sum
		 g_distance+=TIM2CH2_CAPTURE_VAL;		//Get the total high level time
		 g_distance=g_distance*170/1000;      //  Time * speed of sound/2 (round trip), one count 0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//Start the next capture
	 }

}


/**
 * @brief Callback interruption débordement
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if ((TIM2CH2_CAPTURE_STA & 0x80) == 0)  // Pas encore fini
        {
            if (TIM2CH2_CAPTURE_STA & 0x40)  // Front montant capturé
            {
                if ((TIM2CH2_CAPTURE_STA & 0x3F) == 0x3F)
                {
                    // Trop de débordements
                    TIM2CH2_CAPTURE_STA |= 0x80;
                    TIM2CH2_CAPTURE_VAL = 0xFFFF;
                }
                else
                {
                    TIM2CH2_CAPTURE_STA++;
                }
            }
        }
    }
}

/**
 * @brief Callback interruption capture
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if ((TIM2CH2_CAPTURE_STA & 0x80) == 0)  // Pas encore terminé
        {
            if (TIM2CH2_CAPTURE_STA & 0x40)  // Front descendant
            {
                // FIN - Capturer la valeur
                TIM2CH2_CAPTURE_STA |= 0x80;
                TIM2CH2_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                // Reconfigurer pour le prochain front montant
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                               TIM_INPUTCHANNELPOLARITY_RISING);
            }
            else  // Front montant
            {
                // DÉBUT - Initialiser
                TIM2CH2_CAPTURE_STA = 0x40;  // Marquer front montant
                TIM2CH2_CAPTURE_VAL = 0;
                __HAL_TIM_SET_COUNTER(htim, 0);

                // Configurer pour capturer le front descendant
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
                                               TIM_INPUTCHANNELPOLARITY_FALLING);
            }
        }
    }
}

