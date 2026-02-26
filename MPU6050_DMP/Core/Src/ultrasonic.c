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

/**
 * @brief Obtenir la distance mesurée - VERSION SIMPLIFIÉE
 */

void APP_avoid(void)
{
	if(g_distance<14)
	{
		/*
			Move_X = -6;
			delay_us(1);

			Move_X = 0;
			Move_Z = 6;
			delay_us(2); */
		Car_Target_Velocity = -6.0f;   // Reculer
	    Car_Turn_Amplitude_speed = 6.0f; // Tourner
	}
	else
	{
			//Move_X = 6;
			//Move_Z = 0;
		 Car_Target_Velocity = 6.0f;    // Avancer
		  Car_Turn_Amplitude_speed = 0;
	}


}

void Get_Distane(void)
{
	 TRIG_SIG = 1;
	 delay_us(15);
	 TRIG_SIG = 0;
	 if(TIM2CH2_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ //Successfully captured a high level once
	 {
		 g_distance=TIM2CH2_CAPTURE_STA&0X3F;
		 g_distance*=65536;					        //���ʱ���ܺ� Overflow time sum
		 g_distance+=TIM2CH2_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ�� Get the total high level time
		 g_distance=g_distance*170/1000;      //ʱ��*����/2�����أ� һ������0.001ms  Time * speed of sound/2 (round trip), one count 0.001ms
		 TIM2CH2_CAPTURE_STA=0;			//������һ�β��� Start the next capture
	 }
}
/*
void Get_Distane(void)
{
    // Reset des flags avant la mesure
    TIM2CH2_CAPTURE_STA = 0;
    TIM2CH2_CAPTURE_VAL = 0;

    // Envoyer une impulsion TRIG de 15µs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    // Attendre la fin de la capture (max 60ms pour HC-SR04)
    uint32_t start_tick = HAL_GetTick();
    while (!(TIM2CH2_CAPTURE_STA & 0x80))
    {
        if ((HAL_GetTick() - start_tick) > 60)  // Timeout 60ms
        {
            g_distance = 0;  // Hors de portée
            TIM2CH2_CAPTURE_STA = 0;
            return;
        }
    }

    // Calcul de la distance
    uint32_t capture_time = (TIM2CH2_CAPTURE_STA & 0x3F);
    capture_time = capture_time * 65536 + TIM2CH2_CAPTURE_VAL;
    g_distance = (capture_time * 170) / 1000;  // Distance en mm

    // Limiter la distance max à 4000mm (4m)
    if (g_distance > 4000)
    {
        g_distance = 0;
    }
}
*/

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
/*
void Get_Distane(void)
{
    TIM2CH2_CAPTURE_STA = 0;
    TIM2CH2_CAPTURE_VAL = 0;

    // TRIG
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    printf("[DEBUG] Waiting for echo...\r\n");

    uint32_t start_tick = HAL_GetTick();
    while (!(TIM2CH2_CAPTURE_STA & 0x80))
    {
        if ((HAL_GetTick() - start_tick) > 60)
        {
            printf("[DEBUG] TIMEOUT! STA=0x%02X\r\n", TIM2CH2_CAPTURE_STA);
            g_distance = 0;
            TIM2CH2_CAPTURE_STA = 0;
            return;
        }
    }

    printf("[DEBUG] Captured! STA=0x%02X, VAL=%u\r\n",
           TIM2CH2_CAPTURE_STA, TIM2CH2_CAPTURE_VAL);

    uint32_t capture_time = (TIM2CH2_CAPTURE_STA & 0x3F) * 65536 + TIM2CH2_CAPTURE_VAL;
    g_distance = (capture_time * 170) / 1000;

    printf("[DEBUG] Time=%lu µs, Distance=%lu mm\r\n", capture_time, g_distance);
}
*/
