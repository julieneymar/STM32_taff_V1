/*
 * adc.c
 *
 *  Created on: Feb 9, 2026
 *      Author: julie
 */
#include "adc.h"

/**
  * @brief  Récupère une valeur ADC unique d'un canal spécifique
  * @param  hadc: Handle de l'ADC (ex: &hadc1)
  * @param  channel: Canal ADC à lire (ADC_CHANNEL_x)
  * @retval Valeur ADC sur 12 bits (0-4095)
  */
static uint16_t Battery_Get(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    uint16_t adc_value = 0;
    uint32_t timeout_start;


    // 2. Démarrage de la conversion
    HAL_ADC_Start(hadc);

    // 3. Attente de la fin de conversion avec timeout
    timeout_start = HAL_GetTick();
    if (HAL_ADC_PollForConversion(hadc, ADC_TIMEOUT) != HAL_OK)
    {
        // Gestion du timeout
        HAL_ADC_Stop(hadc);
        return 0xFFFF;  // Valeur d'erreur
    }

    // 4. Lecture de la valeur
    adc_value = HAL_ADC_GetValue(hadc);

    // 5. Arrêt de la conversion
    HAL_ADC_Stop(hadc);

    return adc_value;
}

/**
  * @brief  Calcule la moyenne de plusieurs mesures ADC
  * @param  hadc: Handle de l'ADC
  * @param  channel: Canal ADC
  * @param  times: Nombre de mesures pour la moyenne
  * @retval Moyenne des mesures
  */
uint16_t Battery_Get_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint8_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;

    // Vérification des paramètres
    if (times == 0) return 0;

    // Acquisition des mesures
    for (t = 0; t < times; t++)
    {
        temp_val += Battery_Get(hadc, channel);
        HAL_Delay(1);  // Petit délai entre les mesures
    }

    // Calcul de la moyenne avec optimisation
    if (times == 4)
    {
        // Division par 4 optimisée avec décalage
        temp_val = temp_val >> 2;
    }
    else if ((times & (times - 1)) == 0)
    {
        // Si times est une puissance de 2, utiliser le décalage
        uint8_t shift = 0;
        while ((1 << shift) < times) shift++;
        temp_val = temp_val >> shift;
    }
    else
    {
        // Division normale pour les autres cas
        temp_val = temp_val / times;
    }

    return (uint16_t)temp_val;
}

/**
  * @brief  Calcule la tension mesurée après diviseur
  * @param  hadc: Handle de l'ADC
  * @retval Tension en volts (0-3.3V)
  */
float Get_Measure_Voltage(ADC_HandleTypeDef* hadc)
{
    uint16_t adcx;
    float temp;

    // Option 1: Lecture simple
    // adcx = Battery_Get(hadc, BAT_ADC_CH);

    // Option 2: Lecture avec moyennage (recommandé)
    adcx = Battery_Get_Average(hadc, BAT_ADC_CH, 4);

    // Conversion en tension
    // V_mesurée = (Valeur_ADC × Vref) / Résolution
    // Vref = 3.3V, Résolution = 4096 (12 bits)
    temp = (float)adcx * (3.30f / 4096.0f);

    return temp;
}

/**
  * @brief  Calcule la tension réelle de la batterie
  * @param  hadc: Handle de l'ADC
  * @retval Tension de la batterie en volts
  */
float Get_Battery_Voltage(ADC_HandleTypeDef* hadc)
{
    float temp;

    // 1. Lecture de la tension après diviseur
    temp = Get_Measure_Voltage(hadc);

    // 2. Application du coefficient du diviseur
    // Diviseur: R1=10kΩ, R2=3.3kΩ
    // Rapport théorique = (R1+R2)/R2 = (10000+3300)/3300 = 4.0303
    // Coefficient ajusté pour compensation: 4.03
    temp = temp * 4.03f;

    return temp;
}

// Déclaration globale
volatile uint16_t adc_dma_buffer[10];
volatile uint8_t adc_ready = 0;

// Initialisation avec DMA
void Battery_ADC_Init_DMA(ADC_HandleTypeDef* hadc)
{
    // Configuration multi-canaux si nécessaire
    if (HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_dma_buffer, 10) != HAL_OK)
    {
        Error_Handler();
    }
}

// Callback DMA
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    adc_ready = 1;
}

// Lecture avec DMA
float Get_Battery_Voltage_DMA(void)
{
    uint32_t sum = 0;
    uint8_t i;

    if (!adc_ready) return 0.0f;

    // Calcul de la moyenne
    for (i = 0; i < 10; i++)
    {
        sum += adc_dma_buffer[i];
    }

    // Conversion en tension
    float voltage_adc = (float)(sum / 10) * (3.30f / 4096.0f);
    float voltage_bat = voltage_adc * 4.03f;

    adc_ready = 0;
    return voltage_bat;
}


void adc (void){

	// Lecture simple
	 float battery_voltage = Get_Battery_Voltage(&hadc1);
	 // Affichage (via UART)
	 printf("Battery: %.2f V\r\n", battery_voltage);
	 HAL_Delay(1000);
		}
