/*
 * app_ps2.c
 *
 *  Created on: Feb 11, 2026
 *      Author: julie
 */

#include "app_ps2.h"

int g_PS2_LX, g_PS2_LY, g_PS2_RX, g_PS2_RY, g_PS2_KEY;
int g_flag = 1; //  Mode print interlock flag


//Key value testing function
void PS2_Data_Show(void)
{

	g_PS2_LX = PS2_AnologData(PSS_LX);
	g_PS2_LY = PS2_AnologData(PSS_LY);
	g_PS2_RX = PS2_AnologData(PSS_RX);
	g_PS2_RY = PS2_AnologData(PSS_RY);
	g_PS2_KEY = PS2_DataKey();


	if ((g_PS2_LX == 255) && (g_PS2_LY == 255) && (g_PS2_RX == 255) && (g_PS2_RY == 255))
	{
		if (g_flag == 1)
		{
			printf("PS2 mode is RED or GREEN mode \r\n");
			printf("Or PS2 disconnect! \r\n");
			g_flag = 0;
		}
	}

	else
	{
		if (g_flag == 0)
		{
			printf("PS2 mode is RED^GREEN mode \r\n");
			g_flag = 1;
		}
		// Only the red green mode has the correct joystick value ֻ�к���ģʽ������ȷ��ҡ��ֵ Only red and green mode has the correct joystick value
		if (g_PS2_LX > 130 || g_PS2_LX < 100)
			printf("PS2_LX = %d \r\n", g_PS2_LX);
		if (g_PS2_LY > 130 || g_PS2_LY < 100)
			printf("PS2_LY = %d \r\n", g_PS2_LY);
		if (g_PS2_RX > 130 || g_PS2_RX < 100)
			printf("PS2_RX = %d \r\n", g_PS2_RX);
		if (g_PS2_RY > 130 || g_PS2_RY < 100)
			printf("PS2_RY = %d \r\n", g_PS2_RY);
	}

	switch (g_PS2_KEY)
	{
	case PSB_SELECT:
		printf("key = PSB_SELECT\r\n");
		OLED_Draw_Line("key = PSB_SELECT", 1, true, true);
		break;
	case PSB_L3:
		printf("key = PSB_L3\r\n");
		OLED_Draw_Line("key = PSB_L3", 1, true, true);
		break;
	case PSB_R3:
		printf("key = PSB_R3\r\n");
		OLED_Draw_Line("key = PSB_R3", 1, true, true);
		break;
	case PSB_START:
		printf("key = PSB_START\r\n");
		OLED_Draw_Line("key = PSB_START", 1, true, true);
		break;
	case PSB_PAD_UP:
		printf("key = PSB_PAD_UP\r\n");
		OLED_Draw_Line("key = PSB_PAD_UP", 1, true, true);
		break;
	case PSB_PAD_RIGHT:
		printf("key = PSB_PAD_RIGHT\r\n");
		OLED_Draw_Line("key = PSB_PAD_RIGHT", 1, true, true);
		break;
	case PSB_PAD_DOWN:
		printf("key = PSB_PAD_DOWN\r\n");
		OLED_Draw_Line("key = PSB_PAD_DOWN", 1, true, true);
		break;
	case PSB_PAD_LEFT:
		printf("key = PSB_PAD_LEFT\r\n");
		OLED_Draw_Line("key = PSB_PAD_LEFT", 1, true, true);
		break;
	case PSB_L2:
		printf("key = PSB_L2\r\n");
		OLED_Draw_Line("key = PSB_L2", 1, true, true);
		break;
	case PSB_R2:
		printf("key = PSB_R2\r\n");
		OLED_Draw_Line("key = PSB_R2", 1, true, true);
		break;
	case PSB_L1:
		printf("key = PSB_L1\r\n");
		OLED_Draw_Line("key = PSB_L1", 1, true, true);
		break;
	case PSB_R1:
		printf("key = PSB_R1\r\n");
		OLED_Draw_Line("key = PSB_R1", 1, true, true);
		break;
	case PSB_GREEN:
		printf("key = PSB_GREEN\r\n");
		OLED_Draw_Line("key = PSB_GREEN", 1, true, true);
		break;
	case PSB_RED:
		printf("key = PSB_RED\r\n");
		OLED_Draw_Line("key = PSB_RED", 1, true, true);
		break;
	case PSB_BLUE:
		printf("key = PSB_BLUE\r\n");
		OLED_Draw_Line("key = PSB_BLUE", 1, true, true);
		break;
	case PSB_PINK:
		printf("key = PSB_PINK\r\n");
		OLED_Draw_Line("key = PSB_PINK", 1, true, true);
		break;
	}

	delay_ms(100);
}

/**
 * @brief Contrôle du robot via manette PS2
 * Compatible avec robot auto-équilibrant (utilise Car_Target_Velocity et Car_Turn_Amplitude_speed)
 */
void PS2_Control_Car(void)
{

    // Lecture des valeurs analogiques et boutons
    g_PS2_LX = PS2_AnologData(PSS_LX);
    g_PS2_LY = PS2_AnologData(PSS_LY);
    g_PS2_RX = PS2_AnologData(PSS_RX);
    g_PS2_RY = PS2_AnologData(PSS_RY);
    g_PS2_KEY = PS2_DataKey();

    // Vérifier si la manette est déconnectée
    if ((g_PS2_LX == 255) && (g_PS2_LY == 255) && (g_PS2_RX == 255) && (g_PS2_RY == 255))
    {
        if (g_flag == 1)
        {
            printf("⚠️ PS2 disconnected\r\n");
    		//OLED_Draw_Line("PS2 DECONNECTER", 1, true, true);

            g_flag = 0;
        }
        // Arrêter le robot si manette déconnectée
        Car_Target_Velocity = 0;
        Car_Turn_Amplitude_speed = 0;
        return;
    }
    else
    {
        if (g_flag == 0)
        {
            printf("✓ PS2 connected\r\n");
    		//OLED_Draw_Line("PS2 CONNECTER", 1, true, true);

            g_flag = 1;
        }
    }

    // CONTRÔLE PAR JOYSTICK GAUCHE (Prioritaire)

    // Avant-Gauche
    if((g_PS2_LX < 50 && g_PS2_LY < 50) || (g_PS2_RX < 50 && g_PS2_RY < 50))
    {
        Car_Target_Velocity = 6.0f;
        Car_Turn_Amplitude_speed = -6.0f;
		//OLED_Draw_Line("AVANT-GAUCHE", 1, true, true);

    }
    // Avant-Droite
    else if((g_PS2_LX > 150 && g_PS2_LY < 50) || (g_PS2_RX > 150 && g_PS2_RY < 50))
    {
        Car_Target_Velocity = 6.0f;
        Car_Turn_Amplitude_speed = 6.0f;
		//OLED_Draw_Line("AVANT-DROITE", 1, true, true);

    }
    // Arrière-Gauche
    else if((g_PS2_LX < 50 && g_PS2_LY > 150) || (g_PS2_RX < 50 && g_PS2_RY > 150))
    {
        Car_Target_Velocity = -6.0f;
        Car_Turn_Amplitude_speed = -6.0f;
		//OLED_Draw_Line("ARRIERE - GAUCHE", 1, true, true);

    }
    // Arrière-Droite
    else if((g_PS2_LX > 150 && g_PS2_LY > 150) || (g_PS2_RX > 150 && g_PS2_RY > 150))
    {
        Car_Target_Velocity = -6.0f;
        Car_Turn_Amplitude_speed = 6.0f;
		//OLED_Draw_Line("ARRIERE-DROITE", 1, true, true);

    }
    // Avant (priorité haute)
    else if((g_PS2_LY < 90) || (g_PS2_RY < 90))
    {
        Car_Target_Velocity = 6.0f;
        Car_Turn_Amplitude_speed = 0;
		//OLED_Draw_Line("DEVANT", 1, true, true);

    }
    // Arrière
    else if((g_PS2_LY > 150) || (g_PS2_RY > 150))
    {
        Car_Target_Velocity = -6.0f;
        Car_Turn_Amplitude_speed = 0;
		//OLED_Draw_Line("DERRIERE", 1, true, true);

    }
    // Gauche (rotation sur place)
    else if((g_PS2_LX < 90) || (g_PS2_RX < 90))
    {
        Car_Target_Velocity = 0;
        Car_Turn_Amplitude_speed = -6.0f;
		//OLED_Draw_Line("GAUCHE", 1, true, true);

    }
    // Droite (rotation sur place)
    else if((g_PS2_LX > 150) || (g_PS2_RX > 150))
    {
        Car_Target_Velocity = 0;
        Car_Turn_Amplitude_speed = 6.0f;
		//OLED_Draw_Line("DROITE", 1, true, true);

    }
    // Joystick centré + pas de bouton = STOP
    else
    {
        if (g_PS2_KEY == 0)
        {
            Car_Target_Velocity = 0;
            Car_Turn_Amplitude_speed = 0;
        }
    }

    // CONTRÔLE PAR BOUTONS (Peut modifier vitesse)
    switch (g_PS2_KEY)
    {
        // ===== ACCÉLÉRATION (L1/L2) =====
        case PSB_L1:
        case PSB_L2:
            // Augmenter la vitesse
            if(Car_Target_Velocity > 0)
                Car_Target_Velocity += 2.0f;
            else if(Car_Target_Velocity < 0)
                Car_Target_Velocity -= 2.0f;

            // Augmenter vitesse de rotation
            if(Car_Turn_Amplitude_speed > 0)
                Car_Turn_Amplitude_speed += 2.0f;
            else if(Car_Turn_Amplitude_speed < 0)
                Car_Turn_Amplitude_speed -= 2.0f;

            // Limites max
            if(Car_Target_Velocity > 15.0f)
                Car_Target_Velocity = 15.0f;
            if(Car_Target_Velocity < -15.0f)
                Car_Target_Velocity = -15.0f;
            if(Car_Turn_Amplitude_speed > 20.0f)
                Car_Turn_Amplitude_speed = 20.0f;
            if(Car_Turn_Amplitude_speed < -20.0f)
                Car_Turn_Amplitude_speed = -20.0f;

            printf("⚡ SPEED UP: V=%.1f T=%.1f\r\n",
                   Car_Target_Velocity, Car_Turn_Amplitude_speed);
            break;

        // ===== DÉCÉLÉRATION (R1/R2) =====
        case PSB_R1:
        case PSB_R2:
            // Réduire la vitesse
            if(Car_Target_Velocity > 0)
                Car_Target_Velocity -= 2.0f;
            else if(Car_Target_Velocity < 0)
                Car_Target_Velocity += 2.0f;

            // Réduire vitesse de rotation
            if(Car_Turn_Amplitude_speed > 0)
                Car_Turn_Amplitude_speed -= 2.0f;
            else if(Car_Turn_Amplitude_speed < 0)
                Car_Turn_Amplitude_speed += 2.0f;

            // Limites min
            if(Car_Target_Velocity > -2.0f && Car_Target_Velocity < 2.0f)
                Car_Target_Velocity = 0;
            if(Car_Turn_Amplitude_speed > -2.0f && Car_Turn_Amplitude_speed < 2.0f)
                Car_Turn_Amplitude_speed = 0;

            printf("🐌 SPEED DOWN: V=%.1f T=%.1f\r\n",
                   Car_Target_Velocity, Car_Turn_Amplitude_speed);
            break;

        // ===== DIRECTIONS PAR BOUTONS (Si joystick centré) =====
        case PSB_PAD_UP:
        case PSB_GREEN:  // Triangle
            if((g_PS2_LY > 90 && g_PS2_LY < 150) && (g_PS2_LX > 90 && g_PS2_LX < 150))
            {
                Car_Target_Velocity = 6.0f;
                Car_Turn_Amplitude_speed = 0;
                //printf("↑ FORWARD\r\n");
        		//OLED_Draw_Line("DEVANT", 1, true, true);

            }
            break;

        case PSB_PAD_DOWN:
        case PSB_BLUE:  // Cross
            if((g_PS2_LY > 90 && g_PS2_LY < 150) && (g_PS2_LX > 90 && g_PS2_LX < 150))
            {
                Car_Target_Velocity = -6.0f;
                Car_Turn_Amplitude_speed = 0;
               // printf("↓ BACKWARD\r\n");
        		//OLED_Draw_Line("DERRIERE", 1, true, true);

            }
            break;

        case PSB_PAD_LEFT:
        case PSB_PINK:  // Square
            if((g_PS2_LY > 90 && g_PS2_LY < 150) && (g_PS2_LX > 90 && g_PS2_LX < 150))
            {
                Car_Target_Velocity = 0;
                Car_Turn_Amplitude_speed = -6.0f;
             //   printf("← LEFT\r\n");
        		//OLED_Draw_Line("GAUCHE", 1, true, true);

            }
            break;

        case PSB_PAD_RIGHT:
        case PSB_RED:  // Circle
            if((g_PS2_LY > 90 && g_PS2_LY < 150) && (g_PS2_LX > 90 && g_PS2_LX < 150))
            {
                Car_Target_Velocity = 0;
                Car_Turn_Amplitude_speed = 6.0f;
               // printf("→ RIGHT\r\n");
        		//OLED_Draw_Line("DROITE", 1, true, true);

            }
            break;

        // ===== CONTRÔLES SYSTÈME =====
        case PSB_START:
            Stop_Flag = 0;
            printf("▶️ ROBOT STARTED\r\n");
            OLED_Draw_Line("ROBOT ON", 1, true, true);
            break;

        case PSB_SELECT:
            Stop_Flag = 1;
            Car_Target_Velocity = 0;
            Car_Turn_Amplitude_speed = 0;
            printf("■ ROBOT STOPPED\r\n");
            OLED_Draw_Line("ROBOT OFF", 1, true, true);
            break;

        // ===== ARRÊT SI JOYSTICK CENTRÉ =====
        default:
            if(g_flag == 1)  // Mode analog
            {
                // Si joysticks centrés et pas de bouton
                if(((g_PS2_LY > 90 && g_PS2_LY < 140) && (g_PS2_LX > 90 && g_PS2_LX < 140)) &&
                   ((g_PS2_RY > 90 && g_PS2_RY < 140) && (g_PS2_RX > 90 && g_PS2_RX < 140)))
                {
                    Car_Target_Velocity = 0;
                    Car_Turn_Amplitude_speed = 0;
                }
            }
            break;
    }

    // ========================================
    // LIMITES DE SÉCURITÉ FINALES
    // ========================================
    if(Car_Target_Velocity > 15.0f)
        Car_Target_Velocity = 15.0f;
    if(Car_Target_Velocity < -15.0f)
        Car_Target_Velocity = -15.0f;

    if(Car_Turn_Amplitude_speed > 20.0f)
        Car_Turn_Amplitude_speed = 20.0f;
    if(Car_Turn_Amplitude_speed < -20.0f)
        Car_Turn_Amplitude_speed = -20.0f;
}
