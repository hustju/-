/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct gesture
{
	float angz;
	float angx;
	float angy;
	float w;//???
	float v1;//????
	float v2;//????
	float x1;//????
	float x2;//????
	float x;//???
};
struct gesture gest;//????
struct gesture tgest;//????
struct gesture globalgest;//????
struct PID_InitDefStruct
{
	 //????PID??
	float Velcity_Kp;
	float Velcity_Ki;
	float Velcity_Kd;
	float Relcity_Kp;
	float Relcity_Ki;
	float Relcity_Kd;
	float Ur;				//???
	
  uint8_t PID_is_Enable;		//PID??floar Un;					
	float Un;				//?????
  float En_V1;				//???????
	float En_V2;				//???????
	float En_R1;				//???????
	float En_R2;
	float Count;
	int PWM1;		
  int PWM2;	//??PWM?
	
};
struct PID_InitDefStruct p1,p2;
uint8_t rx_buf1[1]={0};
uint8_t rx_buf2[1]={0};
uint8_t rx_buf3[1]={0};
uint8_t receivebuf[10];
int number1=0;
float EnR=0;
int receivelen=0;
float EnV_p1,EnV_p2;
int control,control2=0;
int sign1=0,sign2=0,sign3=0;
void Wheel(int num,int pwm);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//??????
{
  static float catch1[3],catch2[3];
	int judge1,judge2;
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)//?1
	{
		judge1=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
		catch1[1]=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
		catch1[2]=catch1[1]-catch1[0];
		catch1[0]=catch1[1];
		if(catch1[2]<0)
		{
			catch1[2]+=0xffff;
		}
		if(judge1==1)
		{
			gest.x1+=0.0523;
			gest.v1=523/catch1[2];
		}
		if(judge1==0)
		{
		  gest.x1-=0.0523;//????390???
      gest.v1=-523/catch1[2];			
	  }
	}
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)//??2
	{
		judge2=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
		catch2[1]=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		catch2[2]=catch2[1]-catch2[0];
		catch2[0]=catch2[1];
		if(catch2[2]<0)
		{
			catch2[2]+=0xffff;
		}
		if(judge2==0)
		{
			gest.x2+=0.0523;
      gest.v2=523/catch2[2];			
		}
		if(judge2==1)
		{
		  gest.x2-=0.0523;
      gest.v2=-523/catch2[2];			
	  }
	}
}	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//????
{

	if(huart ==&huart3)
	{
		int result=0,i;
		HAL_UART_Receive_IT(&huart3,rx_buf3,1);
		if(rx_buf3[0]=='b')
		{
			control=1;
		}
		else if(rx_buf3[0]=='c'&&control2==0)
		{
			control=2;
		}
		else{
		receivebuf[receivelen]=rx_buf3[0];
		receivelen++;
		if(rx_buf3[0]=='a')
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
			for(i=0;i<receivelen-1;i++)
			{
				result=result*10+receivebuf[i]-48;
			}
			
				EnR=result*3.2-640;
			
			receivelen=0;
		}
	}
	}
}
void Velcity_PWM()
{
	EnV_p1= 0.3-(gest.v1+gest.v2)/2;//???    
  EnV_p2= 0.3-(gest.v1+gest.v2)/2; 
  p1.PWM1 += p1.Velcity_Kp*EnV_p1+p1.Velcity_Kd*(EnV_p1-p1.En_V1)+p1.Velcity_Ki*p1.Count ;//???PID
	p1.En_V2=p1.En_V1;
	p1.En_V1=EnV_p1;
	p2.PWM1 += p2.Velcity_Kp*EnV_p2+p2.Velcity_Kd*(EnV_p2-p2.En_V1)+p2.Velcity_Ki*p2.Count  ;//???PID
	p2.En_V2=p2.En_V1;
	p2.En_V1=EnV_p2;
	if(gest.v1>0.75||gest.v1<0.85)
	{
		if(p1.Count<3)
		{
		p1.Count+=EnV_p1*0.01;
		}
        else
	   {
		   p1.Count=1.5;
	   }			
	}
	else
	{
	  p1.Count=0;	
	}
	if(gest.v2>0.75||gest.v2<0.85)
	{
		if(p2.Count<3)
		{
		p2.Count+=EnV_p2*0.01;
		}
    else
			{
    p2.Count=1.5;
		}
	}
	else
	{
	  p2.Count=0;	
	}
}
void Relcity_PWM()
{
	p1.PWM2=-p1.Relcity_Kp*EnR-p1.Relcity_Kd*(EnR-p1.En_R1);
	p2.PWM2=p2.Relcity_Kp*EnR+p2.Relcity_Kd*(EnR-p2.En_R1);
	p1.En_R1=EnR;
	p2.En_R1=EnR;
	if(p2.PWM2<=-100)
	{
		p2.PWM2=-100;
	}
	if(p1.PWM2<=-100)
	{
		p1.PWM2=-100;
	}
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_UART_Receive_IT(&huart3,rx_buf3,1);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,500);
	p1.Velcity_Kp=50;
	p1.Velcity_Ki=0;
	p1.Velcity_Kd=3;
	p2.Velcity_Kp=50;
	p2.Velcity_Ki=0;
	p2.Velcity_Kd=4;
	p1.Relcity_Kp=0.7;
	p1.Relcity_Ki=0;
	p1.Relcity_Kd=4;
	p2.Relcity_Kp=0.7;
	p2.Relcity_Ki=0;
	p2.Relcity_Kd=3;
	p1.PWM1=0;
	p2.PWM1=0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(EnR==2000)
//	  {
//		  Wheel(1,0);
//		  Wheel(2,0);
//		  HAL_Delay(5000);
//	  }
//	
//    if(p1.PWM2<=-400)
//	{
//		p1.PWM2=-400;
//	}
//	if(p2.PWM2<=-400)
//	{
//		p2.PWM2=-400;
//	}
	
		if(control==1&&sign2==0&&sign3>=2000)
		{
			p1.PWM2=-p1.PWM1;
			p2.PWM2=-p2.PWM1;
		}
		if(control==2)
		{
			p1.PWM2=-p1.PWM1;
			p2.PWM2=-p2.PWM1;
			sign1=1;
		}
		
	  Wheel(1,p1.PWM1+p1.PWM2);
		Wheel(2,p2.PWM1+p2.PWM2);
		if(control==2)
		{
			HAL_Delay(5000);
			control=0;
			control2=1;
		}
		Velcity_PWM();
		Relcity_PWM();
		HAL_Delay(5);
		if(sign1==1)
		{
			sign2++;
			if(sign2==10)
			{
				sign1=0;
				sign2=0;
			}
		}
			
		if(sign3<=2000)
		{
			sign3++;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch,FILE* f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}

void Wheel(int num,int pwm)
{
	int direct;
	if(pwm>1000)
	{
		pwm=1000;
	}
	if(pwm<-1000)
	{
		pwm=-1000;
	}
	
	if(pwm>=0)
	{
		direct=1;
	}
	if(pwm<0)
	{
		direct=0;
	}
	switch(num)
	{
		case 1:
			switch(direct)
			{
				case 1:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pwm);
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);//1
				  break;
				case 0:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000+pwm);
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);//1
				break;
			}
			break;
		
case 2:
			switch(direct)
			{
				case 1:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000-pwm);
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);//2
				  break;
				case 0:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,-pwm);
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);//2
				  break;
			}
		 break;		
		}
				
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */