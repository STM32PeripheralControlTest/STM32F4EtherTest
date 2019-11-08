/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "lwip/sockets.h"
#include "lwip.h"
#include "string.h"
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
/* USER CODE BEGIN Variables */
osThreadId startSocketServerTaskHandle;
volatile int isInitializedFinished = 0;
volatile int isIPSuppliedByDHCP = 0;
volatile struct netif currentNetIf;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void startSocketServerTask(const void* argument);
static void netifStatusCallback(struct netif* netIf);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(socketTask, startSocketServerTask, osPriorityNormal, 0, 512);
  startSocketServerTaskHandle = osThreadCreate(osThread(socketTask), NULL);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */
  isInitializedFinished = 1;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void netifStatusCallback(struct netif* netIf)
{
  isIPSuppliedByDHCP = 1;
  currentNetIf = *netIf;
}
void startSocketServerTask(const void* argument)
{
  while(isInitializedFinished == 0);
  
  SetNetIfStatusCallback(netifStatusCallback);

  while(isIPSuppliedByDHCP == 0);

  volatile ip4_addr_t gotIPAddr = currentNetIf.ip_addr;

  volatile char testChar[30];
  strcpy(testChar, inet_ntoa(gotIPAddr));

  int sockfd = lwip_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

  

  struct sockaddr_in reader_addr;
  memset(&reader_addr, 0, sizeof(reader_addr));
  reader_addr.sin_family = AF_INET;
  reader_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  reader_addr.sin_port = htons(80);

  volatile int status = bind(sockfd, (struct sockaddr*)&reader_addr, sizeof(reader_addr));

  if(status >= 0){
    status = listen(sockfd, 5);
  }

  struct sockaddr_in client_addr;
  int client_addr_len;
  int clientfd = 0;
  while(1){
    if(status >= 0){
      clientfd = accept(sockfd, (struct sockaddr*)&client_addr, &client_addr_len);
    }


    char receiveBuffer[1024] = {0};
    volatile int n = 0;
    if(clientfd >= 0){
      int val = lwip_fcntl(clientfd, F_GETFL, 0);
      lwip_fcntl(clientfd, F_SETFL, val | O_NONBLOCK);

      char buf;
      while(1){

        int readbyte = lwip_read(clientfd, &buf, 1);

        if(readbyte <= 0){
          break;
        }
        
        sprintf(receiveBuffer, "%s%c", receiveBuffer, buf);
        n++;
        if(n >= 1024){
          break;
        }
        else if(n >= 259){
          int x = 0;
          x++;
        }
      }
    }
    char buffer[] = "HTTP/1.1 200 OK\r\n"
			"Date: Sun, 26 Feb 2017 15:13:00 GMT\r\n"
			"Server: Apache/2.2.31 (Amazon)\r\n"
			"Last-Modified: Sun, 26 Feb 2017 15:06:20 GMT\r\n"
			"Accept-Ranges: bytes\r\n"
			"Content-Length: 6\r\n"
			"Connection: close\r\n"
			"Content-Type: text/html; charset=UTF-8\r\n"
			"\r\n"
			"HELLO\r\n";
    
    lwip_send(clientfd, buffer, strlen(buffer), 0);

    close(clientfd);
  }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
