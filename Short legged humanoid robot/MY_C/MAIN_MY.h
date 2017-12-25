#ifndef __MAIN_MY_H
#define __MAIN_MY_H 	
#include "stm32f4xx_hal.h"
#include "PID.h"

#define LEN_R_ON 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)
#define LEN_R_OFF 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
#define LEN_G_ON 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET)
#define LEN_G_OFF 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)

#define KEY_L					HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)
#define KEY_R					HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)

#define ROBOT_ROLL_ERROR (55)
#define ROBOT_PITCH_ERROR (-39)

#define USART_REC_LEN  			100  	//定义最大接收字节数 200
#define NRF_RX_LEN 	12  // 数据通道有效数据宽度
#define NRF_TX_LEN  1

//typedef enum __NRF_MODE{
//	NRF_MODE_TX=1,
//	NRF_MODE_RX
//}NRF_MODE;

/** 
  * @brief  NRF handle Structure definition
  */
typedef struct __NRF_HandleTypeDef
{
//	NRF_MODE 									 MODE;
	
	uint8_t 									 FLAG_RX;
	
//	uint8_t 									 FLAG_SEND;
	
//	uint8_t 									 FLAG_TX_OK;
	
	uint8_t 									 FLAG_RECON;
	
	uint8_t										 REC_CHANNEL;
	
	char 											 BUF_RX[NRF_RX_LEN];
	
	uint8_t 									 BUF_TX[NRF_TX_LEN];
	
	uint8_t 									 BUF2_TX[32];
	
	char 											 BUF2_RX[32];
}NRF_HandleTypeDef;

/** 
  * @brief  ROBOT handle Structure definition
  */
typedef struct __ROBOT_HandleTypeDef
{
	short 										 P_value[24];//用于最终结果
	
	short											 P_value_cul[24];//用于中间计算
	
	uint8_t 									 ACTION_PUSH_UP;
	
	short											 ROLL;
	
	short											 PITCH;
	
	short											 YAW;
	
	short 										 PID_OUT_qh;
	
	short											 PID_OUT_lf;
	
	short											 lf_angle_need;
}ROBOT_HandleTypeDef;

/** 
  * @brief  UART_MY handle Structure definition
  */
typedef struct __UART_MY_HandleTypeDef
{
	uint8_t 									 NEED_SEND;
	
	uint8_t 									 SEND_OK;
	
	uint8_t 									 REC_OK;
	
	char 											 SEND_BUF[USART_REC_LEN+1];
	
	char 											 REC_BUF[USART_REC_LEN+1];
}UART_MY_HandleTypeDef;

/** 
  * @brief  TIMER_MY handle Structure definition
  */
typedef struct __TIMER_MY_HandleTypeDef
{
	short 									 ACTION_TIMER_REMAIN;
	
	uint8_t 									 UART_SEND_TIMER;
	
	uint8_t 									 NRF_Sync;
	
	uint8_t										 NRF_IRQ_OUT_TIME;
}TIMER_MY_HandleTypeDef;

/** 
  * @brief  KEY handle Structure definition
  */
typedef struct __KEY_HandleTypeDef
{
	uint8_t 									 KEY0;
	
	uint8_t 									 KEY_STATUS;
	
}KEY_HandleTypeDef;

/** 
  * @brief  IMU handle Structure definition
  */
typedef struct __IMU_HandleTypeDef{
	float 										 q[4];
	
	float 										 froll;
	
	float 										 fpitch;
	
	float 									 	 fyaw;
	
	short 									   sroll;
	
	short 										 sroll_error;
	
	short 										 spitch;
	
	short 										 syaw;
	
	char  										 key;
}IMU_HandleTypeDef;
/** 
  * @brief  KEY handle Structure definition
  */
typedef struct __GLOVAR_HandleTypeDef
{
	IMU_HandleTypeDef					 IMU[10];	
	
	NRF_HandleTypeDef				 	 NRF;
	
	ROBOT_HandleTypeDef				 ROBOT;
	
	UART_MY_HandleTypeDef			 UART;
	
	TIMER_MY_HandleTypeDef		 TIMER;	
	
	KEY_HandleTypeDef					 KEY;
	
	_increasePID 							 PID_qh;
	
//  DMA_Stream_TypeDef         *Instance;                                                    /*!< Register base address                  */

//  DMA_InitTypeDef            Init;                                                         /*!< DMA communication parameters           */ 

//  HAL_LockTypeDef            Lock;                                                         /*!< DMA locking object                     */  

//  __IO HAL_DMA_StateTypeDef  State;                                                        /*!< DMA transfer state                     */

//  void                       *Parent;                                                      /*!< Parent object state                    */  

//  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA transfer complete callback         */

//  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma); /*!< DMA Half transfer complete callback    */

//  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);   /*!< DMA transfer complete Memory1 callback */

//  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer error callback            */

// __IO uint32_t               ErrorCode;                                                    /*!< DMA Error code                         */

// uint32_t                    StreamBaseAddress;                                            /*!< DMA Stream Base Address                */

// uint32_t                    StreamIndex;                                                  /*!< DMA Stream Index                       */ 
}GLOVAR_HandleTypeDef;

void LIZJ_HAL_INIT(GLOVAR_HandleTypeDef *GloVar);
void NRF_data_handle(GLOVAR_HandleTypeDef *glovar);
void NRF_Reconnection_check(GLOVAR_HandleTypeDef *GloVar);
void Up_UART_DATA(GLOVAR_HandleTypeDef *GloVar);
void usart1_data_send_handle(GLOVAR_HandleTypeDef *GloVar);
void Run_Action_Group(GLOVAR_HandleTypeDef *GloVar,uint8_t Group_num,short speed,uint8_t start_num);
void Run_Action_Group_printf(GLOVAR_HandleTypeDef *GloVar,uint8_t Group_num,short speed,uint8_t start_num);
void decode_UART_rec(GLOVAR_HandleTypeDef *GloVar);
#endif
