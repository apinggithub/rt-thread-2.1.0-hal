/**
  ******************************************************************************
  * @file    ft5216.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file contains all the functions prototypes for the
  *          ft5216.c Touch screen driver.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRV_IIC_CTP_FT5216
#define DRV_IIC_CTP_FT5216

									
#ifdef __cplusplus
extern "C" {
#endif

/* Set Multi-touch as supported */
#if !defined(CTP_MONO_CTP_SUPPORTED)
#define CTP_MULTI_CTP_SUPPORTED        1
#endif /* CTP_MONO_CTP_SUPPORTED */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f1xx_hal.h"

/* Macros --------------------------------------------------------------------*/

#if defined(FT5216_ENABLE_ASSERT)
/* Assert activated */
#define FT5216_ASSERT(__condition__)      do { if(__condition__) \
                                               {  \
                                                 while(1);  \
                                               } \
                                          }while(0)
#else
/* Assert not activated : macro has no effect */
#define FT5216_ASSERT(__condition__)    do { if(__condition__) \
                                             {  \
                                                ;  \
                                             } \
                                            }while(0)
#endif /* FT5216_ENABLE_ASSERT == 1 */

/** @typedef ft5216_handle_TypeDef
 *  ft5216 Handle definition.
 */
																						
																						
																						
struct ft5216_data{
	uint8_t active;
	uint16_t X_phys;
	uint16_t Y_phys;
                  };

#define FT5216_PUT_UP    1									
#define FT5216_PUT_DOWN  0
typedef struct
{
  uint8_t i2cInitialized;

  /* field holding the current number of simultaneous active touches */
  uint8_t currActiveTouchNb;

  /* field holding the touch index currently managed */
  uint8_t currActiveTouchIdx;

} ft5216_handle_TypeDef;

  /** @addtogroup BSP
   * @{
   */

  /** @addtogroup Component
   * @{
   */

/** @defgroup CTP_Driver_structure  Touch Sensor Driver structure
  * @{
  */
typedef struct
{  
  void       (*Init)(uint16_t);
  uint16_t   (*ReadID)(void);
  void       (*Reset)(uint16_t);
  void       (*Start)(uint16_t);
  uint8_t    (*DetectTouch)(uint16_t);
  void       (*GetXY)(uint16_t, uint16_t*, uint16_t*);
  void       (*EnableIT)(uint16_t);
  void       (*ClearIT)(uint16_t);
  uint8_t    (*GetITStatus)(uint16_t);
  void       (*DisableIT)(uint16_t);
}CTP_DrvTypeDef;





  /* Exported types ------------------------------------------------------------*/

  /** @defgroup FT5216_Exported_Types
   * @{
   */

  /* Exported constants --------------------------------------------------------*/

  /** @defgroup FT5216_Exported_Constants
   * @{
   */


static I2C_HandleTypeDef hI2cCtpHandler;



/* Exported constant IO ------------------------------------------------------*/

//#define FT5216_USING_SOFT_I2C	
#define CTP_I2C_ADDRESS                   ((uint16_t)0x70)

/* I2C clock speed configuration (in Hz) 
   WARNING: 
   Make sure that this define is not already declared in other files (ie. 
   stm32746g_discovery.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                       ((uint32_t)100000)
#endif /* I2C_SPEED */

#define CTP_RST_Pin 						GPIO_PIN_5
#define CTP_RST_GPIO_Port 					GPIOC
#define CTP_INT_Pin 						GPIO_PIN_4
#define CTP_INT_GPIO_Port 					GPIOC

/* User can use this section to tailor I2Cx/I2Cx instance used and associated 
   resources */
/* Definition for capacitive touch panel (CTP) I2Cx resources */
#define CTP_I2Cx                             I2C1
#define CTP_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define CTP_DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define CTP_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()

#define CTP_I2Cx_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define CTP_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define CTP_I2Cx_SCL_PIN                     GPIO_PIN_8
#define CTP_I2Cx_SDA_PIN                     GPIO_PIN_9
#define CTP_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
//#define CTP_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C1
#ifdef FT5216_USING_SOFT_I2C
#define CTP_I2Cx_SCL_ON  HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT, CTP_I2Cx_SCL_PIN,GPIO_PIN_SET)
#define CTP_I2Cx_SCL_OFF  HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT, CTP_I2Cx_SCL_PIN,GPIO_PIN_RESET)
#define CTP_I2Cx_SDA_ON  HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT, CTP_I2Cx_SDA_PIN,GPIO_PIN_SET)
#define CTP_I2Cx_SDA_OFF  HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT, CTP_I2Cx_SDA_PIN,GPIO_PIN_RESET)
#define CTP_I2Cx_SDA_READ  HAL_GPIO_ReadPin(CTP_I2Cx_SCL_SDA_GPIO_PORT, CTP_I2Cx_SDA_PIN)
#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRH|=0X40000000;}	 //输入模式，浮空输入模式
#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRH|=0X10000000;}	 //通用推挽输出，输出速度50MHZ
#endif
#define CTP_I2Cx_RST_ON  HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin,GPIO_PIN_SET)
#define CTP_I2Cx_RST_OFF  HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin,GPIO_PIN_RESET)
#define CTP_I2Cx_INT_ON  HAL_GPIO_WritePin(CTP_INT_GPIO_Port, CTP_INT_Pin,GPIO_PIN_SET)





/* I2C interrupt requests */
#define CTP_I2Cx_EV_IRQn                     I2C1_EV_IRQn
#define CTP_I2Cx_ER_IRQn                     I2C1_ER_IRQn

/* Definition for external, camera and Arduino connector I2Cx resources */
////////#define DISCOVERY_EXT_I2Cx                               I2C1
////////#define DISCOVERY_EXT_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C1_CLK_ENABLE()
////////#define DISCOVERY_EXT_DMAx_CLK_ENABLE()                  __HAL_RCC_DMA1_CLK_ENABLE()
////////#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()

////////#define DISCOVERY_EXT_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C1_FORCE_RESET()
////////#define DISCOVERY_EXT_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C1_RELEASE_RESET()

/////////* Definition for I2Cx Pins */
////////#define DISCOVERY_EXT_I2Cx_SCL_PIN                       GPIO_PIN_8
////////#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT             GPIOB
////////#define DISCOVERY_EXT_I2Cx_SCL_SDA_AF                    GPIO_AF4_I2C1
////////#define DISCOVERY_EXT_I2Cx_SDA_PIN                       GPIO_PIN_9

/////////* I2C interrupt requests */
////////#define DISCOVERY_EXT_I2Cx_EV_IRQn                       I2C1_EV_IRQn
////////#define DISCOVERY_EXT_I2Cx_ER_IRQn                       I2C1_ER_IRQn

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated from APB1 source clock = 50 MHz */
/* Due to the big MOFSET capacity for adapting the camera level the rising time is very large (>1us) */
/* 0x40912732 takes in account the big rising and aims a clock of 100khz */
/* this value might be adapted when next Rev Birdie board is available */
#ifndef DISCOVERY_I2Cx_TIMING  
#define DISCOVERY_I2Cx_TIMING                      ((uint32_t)0x40912732)  
#endif /* DISCOVERY_I2Cx_TIMING */





  /* I2C Slave address of touchscreen FocalTech FT5216 */
#define FT5216_I2C_SLAVE_ADDRESS            ((uint8_t)0x70)

  /* Maximum border values of the touchscreen pad */
#define FT5216_MAX_WIDTH                    ((uint16_t)240)     /* Touchscreen pad max width   */
#define FT5216_MAX_HEIGHT                   ((uint16_t)320)     /* Touchscreen pad max height  */

  /* Possible values of driver functions return status */
#define FT5216_STATUS_OK                    ((uint8_t)0x00)
#define FT5216_STATUS_NOT_OK                ((uint8_t)0x01)

  /* Possible values of global variable 'CTP_I2C_Initialized' */
#define FT5216_I2C_NOT_INITIALIZED          ((uint8_t)0x00)
#define FT5216_I2C_INITIALIZED              ((uint8_t)0x01)

  /* Max detectable simultaneous touches */
#define FT5216_MAX_DETECTABLE_CTP         ((uint8_t)0x05)

  /**
   * @brief : Definitions for FT5216 I2C register addresses on 8 bit
   **/

  /* Current mode register of the FT5216 (R/W) */
#define FT5216_DEV_MODE_REG                 ((uint8_t)0x00)

  /* Possible values of FT5216_DEV_MODE_REG */
#define FT5216_DEV_MODE_WORKING             ((uint8_t)0x00)
#define FT5216_DEV_MODE_FACTORY             ((uint8_t)0x04)

#define FT5216_DEV_MODE_MASK                ((uint8_t)0x07)
#define FT5216_DEV_MODE_SHIFT               ((uint8_t)0x04)

 /* Gesture ID register */
#define FT5216_GEST_ID_REG                  ((uint8_t)0x01)

  /* Possible values of FT5216_GEST_ID_REG */
#define FT5216_GEST_ID_NO_GESTURE           ((uint8_t)0x00)
#define FT5216_GEST_ID_MOVE_UP              ((uint8_t)0x10)
#define FT5216_GEST_ID_MOVE_RIGHT           ((uint8_t)0x14)
#define FT5216_GEST_ID_MOVE_DOWN            ((uint8_t)0x18)
#define FT5216_GEST_ID_MOVE_LEFT            ((uint8_t)0x1C)
#define FT5216_GEST_ID_SINGLE_CLICK         ((uint8_t)0x20)
#define FT5216_GEST_ID_DOUBLE_CLICK         ((uint8_t)0x22)
#define FT5216_GEST_ID_ROTATE_CLOCKWISE     ((uint8_t)0x28)
#define FT5216_GEST_ID_ROTATE_C_CLOCKWISE   ((uint8_t)0x29)
#define FT5216_GEST_ID_ZOOM_IN              ((uint8_t)0x40)
#define FT5216_GEST_ID_ZOOM_OUT             ((uint8_t)0x49)

  /* Touch Data Status register : gives number of active touch points (0..5) */
#define FT5216_TD_STAT_REG                  ((uint8_t)0x02)

  /* Values related to FT5216_TD_STAT_REG */
#define FT5216_TD_STAT_MASK                 ((uint8_t)0x0F)
#define FT5216_TD_STAT_SHIFT                ((uint8_t)0x00)

  /* Values Pn_XH and Pn_YH related */
#define FT5216_CTP_EVT_FLAG_PRESS_DOWN    ((uint8_t)0x00)
#define FT5216_CTP_EVT_FLAG_LIFT_UP       ((uint8_t)0x01)
#define FT5216_CTP_EVT_FLAG_CONTACT       ((uint8_t)0x02)
#define FT5216_CTP_EVT_FLAG_NO_EVENT      ((uint8_t)0x03)

#define FT5216_CTP_EVT_FLAG_SHIFT         ((uint8_t)0x06)
#define FT5216_CTP_EVT_FLAG_MASK          ((uint8_t)(3 << FT5216_CTP_EVT_FLAG_SHIFT))

#define FT5216_CTP_POS_MSB_MASK           ((uint8_t)0x0F)
#define FT5216_CTP_POS_MSB_SHIFT          ((uint8_t)0x00)

  /* Values Pn_XL and Pn_YL related */
#define FT5216_CTP_POS_LSB_MASK           ((uint8_t)0xFF)
#define FT5216_CTP_POS_LSB_SHIFT          ((uint8_t)0x00)

#define FT5216_P1_XH_REG                    ((uint8_t)0x03)
#define FT5216_P1_XL_REG                    ((uint8_t)0x04)
#define FT5216_P1_YH_REG                    ((uint8_t)0x05)
#define FT5216_P1_YL_REG                    ((uint8_t)0x06)

/* Touch Pressure register value (R) */
#define FT5216_P1_WEIGHT_REG                ((uint8_t)0x07)

/* Values Pn_WEIGHT related  */
#define FT5216_CTP_WEIGHT_MASK            ((uint8_t)0xFF)
#define FT5216_CTP_WEIGHT_SHIFT           ((uint8_t)0x00)

/* Touch area register */
#define FT5216_P1_MISC_REG                  ((uint8_t)0x08)

/* Values related to FT5216_Pn_MISC_REG */
#define FT5216_CTP_AREA_MASK              ((uint8_t)(0x04 << 4))
#define FT5216_CTP_AREA_SHIFT             ((uint8_t)0x04)

#define FT5216_P2_XH_REG                    ((uint8_t)0x09)
#define FT5216_P2_XL_REG                    ((uint8_t)0x0A)
#define FT5216_P2_YH_REG                    ((uint8_t)0x0B)
#define FT5216_P2_YL_REG                    ((uint8_t)0x0C)
#define FT5216_P2_WEIGHT_REG                ((uint8_t)0x0D)
#define FT5216_P2_MISC_REG                  ((uint8_t)0x0E)

#define FT5216_P3_XH_REG                    ((uint8_t)0x0F)
#define FT5216_P3_XL_REG                    ((uint8_t)0x10)
#define FT5216_P3_YH_REG                    ((uint8_t)0x11)
#define FT5216_P3_YL_REG                    ((uint8_t)0x12)
#define FT5216_P3_WEIGHT_REG                ((uint8_t)0x13)
#define FT5216_P3_MISC_REG                  ((uint8_t)0x14)

#define FT5216_P4_XH_REG                    ((uint8_t)0x15)
#define FT5216_P4_XL_REG                    ((uint8_t)0x16)
#define FT5216_P4_YH_REG                    ((uint8_t)0x17)
#define FT5216_P4_YL_REG                    ((uint8_t)0x18)
#define FT5216_P4_WEIGHT_REG                ((uint8_t)0x19)
#define FT5216_P4_MISC_REG                  ((uint8_t)0x1A)

#define FT5216_P5_XH_REG                    ((uint8_t)0x1B)
#define FT5216_P5_XL_REG                    ((uint8_t)0x1C)
#define FT5216_P5_YH_REG                    ((uint8_t)0x1D)
#define FT5216_P5_YL_REG                    ((uint8_t)0x1E)
#define FT5216_P5_WEIGHT_REG                ((uint8_t)0x1F)
#define FT5216_P5_MISC_REG                  ((uint8_t)0x20)

#define FT5216_P6_XH_REG                    ((uint8_t)0x21)
#define FT5216_P6_XL_REG                    ((uint8_t)0x22)
#define FT5216_P6_YH_REG                    ((uint8_t)0x23)
#define FT5216_P6_YL_REG                    ((uint8_t)0x24)
#define FT5216_P6_WEIGHT_REG                ((uint8_t)0x25)
#define FT5216_P6_MISC_REG                  ((uint8_t)0x26)

#define FT5216_P7_XH_REG                    ((uint8_t)0x27)
#define FT5216_P7_XL_REG                    ((uint8_t)0x28)
#define FT5216_P7_YH_REG                    ((uint8_t)0x29)
#define FT5216_P7_YL_REG                    ((uint8_t)0x2A)
#define FT5216_P7_WEIGHT_REG                ((uint8_t)0x2B)
#define FT5216_P7_MISC_REG                  ((uint8_t)0x2C)

#define FT5216_P8_XH_REG                    ((uint8_t)0x2D)
#define FT5216_P8_XL_REG                    ((uint8_t)0x2E)
#define FT5216_P8_YH_REG                    ((uint8_t)0x2F)
#define FT5216_P8_YL_REG                    ((uint8_t)0x30)
#define FT5216_P8_WEIGHT_REG                ((uint8_t)0x31)
#define FT5216_P8_MISC_REG                  ((uint8_t)0x32)

#define FT5216_P9_XH_REG                    ((uint8_t)0x33)
#define FT5216_P9_XL_REG                    ((uint8_t)0x34)
#define FT5216_P9_YH_REG                    ((uint8_t)0x35)
#define FT5216_P9_YL_REG                    ((uint8_t)0x36)
#define FT5216_P9_WEIGHT_REG                ((uint8_t)0x37)
#define FT5216_P9_MISC_REG                  ((uint8_t)0x38)

#define FT5216_P10_XH_REG                   ((uint8_t)0x39)
#define FT5216_P10_XL_REG                   ((uint8_t)0x3A)
#define FT5216_P10_YH_REG                   ((uint8_t)0x3B)
#define FT5216_P10_YL_REG                   ((uint8_t)0x3C)
#define FT5216_P10_WEIGHT_REG               ((uint8_t)0x3D)
#define FT5216_P10_MISC_REG                 ((uint8_t)0x3E)

  /* Threshold for touch detection */
#define FT5216_TH_GROUP_REG                 ((uint8_t)0x80)

  /* Values FT5216_TH_GROUP_REG : threshold related  */
#define FT5216_THRESHOLD_MASK               ((uint8_t)0xFF)
#define FT5216_THRESHOLD_SHIFT              ((uint8_t)0x00)

  /* Filter function coefficients */
#define FT5216_TH_DIFF_REG                  ((uint8_t)0x85)

  /* Control register */
#define FT5216_CTRL_REG                     ((uint8_t)0x86)

  /* Values related to FT5216_CTRL_REG */

  /* Will keep the Active mode when there is no touching */
#define FT5216_CTRL_KEEP_ACTIVE_MODE        ((uint8_t)0x00)

  /* Switching from Active mode to Monitor mode automatically when there is no touching */
#define FT5216_CTRL_KEEP_AUTO_SWITCH_MONITOR_MODE  ((uint8_t)0x01

  /* The time period of switching from Active mode to Monitor mode when there is no touching */
#define FT5216_TIMEENTERMONITOR_REG         ((uint8_t)0x87)

  /* Report rate in Active mode */
#define FT5216_PERIODACTIVE_REG             ((uint8_t)0x88)

  /* Report rate in Monitor mode */
#define FT5216_PERIODMONITOR_REG            ((uint8_t)0x89)

  /* The value of the minimum allowed angle while Rotating gesture mode */
#define FT5216_RADIAN_VALUE_REG             ((uint8_t)0x91)

  /* Maximum offset while Moving Left and Moving Right gesture */
#define FT5216_OFFSET_LEFT_RIGHT_REG        ((uint8_t)0x92)

  /* Maximum offset while Moving Up and Moving Down gesture */
#define FT5216_OFFSET_UP_DOWN_REG           ((uint8_t)0x93)

  /* Minimum distance while Moving Left and Moving Right gesture */
#define FT5216_DISTANCE_LEFT_RIGHT_REG      ((uint8_t)0x94)

  /* Minimum distance while Moving Up and Moving Down gesture */
#define FT5216_DISTANCE_UP_DOWN_REG         ((uint8_t)0x95)

  /* Maximum distance while Zoom In and Zoom Out gesture */
#define FT5216_DISTANCE_ZOOM_REG            ((uint8_t)0x96)



  /* High 8-bit of LIB Version info */
#define FT5216_LIB_VER_H_REG                ((uint8_t)0xA1)

  /* Low 8-bit of LIB Version info */
#define FT5216_LIB_VER_L_REG                ((uint8_t)0xA2)

  /* Chip Selecting */
#define FT5216_CIPHER_REG                   ((uint8_t)0xA3)

  /* Interrupt mode register (used when in interrupt mode) */
#define FT5216_GMODE_REG                    ((uint8_t)0xA4)

#define FT5216_G_MODE_INTERRUPT_MASK        ((uint8_t)0x03)
#define FT5216_G_MODE_INTERRUPT_SHIFT       ((uint8_t)0x00)

  /* Possible values of FT5216_GMODE_REG */
#define FT5216_G_MODE_INTERRUPT_POLLING     ((uint8_t)0x00)
#define FT5216_G_MODE_INTERRUPT_TRIGGER     ((uint8_t)0x01)

  /* Current power mode the FT5216 system is in (R) */
#define FT5216_PWR_MODE_REG                 ((uint8_t)0xA5)

  /* FT5216 firmware version */
#define FT5216_FIRMID_REG                   ((uint8_t)0xA6)

  /* FT5216 Chip identification register */
#define FT5216_CHIP_ID_REG                  ((uint8_t)0xA3)


  /*  Possible values of FT5216_CHIP_ID_REG */
#define FT5216_ID_VALUE                     ((uint8_t)0x0A)
/* if IC is ft5216 , id is 0x0a ;*/

  /* Release code version */
#define FT5216_RELEASE_CODE_ID_REG          ((uint8_t)0xAF)

  /* Current operating mode the FT5216 system is in (R) */
#define FT5216_STATE_REG                    ((uint8_t)0xBC)



/**
 * @brief  Initialize the ft5216 communication bus
 *         from MCU to FT5216 : ie I2C channel initialization (if required).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
 * @retval None
 */
void ft5216_Init(uint16_t DeviceAddr);

/**
 * @brief  Software Reset the ft5216.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
 * @retval None
 */
void ft5216_Reset(uint16_t DeviceAddr);

/**
 * @brief  Read the ft5216 device ID, pre initialize I2C in case of need to be
 *         able to read the FT5216 device ID, and verify this is a FT5216.
 * @param  DeviceAddr: I2C FT5216 Slave address.
 * @retval The Device ID (two bytes).
 */
uint16_t ft5216_ReadID(void);

/**
 * @brief  Configures the touch Screen IC device to start detecting touches
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
 * @retval None.
 */
void ft5216_CTP_Start(uint16_t DeviceAddr);

/**
 * @brief  Return if there is touches detected or not.
 *         Try to detect new touches and forget the old ones (reset internal global
 *         variables).
 * @param  DeviceAddr: Device address on communication Bus.
 * @retval : Number of active touches detected (can be 0, 1 or 2).
 */
uint8_t ft5216_CTP_DetectTouch(uint16_t DeviceAddr);

/**
 * @brief  Get the touch screen X and Y positions values
 *         Manage multi touch thanks to touch Index global
 *         variable 'ft5216_handle.currActiveTouchIdx'.
 * @param  DeviceAddr: Device address on communication Bus.
 * @param  X: Pointer to X position value
 * @param  Y: Pointer to Y position value
 * @retval None.
 */
void ft5216_CTP_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);

/**
 * @brief  Configure the FT5216 device to generate IT on given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5216).
 * @retval None
 */
void ft5216_CTP_EnableIT(uint16_t DeviceAddr);

/**
 * @brief  Configure the FT5216 device to stop generating IT on the given INT pin
 *         connected to MCU as EXTI.
 * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5216).
 * @retval None
 */
void ft5216_CTP_DisableIT(uint16_t DeviceAddr);

/**
 * @brief  Get IT status from FT5216 interrupt status registers
 *         Should be called Following an EXTI coming to the MCU to know the detailed
 *         reason of the interrupt.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
 * @retval TS interrupts status
 */
uint8_t ft5216_CTP_ITStatus (uint16_t DeviceAddr);

/**
 * @brief  Clear IT status in FT5216 interrupt status clear registers
 *         Should be called Following an EXTI coming to the MCU.
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
 * @retval TS interrupts status
 */
void ft5216_CTP_ClearIT (uint16_t DeviceAddr);

/**** NEW FEATURES enabled when Multi-touch support is enabled ****/

#if (CTP_MULTI_CTP_SUPPORTED == 1)

/**
 * @brief  Get the last touch gesture identification (zoom, move up/down...).
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
 * @param  pGestureId : Pointer to get last touch gesture Identification.
 * @retval None.
 */
void ft5216_CTP_GetGestureID(uint16_t DeviceAddr,uint32_t * pGestureId);

/**
 * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
 *         This touch detailed information contains :
 *         - weight that was applied to this touch
 *         - sub-area of the touch in the touch panel
 *         - event of linked to the touch (press down, lift up, ...)
 * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
 * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
 *                    detailed information.
 * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
 * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
 * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.

 * @retval None.
 */
void ft5216_CTP_GetTouchInfo(uint16_t DeviceAddr,
														uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent);

#endif /* CTP_MULTI_CTP_SUPPORTED == 1 */

/* Imported TS IO functions --------------------------------------------------------*/

/** @defgroup ft5216_Imported_Functions
 * @{
 */

/* CTPSCREEN IO functions */
void            CTP_IO_Init(void);
void            CTP_IO_Write(uint8_t Reg, uint8_t *Value,uint16_t len);
uint8_t         CTP_IO_Read(uint8_t Reg,uint8_t *value,uint16_t len);
void            CTP_IO_Delay(uint32_t Delay);

  /**
   * @}
   */

  /* Imported global variables --------------------------------------------------------*/

  /** @defgroup ft5216_Imported_Globals
   * @{
   */


/* Touch screen driver structure */
extern CTP_DrvTypeDef ft5216_ts_drv;

  /**
   * @}
   */

#ifdef __cplusplus
}
#endif
#endif /* __FT5216_H */


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
