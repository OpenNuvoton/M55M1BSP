/**************************************************************************//**
 * @file     lpgpio_reg.h
 * @version  V1.00
 * @brief    LPGPIO register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPGPIO_REG_H__
#define __LPGPIO_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPGPIO Low Power General Purpose Input/Output Controller (LPGPIO)
    Memory Mapped Structure for LPGPIO Controller
    @{
*/

typedef struct
{
    /**
    * @var LPGPIO_T::MODE
    * Offset: 0x00  LPIOn Mode Control
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |MODE0     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[1]     |MODE1     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[2]     |MODE2     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[3]     |MODE3     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[4]     |MODE4     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[5]     |MODE5     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[6]     |MODE6     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[7]     |MODE7     |LPIOn I/O Pin Mode Control
    * |        |          |Determine each I/O mode of LPIOn pins.
    * |        |          |0 = LPIOn is in Input mode.
    * |        |          |1 = LPIOn is in Push-pull Output mode.
    * |[31]    |LPPDMA_EN |LPPDMA Enable Bit
    * |        |          |This bit is used to enable LPPDMA to access LPGPIO when chip is in NPD0/1/2/3/4.
    * |        |          |0 = LPPDMA cannot access LPGPIO when chip is in NPD0/1/2/3/4.
    * |        |          |1 = LPPDMA can access LPGPIO when chip is in NPD0/1/2/3/4.
    * @var LPGPIO_T::DOUT
    * Offset: 0x04  LPIOn Data Output Value
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |DOUT0     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[1]     |DOUT1     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[2]     |DOUT2     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[3]     |DOUT3     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[4]     |DOUT4     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[5]     |DOUT5     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[6]     |DOUT6     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * |[7]     |DOUT7     |LPIOn Output Value
    * |        |          |Each of these bits controls the status of a LPIOn pin when the LPIOn is configured as Push-pull output mode.
    * |        |          |0 = LPIOn will drive Low if the LPIOn pin is configured as Push-pull output mode.
    * |        |          |1 = LPIOn will drive High if the LPIOn pin is configured as Push-pull output mode.
    * @var LPGPIO_T::PIN
    * Offset: 0x08  LPIOn Pin Value
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |PIN0      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[1]     |PIN1      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[2]     |PIN2      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[3]     |PIN3      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[4]     |PIN4      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[5]     |PIN5      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[6]     |PIN6      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * |[7]     |PIN7      |LPIOn Pin Value
    * |        |          |Each bit of the register reflects the actual status of the respective Pn pin
    * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
    * @var LPGPIO_T::DSRST
    * Offset: 0x0C  LPIOn Data Output Set and Reset Control
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |DSET0     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[1]     |DSET1     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[2]     |DSET2     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[3]     |DSET3     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[4]     |DSET4     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[5]     |DSET5     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[6]     |DSET6     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[7]     |DSET7     |LPIOn Data Ouput Set Control (Write Only)
    * |        |          |Writing 1 to each bit can set LPIOn pin output data to 1.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 1.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DSET[n] will make DOUT[n] reflect the set value.
    * |[16]    |DRESET0   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[17]    |DRESET1   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[18]    |DRESET2   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[19]    |DRESET3   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[20]    |DRESET4   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[21]    |DRESET5   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[22]    |DRESET6   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[23]    |DRESET7   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : If DRESET[n] and DSET[n] are both set, DSET[n] has higher pirority.
    * |        |          |Note 3 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * @var LPGPIO_T::DRST
    * Offset: 0x10  LPIOn Data Output Reset Control
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |DRESET0   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[1]     |DRESET1   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[2]     |DRESET2   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[3]     |DRESET3   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[4]     |DRESET4   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[5]     |DRESET5   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[6]     |DRESET6   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    * |[7]     |DRESET7   |LPIOn Data Ouput Reset Control (Write Only)
    * |        |          |Writing 1 to each bit can reset LPIOn pin output data to 0.
    * |        |          |0 = No action.
    * |        |          |1 = Reset LPIOn to 0.
    * |        |          |Note 1 : This bit field is write only, and reading this field will respond with 0.
    * |        |          |Note 2 : Writing 1 to DRESET[n] will make DOUT[n] reflect the reset value.
    */
    __IO uint32_t MODE;                  /*!< [0x0000] LPIOn Mode Control                                               */
    __IO uint32_t DOUT;                  /*!< [0x0004] LPIOn Data Output Value                                          */
    __I  uint32_t PIN;                   /*!< [0x0008] LPIOn Pin Value                                                  */
    __O  uint32_t DSRST;                 /*!< [0x000c] LPIOn Data Output Set and Reset Control                          */
    __O  uint32_t DRST;                  /*!< [0x0010] LPIOn Data Output Reset Control                                  */
} LPGPIO_T;


/**
    @addtogroup LPGPIO_CONST LPGPIO Bit Field Definition
    Constant Definitions for LPGPIO Controller
@{ */

#define LPGPIO_MODE_MODE_Pos            (0)                                         /*!< LPGPIO_T::MODE: MODE Position              */
#define LPGPIO_MODE_MODE_Msk            (0xfful << LPGPIO_MODE_MODE_Pos)            /*!< LPGPIO_T::MODE: MODE Mask                  */

#define LPGPIO_MODE_MODE0_Pos           (0)                                         /*!< LPGPIO_T::MODE: MODE0 Position             */
#define LPGPIO_MODE_MODE0_Msk           (0x1ul << LPGPIO_MODE_MODE0_Pos)            /*!< LPGPIO_T::MODE: MODE0 Mask                 */

#define LPGPIO_MODE_MODE1_Pos           (1)                                         /*!< LPGPIO_T::MODE: MODE1 Position             */
#define LPGPIO_MODE_MODE1_Msk           (0x1ul << LPGPIO_MODE_MODE1_Pos)            /*!< LPGPIO_T::MODE: MODE1 Mask                 */

#define LPGPIO_MODE_MODE2_Pos           (2)                                         /*!< LPGPIO_T::MODE: MODE2 Position             */
#define LPGPIO_MODE_MODE2_Msk           (0x1ul << LPGPIO_MODE_MODE2_Pos)            /*!< LPGPIO_T::MODE: MODE2 Mask                 */

#define LPGPIO_MODE_MODE3_Pos           (3)                                         /*!< LPGPIO_T::MODE: MODE3 Position             */
#define LPGPIO_MODE_MODE3_Msk           (0x1ul << LPGPIO_MODE_MODE3_Pos)            /*!< LPGPIO_T::MODE: MODE3 Mask                 */

#define LPGPIO_MODE_MODE4_Pos           (4)                                         /*!< LPGPIO_T::MODE: MODE4 Position             */
#define LPGPIO_MODE_MODE4_Msk           (0x1ul << LPGPIO_MODE_MODE4_Pos)            /*!< LPGPIO_T::MODE: MODE4 Mask                 */

#define LPGPIO_MODE_MODE5_Pos           (5)                                         /*!< LPGPIO_T::MODE: MODE5 Position             */
#define LPGPIO_MODE_MODE5_Msk           (0x1ul << LPGPIO_MODE_MODE5_Pos)            /*!< LPGPIO_T::MODE: MODE5 Mask                 */

#define LPGPIO_MODE_MODE6_Pos           (6)                                         /*!< LPGPIO_T::MODE: MODE6 Position             */
#define LPGPIO_MODE_MODE6_Msk           (0x1ul << LPGPIO_MODE_MODE6_Pos)            /*!< LPGPIO_T::MODE: MODE6 Mask                 */

#define LPGPIO_MODE_MODE7_Pos           (7)                                         /*!< LPGPIO_T::MODE: MODE7 Position             */
#define LPGPIO_MODE_MODE7_Msk           (0x1ul << LPGPIO_MODE_MODE7_Pos)            /*!< LPGPIO_T::MODE: MODE7 Mask                 */

#define LPGPIO_MODE_LPPDMA_EN_Pos       (31)                                        /*!< LPGPIO_T::MODE: LPPDMA_EN Position         */
#define LPGPIO_MODE_LPPDMA_EN_Msk       (0x1ul << LPGPIO_MODE_LPPDMA_EN_Pos)        /*!< LPGPIO_T::MODE: LPPDMA_EN Mask             */

#define LPGPIO_DOUT_DOUT_Pos            (0)                                         /*!< LPGPIO_T::DOUT: DOUT Position              */
#define LPGPIO_DOUT_DOUT_Msk            (0xfful << LPGPIO_DOUT_DOUT_Pos)            /*!< LPGPIO_T::DOUT: DOUT Mask                  */

#define LPGPIO_DOUT_DOUT0_Pos           (0)                                         /*!< LPGPIO_T::DOUT: DOUT0 Position             */
#define LPGPIO_DOUT_DOUT0_Msk           (0x1ul << LPGPIO_DOUT_DOUT0_Pos)            /*!< LPGPIO_T::DOUT: DOUT0 Mask                 */

#define LPGPIO_DOUT_DOUT1_Pos           (1)                                         /*!< LPGPIO_T::DOUT: DOUT1 Position             */
#define LPGPIO_DOUT_DOUT1_Msk           (0x1ul << LPGPIO_DOUT_DOUT1_Pos)            /*!< LPGPIO_T::DOUT: DOUT1 Mask                 */

#define LPGPIO_DOUT_DOUT2_Pos           (2)                                         /*!< LPGPIO_T::DOUT: DOUT2 Position             */
#define LPGPIO_DOUT_DOUT2_Msk           (0x1ul << LPGPIO_DOUT_DOUT2_Pos)            /*!< LPGPIO_T::DOUT: DOUT2 Mask                 */

#define LPGPIO_DOUT_DOUT3_Pos           (3)                                         /*!< LPGPIO_T::DOUT: DOUT3 Position             */
#define LPGPIO_DOUT_DOUT3_Msk           (0x1ul << LPGPIO_DOUT_DOUT3_Pos)            /*!< LPGPIO_T::DOUT: DOUT3 Mask                 */

#define LPGPIO_DOUT_DOUT4_Pos           (4)                                         /*!< LPGPIO_T::DOUT: DOUT4 Position             */
#define LPGPIO_DOUT_DOUT4_Msk           (0x1ul << LPGPIO_DOUT_DOUT4_Pos)            /*!< LPGPIO_T::DOUT: DOUT4 Mask                 */

#define LPGPIO_DOUT_DOUT5_Pos           (5)                                         /*!< LPGPIO_T::DOUT: DOUT5 Position             */
#define LPGPIO_DOUT_DOUT5_Msk           (0x1ul << LPGPIO_DOUT_DOUT5_Pos)            /*!< LPGPIO_T::DOUT: DOUT5 Mask                 */

#define LPGPIO_DOUT_DOUT6_Pos           (6)                                         /*!< LPGPIO_T::DOUT: DOUT6 Position             */
#define LPGPIO_DOUT_DOUT6_Msk           (0x1ul << LPGPIO_DOUT_DOUT6_Pos)            /*!< LPGPIO_T::DOUT: DOUT6 Mask                 */

#define LPGPIO_DOUT_DOUT7_Pos           (7)                                         /*!< LPGPIO_T::DOUT: DOUT7 Position             */
#define LPGPIO_DOUT_DOUT7_Msk           (0x1ul << LPGPIO_DOUT_DOUT7_Pos)            /*!< LPGPIO_T::DOUT: DOUT7 Mask                 */

#define LPGPIO_PIN_PIN_Pos              (0)                                         /*!< LPGPIO_T::PIN: PIN Position                */
#define LPGPIO_PIN_PIN_Msk              (0xfful << LPGPIO_PIN_PIN_Pos)              /*!< LPGPIO_T::PIN: PIN Mask                    */

#define LPGPIO_PIN_PIN0_Pos             (0)                                         /*!< LPGPIO_T::PIN: PIN0 Position               */
#define LPGPIO_PIN_PIN0_Msk             (0x1ul << LPGPIO_PIN_PIN0_Pos)              /*!< LPGPIO_T::PIN: PIN0 Mask                   */

#define LPGPIO_PIN_PIN1_Pos             (1)                                         /*!< LPGPIO_T::PIN: PIN1 Position               */
#define LPGPIO_PIN_PIN1_Msk             (0x1ul << LPGPIO_PIN_PIN1_Pos)              /*!< LPGPIO_T::PIN: PIN1 Mask                   */

#define LPGPIO_PIN_PIN2_Pos             (2)                                         /*!< LPGPIO_T::PIN: PIN2 Position               */
#define LPGPIO_PIN_PIN2_Msk             (0x1ul << LPGPIO_PIN_PIN2_Pos)              /*!< LPGPIO_T::PIN: PIN2 Mask                   */

#define LPGPIO_PIN_PIN3_Pos             (3)                                         /*!< LPGPIO_T::PIN: PIN3 Position               */
#define LPGPIO_PIN_PIN3_Msk             (0x1ul << LPGPIO_PIN_PIN3_Pos)              /*!< LPGPIO_T::PIN: PIN3 Mask                   */

#define LPGPIO_PIN_PIN4_Pos             (4)                                         /*!< LPGPIO_T::PIN: PIN4 Position               */
#define LPGPIO_PIN_PIN4_Msk             (0x1ul << LPGPIO_PIN_PIN4_Pos)              /*!< LPGPIO_T::PIN: PIN4 Mask                   */

#define LPGPIO_PIN_PIN5_Pos             (5)                                         /*!< LPGPIO_T::PIN: PIN5 Position               */
#define LPGPIO_PIN_PIN5_Msk             (0x1ul << LPGPIO_PIN_PIN5_Pos)              /*!< LPGPIO_T::PIN: PIN5 Mask                   */

#define LPGPIO_PIN_PIN6_Pos             (6)                                         /*!< LPGPIO_T::PIN: PIN6 Position               */
#define LPGPIO_PIN_PIN6_Msk             (0x1ul << LPGPIO_PIN_PIN6_Pos)              /*!< LPGPIO_T::PIN: PIN6 Mask                   */

#define LPGPIO_PIN_PIN7_Pos             (7)                                         /*!< LPGPIO_T::PIN: PIN7 Position               */
#define LPGPIO_PIN_PIN7_Msk             (0x1ul << LPGPIO_PIN_PIN7_Pos)              /*!< LPGPIO_T::PIN: PIN7 Mask                   */

#define LPGPIO_DSRST_DSET_Pos           (0)                                         /*!< LPGPIO_T::DSRST: DSET Position             */
#define LPGPIO_DSRST_DSET_Msk           (0xfful << LPGPIO_DSRST_DSET_Pos)           /*!< LPGPIO_T::DSRST: DSET Mask                 */

#define LPGPIO_DSRST_DSET0_Pos          (0)                                         /*!< LPGPIO_T::DSRST: DSET0 Position            */
#define LPGPIO_DSRST_DSET0_Msk          (0x1ul << LPGPIO_DSRST_DSET0_Pos)           /*!< LPGPIO_T::DSRST: DSET0 Mask                */

#define LPGPIO_DSRST_DSET1_Pos          (1)                                         /*!< LPGPIO_T::DSRST: DSET1 Position            */
#define LPGPIO_DSRST_DSET1_Msk          (0x1ul << LPGPIO_DSRST_DSET1_Pos)           /*!< LPGPIO_T::DSRST: DSET1 Mask                */

#define LPGPIO_DSRST_DSET2_Pos          (2)                                         /*!< LPGPIO_T::DSRST: DSET2 Position            */
#define LPGPIO_DSRST_DSET2_Msk          (0x1ul << LPGPIO_DSRST_DSET2_Pos)           /*!< LPGPIO_T::DSRST: DSET2 Mask                */

#define LPGPIO_DSRST_DSET3_Pos          (3)                                         /*!< LPGPIO_T::DSRST: DSET3 Position            */
#define LPGPIO_DSRST_DSET3_Msk          (0x1ul << LPGPIO_DSRST_DSET3_Pos)           /*!< LPGPIO_T::DSRST: DSET3 Mask                */

#define LPGPIO_DSRST_DSET4_Pos          (4)                                         /*!< LPGPIO_T::DSRST: DSET4 Position            */
#define LPGPIO_DSRST_DSET4_Msk          (0x1ul << LPGPIO_DSRST_DSET4_Pos)           /*!< LPGPIO_T::DSRST: DSET4 Mask                */

#define LPGPIO_DSRST_DSET5_Pos          (5)                                         /*!< LPGPIO_T::DSRST: DSET5 Position            */
#define LPGPIO_DSRST_DSET5_Msk          (0x1ul << LPGPIO_DSRST_DSET5_Pos)           /*!< LPGPIO_T::DSRST: DSET5 Mask                */

#define LPGPIO_DSRST_DSET6_Pos          (6)                                         /*!< LPGPIO_T::DSRST: DSET6 Position            */
#define LPGPIO_DSRST_DSET6_Msk          (0x1ul << LPGPIO_DSRST_DSET6_Pos)           /*!< LPGPIO_T::DSRST: DSET6 Mask                */

#define LPGPIO_DSRST_DSET7_Pos          (7)                                         /*!< LPGPIO_T::DSRST: DSET7 Position            */
#define LPGPIO_DSRST_DSET7_Msk          (0x1ul << LPGPIO_DSRST_DSET7_Pos)           /*!< LPGPIO_T::DSRST: DSET7 Mask                */

#define LPGPIO_DSRST_DRESET_Pos         (16)                                        /*!< LPGPIO_T::DSRST: DRESET Position           */
#define LPGPIO_DSRST_DRESET_Msk         (0xfful << LPGPIO_DSRST_DRESET_Pos)         /*!< LPGPIO_T::DSRST: DRESET Mask               */

#define LPGPIO_DSRST_DRESET0_Pos        (16)                                        /*!< LPGPIO_T::DSRST: DRESET0 Position          */
#define LPGPIO_DSRST_DRESET0_Msk        (0x1ul << LPGPIO_DSRST_DRESET0_Pos)         /*!< LPGPIO_T::DSRST: DRESET0 Mask              */

#define LPGPIO_DSRST_DRESET1_Pos        (17)                                        /*!< LPGPIO_T::DSRST: DRESET1 Position          */
#define LPGPIO_DSRST_DRESET1_Msk        (0x1ul << LPGPIO_DSRST_DRESET1_Pos)         /*!< LPGPIO_T::DSRST: DRESET1 Mask              */

#define LPGPIO_DSRST_DRESET2_Pos        (18)                                        /*!< LPGPIO_T::DSRST: DRESET2 Position          */
#define LPGPIO_DSRST_DRESET2_Msk        (0x1ul << LPGPIO_DSRST_DRESET2_Pos)         /*!< LPGPIO_T::DSRST: DRESET2 Mask              */

#define LPGPIO_DSRST_DRESET3_Pos        (19)                                        /*!< LPGPIO_T::DSRST: DRESET3 Position          */
#define LPGPIO_DSRST_DRESET3_Msk        (0x1ul << LPGPIO_DSRST_DRESET3_Pos)         /*!< LPGPIO_T::DSRST: DRESET3 Mask              */

#define LPGPIO_DSRST_DRESET4_Pos        (20)                                        /*!< LPGPIO_T::DSRST: DRESET4 Position          */
#define LPGPIO_DSRST_DRESET4_Msk        (0x1ul << LPGPIO_DSRST_DRESET4_Pos)         /*!< LPGPIO_T::DSRST: DRESET4 Mask              */

#define LPGPIO_DSRST_DRESET5_Pos        (21)                                        /*!< LPGPIO_T::DSRST: DRESET5 Position          */
#define LPGPIO_DSRST_DRESET5_Msk        (0x1ul << LPGPIO_DSRST_DRESET5_Pos)         /*!< LPGPIO_T::DSRST: DRESET5 Mask              */

#define LPGPIO_DSRST_DRESET6_Pos        (22)                                        /*!< LPGPIO_T::DSRST: DRESET6 Position          */
#define LPGPIO_DSRST_DRESET6_Msk        (0x1ul << LPGPIO_DSRST_DRESET6_Pos)         /*!< LPGPIO_T::DSRST: DRESET6 Mask              */

#define LPGPIO_DSRST_DRESET7_Pos        (23)                                        /*!< LPGPIO_T::DSRST: DRESET7 Position          */
#define LPGPIO_DSRST_DRESET7_Msk        (0x1ul << LPGPIO_DSRST_DRESET7_Pos)         /*!< LPGPIO_T::DSRST: DRESET7 Mask              */

#define LPGPIO_DRST_DRESET_Pos          (0)                                         /*!< LPGPIO_T::DRST: DRESET Position            */
#define LPGPIO_DRST_DRESET_Msk          (0xfful << LPGPIO_DRST_DRESET_Pos)          /*!< LPGPIO_T::DRST: DRESET Mask                */

#define LPGPIO_DRST_DRESET0_Pos         (0)                                         /*!< LPGPIO_T::DRST: DRESET0 Position           */
#define LPGPIO_DRST_DRESET0_Msk         (0x1ul << LPGPIO_DRST_DRESET0_Pos)          /*!< LPGPIO_T::DRST: DRESET0 Mask               */

#define LPGPIO_DRST_DRESET1_Pos         (1)                                         /*!< LPGPIO_T::DRST: DRESET1 Position           */
#define LPGPIO_DRST_DRESET1_Msk         (0x1ul << LPGPIO_DRST_DRESET1_Pos)          /*!< LPGPIO_T::DRST: DRESET1 Mask               */

#define LPGPIO_DRST_DRESET2_Pos         (2)                                         /*!< LPGPIO_T::DRST: DRESET2 Position           */
#define LPGPIO_DRST_DRESET2_Msk         (0x1ul << LPGPIO_DRST_DRESET2_Pos)          /*!< LPGPIO_T::DRST: DRESET2 Mask               */

#define LPGPIO_DRST_DRESET3_Pos         (3)                                         /*!< LPGPIO_T::DRST: DRESET3 Position           */
#define LPGPIO_DRST_DRESET3_Msk         (0x1ul << LPGPIO_DRST_DRESET3_Pos)          /*!< LPGPIO_T::DRST: DRESET3 Mask               */

#define LPGPIO_DRST_DRESET4_Pos         (4)                                         /*!< LPGPIO_T::DRST: DRESET4 Position           */
#define LPGPIO_DRST_DRESET4_Msk         (0x1ul << LPGPIO_DRST_DRESET4_Pos)          /*!< LPGPIO_T::DRST: DRESET4 Mask               */

#define LPGPIO_DRST_DRESET5_Pos         (5)                                         /*!< LPGPIO_T::DRST: DRESET5 Position           */
#define LPGPIO_DRST_DRESET5_Msk         (0x1ul << LPGPIO_DRST_DRESET5_Pos)          /*!< LPGPIO_T::DRST: DRESET5 Mask               */

#define LPGPIO_DRST_DRESET6_Pos         (6)                                         /*!< LPGPIO_T::DRST: DRESET6 Position           */
#define LPGPIO_DRST_DRESET6_Msk         (0x1ul << LPGPIO_DRST_DRESET6_Pos)          /*!< LPGPIO_T::DRST: DRESET6 Mask               */

#define LPGPIO_DRST_DRESET7_Pos         (7)                                         /*!< LPGPIO_T::DRST: DRESET7 Position           */
#define LPGPIO_DRST_DRESET7_Msk         (0x1ul << LPGPIO_DRST_DRESET7_Pos)          /*!< LPGPIO_T::DRST: DRESET7 Mask               */

/** @} LPGPIO_CONST */
/** @} end of LPGPIO register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPGPIO_REG_H__ */
