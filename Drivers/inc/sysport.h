/**
* @file sysport.h
* @brief system portable functions
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 16.04.2012
*/
#ifndef _SYSPORT_H
#define _SYSPORT_H

#include <stdint.h>
#include <stm32f2xx.h>

#ifdef __cplusplus
extern "C" {
#endif



#if   defined ( __CC_ARM ) /*------------------RealView Compiler -----------------*/
/* ARM armcc specific functions */

#if (__ARMCC_VERSION < 400677)
  #error "Please use ARM Compiler Toolchain V4.0.677 or later!"
#endif


/** \brief  Get Priority Mask and disable interrupts

    This function returns the current state of the priority mask bit from the Priority Mask Register.

    \return               Priority Mask value
 */
static __INLINE uint32_t __disableInterrupts(void)
{
  register uint32_t __regPriMask         __ASM("primask");
  __ASM("cpsid i");
  return(__regPriMask);
}


/** \brief  Set Priority Mask

    This function assigns the given value to the Priority Mask Register.

    \param [in]    priMask  Priority Mask
 */
static __INLINE void __restoreInterrupts(uint32_t priMask)
{
  register uint32_t __regPriMask         __ASM("primask");
  __regPriMask = (priMask);
}
 
 /** \brief  Get Priority Mask and disable interrupts
 
	 This function returns the current state of the priority mask bit from the Priority Mask Register.
 
	 \return			   Priority Mask value
  */
 static __INLINE uint32_t __enableInterrupts(void)
 {
   register uint32_t __regPriMask		  __ASM("primask");
   __ASM("cpsie i");
   return(__regPriMask);
 }
 

#elif defined ( __ICCARM__ ) /*------------------ ICC Compiler -------------------*/
/* IAR iccarm specific functions */

#include <cmsis_iar.h>

#elif defined ( __GNUC__ ) /*------------------ GNU Compiler ---------------------*/
/* GNU gcc specific functions */


/** \brief  Get Priority Mask and disable interrupts

    This function returns the current state of the priority mask bit from the Priority Mask Register.

    \return               Priority Mask value
 */
__attribute__( ( always_inline ) ) static __INLINE uint32_t __disableInterrupts(void)
{
  uint32_t result;

  __ASM volatile ("MRS %0, primask" : "=r" (result) );
  __ASM volatile ("cpsid i");
  return(result);
}


/** \brief  Set Priority Mask

    This function assigns the given value to the Priority Mask Register.

    \param [in]    priMask  Priority Mask
 */
__attribute__( ( always_inline ) ) static __INLINE void __restoreInterrupts(uint32_t priMask)
{
  __ASM volatile ("MSR primask, %0" : : "r" (priMask) );
}

/** \brief  Get Priority Mask and enable interrupts

    This function returns the current state of the priority mask bit from the Priority Mask Register.

    \return               Priority Mask value
 */
__attribute__( ( always_inline ) ) static __INLINE uint32_t __enableInterrupts(void)
{
  uint32_t result;

  __ASM volatile ("MRS %0, primask" : "=r" (result) );
  __ASM volatile ("cpsie i");
  return(result);
}

#endif

extern uint8_t isrLevel;


__attribute__( ( always_inline ) ) static __INLINE int inIsr(void)
{
	register uint32_t key;

	key=__disableInterrupts();
	if (isrLevel==0)
	{
		__restoreInterrupts(key);
		return 0;
	}
	else
	{
		__restoreInterrupts(key);
		return 1;
	}
}

#define ENTER_ISR()	isrLevel++
#define EXIT_ISR()	isrLevel--

#ifdef __cplusplus
}
#endif


#endif

