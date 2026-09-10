/**********************************************************************************************************************
 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *      Project:  Hercules ARM Safety MCUs - F021 Flash API
 *         File:  CGT.gcc.h
 *  Description:  GCC compiler specific info used by the F021 API.
 *                Mirrors CGT.CCS.h for the arm-none-eabi-gcc toolchain.
 *********************************************************************************************************************/

#ifndef CGT_GCC_H_
#define CGT_GCC_H_

/* arm-none-eabi-gcc defines __ARMEB__ when compiled with -mbig-endian */
#if defined(__ARMEB__) || defined(__BIG_ENDIAN__)
#if !defined(_BIG_ENDIAN)
    #define _BIG_ENDIAN
#endif
#elif defined(__ARMEL__) || defined(__LITTLE_ENDIAN__)
#if !defined(_LITTLE_ENDIAN)
    #define _LITTLE_ENDIAN
#endif
#endif

#define ATTRIBUTE_PACKED    __attribute__((packed))

#endif /* CGT_GCC_H_ */

/**********************************************************************************************************************
 *  END OF FILE: CGT.gcc.h
 *********************************************************************************************************************/
