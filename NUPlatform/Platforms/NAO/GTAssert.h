/**
 * @file Platform/linux/GTAssert.h
 *
 * This file contains macros for low level debugging.
 *
 * @author Thomas Röfer
 */
#ifndef __GTAssert_h_
#define __GTAssert_h_

#include <assert.h>
#include <stdio.h>

#undef ASSERT
#undef VERIFY

#ifdef NDEBUG
#define ASSERT(cond) ((void)0)
#define VERIFY(cond) ((void)(cond))
#define PRINT(text) ((void)0)

#else

/**
 * ASSERT prints a message if cond is false and NDEBUG is not defined.
 * ASSERT does not evaluate cond if NDEBUG is defined.
 * Note that printf will not work in early stages of the creation of an Aperios
 * process. Therefore, you will not get any feedback in such a case. If you are
 * not sure whether an assertion failure will be reported at a certain position
 * in the code, try ASSERT(false).
 * @param c The condition to be checked.
 */
#define ASSERT(c) assert(c)

/**
 * VERIFY prints a message if cond is false and NDEBUG is not defined.
 * VERIFY does evaluate cond even if NDEBUG is defined.
 * Note that printf will not work in early stages of the creation of an Aperios
 * process. Therefore, you will not get any feedback in such a case. If you are
 * not sure whether an assertion failure will be reported at a certain position
 * in the code, try ASSERT(false).
 * @param c The condition to be checked.
 */
#define VERIFY(c) assert(c)

/**
 * PRINT prints a text directly to the output if and NDEBUG is not defined.
 * PRINT does not evaluate the parameter if NDEBUG is defined.
 * @param text The text that will be printed.
 */
#define PRINT(text) {fprintf(stderr,text);fprintf(stderr,"\n");}

#endif

#endif //__GTAssert_h_


