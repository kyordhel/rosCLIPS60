   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFGLOBAL BINARY HEADER FILE            */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_globlbin

#define _H_globlbin

#include "modulbin.h"
#include "cstrcbin.h"
#include "globldef.h"

struct bsaveDefglobal
  {
   struct bsaveConstructHeader header;
   long initial;
  };

struct bsaveDefglobalModule
  {
   struct bsaveDefmoduleItemHeader header;
  };  
   
#define DefglobalPointer(i) ((struct defglobal *) (&DefglobalArray[i]))

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GLOBLBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DefglobalBinarySetup(void);
   LOCALE VOID                          *BloadDefglobalModuleReference(int);
#else
   LOCALE VOID                           DefglobalBinarySetup();
   LOCALE VOID                          *BloadDefglobalModuleReference();
#endif

#ifndef _GLOBLBIN_SOURCE_
   extern struct defglobal HUGE_ADDR  *DefglobalArray;
#endif
#endif






