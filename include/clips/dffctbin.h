   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           DEFFACTS BSAVE/BLOAD HEADER FILE          */
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

#if (! RUN_TIME)
#ifndef _H_dffctbin

#define _H_dffctbin

#include "modulbin.h"
#include "cstrcbin.h"
#ifndef _H_constrct
#include "constrct.h"
#endif

struct bsaveDeffacts
  {
   struct bsaveConstructHeader header;
   long assertList;
  };

struct bsaveDeffactsModule
  {
   struct bsaveDefmoduleItemHeader header;
  }; 
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DFFCTBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER   
   LOCALE VOID                           DeffactsBinarySetup(void);
   LOCALE VOID                          *BloadDeffactsModuleReference(int);
#else
   LOCALE VOID                           DeffactsBinarySetup();
   LOCALE VOID                          *BloadDeffactsModuleReference();
#endif

#endif
#endif






