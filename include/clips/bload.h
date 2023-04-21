   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                 BLOAD HEADER FILE                   */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_bload
#define _H_bload

#ifndef _H_extnfunc
#include "extnfunc.h"
#endif
#ifndef _H_exprnbin
#include "exprnbin.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_sysdep
#include "sysdep.h"
#endif
#ifndef _H_symblbin
#include "symblbin.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _BLOAD_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#define FunctionPointer(i) ((struct FunctionDefinition *) (((i) == -1L) ? NULL : FunctionArray[i]))
                                   
#if ANSI_COMPILER  
   LOCALE int                     BloadCommand(void);
   LOCALE BOOLEAN                 Bload(char *);
   LOCALE VOID                    BloadandRefresh(long,unsigned,VOID (*)(VOID *,long));
   LOCALE BOOLEAN                 Bloaded(void);
   LOCALE VOID                    AddBeforeBloadFunction(char *,VOID (*)(void),int);
   LOCALE VOID                    AddAfterBloadFunction(char *,VOID (*)(void),int);
   LOCALE VOID                    AddBloadReadyFunction(char *,int (*)(void),int);
   LOCALE VOID                    AddClearBloadReadyFunction(char *,int (*)(void),int);
   LOCALE VOID                    AddAbortBloadFunction(char *,VOID (*)(void),int);
   LOCALE VOID                    CannotLoadWithBloadMessage(char *);
#else
   LOCALE int                     BloadCommand();
   LOCALE BOOLEAN                 Bload();
   LOCALE VOID                    BloadandRefresh();
   LOCALE BOOLEAN                 Bloaded();
   LOCALE VOID                    AddBeforeBloadFunction();
   LOCALE VOID                    AddAfterBloadFunction();
   LOCALE VOID                    AddBloadReadyFunction();
   LOCALE VOID                    AddClearBloadReadyFunction();
   LOCALE VOID                    AddAbortBloadFunction();
   LOCALE VOID                    CannotLoadWithBloadMessage();
#endif

#ifndef _BLOAD_SOURCE_
   extern char                                  *BinaryPrefixID;
   extern char                                  *BinaryVersionID;
   extern struct FunctionDefinition * HUGE_ADDR *FunctionArray;
#endif
   
#endif







