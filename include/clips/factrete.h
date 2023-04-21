   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*        FACT RETE ACCESS FUNCTIONS HEADER FILE       */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_factrete

#define _H_factrete

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _FACTRETE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE BOOLEAN                        FactGetVarPNFunction1(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactGetVarPNFunction2(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactGetVarPNFunction3(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactGetVarJNFunction1(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactGetVarJNFunction2(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactGetVarJNFunction3(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactSlotLengthTestFunction(VOID *,DATA_OBJECT_PTR);
   LOCALE int                            FactCompVarsJNFunction1(VOID *,DATA_OBJECT_PTR);
   LOCALE int                            FactCompVarsJNFunction2(VOID *,DATA_OBJECT_PTR);
   LOCALE int                            FactCompVarsJNFunction3(VOID *,DATA_OBJECT_PTR);
   LOCALE int                            FactCompVarsPNFunction2(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactConstantPNFunction2(VOID *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        FactConstantPNFunction4(VOID *,DATA_OBJECT_PTR);
   LOCALE int                            SCallStoreMultifield(VOID *,DATA_OBJECT_PTR);
   LOCALE int                            AdjustFieldPosition(struct multifieldMarker *,int,int,int *);
#else   
   LOCALE BOOLEAN                        FactGetVarPNFunction1();
   LOCALE BOOLEAN                        FactGetVarPNFunction2();
   LOCALE BOOLEAN                        FactGetVarPNFunction3();
   LOCALE BOOLEAN                        FactGetVarJNFunction1();
   LOCALE BOOLEAN                        FactGetVarJNFunction2();
   LOCALE BOOLEAN                        FactGetVarJNFunction3();
   LOCALE BOOLEAN                        FactSlotLengthTestFunction();
   LOCALE int                            FactCompVarsJNFunction1();
   LOCALE int                            FactCompVarsJNFunction2();
   LOCALE int                            FactCompVarsJNFunction3();
   LOCALE int                            FactCompVarsPNFunction2();
   LOCALE BOOLEAN                        FactConstantPNFunction2();
   LOCALE BOOLEAN                        FactConstantPNFunction4();
   LOCALE int                            SCallStoreMultifield();
   LOCALE int                            AdjustFieldPosition();
#endif 

#endif






