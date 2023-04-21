   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         FACT RETE PRINT FUNCTIONS HEADER FILE       */
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

#ifndef _H_factprt

#define _H_factprt

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _FACTPRT_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           PrintFactJNCompVars1(char *,VOID *);
   LOCALE VOID                           PrintFactJNCompVars2(char *,VOID *);
   LOCALE VOID                           PrintFactJNCompVars3(char *,VOID *);
   LOCALE VOID                           PrintFactPNCompVars2(char *,VOID *);
   LOCALE VOID                           PrintFactGetVarJN1(char *,VOID *);
   LOCALE VOID                           PrintFactGetVarJN2(char *,VOID *);
   LOCALE VOID                           PrintFactGetVarJN3(char *,VOID *);
   LOCALE VOID                           PrintFactGetVarPN1(char *,VOID *);
   LOCALE VOID                           PrintFactGetVarPN2(char *,VOID *);
   LOCALE VOID                           PrintFactGetVarPN3(char *,VOID *);
   LOCALE VOID                           PrintSlotLengthTest(char *,VOID *);
   LOCALE VOID                           PrintPNConstant2(char *,VOID *);
   LOCALE VOID                           PrintPNConstant4(char *,VOID *);
#else   
   LOCALE VOID                           PrintFactJNCompVars1();
   LOCALE VOID                           PrintFactJNCompVars2();
   LOCALE VOID                           PrintFactJNCompVars3();
   LOCALE VOID                           PrintFactPNCompVars2();
   LOCALE VOID                           PrintFactGetVarJN1();
   LOCALE VOID                           PrintFactGetVarJN2();
   LOCALE VOID                           PrintFactGetVarJN3();
   LOCALE VOID                           PrintFactGetVarPN1();
   LOCALE VOID                           PrintFactGetVarPN2();
   LOCALE VOID                           PrintFactGetVarPN3();
   LOCALE VOID                           PrintSlotLengthTest();
   LOCALE VOID                           PrintPNConstant2();
   LOCALE VOID                           PrintPNConstant4();
#endif 

#endif






