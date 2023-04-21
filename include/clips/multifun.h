   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           MULTIFIELD FUNCTIONS HEADER FILE          */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary Riley and Brian Donnell                         */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_multifun
#define _H_multifun

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MULTIFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

   LOCALE int                     ReplaceMultiValueField(struct dataObject *,struct dataObject *,
                                                         int,int,struct dataObject *,char *);
   LOCALE int                     InsertMultiValueField(struct dataObject *,struct dataObject *,
                                                        int,struct dataObject *,char *);
   LOCALE int                     DeleteMultiValueField(struct dataObject *,struct dataObject *,
                                                        int,int,char *);

#else

   LOCALE int                     ReplaceMultiValueField();
   LOCALE int                     InsertMultiValueField();
   LOCALE int                     DeleteMultiValueField();

#endif

#endif





