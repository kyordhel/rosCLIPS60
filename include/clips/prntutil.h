   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              PRINT UTILITY HEADER FILE              */
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

#ifndef _H_prntutil
#define _H_prntutil

#ifndef _H_moduldef
#include "moduldef.h"
#endif

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _PRNTUTIL_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           PrintInChunks(char *,char *);
   LOCALE VOID                           PrintFloat(char *,double);
   LOCALE VOID                           PrintLongInteger(char *,long);
   LOCALE VOID                           PrintAtom(char *,int,VOID *);
   LOCALE VOID                           PrintTally(char *,long,char *,char *);
   LOCALE char                          *FloatToString(double);
   LOCALE char                          *LongIntegerToString(long);
   LOCALE VOID                           SyntaxErrorMessage(char *);
   LOCALE VOID                           CLIPSSystemError(char *,int);
   LOCALE VOID                           ListItemsDriver(char *,struct defmodule *,
                                                         char *,char *,
                                                          VOID *(*)(VOID *), 
                                                          char *(*)(VOID *), 
                                                          VOID (*)(char *,VOID *),
                                                          int (*)(VOID *));
   LOCALE VOID                           PrintErrorID(char *,int,int);
   LOCALE VOID                           PrintWarningID(char *,int,int);
   LOCALE VOID                           CantFindItemErrorMessage(char *,char *);
   LOCALE VOID                           CantDeleteItemErrorMessage(char *,char *);
   LOCALE VOID                           AlreadyParsedErrorMessage(char *,char *);
   LOCALE VOID                           LocalVariableErrorMessage(char *);
   LOCALE VOID                           DivideByZeroErrorMessage(char *);
#else
   LOCALE VOID                           PrintInChunks();
   LOCALE VOID                           PrintFloat();
   LOCALE VOID                           PrintLongInteger();
   LOCALE VOID                           PrintAtom();
   LOCALE VOID                           PrintTally();
   LOCALE char                          *FloatToString();
   LOCALE char                          *LongIntegerToString();
   LOCALE VOID                           SyntaxErrorMessage();
   LOCALE VOID                           CLIPSSystemError();
   LOCALE VOID                           ListItemsDriver();
   LOCALE VOID                           PrintErrorID();
   LOCALE VOID                           PrintWarningID();
   LOCALE VOID                           CantFindItemErrorMessage();
   LOCALE VOID                           CantDeleteItemErrorMessage();
   LOCALE VOID                           AlreadyParsedErrorMessage();
   LOCALE VOID                           LocalVariableErrorMessage();
   LOCALE VOID                           DivideByZeroErrorMessage();
#endif 
   
#ifndef _PRNTUTIL_SOURCE_
   extern BOOLEAN                     PreserveEscapedCharacters;
   extern BOOLEAN                     AddressesToStrings;
   extern BOOLEAN                     InstanceAddressesToNames;
#endif

#endif






