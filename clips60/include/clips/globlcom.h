   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFGLOBAL COMMANDS HEADER FILE           */
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

#ifndef _H_globlcom
#define _H_globlcom
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GLOBLCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER  
   LOCALE VOID                           DefglobalCommands(void);
   LOCALE int                            SetResetGlobalsCommand(void);
   LOCALE BOOLEAN                        SetResetGlobals(int);
   LOCALE int                            GetResetGlobalsCommand(void);
   LOCALE BOOLEAN                        GetResetGlobals(void);  
   LOCALE VOID                           ShowDefglobalsCommand(void);
   LOCALE VOID                           ShowDefglobals(char *,VOID *);
#else  
   LOCALE VOID                           DefglobalCommands();
   LOCALE int                            SetResetGlobalsCommand();
   LOCALE BOOLEAN                        SetResetGlobals();
   LOCALE int                            GetResetGlobalsCommand();
   LOCALE BOOLEAN                        GetResetGlobals();  
   LOCALE VOID                           ShowDefglobalsCommand();
   LOCALE VOID                           ShowDefglobals();
#endif

#endif


