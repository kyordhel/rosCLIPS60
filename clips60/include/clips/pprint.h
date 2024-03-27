   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               PRETTY PRINT HEADER FILE              */
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

#ifndef _H_pprint
#define _H_pprint

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _PPRINT_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE int                            FlushPPBuffer(void);
   LOCALE VOID                           DestroyPPBuffer(void);
   LOCALE VOID                           SavePPBuffer(char *);
   LOCALE VOID                           PPBackup(void);
   LOCALE char                          *CopyPPBuffer(void);
   LOCALE char                          *GetPPBuffer(void);
   LOCALE VOID                           PPCRAndIndent(void);
   LOCALE VOID                           IncrementIndentDepth(int);  
   LOCALE int                            DecrementIndentDepth(int);  
   LOCALE int                            SetIndentDepth(int);  
   LOCALE VOID                           SetPPBufferStatus(int);
   LOCALE int                            GetPPBufferStatus(void);
#else 
   LOCALE int                            FlushPPBuffer();
   LOCALE VOID                           DestroyPPBuffer();
   LOCALE VOID                           SavePPBuffer();
   LOCALE VOID                           PPBackup();
   LOCALE char                          *CopyPPBuffer();
   LOCALE char                          *GetPPBuffer();
   LOCALE VOID                           PPCRAndIndent();
   LOCALE VOID                           IncrementIndentDepth();  
   LOCALE int                            DecrementIndentDepth();  
   LOCALE int                            SetIndentDepth();  
   LOCALE VOID                           SetPPBufferStatus();
   LOCALE int                            GetPPBufferStatus();
#endif

#endif




