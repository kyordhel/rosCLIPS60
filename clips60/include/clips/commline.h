   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*              COMMAND LINE HEADER FILE               */
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

#ifndef _H_commline

#define _H_commline

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _COMMLINE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE int                            ExpandCommandString(int);
   LOCALE VOID                           FlushCommandString(void);
   LOCALE VOID                           SetCommandString(char *);
   LOCALE VOID                           AppendCommandString(char *);
   LOCALE char                          *GetCommandString(void);
   LOCALE int                            CompleteCommand(char *);
   LOCALE VOID                           CommandLoop(void);
   LOCALE VOID                           PrintPrompt(void);
   LOCALE VOID                           SetMemoryStatusFunction(int (*)(void));
   LOCALE BOOLEAN                        RouteCommand(char *);
   LOCALE int                          (*SetEventFunction(int (*)(void)))(void);
   LOCALE BOOLEAN                        TopLevelCommand(void);
#else
   LOCALE int                            ExpandCommandString();
   LOCALE VOID                           FlushCommandString();
   LOCALE VOID                           SetCommandString();
   LOCALE VOID                           AppendCommandString();
   LOCALE char                          *GetCommandString();
   LOCALE int                            CompleteCommand();
   LOCALE VOID                           CommandLoop();
   LOCALE VOID                           PrintPrompt();
   LOCALE VOID                           SetMemoryStatusFunction();
   LOCALE BOOLEAN                        RouteCommand();
   LOCALE int                          (*SetEventFunction())();
   LOCALE BOOLEAN                        TopLevelCommand();
#endif

#ifndef _COMMLINE_SOURCE_
   extern int                     EvaluatingTopLevelCommand;
#endif

#endif
