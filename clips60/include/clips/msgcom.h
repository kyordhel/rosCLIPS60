   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_msgcom
#define _H_msgcom

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MSGCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif
   
#define INIT_STRING   "init"
#define DELETE_STRING "delete"
#define PRINT_STRING  "print"

#if ANSI_COMPILER
LOCALE VOID SetupMessageHandlers(void);
LOCALE char *GetDefmessageHandlerName(VOID *,unsigned);
LOCALE char *GetDefmessageHandlerType(VOID *,unsigned);
LOCALE unsigned GetNextDefmessageHandler(VOID *,unsigned);
#if DEBUGGING_FUNCTIONS
LOCALE BOOLEAN GetDefmessageHandlerWatch(VOID *,unsigned);
LOCALE VOID SetDefmessageHandlerWatch(int,VOID *,unsigned);
#endif
LOCALE unsigned FindDefmessageHandler(VOID *,char *,char *);
LOCALE int IsDefmessageHandlerDeletable(VOID *,unsigned);
LOCALE VOID UndefmessageHandlerCommand(void);
LOCALE int UndefmessageHandler(VOID *,unsigned);

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDefmessageHandlerCommand(void);
LOCALE VOID ListDefmessageHandlersCommand(void);
LOCALE VOID PreviewSendCommand(void);
LOCALE char *GetDefmessageHandlerPPForm(VOID *,unsigned);
LOCALE VOID ListDefmessageHandlers(char *,VOID *,int);
LOCALE VOID PreviewSend(char *,VOID *,char *);
LOCALE long DisplayHandlersInLinks(char *,PACKED_CLASS_LINKS *,unsigned);
#endif

#else

LOCALE VOID SetupMessageHandlers();
LOCALE char *GetDefmessageHandlerName();
LOCALE char *GetDefmessageHandlerType();
LOCALE unsigned GetNextDefmessageHandler();
#if DEBUGGING_FUNCTIONS
LOCALE BOOLEAN GetDefmessageHandlerWatch();
LOCALE VOID SetDefmessageHandlerWatch();
#endif
LOCALE unsigned FindDefmessageHandler();
LOCALE int IsDefmessageHandlerDeletable();
LOCALE VOID UndefmessageHandlerCommand();
LOCALE int UndefmessageHandler();

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDefmessageHandlerCommand();
LOCALE VOID ListDefmessageHandlersCommand();
LOCALE VOID PreviewSendCommand();
LOCALE char *GetDefmessageHandlerPPForm();
LOCALE VOID ListDefmessageHandlers();
LOCALE VOID PreviewSend();
LOCALE long DisplayHandlersInLinks();
#endif

#endif

#ifndef _MSGCOM_SOURCE_
#endif

#endif





