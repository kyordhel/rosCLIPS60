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
/* Purpose: Message-passing support functions                */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_msgfun
#define _H_msgfun

typedef struct handlerSlotReference
  {
   unsigned short classID;
   unsigned slotID;
  } HANDLER_SLOT_REFERENCE;

#ifndef _H_object
#include "object.h"
#endif
#include "msgpass.h"

#define BEGIN_TRACE ">>"
#define END_TRACE   "<<"

/* =================================================================================
   Message-handler types - don't change these values: a string array depends on them
   ================================================================================= */
#define MAROUND        0
#define MBEFORE        1
#define MPRIMARY       2
#define MAFTER         3
#define MERROR         4

#define LOOKUP_HANDLER_INDEX   0
#define LOOKUP_HANDLER_ADDRESS 1

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MSGFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID UnboundHandlerErr(void);
LOCALE VOID PrintNoHandlerError(char *);
LOCALE int CheckHandlerArgCount(void);
LOCALE VOID SlotAccessViolationError(char *,BOOLEAN,VOID *);
LOCALE VOID SlotVisibilityViolationError(SLOT_DESC *,DEFCLASS *);

#if ! RUN_TIME
LOCALE VOID NewSystemHandler(char *,char *,char *,int);
LOCALE HANDLER *InsertHandlerHeader(DEFCLASS *,SYMBOL_HN *,int);
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE HANDLER *NewHandler(void);
LOCALE int HandlersExecuting(DEFCLASS *);
LOCALE int DeleteHandler(DEFCLASS *,SYMBOL_HN *,int,int);
LOCALE VOID DeallocateMarkedHandlers(DEFCLASS *);
#endif
LOCALE int HandlerType(char *,char *);
LOCALE int CheckCurrentMessage(char *,int);
LOCALE VOID PrintHandler(char *,HANDLER *,int);
LOCALE HANDLER *FindHandlerByAddress(DEFCLASS *,SYMBOL_HN *,unsigned);
LOCALE int FindHandlerByIndex(DEFCLASS *,SYMBOL_HN *,unsigned);
LOCALE int FindHandlerNameGroup(DEFCLASS *,SYMBOL_HN *);
LOCALE VOID HandlerDeleteError(char *);

#if DEBUGGING_FUNCTIONS
LOCALE VOID DisplayCore(char *,HANDLER_LINK *,int);
LOCALE HANDLER_LINK *FindPreviewApplicableHandlers(DEFCLASS *,SYMBOL_HN *);
LOCALE VOID WatchMessage(char *,char *);
LOCALE VOID WatchHandler(char *,HANDLER_LINK *,char *);
#endif

#else

LOCALE VOID UnboundHandlerErr();
LOCALE VOID PrintNoHandlerError();
LOCALE int CheckHandlerArgCount();
LOCALE VOID SlotAccessViolationError();
LOCALE VOID SlotVisibilityViolationError();

#if ! RUN_TIME
LOCALE VOID NewSystemHandler();
LOCALE HANDLER *InsertHandlerHeader();
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE HANDLER *NewHandler();
LOCALE int HandlersExecuting();
LOCALE int DeleteHandler();
LOCALE VOID DeallocateMarkedHandlers();
#endif
LOCALE int HandlerType();
LOCALE int CheckCurrentMessage();
LOCALE VOID PrintHandler();
LOCALE HANDLER *FindHandlerByAddress();
LOCALE int FindHandlerByIndex();
LOCALE int FindHandlerNameGroup();
LOCALE VOID HandlerDeleteError();

#if DEBUGGING_FUNCTIONS
LOCALE VOID DisplayCore();
LOCALE HANDLER_LINK *FindPreviewApplicableHandlers();
LOCALE VOID WatchMessage();
LOCALE VOID WatchHandler();
#endif

#endif
      
#ifndef _MSGFUN_SOURCE_
extern SYMBOL_HN *INIT_SYMBOL,*DELETE_SYMBOL;
extern char *hndquals[];

#if DEBUGGING_FUNCTIONS
extern int WatchHandlers,WatchMessages;
#endif
#endif

#endif







