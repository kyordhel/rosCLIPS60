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

#ifndef _H_msgpass
#define _H_msgpass

#define GetActiveInstance() ((INSTANCE_TYPE *) GetNthMessageArgument(0)->value)

#ifndef _H_object
#include "object.h"
#endif

typedef struct messageHandlerLink
  {
   HANDLER *hnd;
   struct messageHandlerLink *nxt;
  } HANDLER_LINK;

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MSGPASS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID DirectMessage(SYMBOL_HN *,INSTANCE_TYPE *,
                          DATA_OBJECT *,EXPRESSION *);
LOCALE VOID Send(DATA_OBJECT *,char *,char *,DATA_OBJECT *);
VOID DestroyHandlerLinks(HANDLER_LINK *);
LOCALE VOID SendCommand(DATA_OBJECT *);
LOCALE DATA_OBJECT *GetNthMessageArgument(int);

#if IMPERATIVE_MESSAGE_HANDLERS
LOCALE int NextHandlerAvailable(void);
LOCALE VOID CallNextHandler(DATA_OBJECT *);
#endif

LOCALE VOID FindApplicableOfName(DEFCLASS *,HANDLER_LINK *[],
                                 HANDLER_LINK *[],SYMBOL_HN *);
LOCALE HANDLER_LINK *JoinHandlerLinks(HANDLER_LINK *[],HANDLER_LINK *[],SYMBOL_HN *);

LOCALE VOID PrintHandlerSlotGetFunction(char *,VOID *);
LOCALE BOOLEAN HandlerSlotGetFunction(VOID *,DATA_OBJECT *);
LOCALE VOID PrintHandlerSlotPutFunction(char *,VOID *);
LOCALE BOOLEAN HandlerSlotPutFunction(VOID *,DATA_OBJECT *);
LOCALE VOID DynamicHandlerGetSlot(DATA_OBJECT *);
LOCALE VOID DynamicHandlerPutSlot(DATA_OBJECT *);

#else

LOCALE VOID DirectMessage();
LOCALE VOID Send();
LOCALE VOID DestroyHandlerLinks();
LOCALE VOID SendCommand();
LOCALE DATA_OBJECT *GetNthMessageArgument();

#if IMPERATIVE_MESSAGE_HANDLERS
LOCALE int NextHandlerAvailable();
LOCALE VOID CallNextHandler();
#endif

LOCALE VOID FindApplicableOfName();
LOCALE HANDLER_LINK *JoinHandlerLinks();

LOCALE VOID PrintHandlerSlotGetFunction();
LOCALE BOOLEAN HandlerSlotGetFunction();
LOCALE VOID PrintHandlerSlotPutFunction();
LOCALE BOOLEAN HandlerSlotPutFunction();
LOCALE VOID DynamicHandlerGetSlot();
LOCALE VOID DynamicHandlerPutSlot();

#endif
      
#ifndef _MSGPASS_SOURCE_
extern SYMBOL_HN *CurrentMessageName;
extern HANDLER_LINK *CurrentCore;
#endif

#endif







