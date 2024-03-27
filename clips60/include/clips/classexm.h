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

#ifndef _H_classexm
#define _H_classexm

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CLASSEXM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if DEBUGGING_FUNCTIONS
LOCALE VOID BrowseClassesCommand(void);
LOCALE VOID BrowseClasses(char *,VOID *);
LOCALE VOID DescribeClassCommand(void);
LOCALE VOID DescribeClass(char *,VOID *);
#endif

LOCALE char *GetCreateAccessorString(VOID *);

LOCALE SYMBOL_HN *GetDefclassModuleCommand(void);
LOCALE BOOLEAN SuperclassPCommand(void);
LOCALE BOOLEAN SuperclassP(VOID *,VOID *);
LOCALE BOOLEAN SubclassPCommand(void);
LOCALE BOOLEAN SubclassP(VOID *,VOID *);
LOCALE int SlotExistPCommand(void);
LOCALE BOOLEAN SlotExistP(VOID *,char *,BOOLEAN);
LOCALE int MessageHandlerExistPCommand(void);
LOCALE BOOLEAN SlotWritablePCommand(void);
LOCALE BOOLEAN SlotWritableP(VOID *,char *);
LOCALE BOOLEAN SlotInitablePCommand(void);
LOCALE BOOLEAN SlotInitableP(VOID *,char *);
LOCALE BOOLEAN SlotPublicPCommand(void);
LOCALE BOOLEAN SlotPublicP(VOID *,char *);
LOCALE BOOLEAN SlotDirectAccessPCommand(void);
LOCALE BOOLEAN SlotDirectAccessP(VOID *,char *);
LOCALE int ClassExistPCommand(void);

#else

#if DEBUGGING_FUNCTIONS
LOCALE VOID BrowseClassesCommand();
LOCALE VOID BrowseClasses();
LOCALE VOID DescribeClassCommand();
LOCALE VOID DescribeClass();
#endif

LOCALE char *GetCreateAccessorString();

LOCALE SYMBOL_HN *GetDefclassModuleCommand();
LOCALE BOOLEAN SuperclassPCommand();
LOCALE BOOLEAN SuperclassP();
LOCALE BOOLEAN SubclassPCommand();
LOCALE BOOLEAN SubclassP();
LOCALE int SlotExistPCommand();
LOCALE BOOLEAN SlotExistP();
LOCALE int MessageHandlerExistPCommand();
LOCALE BOOLEAN SlotWritablePCommand();
LOCALE BOOLEAN SlotWritableP();
LOCALE BOOLEAN SlotInitablePCommand();
LOCALE BOOLEAN SlotInitableP();
LOCALE BOOLEAN SlotPublicPCommand();
LOCALE BOOLEAN SlotPublicP();
LOCALE BOOLEAN SlotDirectAccessPCommand();
LOCALE BOOLEAN SlotDirectAccessP();
LOCALE int ClassExistPCommand();

#endif

#ifndef _CLASSEXM_SOURCE_
#endif

#endif





