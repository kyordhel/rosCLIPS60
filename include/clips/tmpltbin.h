   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFTEMPLATE BSAVE/BLOAD HEADER FILE         */
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

#if (! RUN_TIME)
#ifndef _H_tmpltbin

#define _H_tmpltbin
  
struct bsaveTemplateSlot
  {
   unsigned long slotName;
   unsigned int multislot : 1;
   unsigned int noDefault : 1;
   unsigned int defaultPresent : 1;
   unsigned int defaultDynamic : 1;
   long constraints;
   long defaultList;
   long next;
  };

struct bsaveDeftemplate;
struct bsaveDeftemplateModule;

#include "cstrcbin.h"
  
struct bsaveDeftemplate
  {
   struct bsaveConstructHeader header;
   long slotList;  
   unsigned int implied : 1;
   unsigned int numberOfSlots : 15; 
   long patternNetwork;  
  };

#include "modulbin.h"
 
struct bsaveDeftemplateModule
  {
   struct bsaveDefmoduleItemHeader header;
  }; 
  
#define DeftemplatePointer(i) ((struct deftemplate *) (&DeftemplateArray[i]))

#ifndef _H_tmpltdef
#include "tmpltdef.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _TMPLTBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER   
   LOCALE VOID                           DeftemplateBinarySetup(void);
   LOCALE VOID                          *BloadDeftemplateModuleReference(int);
#else
   LOCALE VOID                           DeftemplateBinarySetup();
   LOCALE VOID                          *BloadDeftemplateModuleReference();
#endif

#ifndef _TEMPLATE_SOURCE_
   extern struct deftemplate HUGE_ADDR  *DeftemplateArray;
#endif

#endif
#endif







