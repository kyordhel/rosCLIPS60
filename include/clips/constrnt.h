   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                CONSTRAINT HEADER FILE               */
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

#ifndef _H_constrnt
#define _H_constrnt

struct constraintRecord;

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CONSTRNT_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

struct constraintRecord
  {      
   unsigned int anyAllowed : 1;
   unsigned int symbolsAllowed : 1;
   unsigned int stringsAllowed : 1;
   unsigned int floatsAllowed : 1;
   unsigned int integersAllowed : 1;
   unsigned int instanceNamesAllowed : 1;
   unsigned int instanceAddressesAllowed : 1;
   unsigned int externalAddressesAllowed : 1;
   unsigned int multifieldsAllowed : 1;
   unsigned int factAddressesAllowed : 1;
   unsigned int anyRestriction : 1;
   unsigned int symbolRestriction : 1;
   unsigned int stringRestriction : 1;
   unsigned int floatRestriction : 1;
   unsigned int integerRestriction : 1;
   unsigned int instanceNameRestriction : 1;
   unsigned short bsaveIndex;
   struct expr *restrictionList;
   struct expr *minValue;
   struct expr *maxValue;
   struct expr *minFields; 
   struct expr *maxFields;
   struct constraintRecord *next;
   int bucket;
   int count;  
  };
  
typedef struct constraintRecord CONSTRAINT_RECORD;

#define SIZE_CONSTRAINT_HASH  167

#if ANSI_COMPILER  
   LOCALE VOID                           InitializeConstraints(VOID);
   LOCALE int                            GDCCommand(void);
   LOCALE int                            SDCCommand(void);
   LOCALE int                            GSCCommand(void);
   LOCALE int                            SSCCommand(void);
   LOCALE BOOLEAN                        SetDynamicConstraintChecking(int);
   LOCALE BOOLEAN                        GetDynamicConstraintChecking(VOID);
   LOCALE BOOLEAN                        SetStaticConstraintChecking(int);
   LOCALE BOOLEAN                        GetStaticConstraintChecking(VOID);
#if (! BLOAD_ONLY) && (! RUN_TIME)
   LOCALE int                            HashConstraint(struct constraintRecord *);
   LOCALE struct constraintRecord       *AddConstraint(struct constraintRecord *);
#endif
#if (! RUN_TIME)
   LOCALE VOID                           RemoveConstraint(struct constraintRecord *);
#endif
#else
   LOCALE VOID                           InitializeConstraints();
   LOCALE int                            GDCCommand();
   LOCALE int                            SDCCommand();
   LOCALE int                            GSCCommand();
   LOCALE int                            SSCCommand();
   LOCALE BOOLEAN                        SetDynamicConstraintChecking();
   LOCALE BOOLEAN                        GetDynamicConstraintChecking();
   LOCALE BOOLEAN                        SetStaticConstraintChecking();
   LOCALE BOOLEAN                        GetStaticConstraintChecking();
#if (! BLOAD_ONLY) && (! RUN_TIME)
   LOCALE int                            HashConstraint();
   LOCALE struct constraintRecord       *AddConstraint();
#endif
#if (! RUN_TIME)
   LOCALE VOID                           RemoveConstraint();
#endif
#endif 

#ifndef _CONSTRNT_SOURCE_
   extern struct constraintRecord   **ConstraintHashtable;
#endif

#endif




