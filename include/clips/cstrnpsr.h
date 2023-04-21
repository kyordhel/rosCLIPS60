   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             CONSTRAINT PARSER HEADER FILE           */
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

#ifndef _H_cstrnpsr
#define _H_cstrnpsr

#ifndef _H_constrnt
#include "constrnt.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRNPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

struct constraintParseRecord
  {  
   unsigned int type : 1;
   unsigned int range : 1;
   unsigned int allowedSymbols : 1;
   unsigned int allowedStrings : 1;
   unsigned int allowedLexemes : 1;
   unsigned int allowedFloats : 1;
   unsigned int allowedIntegers : 1;
   unsigned int allowedNumbers : 1;
   unsigned int allowedValues : 1;
   unsigned int allowedInstanceNames : 1;
   unsigned int cardinality : 1;
  };
  
typedef struct constraintParseRecord CONSTRAINT_PARSE_RECORD;

#if ANSI_COMPILER  
   LOCALE BOOLEAN                        CheckConstraintParseConflicts(CONSTRAINT_RECORD *);
   LOCALE VOID                           AttributeConflictErrorMessage(char *,char *);
#if (! RUN_TIME) && (! BLOAD_ONLY)
   LOCALE VOID                           InitializeConstraintParseRecord(CONSTRAINT_PARSE_RECORD *);
   LOCALE BOOLEAN                        StandardConstraint(char *);
   LOCALE BOOLEAN                        ParseStandardConstraint(char *,char *,
                                                                 CONSTRAINT_RECORD *,
                                                                 CONSTRAINT_PARSE_RECORD *,
                                                                 int);
   LOCALE VOID                           OverlayConstraint(CONSTRAINT_PARSE_RECORD *,
                                                           CONSTRAINT_RECORD *,CONSTRAINT_RECORD *);  
#endif
#else
   LOCALE BOOLEAN                        CheckConstraintParseConflicts();
   LOCALE VOID                           AttributeConflictErrorMessage();
#if (! RUN_TIME) && (! BLOAD_ONLY)
   LOCALE VOID                           InitializeConstraintParseRecord();
   LOCALE BOOLEAN                        StandardConstraint();
   LOCALE BOOLEAN                        ParseStandardConstraint();
   LOCALE VOID                           OverlayConstraint();  
#endif
#endif 

#endif




