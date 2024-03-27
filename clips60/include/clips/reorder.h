   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 REORDER HEADER FILE                 */
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

#ifndef _H_reorder
#define _H_reorder

struct lhsParseNode;

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_ruledef
#include "ruledef.h"
#endif
#ifndef _H_pattern
#include "pattern.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _REORDER_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

/***********************************************************************/
/* lhsParseNode structure: Stores information about the intermediate   */  
/*   parsed representation of the lhs of a rule.                       */
/***********************************************************************/
struct lhsParseNode
  {
   int type;  
   VOID *value;                 
   unsigned int negated : 1;   
   unsigned int logical : 1;    
   unsigned int multifieldSlot : 1;  
   unsigned int bindingVariable : 1; 
   unsigned int derivedConstraints : 1;
   unsigned int userCE : 1;
   unsigned int whichCE : 7;
   unsigned int marked : 1;
   unsigned int withinMultifieldSlot : 1;
   unsigned int multiFieldsBefore : 7;
   unsigned int multiFieldsAfter : 7;
   unsigned int singleFieldsBefore : 7;
   unsigned int singleFieldsAfter : 7;
   struct constraintRecord *constraints;
   struct lhsParseNode *referringNode;
   struct patternParser *patternType;
   int pattern;
   int index;
   struct symbolHashNode *slot;
   int slotNumber;
   int beginNandDepth;
   int endNandDepth;
   struct expr *networkTest;
   struct lhsParseNode *expression;
   VOID *userData;
   struct lhsParseNode *right;
   struct lhsParseNode *bottom;
  };
  
#if ANSI_COMPILER
LOCALE struct lhsParseNode           *ReorderPatterns(struct lhsParseNode *,int *);
LOCALE struct lhsParseNode           *CopyLHSParseNodes(struct lhsParseNode *);
LOCALE VOID                           CopyLHSParseNode(struct lhsParseNode *,struct lhsParseNode *,int);
LOCALE struct lhsParseNode           *GetLHSParseNode(void);
LOCALE VOID                           ReturnLHSParseNodes(struct lhsParseNode *);
LOCALE struct lhsParseNode           *ExpressionToLHSParseNodes(struct expr *);
LOCALE struct expr                   *LHSParseNodesToExpression(struct lhsParseNode *);
LOCALE VOID                           AddInitialPatterns(struct lhsParseNode *);
#else
LOCALE struct lhsParseNode           *ReorderPatterns();
LOCALE struct lhsParseNode           *CopyLHSParseNodes();
LOCALE VOID                           CopyLHSParseNode();
LOCALE struct lhsParseNode           *GetLHSParseNode();
LOCALE VOID                           ReturnLHSParseNodes();
LOCALE struct lhsParseNode           *ExpressionToLHSParseNodes();
LOCALE struct expr                   *LHSParseNodesToExpression();
LOCALE VOID                           AddInitialPatterns();
#endif

#endif



 
  




