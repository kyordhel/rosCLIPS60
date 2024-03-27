   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  EXPRESSION MODULE                  */
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

#define _EXPRESSN_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#if ANSI_COMPILER
#include <stdlib.h>
#endif
#include <string.h>
#include <ctype.h>

#include "clipsmem.h"
#include "router.h"
#include "extnfunc.h"
#include "exprnops.h"
#include "prntutil.h"
#include "evaluatn.h"

#include "expressn.h"

#define PRIME_ONE   257
#define PRIME_TWO   263
#define PRIME_THREE 269

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle VOID              *PTR_AND;
   globle VOID              *PTR_OR;
   globle VOID              *PTR_EQ;
   globle VOID              *PTR_NEQ;
   globle VOID              *PTR_NOT;
   globle EXPRESSION_HN    **ExpressionHashTable = NULL;

/****************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS  */
/****************************************/

#if ANSI_COMPILER
#if (! RUN_TIME)
   static EXPRESSION_HN          *FindHashedExpression(EXPRESSION *,unsigned *,EXPRESSION_HN **);
   static unsigned                HashExpression(EXPRESSION *);
#endif
#else
#if (! RUN_TIME)
   static EXPRESSION_HN          *FindHashedExpression();
   static unsigned                HashExpression();
#endif
#endif

/***************************************************/
/* InitExpressionData: Initializes the function    */
/*   pointers used in generating some expressions. */
/***************************************************/
globle VOID InitExpressionData()
  {
   register unsigned i;
   
   InitExpressionPointers();

   ExpressionHashTable = (EXPRESSION_HN **)
     gm2((int) (sizeof(EXPRESSION_HN *) * EXPRESSION_HASH_SIZE));
   for (i = 0 ; i < EXPRESSION_HASH_SIZE ; i++)
     ExpressionHashTable[i] = NULL;
  }
  
/***************************************************/
/* InitExpressionPointers: Initializes the function    */
/*   pointers used in generating some expressions. */
/***************************************************/
globle VOID InitExpressionPointers()
  {
   PTR_AND          = (VOID *) FindFunction("and");
   PTR_OR           = (VOID *) FindFunction("or");
   PTR_EQ           = (VOID *) FindFunction("eq");
   PTR_NEQ          = (VOID *) FindFunction("neq");
   PTR_NOT          = (VOID *) FindFunction("not");

   if ((PTR_AND == NULL) || (PTR_OR == NULL) ||
       (PTR_EQ == NULL) || (PTR_NEQ == NULL) || (PTR_NOT == NULL))
     {
      CLIPSSystemError("EXPRESSN",1);
      ExitCLIPS(4);
     }
  }
  
#if (! RUN_TIME)

/********************************************************************/
/* ExpressionInstall:  Increments all occurrences in the hash table */
/*   of symbols found in an expression composed of test structures. */
/********************************************************************/
globle VOID ExpressionInstall(expression)
  struct expr *expression;
  {
   if (expression == NULL) return;

   while (expression != NULL)
     {
      AtomInstall(expression->type,expression->value); 
      ExpressionInstall(expression->argList);
      expression = expression->nextArg;
     }
  }

/**********************************************************************/
/* ExpressionDeinstall:  Increments all occurrences in the hash table */
/*   of symbols found in an expression composed of test structures.   */
/**********************************************************************/
globle VOID ExpressionDeinstall(expression)
  struct expr *expression;
  {
   if (expression == NULL) return;

   while (expression != NULL)
     {
      AtomDeinstall(expression->type,expression->value); 
      ExpressionDeinstall(expression->argList);
      expression = expression->nextArg;
     }
  }

/*********************************************************************************/
/* PackExpression: Copies an expression (created using multiple memory requests) */
/*   into an array (created using a single memory request) while maintaining all */
/*   appropriate links in the expression. A packed expression requires less      */
/*   total memory because it reduces the overhead required for multiple memory   */
/*   allocations.                                                                */
/*********************************************************************************/
globle struct expr *PackExpression(original)
  struct expr *original;
  {
   struct expr *packPtr;

   if (original == NULL) return (NULL);
   packPtr = (struct expr *) gm3((long) sizeof (struct expr) * (long) ExpressionSize(original));
   ListToPacked(original,packPtr,0);
   return(packPtr);
  }

/***********************************************************/
/* ListToPacked: Copies a list of expressions to an array. */
/***********************************************************/
globle int ListToPacked(original,destination,count)
  struct expr *original, *destination;
  int count;
  {
   long i;

   if (original == NULL) { return(count); }

   while (original != NULL)
     {
      i = count;
      count++;

      destination[i].type = original->type;
      destination[i].value = original->value;

      if (original->argList == NULL)
        { destination[i].argList = NULL; }
      else
        {
         destination[i].argList = &destination[(long) count];
         count = ListToPacked(original->argList,destination,count);
        }

      if (original->nextArg == NULL)
        { destination[i].nextArg = NULL; }
      else
        { destination[i].nextArg = &destination[(long) count]; }

      original = original->nextArg;
     }

   return(count);
  }

/***************************************************************/
/* ReturnPackedExpression: Returns a packed expression created */
/*   using PackExpression to the memory manager.               */
/***************************************************************/
globle VOID ReturnPackedExpression(packPtr)
  struct expr *packPtr;
  {
   if (packPtr != NULL)
     { rm3((VOID *) packPtr,(long) sizeof (struct expr) * ExpressionSize(packPtr)); }
  }

#endif

/******************************************************************************/
/* ReturnExpression:  Returns a multiply linked list of expr data structures. */
/******************************************************************************/
globle VOID ReturnExpression(waste)
  struct expr *waste;
  {
   register struct expr *tmp;

   while (waste != NULL)
     {
      if (waste->argList != NULL)
        ReturnExpression(waste->argList);
      tmp = waste;
      waste = waste->nextArg;
      rtn_struct(expr,tmp);
     }
  }
   
#if (! RUN_TIME)

/***************************************************
  NAME         : FindHashedExpression
  DESCRIPTION  : Determines if a given expression
                 is in the expression hash table
  INPUTS       : 1) The expression
                 2) A buffer to hold the hash
                    value
                 3) A buffer to hold the previous
                    node in the hash chain
  RETURNS      : The expression hash table entry
                 (NULL if not found)
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static EXPRESSION_HN *FindHashedExpression(exp,hashval,prv)
  EXPRESSION *exp;
  unsigned *hashval;
  EXPRESSION_HN **prv;
  {
   EXPRESSION_HN *exphash;
   
   if (exp == NULL)
     return(NULL);
   *hashval = HashExpression(exp);
   *prv = NULL;
   exphash = ExpressionHashTable[*hashval];
   while (exphash != NULL)
     {
      if (IdenticalExpression(exphash->exp,exp))
        return(exphash);
      *prv = exphash;
      exphash = exphash->nxt;
     }
   return(NULL);
  }
  
/***************************************************
  NAME         : HashExpression
  DESCRIPTION  : Assigns a deterministic number to
                 an expression
  INPUTS       : The expression
  RETURNS      : The "value" of the expression
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static unsigned HashExpression(exp)
  EXPRESSION *exp;
  {
   unsigned long tally = PRIME_THREE;
   
   if (exp->argList != NULL)
     tally += HashExpression(exp->argList) * PRIME_ONE;
   while (exp != NULL)
     {
      tally += exp->type * PRIME_TWO;
      tally += (unsigned long) exp->value;
      exp = exp->nextArg;
     }
   return((unsigned) (tally % EXPRESSION_HASH_SIZE));
  }
  
/***************************************************
  NAME         : RemoveHashedExpression
  DESCRIPTION  : Removes a hashed expression from
                 the hash table
  INPUTS       : The expression
  RETURNS      : Nothing useful
  SIDE EFFECTS : Hash node removed (or use count
                 decremented).  If the hash node
                 is removed, the expression is
                 deinstalled and deleted
  NOTES        : If the expression is in use by
                 others, then the use count is
                 merley decremented
 ***************************************************/
globle VOID RemoveHashedExpression(exp)
  EXPRESSION *exp;
  {
   EXPRESSION_HN *exphash,*prv;
   unsigned hashval;
   
   exphash = FindHashedExpression(exp,&hashval,&prv);
   if (exphash == NULL)
     return;
   if (--exphash->count != 0)
     return;
   if (prv == NULL)
     ExpressionHashTable[hashval] = exphash->nxt;
   else
     prv->nxt = exphash->nxt;
   ExpressionDeinstall(exphash->exp);
   ReturnPackedExpression(exphash->exp);
   rtn_struct(exprHashNode,exphash);
  }
     
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)

/*****************************************************
  NAME         : AddHashedExpression
  DESCRIPTION  : Adds a new expression to the
                 expression hash table (or increments
                 the use count if it is already there)
  INPUTS       : The (new) expression
  RETURNS      : A pointer to the (new) hash node
  SIDE EFFECTS : Adds the new hash node or increments
                 the count of an existing one
  NOTES        : It is the caller's responsibility to
                 delete the passed expression.  This
                 routine copies, packs and installs
                 the given expression
 *****************************************************/
globle EXPRESSION *AddHashedExpression(exp)
  EXPRESSION *exp;
  {
   EXPRESSION_HN *prv,*exphash;
   unsigned hashval;
   
   if (exp == NULL) return(NULL);
   exphash = FindHashedExpression(exp,&hashval,&prv);
   if (exphash != NULL)
     {
      exphash->count++;
      return(exphash->exp);
     }
   exphash = get_struct(exprHashNode);
   exphash->hashval = hashval;
   exphash->count = 1;
   exphash->exp = PackExpression(exp);
   ExpressionInstall(exphash->exp);
   exphash->nxt = ExpressionHashTable[exphash->hashval];
   ExpressionHashTable[exphash->hashval] = exphash;
   exphash->bsaveID = 0L;
   return(exphash->exp);
  }
  
#endif

#if (BLOAD_AND_BSAVE || BLOAD_ONLY || BLOAD || CONSTRUCT_COMPILER) && (! RUN_TIME)

/***************************************************
  NAME         : HashedExpressionIndex
  DESCRIPTION  : Finds the expression bload array
                 index for a hashed expression
  INPUTS       : The expression
  RETURNS      : The bload index
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle long HashedExpressionIndex(exp)
  EXPRESSION *exp;
  {
   EXPRESSION_HN *exphash,*prv;
   unsigned hashval;
   
   if (exp == NULL)
     return(-1L);
   exphash = FindHashedExpression(exp,&hashval,&prv);
   return((exphash != NULL) ? exphash->bsaveID : -1L);
  }
  
#endif

