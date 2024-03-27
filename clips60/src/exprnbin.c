   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             EXPRESSION BSAVE/BLOAD MODULE           */
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

#define _EXPRNBIN_SOURCE_

#include "setup.h"

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "dffctdef.h"
#include "moduldef.h"
#include "constrct.h"
#include "extnfunc.h"
#include "bload.h"
#include "bsave.h"

#if DEFRULE_CONSTRUCT
#include "network.h"
#endif

#if DEFGENERIC_CONSTRUCT
#include "genrcbin.h"
#endif

#if DEFFUNCTION_CONSTRUCT
#include "dffnxbin.h"
#endif

#if DEFTEMPLATE_CONSTRUCT
#include "tmpltbin.h"
#endif

#if DEFGLOBAL_CONSTRUCT
#include "globlbin.h"
#endif

#if OBJECT_SYSTEM
#include "objbin.h"
#include "insfun.h"
#include "inscom.h"
#endif

#include "exprnbin.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static  long                                  NumberOfExpressions;
   
/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct expr HUGE_ADDR                 *ExpressionArray;
   globle long int                               ExpressionCount;
   
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                        UpdateExpression(VOID *,long);
#else
   static VOID                        UpdateExpression();
#endif

/*************************************************************************/
/* Allocate Expressions: Loads in the expressions (i.e. test structures) */
/*   used by the binary image.                                           */
/*************************************************************************/
globle VOID AllocateExpressions()
  {
   unsigned long space;

   GenRead((VOID *) &NumberOfExpressions,(unsigned long) sizeof(long));
   if (NumberOfExpressions == 0L)
     ExpressionArray = NULL;
   else
     {
      space = NumberOfExpressions * sizeof(struct expr);
      ExpressionArray = (struct expr HUGE_ADDR *) genlongalloc(space);
     }
  }
  
/*************************************************************************/
/* Refresh Expressions: Refreshes pointers in expressions                */
/*************************************************************************/
globle VOID RefreshExpressions()
  {
   if (ExpressionArray == NULL) return;
   
   BloadandRefresh(NumberOfExpressions,
                   (unsigned) sizeof(BSAVE_EXPRESSION),UpdateExpression);
  }
  
/*********************************************************
  NAME         : UpdateExpression
  DESCRIPTION  : Given a bloaded expression buffer,
                   this routine refreshes the pointers
                   in the expression array
  INPUTS       : 1) a bloaded expression buffer
                 2) the index of the expression to refresh
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expression updated
  NOTES        : None
 *********************************************************/
static VOID UpdateExpression(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_EXPRESSION *bexp;
   long index;
   
   bexp = (BSAVE_EXPRESSION *) buf;
   ExpressionArray[obji].type = bexp->type;
   switch(bexp->type)
     {
      case FCALL:
        ExpressionArray[obji].value = (VOID *) FunctionArray[bexp->value];
        break;

      case GCALL:
#if DEFGENERIC_CONSTRUCT
        ExpressionArray[obji].value = (VOID *) GenericPointer(bexp->value);
#else
        ExpressionArray[obji].value = NULL;
#endif
        break;

      case PCALL:
#if DEFFUNCTION_CONSTRUCT
        ExpressionArray[obji].value = (VOID *) DeffunctionPointer(bexp->value);
#else
        ExpressionArray[obji].value = NULL;
#endif
        break;
              
      case DEFTEMPLATE_PTR:
#if DEFTEMPLATE_CONSTRUCT
        ExpressionArray[obji].value = (VOID *) DeftemplatePointer(bexp->value);
#else
        ExpressionArray[obji].value = NULL;
#endif
        break;
     
     case DEFCLASS_PTR:
#if OBJECT_SYSTEM
        ExpressionArray[obji].value = (VOID *) DefclassPointer(bexp->value);
#else
        ExpressionArray[obji].value = NULL;
#endif
        break;

      case DEFGLOBAL_PTR:
      
#if DEFGLOBAL_CONSTRUCT
        ExpressionArray[obji].value = (VOID *) DefglobalPointer(bexp->value);
#else
        ExpressionArray[obji].value = NULL;
#endif
        break;


      case INTEGER:
        ExpressionArray[obji].value = (VOID *) IntegerArray[bexp->value];
        IncrementIntegerCount((INTEGER_HN *) ExpressionArray[obji].value);
        break;

      case FLOAT:
        ExpressionArray[obji].value = (VOID *) FloatArray[bexp->value];
        IncrementFloatCount((FLOAT_HN *) ExpressionArray[obji].value);
        break;

      case INSTANCE_NAME:
#if ! OBJECT_SYSTEM
        ExpressionArray[obji].type = SYMBOL;
#endif
      case GBL_VARIABLE:
      case SYMBOL:
      case STRING:
        ExpressionArray[obji].value = (VOID *) SymbolArray[bexp->value];
        IncrementSymbolCount((SYMBOL_HN *) ExpressionArray[obji].value);
        break;

#if DEFTEMPLATE_CONSTRUCT      
      case FACT_ADDRESS:
        ExpressionArray[obji].value = (VOID *) &DummyFact;
        IncrementFactCount(ExpressionArray[obji].value);
        break;
#endif

#if OBJECT_SYSTEM        
      case INSTANCE_ADDRESS:
        ExpressionArray[obji].value = (VOID *) &DummyInstance;
        IncrementInstanceCount(ExpressionArray[obji].value);
        break;
#endif
        
      case EXTERNAL_ADDRESS:
        ExpressionArray[obji].value = NULL;
        break;

      default:        
        if (PrimitivesArray[bexp->type] == NULL) break;
        if (PrimitivesArray[bexp->type]->bitMap) 
          {
           ExpressionArray[obji].value = (VOID *) BitMapArray[bexp->value];
           IncrementBitMapCount((BITMAP_HN *) ExpressionArray[obji].value);
          }
        break;
     }

   index = (long int) bexp->nextArg;
   if (index == -1L)
     { ExpressionArray[obji].nextArg = NULL; }
   else
     { ExpressionArray[obji].nextArg = (struct expr *) &ExpressionArray[index]; }

   index = (long int) bexp->argList;
   if (index == -1L)
     { ExpressionArray[obji].argList = NULL; }
   else
     { ExpressionArray[obji].argList = (struct expr *) &ExpressionArray[index]; }
  }

/*************************************************************************/
/* ClearBloadedExpressions:                                           */
/*************************************************************************/
globle VOID ClearBloadedExpressions()
  {   
   unsigned long int i, space;
   
   /*==========================*/
   /* Update the symbol table. */
   /*==========================*/

   for (i = 0; i < NumberOfExpressions; i++)
     {
      switch (ExpressionArray[i].type)
        {
         case SYMBOL          :
         case STRING          :
         case INSTANCE_NAME   :
         case GBL_VARIABLE    :
           DecrementSymbolCount((SYMBOL_HN *) ExpressionArray[i].value);
           break;
         case FLOAT           :
           DecrementFloatCount((FLOAT_HN *) ExpressionArray[i].value);
           break;
         case INTEGER         :
           DecrementIntegerCount((INTEGER_HN *) ExpressionArray[i].value);
           break;

#if DEFTEMPLATE_CONSTRUCT   
         case FACT_ADDRESS    :
           DecrementFactCount(ExpressionArray[i].value);
           break;
#endif

#if OBJECT_SYSTEM
         case INSTANCE_ADDRESS :
           DecrementInstanceCount(ExpressionArray[i].value);
           break;
#endif

         default:        
           if (PrimitivesArray[ExpressionArray[i].type] == NULL) break;
           if (PrimitivesArray[ExpressionArray[i].type]->bitMap) 
             { DecrementBitMapCount((BITMAP_HN *) ExpressionArray[i].value); }
           break;
        }
     }

   /*========================*/
   /* Free expression array. */
   /*========================*/

   space = NumberOfExpressions * sizeof(struct expr);
   if (space != 0) genlongfree((VOID *) ExpressionArray,space);
  }


#if BLOAD_AND_BSAVE

/***************************************************
  NAME         : FindHashedExpressions
  DESCRIPTION  : Sets the bsave expression array
                 indices for hashed expression nodes
                 and marks the items needed by
                 these expressions
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Atoms marked and ids set
  NOTES        : None
 ***************************************************/
globle VOID FindHashedExpressions()
  {
   register unsigned i;
   EXPRESSION_HN *exphash;
   
   for (i = 0 ; i < EXPRESSION_HASH_SIZE ; i++)
     for (exphash = ExpressionHashTable[i] ; exphash != NULL ; exphash = exphash->nxt)
       {
        MarkNeededItems(exphash->exp);
        exphash->bsaveID = ExpressionCount;
        ExpressionCount += ExpressionSize(exphash->exp);
       }
  }

/***************************************************
  NAME         : BsaveHashedExpressions
  DESCRIPTION  : Writes out hashed expressions
  INPUTS       : Bsave file stream pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expressions written
  NOTES        : None
 ***************************************************/
globle VOID BsaveHashedExpressions(fp)
  FILE *fp;
  {
   register unsigned i;
   EXPRESSION_HN *exphash;
   
   for (i = 0 ; i < EXPRESSION_HASH_SIZE ; i++)
     for (exphash = ExpressionHashTable[i] ; exphash != NULL ; exphash = exphash->nxt)
       BsaveExpression(exphash->exp,fp);
  }
  
/***************************************************************/
/* BsaveConstructExpressions: Writes all expression needed by  */
/*   constructs for this binary image to the binary save file. */
/***************************************************************/
globle VOID BsaveConstructExpressions(fp)
  FILE *fp;
  {
   struct BinaryItem *biPtr;

   biPtr = ListOfBinaryItems;
   while (biPtr != NULL)
     {
      if (biPtr->expressionFunction != NULL) (*biPtr->expressionFunction)(fp);
      biPtr = biPtr->next;
     }
  }

/************************************************************************/
/* BsaveExpression: Recursively saves an expression to the binary file. */
/************************************************************************/
globle VOID BsaveExpression(testPtr,fp)
  struct expr *testPtr;
  FILE *fp;
  {
   BSAVE_EXPRESSION newTest;
   long int newIndex;

   while (testPtr != NULL)
     {
      ExpressionCount++;

      /*================*/
      /* Copy the type. */
      /*================*/

      newTest.type = testPtr->type;

      /*========================================*/
      /* Convert the argList slot to an index. */
      /*========================================*/

      if (testPtr->argList == NULL)
        { newTest.argList = -1L; }
      else
        { newTest.argList = ExpressionCount; }

      /*========================================*/
      /* Convert the nextArg slot to an index. */
      /*========================================*/

      if (testPtr->nextArg == NULL)
        { newTest.nextArg = -1L; }
      else
        {
         newIndex = ExpressionCount + ExpressionSize(testPtr->argList);
         newTest.nextArg = newIndex;
        }

      /*=========================*/
      /* Convert the value slot. */
      /*=========================*/

      switch(testPtr->type)
        {
         case FLOAT:
           newTest.value = (unsigned long) ((FLOAT_HN *) testPtr->value)->bucket;
           break;

         case INTEGER:
           newTest.value = (unsigned long) ((INTEGER_HN *) testPtr->value)->bucket;
           break;

         case FCALL:
           newTest.value = (unsigned long) ((struct FunctionDefinition *) 
                                  testPtr->value)->bsaveIndex;
           break;

         case GCALL:
#if DEFGENERIC_CONSTRUCT
           if (testPtr->value != NULL)
             newTest.value = ((struct constructHeader *) testPtr->value)->bsaveID;
           else
#endif
             newTest.value = -1L;
           break;

         case PCALL:
#if DEFFUNCTION_CONSTRUCT
           if (testPtr->value != NULL)
             newTest.value = ((struct constructHeader *) testPtr->value)->bsaveID;
           else
#endif
             newTest.value = -1L;
           break;
           
         case DEFTEMPLATE_PTR:
#if DEFTEMPLATE_CONSTRUCT
           if (testPtr->value != NULL)
             newTest.value = ((struct constructHeader *) testPtr->value)->bsaveID;
           else
#endif
             newTest.value = -1L;
           break;
         
         case DEFCLASS_PTR:
#if OBJECT_SYSTEM
           if (testPtr->value != NULL)
             newTest.value = ((struct constructHeader *) testPtr->value)->bsaveID;
           else
#endif
             newTest.value = -1L;
           break;

         case DEFGLOBAL_PTR:
#if DEFGLOBAL_CONSTRUCT
           if (testPtr->value != NULL) 
             newTest.value = ((struct defglobal *) testPtr->value)->header.bsaveID;
           else
#endif
             newTest.value = -1L;
           break;
           
#if OBJECT_SYSTEM
         case INSTANCE_NAME:
#endif
         case SYMBOL:
         case GBL_VARIABLE:
         case STRING:
           newTest.value = (unsigned long) ((SYMBOL_HN *) testPtr->value)->bucket;
           break;
         
         case FACT_ADDRESS:
         case INSTANCE_ADDRESS:
         case EXTERNAL_ADDRESS:
           newTest.value = -1L;
           break;
           
         default:
           if (PrimitivesArray[testPtr->type] == NULL) break;
           if (PrimitivesArray[testPtr->type]->bitMap) 
             { newTest.value = (unsigned long) ((BITMAP_HN *) testPtr->value)->bucket; }
           break;
        }

     /*===========================*/
     /* Write out the expression. */
     /*===========================*/

     GenWrite(&newTest,(unsigned long) sizeof(BSAVE_EXPRESSION),fp);

     /*==========================*/
     /* Write out argument list. */
     /*==========================*/

     if (testPtr->argList != NULL)
       {
        BsaveExpression(testPtr->argList,fp);
       }

     testPtr = testPtr->nextArg;
    }
  }
   
#endif

#endif /* (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) */
