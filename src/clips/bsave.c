   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                     BSAVE MODULE                    */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _BSAVE_SOURCE_

#include "setup.h"

#include "clipsmem.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"
#include "cstrnbin.h"
#include "moduldef.h"
#include "symblbin.h"
#include "bload.h"

#include "bsave.h"

/*******************/
/* DATA STRUCTURES */
/*******************/

#if BLOAD_AND_BSAVE
typedef struct bloadcntsv
  {
   long val;
   struct bloadcntsv *nxt;
  } BLOADCNTSV;
#endif
  
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if BLOAD_AND_BSAVE
   static VOID                        FindNeededItems(void);
   static VOID                        InitializeFunctionNeededFlags(void);
   static VOID                        WriteNeededFunctions(FILE *);
   static unsigned long int           FunctionBinarySize(void);
   static VOID                        WriteBinaryHeader(FILE *);
   static VOID                        WriteBinaryFooter(FILE *);
#endif
#else
#if BLOAD_AND_BSAVE
   static VOID                        FindNeededItems();
   static VOID                        InitializeFunctionNeededFlags();
   static VOID                        WriteNeededFunctions();
   static unsigned long int           FunctionBinarySize();
   static VOID                        WriteBinaryHeader();
   static VOID                        WriteBinaryFooter();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct BinaryItem         *ListOfBinaryItems = NULL;

/****************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS  */
/****************************************/

#if BLOAD_AND_BSAVE
   static BLOADCNTSV                *BloadCountSaveTop = NULL;
#endif

/**************************************************/
/* BsaveCommand: Handles top level bsave command. */
/**************************************************/
globle int BsaveCommand()
  {
#if (! RUN_TIME) && BLOAD_AND_BSAVE
   char *fileName;

   if (ArgCountCheck("bsave",EXACTLY,1) == -1) return(CLIPS_FALSE);
   fileName = GetFileName("bsave",1);
   if (fileName != NULL)
     { if (Bsave(fileName)) return(CLIPS_TRUE); }
#endif
   return(CLIPS_FALSE);
  }

#if BLOAD_AND_BSAVE

/********************************************************/
/* Bsave: Stores the binary representation of the CLIPS */
/*   environment to a file.                             */
/********************************************************/
globle BOOLEAN Bsave(fileName)
  char *fileName;
  {
   FILE *fp;
   struct BinaryItem *biPtr;
   char constructBuffer[CONSTRUCT_HEADER_SIZE];
   long saveExpressionCount;
   
   if (Bloaded())
     {
      PrintErrorID("BSAVE",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot perform a binary save while a binary load is in effect.\n");
      return(0);
     }

   /*================*/
   /* Open the file. */
   /*================*/

   if ((fp = fopen(fileName,"wb")) == NULL)
     {
      OpenErrorMessage("bsave",fileName);
      return(0);
     }
     
   /*==============================*/
   /* Remember the current module. */
   /*==============================*/
   
   SaveCurrentModule();
   
   /*==================================*/
   /* Write binary header to the file. */
   /*==================================*/

   WriteBinaryHeader(fp);

   /*=============================*/
   /* Initialize count variables. */
   /*=============================*/

   ExpressionCount = 0;
   InitializeFunctionNeededFlags();
   InitAtomicValueNeededFlags();
   FindHashedExpressions();
   FindNeededItems();
   SetAtomicValueIndices(CLIPS_FALSE);

   /*===============================*/
   /* Save the functions and atoms. */
   /*===============================*/

   WriteNeededFunctions(fp);
   
   WriteNeededAtomicValues(fp);
   
   GenWrite((VOID *) &ExpressionCount,(unsigned long) sizeof(unsigned long),fp);

   /*============================================================*/
   /* Save the numbers indicating the amount of memory necessary */
   /* to store the constructs.                                   */
   /*============================================================*/

   biPtr = ListOfBinaryItems;
   while (biPtr != NULL)
     {
      if (biPtr->bsaveStorageFunction != NULL)
        {
         strncpy(constructBuffer,biPtr->name,CONSTRUCT_HEADER_SIZE);
         GenWrite(constructBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE,fp);
         (*biPtr->bsaveStorageFunction)(fp);
        }
      biPtr = biPtr->next;
     }
   WriteBinaryFooter(fp);

   /*===================*/
   /* Save expressions. */
   /*===================*/

   ExpressionCount = 0;
   BsaveHashedExpressions(fp);
   saveExpressionCount = ExpressionCount;
   BsaveConstructExpressions(fp);
   
   /*======================================*/
   /* Save the constraints and constructs. */
   /*======================================*/

   ExpressionCount = saveExpressionCount;
   WriteNeededConstraints(fp);
   
   biPtr = ListOfBinaryItems;
   while (biPtr != NULL)
     {
      if (biPtr->bsaveFunction != NULL)
        {
         strncpy(constructBuffer,biPtr->name,CONSTRUCT_HEADER_SIZE);
         GenWrite(constructBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE,fp);
         (*biPtr->bsaveFunction)(fp);
        }
      biPtr = biPtr->next;
     }
     
   WriteBinaryFooter(fp);

   /*===========*/
   /* Clean up. */
   /*===========*/

   RestoreAtomicValueBuckets();

   /*=================*/
   /* Close the file. */
   /*=================*/

   fclose(fp);

   /*=============================*/
   /* Restore the current module. */
   /*=============================*/
   
   RestoreCurrentModule();
   
   /*========================================*/
   /* Return CLIPS_TRUE to indicate success. */
   /*========================================*/

   return(CLIPS_TRUE);
  }

/****************************************************************************/
/* InitializeFunctionNeededFlags:                                                             */
/****************************************************************************/
static VOID InitializeFunctionNeededFlags()
  {
   struct FunctionDefinition *functionList;

   functionList = GetFunctionList();

   while (functionList != NULL)
     {
      functionList->bsaveIndex = 0;
      functionList = functionList->next;
     }
  }
  
/**********************************************************/
/* FindNeededItems: Searches through the constructs for   */
/*   the functions, constraints, or atoms that are needed */
/*   by that construct. This routine also counts the      */
/*   number of expressions in use (through a global)      */
/**********************************************************/
static VOID FindNeededItems()
  {
   struct BinaryItem *biPtr;

   biPtr = ListOfBinaryItems;
   while (biPtr != NULL)
     {
      if (biPtr->findFunction != NULL) (*biPtr->findFunction)();
      biPtr = biPtr->next;
     }
  }

/****************************************************/
/* WriteNeededFunctions: Writes the names of needed */
/*   functions to the binary save file.             */
/****************************************************/
static VOID WriteNeededFunctions(fp)
  FILE *fp;
  {
   unsigned long int space, count = 0, length;
   struct FunctionDefinition *functionList;

   /*==============================================*/
   /* Assign each function an index if it is used. */
   /*==============================================*/

   functionList = GetFunctionList();

   while (functionList != NULL)
     {
      if (functionList->bsaveIndex)
        functionList->bsaveIndex = (short int) count++;
      else functionList->bsaveIndex = -1;
      functionList = functionList->next;
     }

   /*=======================================================*/
   /* Determine the number of function names to be written. */
   /*=======================================================*/

   GenWrite(&count,(unsigned long) sizeof(unsigned long int),fp);
   if (count == 0)
     {
      GenWrite(&count,(unsigned long) sizeof(unsigned long int),fp);
      return;
     }

   /*=========================================================*/
   /* Determine the amount of space needed for the functions. */
   /*=========================================================*/

   space = FunctionBinarySize();
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);

   /*===============================*/
   /* Write out the function names. */
   /*===============================*/

   functionList = GetFunctionList();

   while (functionList != NULL)
     {
      if (functionList->bsaveIndex >= 0)
        {
         length = strlen(ValueToString(functionList->callFunctionName)) + 1;
         GenWrite(ValueToString(functionList->callFunctionName),(unsigned long) length,fp);
        }
      functionList = functionList->next;
     }
  }

/****************************************************************/
/* FunctionBinarySize: Determines the number of bytes needed to */
/*   save all of the function names in the binary save file.    */
/****************************************************************/
static unsigned long int FunctionBinarySize()
  {
   unsigned long int size = 0;
   struct FunctionDefinition *functionList;

   functionList = GetFunctionList();

   while (functionList != NULL)
     {
      if (functionList->bsaveIndex >= 0)
        { size += strlen(ValueToString(functionList->callFunctionName)) + 1; }
      functionList = functionList->next;
     }

   return(size);
  }

/****************************************************/
/* SaveBloadCount:                                  */
/****************************************************/
globle VOID SaveBloadCount(cnt)
  long cnt;
  {
   BLOADCNTSV *tmp,*prv;
   
   tmp = get_struct(bloadcntsv);
   tmp->val = cnt;
   tmp->nxt = NULL;
   if (BloadCountSaveTop == NULL)
     BloadCountSaveTop = tmp;
   else
     {
      prv = BloadCountSaveTop;
      while (prv->nxt != NULL)
        prv = prv->nxt;
      prv->nxt = tmp;
     }
  }
  
/****************************************************/
/* RestoreBloadCount:                               */
/****************************************************/
globle VOID RestoreBloadCount(cnt)
  long *cnt;
  {
   BLOADCNTSV *tmp;

   *cnt = BloadCountSaveTop->val;
   tmp = BloadCountSaveTop;
   BloadCountSaveTop = BloadCountSaveTop->nxt;
   rtn_struct(bloadcntsv,tmp);
  }
  
/**************************************************************/
/* MarkNeededItems: Examines an expression to determine which */
/*   items are necessary to evaluate the expression.          */
/**************************************************************/
globle VOID MarkNeededItems(testPtr)
  struct expr *testPtr;
  {
   while (testPtr != NULL)
     {
      switch (testPtr->type)
        {
         case SYMBOL:
         case STRING:
         case GBL_VARIABLE:
         case INSTANCE_NAME:
            ((SYMBOL_HN *) testPtr->value)->neededSymbol = CLIPS_TRUE;
            break;

         case FLOAT:
            ((FLOAT_HN *) testPtr->value)->neededFloat = CLIPS_TRUE;
            break;

         case INTEGER:
            ((INTEGER_HN *) testPtr->value)->neededInteger = CLIPS_TRUE;
            break;

         case FCALL:
            ((struct FunctionDefinition *) testPtr->value)->bsaveIndex = CLIPS_TRUE;
            break;
         
         default:
           if (PrimitivesArray[testPtr->type] == NULL) break;
           if (PrimitivesArray[testPtr->type]->bitMap) 
             { ((BITMAP_HN *) testPtr->value)->neededBitMap = CLIPS_TRUE; }
           break;

        }
        
      if (testPtr->argList != NULL)
        { MarkNeededItems(testPtr->argList); }

      testPtr = testPtr->nextArg;
     }
  }
  
/*****************************************************************/
/* WriteBinaryHeader:               */
/*****************************************************************/
static VOID WriteBinaryHeader(fp)
  FILE *fp;
  {
   GenWrite(BinaryPrefixID,(unsigned long) strlen(BinaryPrefixID) + 1,fp);
   GenWrite(BinaryVersionID,(unsigned long) strlen(BinaryVersionID) + 1,fp);
  }

/*****************************************************************/
/* WriteBinaryFooter:               */
/*****************************************************************/
static VOID WriteBinaryFooter(fp)
  FILE *fp;
  {
   char footerBuffer[CONSTRUCT_HEADER_SIZE];

   strncpy(footerBuffer,BinaryPrefixID,CONSTRUCT_HEADER_SIZE);
   GenWrite(footerBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE,fp);
  }

#endif

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
/********************/
/* AddBinaryItem:   */
/********************/
globle BOOLEAN AddBinaryItem(name,priority,findFunction,expressionFunction,
                      bsaveStorageFunction,bsaveFunction,
                      bloadStorageFunction,bloadFunction,clearFunction)
  char *name;
  int priority;
#if ANSI_COMPILER
  VOID (*findFunction)(void);
  VOID (*expressionFunction)(FILE *);
  VOID (*bsaveStorageFunction)(FILE *);
  VOID (*bsaveFunction)(FILE *);
  VOID (*bloadStorageFunction)(void);
  VOID (*bloadFunction)(void);
  VOID (*clearFunction)(void);
#else
  VOID (*findFunction)();
  VOID (*expressionFunction)();
  VOID (*bsaveStorageFunction)();
  VOID (*bsaveFunction)();
  VOID (*bloadStorageFunction)();
  VOID (*bloadFunction)();
  VOID (*clearFunction)();
#endif
  {
   struct BinaryItem *newPtr, *currentPtr, *lastPtr = NULL;

   newPtr = get_struct(BinaryItem);

   newPtr->name = name;
   newPtr->findFunction = findFunction;
   newPtr->expressionFunction = expressionFunction;
   newPtr->bsaveStorageFunction = bsaveStorageFunction;
   newPtr->bsaveFunction = bsaveFunction;
   newPtr->bloadStorageFunction = bloadStorageFunction;
   newPtr->bloadFunction = bloadFunction;
   newPtr->clearFunction = clearFunction;
   newPtr->priority = priority;

   if (ListOfBinaryItems == NULL)
     {
      newPtr->next = NULL;
      ListOfBinaryItems = newPtr;
      return(1);
     }

   currentPtr = ListOfBinaryItems;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = ListOfBinaryItems;
      ListOfBinaryItems = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }

   return(1);
  }

#endif

#if BLOAD_AND_BSAVE || BSAVE_INSTANCES

/****************************************************/
/* GenWrite: Generic routine for writing to a file. */
/*   No machine specific code as of yet.            */
/****************************************************/
globle VOID GenWrite(dataPtr,size,fp)
  VOID *dataPtr;
  unsigned long size;
  FILE *fp;
  {
   if (size == 0) return;
   fwrite(dataPtr,(int) size,1,fp);
  }

#endif





