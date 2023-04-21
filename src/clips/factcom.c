   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                FACT COMMANDS MODULE                 */
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

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT

#define _FACTCOM_SOURCE_

#include "clipsmem.h"
#include "exprnpsr.h"
#include "factmngr.h"
#include "argacces.h"
#include "match.h"
#include "router.h"
#include "scanner.h"
#include "constant.h"
#include "factrhs.h"
#include "factmch.h"
#include "extnfunc.h"
#include "tmpltpsr.h"
#include "facthsh.h"
#include "modulutl.h"

#include "tmpltdef.h"
#include "tmpltfun.h"

#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
#include "bload.h"
#endif

#include "factcom.h"

#define INVALID     -2L
#define UNSPECIFIED -1L

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if (! RUN_TIME)
   static struct expr            *AssertParse(struct expr *,char *);
#endif
#if DEBUGGING_FUNCTIONS
   static long int                GetFactsArgument(int,int);
#endif
   static struct expr            *StandardLoadFact(char *,struct token *);
#else
#if (! RUN_TIME)
   static struct expr            *AssertParse();
#endif
#if DEBUGGING_FUNCTIONS
   static long int                GetFactsArgument();
#endif
   static struct expr            *StandardLoadFact();
#endif

/************************************************************/
/* InitFactCommands: Makes appropriate DefineFunction calls */
/*   to notify CLIPS of functions defined in this module.   */
/************************************************************/
globle VOID InitFactCommands()
  {
#if ! RUN_TIME
#if DEBUGGING_FUNCTIONS
   DefineFunction2("facts",        'v', PTIF FactsCommand,        "FactsCommand", "*4iu");
#endif

   DefineFunction("assert",       'u', PTIF AssertCommand,  "AssertCommand");
   DefineFunction2("retract",      'v', PTIF RetractCommand, "RetractCommand","1*z");
   DefineFunction2("assert-string",       'u', PTIF AssertStringFunction,   "AssertStringFunction", "11s");
   DefineFunction2("str-assert",       'u', PTIF AssertStringFunction,   "AssertStringFunction", "11s");

   DefineFunction2("get-fact-duplication",'b',
                   GetFactDuplicationCommand,"GetFactDuplicationCommand", "00");
   DefineFunction2("set-fact-duplication",'b',
                   SetFactDuplicationCommand,"SetFactDuplicationCommand", "11");

   DefineFunction2("save-facts",    'b', PTIF SaveFactsCommand, "SaveFactsCommand", "1*wk");
   DefineFunction2("load-facts",    'b', PTIF LoadFactsCommand, "LoadFactsCommand", "11k");
   DefineFunction2("fact-index",    'l', PTIF FactIndexFunction,"FactIndexFunction", "11y");
#endif

#if (! RUN_TIME)
   AddFunctionParser("assert",AssertParse);
   FuncSeqOvlFlags("assert",CLIPS_FALSE,CLIPS_FALSE);
#endif
  }

/*************************************************/
/* AssertCommand: Implements the assert command. */
/*************************************************/
globle VOID AssertCommand(rv)
  DATA_OBJECT_PTR rv;
  {
   struct deftemplate *theDeftemplate;
   struct field *theField;
   DATA_OBJECT theValue;
   struct expr *theExpression;
   struct templateSlot *slotPtr;
   struct fact *newFact;
   int error = CLIPS_FALSE;
   int i;
   struct fact *theFact;

   /*===================================================*/
   /* Set the default return value to the symbol FALSE. */
   /*===================================================*/

   SetpType(rv,SYMBOL);
   SetpValue(rv,CLIPSFalseSymbol);

   /*====================================================*/
   /* Determine if a deftemplate fact is being asserted. */
   /*====================================================*/

   theExpression = GetFirstArgument();
   theDeftemplate = (struct deftemplate *) theExpression->value;
   
   /* Is the following code necessary? */
   
   if (theDeftemplate == NULL) 
#if (! RUN_TIME) && (! BLOAD_ONLY)
     { 
#if BLOAD || BLOAD_AND_BSAVE
      if (Bloaded()) return;
#endif

      theDeftemplate = CreateImpliedDeftemplate((SYMBOL_HN *) theExpression->value,CLIPS_TRUE); 
     }
#else
     { return; }
#endif
   
   /*=======================================*/
   /* Create the fact and store the name of */
   /* the deftemplate as the 1st field.     */
   /*=======================================*/
   
   if (theDeftemplate->implied == CLIPS_FALSE) 
     { 
      newFact = CreateFactBySize((int) theDeftemplate->numberOfSlots);
      slotPtr = theDeftemplate->slotList;
     }
   else 
     {
      newFact = CreateFactBySize(1);
      if (theExpression->nextArg == NULL)  
        { 
         newFact->theProposition.theFields[0].type = MULTIFIELD;
         newFact->theProposition.theFields[0].value = CreateMultifield2(0);
        }
      slotPtr = NULL;
     }
   
   newFact->whichDeftemplate = theDeftemplate;
   
   theField = newFact->theProposition.theFields;
   
   theExpression = theExpression->nextArg;
   i = 0;
   while (theExpression != NULL)
     {
      EvaluateExpression(theExpression,&theValue);   
      
      if ((slotPtr != NULL) ?
          (slotPtr->multislot == CLIPS_FALSE) && (theValue.type == MULTIFIELD) :
          CLIPS_FALSE)
        {
         MultiIntoSingleFieldSlotError(slotPtr,theDeftemplate);
         theValue.type = SYMBOL;
         theValue.value = CLIPSFalseSymbol;
         error = CLIPS_TRUE;
        }
        
      theField[i].type = theValue.type;
      theField[i].value = theValue.value;
      theExpression = theExpression->nextArg;
      if (slotPtr != NULL) slotPtr = slotPtr->next;
      i++;
     }

   if (error) 
     {
      ReturnFact(newFact);
      return;
     }
     
   /*================================*/
   /* Add the fact to the fact-list. */
   /*================================*/

   theFact = (struct fact *) Assert((VOID *) newFact);

   /*========================================*/
   /* The asserted fact is the return value. */
   /*========================================*/

   if (theFact != NULL)
     {
      SetpType(rv,FACT_ADDRESS);
      SetpValue(rv,(VOID *) theFact);
     }
     
   return;
  }
  
/****************************************/
/* RetractCommand: CLIPS access routine */
/*   for the retract command.           */
/****************************************/
globle VOID RetractCommand()
  {
   long int factIndex;
   int found_fact;
   struct fact *ptr;
   struct expr *theArgument;
   DATA_OBJECT theResult;
   int argNumber = 1;

   theArgument = GetFirstArgument();

   while (theArgument != NULL)
     {
      EvaluateExpression(theArgument,&theResult);
      if (theResult.type == INTEGER)
        {
         factIndex = ValueToLong(theResult.value);
         if (factIndex < 0)
           {            
            ExpectedTypeError1("retract",argNumber,"fact-address, fact-index, or the symbol *");
            return;
           }
         found_fact = CLIPS_FALSE;
         ptr = (struct fact *) GetNextFact(NULL);
         while (ptr != NULL)
           {
            if (ptr->factIndex == factIndex)
              {
               Retract((VOID *) ptr);
               ptr = NULL;
               found_fact = CLIPS_TRUE;
              }
            else
              { ptr = ptr->nextFact; }
           }
           
         if (found_fact == CLIPS_FALSE)
           {
            char tempBuffer[20];
            sprintf(tempBuffer,"f-%ld",factIndex);
            CantFindItemErrorMessage("fact",tempBuffer);
           }
        }
      else if (theResult.type == FACT_ADDRESS)
        { Retract(theResult.value); }
      else if ((theResult.type == SYMBOL) ?
               (strcmp(ValueToString(theResult.value),"*") == 0) : CLIPS_FALSE)
        {
         RemoveAllFacts();
         return;
        }
      else
        {
         ExpectedTypeError1("retract",argNumber,"fact-address, fact-index, or the symbol *");
         SetEvaluationError(TRUE);
        }

      argNumber++;
      theArgument = GetNextArgument(theArgument);
     }
  }

/***************************************************************************/
/* SetFactDuplicationCommand: Implements the set-fact-duplication command. */
/***************************************************************************/
globle int SetFactDuplicationCommand()
  {
   int oldValue;
   DATA_OBJECT theValue;

   oldValue = GetFactDuplication();

   if (ArgCountCheck("set-fact-duplication",EXACTLY,1) == -1)
     { return(oldValue); }

   RtnUnknown(1,&theValue);

   if ((theValue.value == CLIPSFalseSymbol) && (theValue.type == SYMBOL))
     { SetFactDuplication(CLIPS_FALSE); }
   else
     { SetFactDuplication(CLIPS_TRUE); }

   return(oldValue);
  }

/***************************************************************************/
/* GetFactDuplicationCommand: Implements the get-fact-duplication command. */
/***************************************************************************/
globle int GetFactDuplicationCommand()
  {
   int oldValue;

   oldValue = GetFactDuplication();

   if (ArgCountCheck("get-fact-duplication",EXACTLY,0) == -1)
     { return(oldValue); }

   return(oldValue);
  }

/*******************************************/
/* FactIndexFunction: CLIPS access routine */
/*   for the fact-index function.          */
/*******************************************/
globle long int FactIndexFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("fact-index",EXACTLY,1) == -1) return(-1L);

   RtnUnknown(1,&item);

   if (GetType(item) == FACT_ADDRESS)
     { 
      if (((struct fact *) GetValue(item))->garbage) return(-1L);
      else return (FactIndex(GetValue(item))); 
     }
   else
     {
      ExpectedTypeError1("fact-index",1,"fact-address");
      return(-1L);
     }
  }

#if DEBUGGING_FUNCTIONS

/***********************************************/
/* FactsCommand: Implements the facts command. */
/***********************************************/
globle VOID FactsCommand()
  {
   int argumentCount;
   long int start = UNSPECIFIED, end, max;
   struct defmodule *theModule;
   DATA_OBJECT theValue;
   int argOffset = 0;

   /*=========================================================*/
   /* Determine the number of arguments to the facts command. */
   /*=========================================================*/
   
   if ((argumentCount = ArgCountCheck("facts",NO_MORE_THAN,4)) == -1) return;
   
   /*=========================================*/
   /* See if a module or start index has been */
   /* specified as the first argument.        */
   /*=========================================*/
   
   theModule = ((struct defmodule *) GetCurrentModule());
   if (argumentCount > 0)
     {
      RtnUnknown(1,&theValue);
      if (theValue.type == SYMBOL)
        {
         theModule = (struct defmodule *) FindDefmodule(ValueToString(theValue.value));
         if ((theModule == NULL) && (strcmp(ValueToString(theValue.value),"*") != 0))
           {
            SetEvaluationError(CLIPS_TRUE);
            CantFindItemErrorMessage("defmodule",ValueToString(theValue.value));
            return;
           }
           
         if ((start = GetFactsArgument(2,argumentCount)) == INVALID) return;
           
         argOffset = 1;
        }
      else if (theValue.type == INTEGER)
        {   
         start = DOToLong(theValue);
         if (start < 0)
           {
            ExpectedTypeError1("facts",1,"symbol or positive number");
            SetHaltExecution(CLIPS_TRUE);
            SetEvaluationError(CLIPS_TRUE);
            return;
           }
         argOffset = 0;
        }
      else
        {  
         ExpectedTypeError1("facts",1,"symbol or positive number");
         SetHaltExecution(CLIPS_TRUE);
         SetEvaluationError(CLIPS_TRUE);
         return;
        }
     }

   /*==========================*/
   /* Get the other arguments. */
   /*==========================*/
   
   if ((end = GetFactsArgument(2 + argOffset,argumentCount)) == INVALID) return;
   if ((max = GetFactsArgument(3 + argOffset,argumentCount)) == INVALID) return;

   /*=================*/
   /* List the facts. */
   /*=================*/
   
   Facts(WDISPLAY,theModule,(long) start,(long) end,(long) max);
  }

/**************************************************/
/* Facts: C access routine for the facts command. */
/**************************************************/
globle VOID Facts(logicalName,vTheModule,start,end,max)
  char *logicalName;
  VOID *vTheModule;
  long start, end, max;
  {
   struct fact *factPtr;
   long count = 0;
   struct defmodule *oldModule, *theModule = (struct defmodule *) vTheModule;
   int allModules = CLIPS_FALSE;

   oldModule = ((struct defmodule *) GetCurrentModule());
   
   if (theModule == NULL) allModules = CLIPS_TRUE;
   else SetCurrentModule((VOID *) theModule);
   
   if (allModules) factPtr = (struct fact *) GetNextFact(NULL);
   else factPtr = (struct fact *) GetNextFactInScope(NULL);
   while (factPtr != NULL)
     {
      if (GetHaltExecution() == CLIPS_TRUE) 
        {
         SetCurrentModule((VOID *) oldModule);
         return;
        }
      
      if ((factPtr->factIndex > end) && (end != UNSPECIFIED))
        {
         PrintTally(logicalName,count,"fact","facts");
         SetCurrentModule((VOID *) oldModule);
         return;
        }

      if (max == 0)
        {
         PrintTally(logicalName,count,"fact","facts");
         SetCurrentModule((VOID *) oldModule);
         return;
        }

      if (factPtr->factIndex >= start)
        {
         PrintFactWithIdentifier(WDISPLAY,factPtr);
         PrintCLIPS(logicalName,"\n");
         count++;
         if (max > 0) max--;
        }

      if (allModules) factPtr = (struct fact *) GetNextFact(factPtr);
      else factPtr = (struct fact *) GetNextFactInScope(factPtr);
     }

   PrintTally(logicalName,count,"fact","facts");
   SetCurrentModule((VOID *) oldModule);
  }
  
/****************************************************************/
/* GetFactsArgument: Returns an argument for the facts command. */
/*  -1 indicates that no value was specified.                   */
/*  -2 indicates that the value specified is invalid.           */
/****************************************************************/
static long int GetFactsArgument(whichOne,argumentCount)
  int whichOne, argumentCount;
  {
   long int fint;
   DATA_OBJECT theValue;

   if (whichOne > argumentCount) return(UNSPECIFIED);

   if (ArgTypeCheck("facts",whichOne,INTEGER,&theValue) == CLIPS_FALSE) return(INVALID);
   fint = DOToLong(theValue);
   if (fint < 0)
     {
      ExpectedTypeError1("facts",whichOne,"positive number");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(INVALID);
     }
     
   return(fint);
  }

#endif

/****************************************************/
/* AssertStringFunction: CLIPS defined function for */
/*   converting a string to a fact.                 */
/*   Syntax: (assert-string <string>)               */
/****************************************************/
globle VOID AssertStringFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT argPtr;
   struct fact *theFact;
   
   /*===================================================*/
   /* Set the default return value to the symbol FALSE. */
   /*===================================================*/

   SetpType(returnValue,SYMBOL);
   SetpValue(returnValue,CLIPSFalseSymbol);

   /*=====================================================*/
   /* Check for the correct number and type of arguments. */
   /*=====================================================*/
   
   if (ArgCountCheck("assert-string",EXACTLY,1) == -1) return;

   if (ArgTypeCheck("assert-string",1,STRING,&argPtr) == CLIPS_FALSE)
     { return; }

   /*==========================================*/
   /* Call the driver routine for converting a */
   /* string to a fact and then assert it.     */
   /*==========================================*/
   
   theFact = (struct fact *) AssertString(DOToString(argPtr));
   if (theFact != NULL)
     {
      SetpType(returnValue,FACT_ADDRESS);
      SetpValue(returnValue,(VOID *) theFact);
     }
     
   return;
  }

/******************************************/
/* SaveFactsCommand: CLIPS access routine */
/*   for the save-facts command.          */
/******************************************/
globle int SaveFactsCommand()
  {
   char *fileName;
   int numArgs, saveCode = LOCAL_SAVE;
   char *argument;
   DATA_OBJECT theValue;
   struct expr *theList = NULL;

   if ((numArgs = ArgCountCheck("save-facts",AT_LEAST,1)) == -1) return(CLIPS_FALSE);
   
   if ((fileName = GetFileName("save-facts",1)) == NULL) return(CLIPS_FALSE);
   
   if (numArgs > 1)
     {
      if (ArgTypeCheck("save-facts",2,SYMBOL,&theValue) == CLIPS_FALSE) return(CLIPS_FALSE);

      argument = DOToString(theValue);

      if (strcmp(argument,"local") == 0)
        { saveCode = LOCAL_SAVE; }
      else if (strcmp(argument,"visible") == 0)
        { saveCode = VISIBLE_SAVE; }
      else
        {
         ExpectedTypeError1("save-facts",2,"symbol with value local or visible");
         return(CLIPS_FALSE);
        }
     }
   
   if (numArgs > 2) theList = GetFirstArgument()->nextArg->nextArg;

   if (SaveFacts(fileName,saveCode,theList) == CLIPS_FALSE)
     {
      return(CLIPS_FALSE);
     }

   return(CLIPS_TRUE);
  }

/******************************************/
/* LoadFactsCommand: CLIPS access routine */
/*   for the load-facts command.          */
/******************************************/
globle int LoadFactsCommand()
  {
   char *fileName;

   if (ArgCountCheck("load-facts",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if ((fileName = GetFileName("load-facts",1)) == NULL) return(CLIPS_FALSE);

   if (LoadFacts(fileName) == CLIPS_FALSE) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }

/***********************************************************/
/* SaveFacts: C access routine for the save-facts command. */
/***********************************************************/
globle BOOLEAN SaveFacts(fileName,saveCode,theList)
  char *fileName;
  int saveCode;
  struct expr *theList;
  {
   int tempValue1, tempValue2, tempValue3;
   struct fact *list;
   FILE *filePtr;
   struct defmodule *theModule;
   struct expr *tempList;
   DATA_OBJECT_PTR theDOArray;
   struct deftemplate *theDeftemplate;
   int count, i, tempCount, printFact;

   /*======================================================*/
   /* Open the file. Use either "fast save" or I/O Router. */
   /*======================================================*/

   if ((filePtr = fopen(fileName,"w")) == NULL)
     {
      OpenErrorMessage("save-facts",fileName);
      return(CLIPS_FALSE);
     }

   SetFastSave(filePtr);

   /*===========================================*/
   /* Set the print flags so that addresses and */
   /* strings are printed properly to the file. */
   /*===========================================*/

   tempValue1 = PreserveEscapedCharacters;
   PreserveEscapedCharacters = CLIPS_TRUE;
   tempValue2 = AddressesToStrings;
   AddressesToStrings = CLIPS_TRUE;
   tempValue3 = InstanceAddressesToNames;
   InstanceAddressesToNames = CLIPS_TRUE;

   /*===================================================*/
   /* Determine the list of specific facts to be saved. */
   /*===================================================*/
   
   if (theList != NULL)
     {   
      for (tempList = theList, count = 0;
           tempList != NULL;
           tempList = tempList->nextArg, count++);
      theDOArray = (DATA_OBJECT_PTR) gm3((long) sizeof(DATA_OBJECT) * count);
      
      for (tempList = theList, i = 0 ; 
           i < count; 
           tempList = tempList->nextArg, i++)
        {
         EvaluateExpression(tempList,&theDOArray[i]);
         
         if (EvaluationError)
           {
            rm3(theDOArray,(long) sizeof(DATA_OBJECT) * count);
            PreserveEscapedCharacters = tempValue1;
            AddressesToStrings = tempValue2;
            InstanceAddressesToNames = tempValue3;
            fclose(filePtr);
            SetFastSave(NULL);
            return(CLIPS_FALSE);
           }
           
         if (theDOArray[i].type != SYMBOL)
           { 
            ExpectedTypeError1("save-facts",3+i,"symbol");
            rm3(theDOArray,(long) sizeof(DATA_OBJECT) * count);
            PreserveEscapedCharacters = tempValue1;
            AddressesToStrings = tempValue2;
            InstanceAddressesToNames = tempValue3;
            fclose(filePtr);
            SetFastSave(NULL);
            return(CLIPS_FALSE);
           }
           
         if (saveCode == LOCAL_SAVE)
           {
            theDeftemplate = (struct deftemplate *)
                            FindDeftemplate(ValueToString(theDOArray[i].value));
            if (theDeftemplate == NULL)
              {
               ExpectedTypeError1("save-facts",3+i,"local deftemplate name");
               rm3(theDOArray,(long) sizeof(DATA_OBJECT) * count);
               PreserveEscapedCharacters = tempValue1;
               AddressesToStrings = tempValue2;
               InstanceAddressesToNames = tempValue3;
               fclose(filePtr);
               SetFastSave(NULL);
               return(CLIPS_FALSE);
              }
           }
         else if (saveCode == VISIBLE_SAVE)
           {
            theDeftemplate = (struct deftemplate *)
              FindImportedConstruct("deftemplate",NULL,
                                    ValueToString(theDOArray[i].value),
                                    &tempCount,CLIPS_TRUE,NULL);
            if (theDeftemplate == NULL)
              {
               ExpectedTypeError1("save-facts",3+i,"visible deftemplate name");
               rm3(theDOArray,(long) sizeof(DATA_OBJECT) * count);
               PreserveEscapedCharacters = tempValue1;
               AddressesToStrings = tempValue2;
               InstanceAddressesToNames = tempValue3;
               fclose(filePtr);
               SetFastSave(NULL);
               return(CLIPS_FALSE);
              }
           }
           
         theDOArray[i].type = DEFTEMPLATE_PTR;
         theDOArray[i].value = (VOID *) theDeftemplate;
        }
     }
     
   /*=================*/
   /* Save the facts. */
   /*=================*/
   
   switch (saveCode)
     {
      case VISIBLE_SAVE:
        list = (struct fact *) GetNextFactInScope(NULL);
        while (list != NULL)
          {
           if (theList == NULL) printFact = CLIPS_TRUE;
           else 
             {
              printFact = CLIPS_FALSE;
              for (i = 0; i < count; i++)
                {
                 if (theDOArray[i].value == (VOID *) list->whichDeftemplate)
                   {
                    printFact = CLIPS_TRUE;
                    break;
                   }
                }
             }
            
           if (printFact)
             { 
              PrintFact((char *) filePtr,list);
              PrintCLIPS((char *) filePtr,"\n");
             }
             
           list = (struct fact *) GetNextFactInScope(list);
          }
        break;
        
      case LOCAL_SAVE:
        theModule = ((struct defmodule *) GetCurrentModule());
        list = (struct fact *) GetNextFactInScope(NULL);
        while (list != NULL)
          {
           if (list->whichDeftemplate->header.whichModule->theModule != theModule)
             { printFact = CLIPS_FALSE; }
           else if (theList == NULL) printFact = CLIPS_TRUE;
           else 
             {
              printFact = CLIPS_FALSE;
              for (i = 0; i < count; i++)
                {
                 if (theDOArray[i].value == (VOID *) list->whichDeftemplate)
                   {
                    printFact = CLIPS_TRUE;
                    break;
                   }
                }
             }
            
           if (printFact)
             { 
              PrintFact((char *) filePtr,list);
              PrintCLIPS((char *) filePtr,"\n");
             }
           list = (struct fact *) GetNextFactInScope(list);
          }
        break;
     }
     
   /*==========================*/
   /* Restore the print flags. */
   /*==========================*/
   
   PreserveEscapedCharacters = tempValue1;
   AddressesToStrings = tempValue2;
   InstanceAddressesToNames = tempValue3;
   
   /*=================*/
   /* Close the file. */
   /*=================*/

   fclose(filePtr);
   SetFastSave(NULL);
   
   if (theList != NULL) rm3(theDOArray,(long) sizeof(DATA_OBJECT) * count);

   return(CLIPS_TRUE);
  }

/***********************************************************/
/* LoadFacts: C access routine for the load-facts command. */
/***********************************************************/
globle BOOLEAN LoadFacts(fileName)
  char *fileName;
  {
   FILE *filePtr;
   struct token theToken;
   struct expr *testPtr;
   DATA_OBJECT rv;

   /*======================================================*/
   /* Open the file. Use either "fast save" or I/O Router. */
   /*======================================================*/

   if ((filePtr = fopen(fileName,"r")) == NULL)
     {
      OpenErrorMessage("load-facts",fileName);
      return(CLIPS_FALSE);
     }

   SetFastLoad(filePtr);

   /*=================*/
   /* Load the facts. */
   /*=================*/

   theToken.type = LPAREN;
   while (theToken.type != STOP)
     {
      testPtr = StandardLoadFact((char *) filePtr,&theToken);
      if (testPtr == NULL) theToken.type = STOP;
      else EvaluateExpression(testPtr,&rv);
      ReturnExpression(testPtr);
     }

   /*=================*/
   /* Close the file. */
   /*=================*/

   SetFastLoad(NULL);
   fclose(filePtr);

   if (EvaluationError) return(CLIPS_FALSE);
   return(CLIPS_TRUE);
  }

/**************************************************************************/
/* StandardLoadFact: Loads a single fact from the specified logical name. */
/**************************************************************************/
static struct expr *StandardLoadFact(logicalName,theToken)
  char *logicalName;
  struct token *theToken;
  {
   int error = CLIPS_FALSE;
   struct expr *temp;

   GetToken(logicalName,theToken);
   if (theToken->type != LPAREN) return(NULL);

   temp = GenConstant(FCALL,FindFunction("assert"));
   temp->argList = GetRHSPattern(logicalName,theToken,&error,
                                  CLIPS_TRUE,CLIPS_FALSE,CLIPS_TRUE,RPAREN);

   if (error == CLIPS_TRUE)
     {
      PrintCLIPS(WERROR,"Function load-facts encountered an error\n");
      SetEvaluationError(CLIPS_TRUE);
      ReturnExpression(temp);
      return(NULL);
     }

   if (ExpressionContainsVariables(temp,CLIPS_TRUE))
     {
      ReturnExpression(temp);
      return(NULL);
     }

   return(temp);
  }

#if (! RUN_TIME)
/**************************************************************/
/* AssertParse: Purpose is to parse the assert statement. The */
/*   parse of the statement is the return value.              */
/*   Assert syntax:                                           */
/*     (assert (<field>+))                                    */
/**************************************************************/
static struct expr *AssertParse(top,logicalName)
  struct expr *top;
  char *logicalName;
  {
   int error;
   struct expr *rv;
   struct token theToken;

   ReturnExpression(top);
   SavePPBuffer(" ");
   IncrementIndentDepth(8);
   rv = BuildRHSAssert(logicalName,&theToken,&error,CLIPS_TRUE,CLIPS_TRUE,"assert command");
   DecrementIndentDepth(8);
   return(rv);
  }

#endif

#endif


