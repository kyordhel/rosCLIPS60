   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*              CONSTRUCT PARSER MODULE                */
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

#define _CSTRCPSR_SOURCE_

#include "setup.h"

#if (! RUN_TIME) && (! BLOAD_ONLY)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "router.h"
#include "watch.h"
#include "constrct.h" 
#include "prcdrpsr.h"
#include "exprnpsr.h"
#include "modulutl.h"
#include "modulpsr.h"

#include "cstrcpsr.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     FindConstructBeginning(char *,struct token *,int,int *);
#else
   static int                     FindConstructBeginning();
#endif

/************************************************/
/* Load: C access routine for the load command. */
/************************************************/
globle int Load(fileName)
  char *fileName;
  {
   FILE *theFile;
   int noErrorsDetected;

   /*=======================================*/
   /* Open the file specified by file name. */
   /*=======================================*/

   if ((theFile = fopen(fileName,"r")) == NULL) return(CLIPS_FALSE);
     
   /*=========================*/
   /* Read in the constructs. */
   /*=========================*/
   
   SetFastLoad(theFile);
   noErrorsDetected = LoadConstructsFromLogicalName((char *) theFile);
   SetFastLoad(NULL);

   /*=================*/
   /* Close the file. */
   /*=================*/

   fclose(theFile);
   
   /*========================================*/
   /* If no errors occurred during the load, */
   /* return TRUE, otherwise return FALSE.   */
   /*========================================*/
   
   if (noErrorsDetected) return(1);
   
   return(-1);
  }

/******************************************************************/
/* LoadConstructsFromLogicalName:  Loads a set of constructs into */
/*   the current CLIPS environment from a specified logical name. */
/******************************************************************/
globle int LoadConstructsFromLogicalName(readSource)
  char *readSource;
  {
   int constructFlag;
   struct token theToken;
   int noErrors = CLIPS_TRUE;
   int foundConstruct;

   /*==================================================*/
   /* Parse the file until the end of file is reached. */
   /*==================================================*/

   if (CurrentEvaluationDepth == 0) SetHaltExecution(CLIPS_FALSE);
   SetEvaluationError(CLIPS_FALSE);
   
   GetToken(readSource,&theToken);
   foundConstruct = FindConstructBeginning(readSource,&theToken,CLIPS_FALSE,&noErrors);

   while ((foundConstruct == CLIPS_TRUE) && (GetHaltExecution() == CLIPS_FALSE))
     {
      FlushPPBuffer();
      constructFlag = ParseConstruct(ValueToString(theToken.value),readSource);
      if (constructFlag == 1)
        {
         PrintCLIPS(WERROR,"\nERROR:\n");
         PrintInChunks(WERROR,GetPPBuffer());
         PrintCLIPS(WERROR,"\n");
         noErrors = CLIPS_FALSE;
         GetToken(readSource,&theToken);
         foundConstruct = FindConstructBeginning(readSource,&theToken,CLIPS_TRUE,&noErrors); 
        }
      else
        { 
         GetToken(readSource,&theToken);
         foundConstruct = FindConstructBeginning(readSource,&theToken,CLIPS_FALSE,&noErrors);
        }
     }

   /*========================================================*/
   /* Print a carriage return if a single character is being */
   /* printed to indicate constructs are being processed.    */
   /*========================================================*/

#if DEBUGGING_FUNCTIONS
   if ((GetWatchItem("compilations") != CLIPS_TRUE) && GetPrintWhileLoading())
#else
   if (GetPrintWhileLoading())
#endif
     { PrintCLIPS(WDIALOG,"\n"); }
     
   /*=============================================================*/
   /* Once the load is complete, destroy the pretty print buffer. */
   /* This frees up any memory that was used to create the pretty */
   /* print forms for constructs during parsing. Thus calls to    */
   /* the mem-used function will accurately reflect the amount of */
   /* memory being used after a load command.                     */
   /*=============================================================*/
                  
   DestroyPPBuffer();
   
   /*==========================================================*/
   /* Return a boolean flag which indicates whether any errors */
   /* were encountered while loading the constructs.           */
   /*==========================================================*/
   
   return(noErrors);
  }

/********************************************************************/
/* FindConstructBeginning: Searches for a left parenthesis followed */
/*   by the name of a valid construct. Used by the load command to  */
/*   find the next construct to be parsed. Returns TRUE is the      */
/*   beginning of a construct was found, otherwise FALSE.           */
/********************************************************************/
static int FindConstructBeginning(readSource,theToken,errorCorrection,noErrors)
  char *readSource;
  struct token *theToken;
  int errorCorrection;
  int *noErrors;
  {
   int leftParenthesisFound = CLIPS_FALSE;
   int firstAttempt = CLIPS_TRUE;
   
   /*===================================================*/
   /* Process tokens until the beginning of a construct */
   /* is found or there are no more tokens.             */
   /*===================================================*/
   
   while (theToken->type != STOP)
     {
      /*=====================================================*/
      /* Constructs begin with a left parenthesis. Make note */
      /* that the opening parenthesis has been found.        */
      /*=====================================================*/
      
      if (theToken->type == LPAREN) 
        { leftParenthesisFound = CLIPS_TRUE; }
        
      /*=================================================================*/
      /* The name of the construct follows the opening left parenthesis. */
      /* If it is the name of a valid construct, then return TRUE.       */
      /* Otherwise, reset the flags to look for the beginning of a       */
      /* construct. If error correction is being performed (i.e. the     */
      /* last construct parsed had an error in it), then don't bother to */
      /* print an error message, otherwise, print an error message.      */
      /*=================================================================*/
      
      else if ((theToken->type == SYMBOL) && (leftParenthesisFound == CLIPS_TRUE))
        {
         if (FindConstruct(ValueToString(theToken->value)) != NULL) return(CLIPS_TRUE);
         
         if (firstAttempt && (! errorCorrection))
           {
            errorCorrection = CLIPS_TRUE;
            *noErrors = CLIPS_FALSE;
            PrintErrorID("CSTRCPSR",1,CLIPS_TRUE);
            PrintCLIPS(WERROR,"Expected the beginning of a construct.\n");
           }
           
         firstAttempt = CLIPS_FALSE;
         leftParenthesisFound = CLIPS_FALSE;
        }
        
      /*====================================================================*/
      /* Any token encountered other than a left parenthesis or a construct */
      /* name following a left parenthesis is illegal. Again, if error      */
      /* correction is in progress, no error message is printed, otherwise, */
      /*  an error message is printed.                                      */
      /*====================================================================*/
      
      else
        { 
         if (firstAttempt && (! errorCorrection))
           {
            errorCorrection = CLIPS_TRUE;
            *noErrors = CLIPS_FALSE;
            PrintErrorID("CSTRCPSR",1,CLIPS_TRUE);
            PrintCLIPS(WERROR,"Expected the beginning of a construct.\n");
           }
           
         firstAttempt = CLIPS_FALSE;
         leftParenthesisFound = CLIPS_FALSE;
        }
        
      /*============================================*/
      /* Move on to the next token to be processed. */
      /*============================================*/
      
      GetToken(readSource,theToken);
     }
     
   /*===================================================================*/
   /* Couldn't find the beginning of a construct, so FALSE is returned. */
   /*===================================================================*/
     
   return(CLIPS_FALSE);
  }

/***********************************************************/
/* ParseConstruct: Parses a construct. Returns an integer. */
/*   -1 if the construct name has no parsing function, 0   */
/*   if the construct was parsed successfully, and 1 if    */
/*   the construct was parsed unsuccessfully.              */
/***********************************************************/
globle int ParseConstruct(name,logicalName)
  char *name, *logicalName;
  {
   struct construct *currentPtr;
   int rv, ov;

   currentPtr = FindConstruct(name);
   if (currentPtr == NULL) return(-1);
   
   ov = GetHaltExecution();

   SetEvaluationError(CLIPS_FALSE);
   SetHaltExecution(CLIPS_FALSE);
   ClearParsedBindNames();
   
   PushRtnBrkContexts();
   ReturnContext = CLIPS_FALSE;
   BreakContext = CLIPS_FALSE;
   rv = (*currentPtr->parseFunction)(logicalName);
   PopRtnBrkContexts();
   
   ClearParsedBindNames();
   SetPPBufferStatus(OFF);
   SetHaltExecution(ov);
   return(rv);
  }
  
/******************************************************************/
/* GetConstructNameAndComment:  Get the name and comment field of */
/*   a construct. Returns name of the construct if no errors are  */
/*   detected, otherwise returns NULL.                            */
/******************************************************************/
globle SYMBOL_HN *GetConstructNameAndComment(readSource,inputToken,constructName,
                                             findFunction,deleteFunction,
                                             constructSymbol,fullMessageCR,getComment,
                                             moduleNameAllowed)
  char *readSource;
  struct token *inputToken;
  char *constructName;
#if ANSI_COMPILER
  VOID *(*findFunction)(char *);
  int (*deleteFunction)(VOID *);
#else
  VOID *(*findFunction)();
  int (*deleteFunction)();
#endif
  char *constructSymbol;
  int fullMessageCR, getComment;
  int moduleNameAllowed;
  {
   SYMBOL_HN *name, *moduleName;
   int redefining = CLIPS_FALSE;
   VOID *theConstruct;
   int separatorPosition;
   struct defmodule *theModule;

   /*============================================*/
   /* Next token should be the name of the rule. */
   /*============================================*/

   GetToken(readSource,inputToken);
   if (inputToken->type != SYMBOL)
     {
      PrintErrorID("CSTRCPSR",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Missing name for ");
      PrintCLIPS(WERROR,constructName);
      PrintCLIPS(WERROR," construct\n");
      return(NULL);
     }

   name = (SYMBOL_HN *) inputToken->value;

   /*===============================*/
   /* Determine the current module. */
   /*===============================*/
   
   separatorPosition = FindModuleSeparator(ValueToString(name));   
   if (separatorPosition)
     {
      if (moduleNameAllowed == CLIPS_FALSE)
        {
         SyntaxErrorMessage("module specifier");
         return(NULL);
        }
        
      moduleName = ExtractModuleName(separatorPosition,ValueToString(name));
      if (moduleName == NULL)
        {
         SyntaxErrorMessage("construct name");
         return(NULL);
        }
        
      theModule = (struct defmodule *) FindDefmodule(ValueToString(moduleName));
      if (theModule == NULL)
        {
         CantFindItemErrorMessage("defmodule",ValueToString(moduleName));
         return(NULL);
        }
        
      SetCurrentModule((VOID *) theModule);
      name = ExtractConstructName(separatorPosition,ValueToString(name));
     }
     
   /*=====================================================*/
   /* If the module was not specified, record the current */
   /* module name as part of the pretty-print form.       */
   /*=====================================================*/
   
   else 
     {
      theModule = ((struct defmodule *) GetCurrentModule());
      if (moduleNameAllowed)
        {
         PPBackup();
         SavePPBuffer(GetDefmoduleName(theModule));
         SavePPBuffer("::");
         SavePPBuffer(ValueToString(name));
        }
     }

   /*==================================================================*/
   /* Check for import/export conflicts from the construct definition. */
   /*==================================================================*/

#if DEFMODULE_CONSTRUCT
   if (FindImportExportConflict(constructName,theModule,ValueToString(name)))
     {
      ImportExportConflictMessage(constructName,ValueToString(name),NULL,NULL);
      return(NULL);
     }
#endif
     
   /*==============================================================*/
   /* Remove the construct if it is already in the knowledge base. */
   /*==============================================================*/

   if (findFunction != NULL)
     {
      theConstruct = (*findFunction)(ValueToString(name));
      if (theConstruct != NULL)
        {
         redefining = CLIPS_TRUE;
         if (deleteFunction != NULL)
           {
            if ((*deleteFunction)(theConstruct) == CLIPS_FALSE)
              {
               PrintErrorID("CSTRCPSR",4,CLIPS_TRUE);
               PrintCLIPS(WERROR,"Cannot redefine ");
               PrintCLIPS(WERROR,constructName);
               PrintCLIPS(WERROR," ");
               PrintCLIPS(WERROR,ValueToString(name));
               PrintCLIPS(WERROR," while it is in use.\n");
               return(NULL);
              }
           }
        }
     }

   /*===============================================================*/
   /* If rules being watched, indicate that rule is being compiled. */
   /*===============================================================*/

#if DEBUGGING_FUNCTIONS
   if ((GetWatchItem("compilations") == CLIPS_TRUE) && GetPrintWhileLoading())
     {
      if (redefining) PrintCLIPS(WDIALOG,"Redefining ");
      else PrintCLIPS(WDIALOG,"Defining ");

      PrintCLIPS(WDIALOG,constructName);
      PrintCLIPS(WDIALOG,": ");
      PrintCLIPS(WDIALOG,ValueToString(name));

      if (fullMessageCR) PrintCLIPS(WDIALOG,"\n");
      else PrintCLIPS(WDIALOG," ");
     }
   else 
#endif
     { if (GetPrintWhileLoading()) PrintCLIPS(WDIALOG,constructSymbol); }

   /*===========================*/
   /* Get comment if it exists. */
   /*===========================*/

   GetToken(readSource,inputToken);
   if ((inputToken->type == STRING) && getComment)
     {
      PPBackup();
      SavePPBuffer(" ");
      SavePPBuffer(inputToken->printForm);
      GetToken(readSource,inputToken);
      if (inputToken->type != RPAREN)
        {
         PPBackup();
         SavePPBuffer("\n   ");
         SavePPBuffer(inputToken->printForm);
        }
     }
   else if (inputToken->type != RPAREN)
     {
      PPBackup();
      SavePPBuffer("\n   ");
      SavePPBuffer(inputToken->printForm);
     }

   return(name);
  }
  
/****************************************/
/* RemoveConstructFromModule: Removes a */
/*    construct from its module's list  */
/****************************************/
globle VOID RemoveConstructFromModule(theConstruct)
  struct constructHeader *theConstruct;
  {
   struct constructHeader *lastConstruct,*currentConstruct;
   
   lastConstruct = NULL;
   currentConstruct = theConstruct->whichModule->firstItem;
   while (currentConstruct != theConstruct)
     {
      lastConstruct = currentConstruct;
      currentConstruct = currentConstruct->next;  
     }
   
   if (currentConstruct == NULL)
     {
      CLIPSSystemError("CSTRCPSR",1);
      ExitCLIPS(2);
     }
       
   if (lastConstruct == NULL)
     theConstruct->whichModule->firstItem = theConstruct->next;
   else
     lastConstruct->next = theConstruct->next;
     
   if (theConstruct == theConstruct->whichModule->lastItem)
     theConstruct->whichModule->lastItem = lastConstruct;
  }
  
/****************************************/
/* ImportExportConflictMessage:   */
/****************************************/
globle VOID ImportExportConflictMessage(constructName,itemName,causedByConstruct,causedByName)
  char *constructName, *itemName;
  char *causedByConstruct, *causedByName;
  {
   PrintErrorID("CSTRCPSR",3,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Cannot define ");
   PrintCLIPS(WERROR,constructName);
   PrintCLIPS(WERROR," ");
   PrintCLIPS(WERROR,itemName);
   PrintCLIPS(WERROR," because of an import/export conflict");
   
   if (causedByConstruct == NULL) PrintCLIPS(WERROR,".\n");
   else
     {
      PrintCLIPS(WERROR," caused by the ");
      PrintCLIPS(WERROR,causedByConstruct);
      PrintCLIPS(WERROR," ");
      PrintCLIPS(WERROR,causedByName);
      PrintCLIPS(WERROR,".\n");
     }
  }
  
#endif



