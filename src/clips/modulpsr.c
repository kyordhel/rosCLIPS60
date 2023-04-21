   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              DEFMODULE PARSER MODULE                */
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

#define _MODULPSR_SOURCE_

#include "setup.h"

#if DEFMODULE_CONSTRUCT && (! RUN_TIME) && (! BLOAD_ONLY)

#include <stdio.h>
#include <string.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "constant.h"
#include "router.h"
#include "extnfunc.h"
#include "argacces.h"
#include "cstrcpsr.h"
#include "constrct.h"
#include "modulutl.h"
#include "utility.h"

#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#include "modulpsr.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct portConstructItem    *ListOfPortConstructItems = NULL;
   static long                         NumberOfDefmodules;
   static struct callFunctionItem     *AfterModuleDefinedFunctions = NULL;
   
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                        ParsePortSpecifications(char *,struct token *,
                                                             struct defmodule *);
   static int                        ParseImportSpec(char *,struct token *,
                                                     struct defmodule *);
   static int                        ParseExportSpec(char *,struct token *,
                                                     struct defmodule *,
                                                     struct defmodule *);
   static BOOLEAN                    DeleteDefmodule(VOID *);
   static int                        FindMultiImportConflict(struct defmodule *);
   static VOID                       NotExportedErrorMessage(char *,char *,char *);
#else
   static int                        ParsePortSpecifications();
   static int                        ParseImportSpec();
   static int                        ParseExportSpec();
   static BOOLEAN                    DeleteDefmodule();
   static int                        FindMultiImportConflict();
   static VOID                       NotExportedErrorMessage();
#endif

/************************************************************/
/* GetNumberOfDefmodules:                                   */
/************************************************************/
globle long GetNumberOfDefmodules()
  {
   return(NumberOfDefmodules);
  }
  
/************************************************************/
/* SetNumberOfDefmodules:                                   */
/************************************************************/
globle VOID SetNumberOfDefmodules(value)
  long value;
  {
   NumberOfDefmodules = value;
  }
  
/**********************************************************/
/* AddAfterModuleChangeFunction:                          */
/**********************************************************/
globle VOID AddAfterModuleDefinedFunction(name,func,priority)
  char *name;
  VOID (*func)(VOID_ARG);
  int priority;
  {
   AfterModuleDefinedFunctions = 
     AddFunctionToCallList(name,priority,func,AfterModuleDefinedFunctions);
  }    

/************************************************************/
/* AddPortConstructItem:                                    */
/************************************************************/
globle VOID AddPortConstructItem(theName,theType)
  char *theName;
  int theType;
  {   
   struct portConstructItem *newItem;
   
   newItem = get_struct(portConstructItem);
   newItem->constructName = theName;
   newItem->typeExpected = theType;
   newItem->next = ListOfPortConstructItems;
   ListOfPortConstructItems = newItem;
  }

/*************************************************************/
/* ParseDefmodule: Coordinates all actions necessary for the */
/*   addition of a defmodule construct into the current      */
/*   environment. Called when parsing a construct after the  */
/*   defmodule keyword has been found.                       */
/*************************************************************/
globle int ParseDefmodule(readSource)
  char *readSource;
  {
   SYMBOL_HN *defmoduleName;
   struct defmodule *newDefmodule;
   struct token inputToken;
   int i;
   struct moduleItem *theItem;
   struct portItem *portSpecs, *nextSpec;
   struct defmoduleItemHeader *theHeader;
   struct callFunctionItem *defineFunctions;
   struct defmodule *redefiningMainModule = NULL;
   int parseError;

   SetPPBufferStatus(ON);

   FlushPPBuffer();
   SetIndentDepth(3);
   SavePPBuffer("(defmodule ");

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE)
     {
      CannotLoadWithBloadMessage("defmodule");
      return(CLIPS_TRUE);
     }
#endif

   /*=====================================================*/
   /* Parse the name and comment fields of the defmodule. */
   /* Remove the defmodule if it already exists.          */
   /*=====================================================*/

   defmoduleName = GetConstructNameAndComment(readSource,&inputToken,"defmodule",
                                              FindDefmodule,DeleteDefmodule,"+",
                                              CLIPS_TRUE,CLIPS_TRUE,CLIPS_FALSE);
   if (defmoduleName == NULL) { return(CLIPS_TRUE); }
   
   if (strcmp(ValueToString(defmoduleName),"MAIN") == 0)
     { redefiningMainModule = (struct defmodule *) FindDefmodule("MAIN"); }
     
   /*===============================*/
   /* Finish parsing the defmodule. */
   /*===============================*/

   if (redefiningMainModule == NULL) 
     {
      newDefmodule = get_struct(defmodule);
      newDefmodule->name = defmoduleName;
      newDefmodule->next = NULL;
     }
   else newDefmodule = redefiningMainModule; 
   
   newDefmodule->importList = NULL;
   newDefmodule->exportList = NULL;
      
   parseError = ParsePortSpecifications(readSource,&inputToken,newDefmodule);
   
   if (! parseError) parseError = FindMultiImportConflict(newDefmodule);
   
   if (parseError)
     {
      while (newDefmodule->importList != NULL)
        { 
         nextSpec = newDefmodule->importList->next;
         rtn_struct(portItem,newDefmodule->importList);
         newDefmodule->importList = nextSpec;
        }
   
      while (newDefmodule->exportList != NULL)
        { 
         nextSpec = newDefmodule->exportList->next;
         rtn_struct(portItem,newDefmodule->exportList);
         newDefmodule->exportList = nextSpec;
        }  

      if (redefiningMainModule == NULL) rtn_struct(defmodule,newDefmodule);
      
      return(CLIPS_TRUE);
     }

   SavePPBuffer("\n");
   
   /*=================================*/
   /* Create the defmodule structure. */
   /*=================================*/
   
   if (redefiningMainModule == NULL)
     { IncrementSymbolCount(newDefmodule->name); }
   else
     {
      if ((newDefmodule->importList != NULL) ||
          (newDefmodule->exportList != NULL))
        { MainModuleRedefinable = CLIPS_FALSE; }
     }
   
   for (portSpecs = newDefmodule->importList; portSpecs != NULL; portSpecs = portSpecs->next)
     { 
      if (portSpecs->moduleName != NULL) IncrementSymbolCount(portSpecs->moduleName);
      if (portSpecs->constructType != NULL) IncrementSymbolCount(portSpecs->constructType);
      if (portSpecs->constructName != NULL) IncrementSymbolCount(portSpecs->constructName);
     }
   
   for (portSpecs = newDefmodule->exportList; portSpecs != NULL; portSpecs = portSpecs->next)
     { 
      if (portSpecs->moduleName != NULL) IncrementSymbolCount(portSpecs->moduleName);
      if (portSpecs->constructType != NULL) IncrementSymbolCount(portSpecs->constructType);
      if (portSpecs->constructName != NULL) IncrementSymbolCount(portSpecs->constructName);
     }   

   if (redefiningMainModule != NULL) { /* Do nothing */ }
   else if (NumberOfModuleItems == 0) newDefmodule->itemsArray = NULL;
   else 
     {
      newDefmodule->itemsArray = (struct defmoduleItemHeader **) gm2((int) sizeof(VOID *) * NumberOfModuleItems);
      for (i = 0, theItem = ListOfModuleItems;
           i < NumberOfModuleItems, theItem != NULL; 
           i++, theItem = theItem->next)
        { 
         if (theItem->allocateFunction == NULL)
           { newDefmodule->itemsArray[i] = NULL; }
         else
           {
            newDefmodule->itemsArray[i] = (struct defmoduleItemHeader *)
                                          (*theItem->allocateFunction)();
            theHeader = (struct defmoduleItemHeader *) newDefmodule->itemsArray[i];
            theHeader->theModule = newDefmodule;
            theHeader->firstItem = NULL;
            theHeader->lastItem = NULL;
           }
        }
     }
   
   /*=======================================*/
   /* Save the pretty print representation. */
   /*=======================================*/
   
   if (GetConserveMemory() == CLIPS_TRUE)
     { newDefmodule->ppForm = NULL; }
   else
     { newDefmodule->ppForm = CopyPPBuffer(); }

   /*==============================================*/
   /* Add the defmodule to the list of defmodules. */
   /*==============================================*/
   
   if (redefiningMainModule == NULL)
     {
      if (LastDefmodule == NULL) ListOfDefmodules = newDefmodule;
      else LastDefmodule->next = newDefmodule;
      LastDefmodule = newDefmodule;
      newDefmodule->bsaveID = NumberOfDefmodules++;
     }

   SetCurrentModule((VOID *) newDefmodule);

   /*=========================================*/
   /* Call any functions required by other    */
   /* constructs when a new module is defined */
   /*=========================================*/
   
   defineFunctions = AfterModuleDefinedFunctions;
   while (defineFunctions != NULL)
     {
      (* (VOID (*)(VOID_ARG)) defineFunctions->func)();
      defineFunctions = defineFunctions->next;
     }
           
   /*===============================================*/
   /* Defmodule successfully parsed with no errors. */
   /*===============================================*/

   return(CLIPS_FALSE);
  }

/*************************************************************/
/* DeleteDefmodule: Used by the parsing routine to determine */
/*   if a module can be redefined. Only the MAIN module can  */
/*   be redefined (and it can only be redefined once).       */
/*************************************************************/
static BOOLEAN DeleteDefmodule(theConstruct)
  VOID *theConstruct;
  { 
   if (strcmp(GetDefmoduleName(theConstruct),"MAIN") == 0)
     { return(MainModuleRedefinable); }
     
   return(CLIPS_FALSE);
  }
  
/*********************************************************/
/* ParsePortSpecifications: Parses the import and export */
/*   specifications found in a defmodule construct.      */
/*********************************************************/
static int ParsePortSpecifications(readSource,theToken,theDefmodule)
  char *readSource;
  struct token *theToken;
  struct defmodule *theDefmodule;
  {
   int error;
   
   theDefmodule->importList = NULL;
   theDefmodule->exportList = NULL;
   
   while (theToken->type != RPAREN)
     {  
      /*========================================*/
      /* Look for the opening left parenthesis. */
      /*========================================*/
      
      if (theToken->type != LPAREN)
        {
         SyntaxErrorMessage("defmodule");
         return(CLIPS_TRUE);
        }
          
      /*=====================================*/
      /* Look for the import/export keyword. */
      /*=====================================*/
      
      GetToken(readSource,theToken);
      
      if (theToken->type != SYMBOL)
        {
         SyntaxErrorMessage("defmodule");
         return(CLIPS_TRUE);
        }
        
      if (strcmp(ValueToString(theToken->value),"import") == 0)
        { 
         error = ParseImportSpec(readSource,theToken,theDefmodule);
        }       
      else if (strcmp(ValueToString(theToken->value),"export") == 0)
        { 
         error = ParseExportSpec(readSource,theToken,theDefmodule,NULL);
        }
      else
        {
         SyntaxErrorMessage("defmodule");
         return(CLIPS_TRUE);
        }
      
      if (error) return(CLIPS_TRUE);
      
      /*============================================*/
      /* Begin parsing the next port specification. */
      /*============================================*/
      
      PPCRAndIndent();
      GetToken(readSource,theToken);
      
      if (theToken->type == RPAREN)
        {
         PPBackup();
         PPBackup();
         SavePPBuffer(")");
        }
     }
     
   return(CLIPS_FALSE);
  }
  
/**********************************************************/
/* ParseImportSpec: Parses import specifications found in */
/*   a defmodule construct.                               */
/*                                                        */
/* <import-spec> ::= (import <module-name> <port-item>)   */
/*                                                        */
/* <port-item>   ::= ?ALL |                               */
/*                   ?NONE |                              */
/*                   <construct-name> ?ALL |              */
/*                   <construct-name> ?NONE |             */
/*                   <construct-name> <names>*            */
/**********************************************************/
static int ParseImportSpec(readSource,theToken,newModule)
  char *readSource;
  struct token *theToken;
  struct defmodule *newModule;
  {
   struct defmodule *theModule;
   struct portItem *thePort, *oldImportSpec;
   
   SavePPBuffer(" ");
      
   /*===========================*/
   /* Look for the module name. */
   /*===========================*/
      
   GetToken(readSource,theToken);
      
   if (theToken->type != SYMBOL)
     {
      SyntaxErrorMessage("defmodule import specification");
      return(CLIPS_TRUE);
     }
   
   /*=====================================*/
   /* Verify the existence of the module. */
   /*=====================================*/
   
   if ((theModule = (struct defmodule *)
                    FindDefmodule(ValueToString(theToken->value))) == NULL)
     {
      CantFindItemErrorMessage("defmodule",ValueToString(theToken->value));
      return(CLIPS_TRUE);
     }
     
   /*========================================================*/
   /* If the specified module doesn't export any constructs, */
   /* then the import specification is meaningless.          */
   /*========================================================*/
   
   if (theModule->exportList == NULL)
     {
      NotExportedErrorMessage(GetDefmoduleName(theModule),NULL,NULL);
      return(CLIPS_TRUE);
     }
     
   /*=========================================================*/
   /* Parse the remaining portion of the import specification */
   /* and return if an error occurs.                          */
   /*=========================================================*/
   
   oldImportSpec = newModule->importList;
   if (ParseExportSpec(readSource,theToken,newModule,theModule)) return(CLIPS_TRUE);
   
   /*=============================================================*/
   /* If the ?NONE keyword was used with the import spec, then no */
   /* constructs were actually imported and the import spec does  */
   /* not need to be checked for conflicts.                       */
   /*=============================================================*/
   
   if (newModule->importList == oldImportSpec) return(CLIPS_FALSE);

   /*==================================================*/
   /* Determine if any constructs of the type imported */
   /* are exported from the specified module.          */
   /*==================================================*/
   
   if (newModule->importList->constructType != NULL)
     {
      int found;

      found = CLIPS_FALSE;
      for (thePort = theModule->exportList;
           (thePort != NULL) && (! found);
           thePort = thePort->next)
        {
         if (thePort->constructType == NULL) found = CLIPS_TRUE;
         else if (thePort->constructType == newModule->importList->constructType)
           {
            if (newModule->importList->constructName == NULL) found = CLIPS_TRUE;
            else if (thePort->constructName == NULL) found = CLIPS_TRUE;
            else if (thePort->constructName == newModule->importList->constructName)
              { found = CLIPS_TRUE; }
           }
        }
           
      if (! found)
        {
         if (newModule->importList->constructName == NULL)
           { 
            NotExportedErrorMessage(GetDefmoduleName(theModule),
                                    ValueToString(newModule->importList->constructType),
                                    NULL);
           }
         else 
           { 
            NotExportedErrorMessage(GetDefmoduleName(theModule),
                                    ValueToString(newModule->importList->constructType),
                                    ValueToString(newModule->importList->constructName));
           }
         return(CLIPS_TRUE);
        }
     }
     
   /*======================================================*/
   /* Make sure all imported items are also exported. Each */
   /* named construct in the import list is checked to see */
   /* if it can be imported from the MAIN module. Since    */
   /* the MAIN module imports from all modules and exports */
   /* to all modules, the MAIN module can be used to       */
   /* find all exports from a particular module.           */
   /*======================================================*/
   
   SaveCurrentModule();
   SetCurrentModule((VOID *) newModule);
   
   for (thePort = newModule->importList;
        thePort != NULL;
        thePort = thePort->next)
     {
      int count;
      
      if ((thePort->constructType != NULL) && (thePort->constructName != NULL))
        {
        /*
         theModule = (struct defmodule *) FindDefmodule(ValueToString(thePort->moduleName));
         if (FindImportedConstruct(ValueToString(thePort->constructType),theModule,
                                   ValueToString(thePort->constructName),&count,
                                   CLIPS_TRUE,CLIPS_FALSE) == NULL)
           {
            NotExportedErrorMessage(GetDefmoduleName(theModule),
                                    ValueToString(thePort->constructType),
                                    ValueToString(thePort->constructName));
            RestoreCurrentModule();
            return(CLIPS_TRUE);
           }
         */
         
         theModule = (struct defmodule *) 
                     FindDefmodule(ValueToString(thePort->moduleName));
         SetCurrentModule(theModule);
         if (FindImportedConstruct(ValueToString(thePort->constructType),NULL,
                                   ValueToString(thePort->constructName),&count,
                                   CLIPS_TRUE,CLIPS_FALSE) == NULL)
           {
            NotExportedErrorMessage(GetDefmoduleName(theModule),
                                    ValueToString(thePort->constructType),
                                    ValueToString(thePort->constructName));
            RestoreCurrentModule();
            return(CLIPS_TRUE);
           }

        }
     }
     
   RestoreCurrentModule();
   
   /*===============================================*/
   /* The import list has been successfully parsed. */
   /*===============================================*/
   
   return(CLIPS_FALSE);
  }  

/**********************************************************/
/* ParseExportSpec: Parses export specifications found in */
/*   a defmodule construct.                               */
/*                                                        */
/* <export-spec> ::= (export <port-item>)                 */
/*                                                        */
/* <port-item>   ::= ?ALL |                               */
/*                   ?NONE |                              */
/*                   <construct-name> ?ALL |              */
/*                   <construct-name> ?NONE |             */
/*                   <construct-name> <names>+            */
/**********************************************************/
static int ParseExportSpec(readSource,theToken,newModule,importModule)
  char *readSource;
  struct token *theToken;
  struct defmodule *newModule;
  struct defmodule *importModule;
  {
   struct portItem *newPort;
   SYMBOL_HN *theConstruct, *moduleName;
   struct portConstructItem *thePortConstruct; 
   char *errorMessage; 
      
   if (importModule != NULL) 
     {
      errorMessage = "defmodule import specification";
      moduleName = importModule->name;
     }
   else 
     {
      errorMessage = "defmodule export specification";
      moduleName = NULL;
     }
   
   /*=================================================*/
   /* If the next token is the special variable ?ALL, */
   /* then all exported constructs from the specified */
   /* module will be imported. If the next token is   */
   /* the special variable ?NONE, then no constructs  */
   /* from the specified module will be imported.     */
   /*=================================================*/
   
   SavePPBuffer(" ");
   GetToken(readSource,theToken);
        
   if (theToken->type == SF_VARIABLE)
     {
      if (strcmp(ValueToString(theToken->value),"ALL") == 0)
        {
         newPort = (struct portItem *) get_struct(portItem);
         newPort->moduleName = moduleName;
         newPort->constructType = NULL;
         newPort->constructName = NULL;
         newPort->next = NULL;
        }
      else if (strcmp(ValueToString(theToken->value),"NONE") == 0)
        { newPort = NULL; }
      else
        {
         SyntaxErrorMessage(errorMessage);
         return(CLIPS_TRUE);
        }
        
      GetToken(readSource,theToken);
      
      if (theToken->type != RPAREN)
        {
         if (newPort != NULL) rtn_struct(portItem,newPort);
         PPBackup();
         SavePPBuffer(" ");
         SavePPBuffer(theToken->printForm);
         SyntaxErrorMessage(errorMessage);
         return(CLIPS_TRUE);
        }
        
      if (newPort != NULL) 
        {
         if (importModule != NULL)
           {
            newPort->next = newModule->importList;
            newModule->importList = newPort;
           }
         else
           {
            newPort->next = newModule->exportList;
            newModule->exportList = newPort;
           }
        }
        
      return(CLIPS_FALSE);
     }
   
   /*========================================================*/
   /* If the ?ALL and ?NONE keywords were not used, then the */
   /* token must be the name of an importable construct.     */
   /*========================================================*/
        
   if (theToken->type != SYMBOL)
     {
      SyntaxErrorMessage(errorMessage);
      return(CLIPS_TRUE);
     }
     
   theConstruct = (SYMBOL_HN *) theToken->value;

   if ((thePortConstruct = ValidPortConstructItem(ValueToString(theConstruct))) == NULL)
     {    
      SyntaxErrorMessage(errorMessage);
      return(CLIPS_TRUE);
     }
   
   /*============================================================*/
   /* If the next token is the special variable ?ALL, then all   */
   /* constructs of the specified type that are exported from    */
   /* the specified module will be imported. If the next token   */
   /* is the special variable ?NONE, then no constructs of the   */
   /* specified type will be imported from the specified module. */
   /*============================================================*/

   SavePPBuffer(" ");
   GetToken(readSource,theToken);
   
   if (theToken->type == SF_VARIABLE)
     {
      if (strcmp(ValueToString(theToken->value),"ALL") == 0)
        {
         newPort = (struct portItem *) get_struct(portItem);
         newPort->moduleName = moduleName;
         newPort->constructType = theConstruct;
         newPort->constructName = NULL;
         newPort->next = NULL;
        }
      else if (strcmp(ValueToString(theToken->value),"NONE") == 0)
        { newPort = NULL; }
      else
        {
         SyntaxErrorMessage(errorMessage);
         return(CLIPS_TRUE);
        }
        
      GetToken(readSource,theToken);
      
      if (theToken->type != RPAREN)
        {
         if (newPort != NULL) rtn_struct(portItem,newPort);
         PPBackup();
         SavePPBuffer(" ");
         SavePPBuffer(theToken->printForm);
         SyntaxErrorMessage(errorMessage);
         return(CLIPS_TRUE);
        }
        
      if (newPort != NULL) 
        {
         if (importModule != NULL)
           {
            newPort->next = newModule->importList;
            newModule->importList = newPort;
           }
         else
           {
            newPort->next = newModule->exportList;
            newModule->exportList = newPort;
           }
        }
      
      return(CLIPS_FALSE);
     }
     
   /*====================================*/
   /* There must be at least one item in */
   /* the import/export list.            */
   /*====================================*/
   
   if (theToken->type == RPAREN)
     {
      SyntaxErrorMessage(errorMessage);
      return(CLIPS_TRUE);
     }
     
   /*=====================================*/
   /* Read in the list of imported items. */
   /*=====================================*/
         
   while (theToken->type != RPAREN)
     {      
      if (theToken->type != thePortConstruct->typeExpected)
        {
         SyntaxErrorMessage(errorMessage);
         return(CLIPS_TRUE);
        }
      
      /* Check for existence of the item. */
      
      newPort = (struct portItem *) get_struct(portItem);
      newPort->moduleName = moduleName;
      newPort->constructType = theConstruct;
      newPort->constructName = (SYMBOL_HN *) theToken->value;
      
      if (importModule != NULL)
        {
         newPort->next = newModule->importList;
         newModule->importList = newPort;
        }
      else
        {
         newPort->next = newModule->exportList;
         newModule->exportList = newPort;
        }
     
      SavePPBuffer(" ");
      GetToken(readSource,theToken);
     }
 
   PPBackup();
   PPBackup();
   SavePPBuffer(")");
  
   return(CLIPS_FALSE);
  }
  
/*************************************************************/
/* ValidPortConstructItem: Returns TRUE if a given construct */
/*   name is in the list of constructs which can be exported */
/*   and imported, otherwise FALSE is returned.              */
/*************************************************************/
globle struct portConstructItem *ValidPortConstructItem(theName)
  char *theName;
  {   
   struct portConstructItem *theItem;
   
   for (theItem = ListOfPortConstructItems; 
        theItem != NULL; 
        theItem = theItem->next)
     { if (strcmp(theName,theItem->constructName) == 0) return(theItem); }
     
   return(NULL);
  } 
  
/**********************************************************/
/* FindMultiImportConflict:          */
/**********************************************************/
static int FindMultiImportConflict(theModule)
  struct defmodule *theModule;
  {
   struct defmodule *testModule;
   int count;
   struct portConstructItem *thePCItem;
   struct construct *theConstruct;
   VOID *theCItem;
      
   SaveCurrentModule();
   
   for (testModule = (struct defmodule *) GetNextDefmodule(NULL);
        testModule != NULL;
        testModule = (struct defmodule *) GetNextDefmodule(testModule))
     {
      for (thePCItem = ListOfPortConstructItems; 
           thePCItem != NULL; 
           thePCItem = thePCItem->next) 
        {
         theConstruct = FindConstruct(thePCItem->constructName);
         
         SetCurrentModule((VOID *) testModule);
         for (theCItem = (*theConstruct->getNextItemFunction)(NULL);
              theCItem != NULL;
              theCItem = (*theConstruct->getNextItemFunction)(theCItem))
            {     
             SetCurrentModule((VOID *) theModule);
             FindImportedConstruct(thePCItem->constructName,NULL,
                                   ValueToString((*theConstruct->getConstructNameFunction)(theCItem)),
                                   &count,CLIPS_FALSE,NULL);
             if (count > 1)
               {
                ImportExportConflictMessage("defmodule",GetDefmoduleName(theModule),
                                            thePCItem->constructName,
                                            ValueToString((*theConstruct->getConstructNameFunction)(theCItem)));
                RestoreCurrentModule();
                return(CLIPS_TRUE);
               }
             SetCurrentModule((VOID *) testModule);
            }
        }
     }
     
   RestoreCurrentModule();
   return(CLIPS_FALSE);
  }

/**********************************************************/
/* NotExportedErrorMessage: Generalized error message         */
/**********************************************************/
static VOID NotExportedErrorMessage(theModule,theConstruct,theName)
  char *theModule;
  char *theConstruct;
  char *theName;
  {
   PrintErrorID("MODULPSR",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Module ");
   PrintCLIPS(WERROR,theModule);
   PrintCLIPS(WERROR," does not export ");
   
   if (theConstruct == NULL) PrintCLIPS(WERROR,"any constructs");
   else if (theName == NULL)
     {
      PrintCLIPS(WERROR,"any ");
      PrintCLIPS(WERROR,theConstruct);
      PrintCLIPS(WERROR," constructs");
     }
   else
     {
      PrintCLIPS(WERROR,"the ");
      PrintCLIPS(WERROR,theConstruct);
      PrintCLIPS(WERROR," ");
      PrintCLIPS(WERROR,theName);
     }
   
   PrintCLIPS(WERROR,".\n");
  }

/*************************************************************/
/* FindImportExportConflict: Determines if the definition of */
/*   a construct would cause an import/export conflict. The  */
/*   construct is not yet defined when this function is      */
/*   called. TRUE is returned if an import/export conflicts  */
/*   is found, otherwise FALSE is returned.                  */
/*************************************************************/
globle int FindImportExportConflict(constructName,matchModule,findName)
  char *constructName;
  struct defmodule *matchModule;
  char *findName;
  {
   struct defmodule *theModule;
   struct moduleItem *theModuleItem;
   int count;
   
   /*===========================================================*/
   /* If the construct type can't be imported or exported, then */
   /* it's not possible to have an import/export conflict.      */
   /*===========================================================*/
   
   if (ValidPortConstructItem(constructName) == NULL) return(CLIPS_FALSE);
   
   /*============================================*/
   /* There module name should already have been */
   /* separated fromthe construct's name.        */
   /*============================================*/
   
   if (FindModuleSeparator(findName)) return(CLIPS_FALSE);
   
   /*===============================================================*/
   /* The construct must be capable of being stored within a module */
   /* (this test should never fail). The construct must also have   */
   /* a find function associated with it so we can actually look    */
   /* for import/export conflicts.                                  */
   /*===============================================================*/
   
   if ((theModuleItem = FindModuleItem(constructName)) == NULL) return(CLIPS_FALSE);

   if (theModuleItem->findFunction == NULL) return(CLIPS_FALSE);
        
   /*==========================*/
   /* Save the current module. */
   /*==========================*/
   
   SaveCurrentModule();
   
   /*================================================================*/
   /* Look at each module and count each definition of the specified */
   /* construct which is visible to the module. If more than one     */
   /* definition is visible, then an import/export conflict exists   */
   /* and TRUE is returned.                                          */
   /*================================================================*/
   
   for (theModule = (struct defmodule *) GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = (struct defmodule *) GetNextDefmodule(theModule))
     {
      SetCurrentModule((VOID *) theModule);
         
      FindImportedConstruct(constructName,NULL,findName,&count,CLIPS_TRUE,matchModule);
      if (count > 1)
        {
         RestoreCurrentModule();
         return(CLIPS_TRUE);
        }
     }
     
   /*==========================================*/
   /* Restore the current module. No conflicts */
   /* were detected so FALSE is returned.      */
   /*==========================================*/
   
   RestoreCurrentModule();
   return(CLIPS_FALSE);
  }

#endif /* DEFMODULE_CONSTRUCT && (! RUN_TIME) && (! BLOAD_ONLY) */


