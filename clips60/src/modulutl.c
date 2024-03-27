   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              DEFMODULE UTILITY MODULE               */
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

#define _MODULUTL_SOURCE_

#include "setup.h"

#include "clipsmem.h"
#include "router.h"

#include "modulpsr.h"
#include "modulutl.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                      *SearchImportedConstructModules(struct symbolHashNode *,
                                              struct defmodule *,
                                              struct moduleItem *,struct symbolHashNode *,
                                              int *,int,struct defmodule *);
#else
   static VOID                      *SearchImportedConstructModules();
#endif

/********************************************************************/
/* FindModuleSeparator: Finds the :: separator which delineates the */
/*   boundary between a module name and a construct name. The value */
/*   zero is returned if the separator is not found, otherwise the  */
/*   position of the second colon within the string is returned.    */
/********************************************************************/
globle int FindModuleSeparator(theString)
  char *theString;
  {
   int i, foundColon;
   
   for (i = 0, foundColon = CLIPS_FALSE; theString[i] != EOS; i++)
     {
      if (theString[i] == ':')
        {
         if (foundColon) return(i);
         foundColon = CLIPS_TRUE;
        }
      else
        { foundColon = CLIPS_FALSE; }
     }
     
   return(CLIPS_FALSE);
  }
  
/*******************************************************************/
/* ExtractModuleName: Given the position of the :: separator and a */
/*   module/construct name joined using the separator, returns a   */
/*   symbol reference to the module name (or NULL if a module name */
/*   cannot be extracted).                                         */
/*******************************************************************/
globle SYMBOL_HN *ExtractModuleName(thePosition,theString)
  int thePosition;
  char *theString;
  {
   char *newString;
   SYMBOL_HN *returnValue;
   
   if (thePosition <= 1) return(NULL);
   
   newString = (char *) gm2(thePosition);
   
   strncpy(newString,theString,thePosition - 1);
   newString[thePosition-1] = EOS;
   
   returnValue = (SYMBOL_HN *) AddSymbol(newString);
   
   rm(newString,thePosition);
   
   return(returnValue);
  }

/********************************************************************/
/* ExtractConstructName: Given the position of the :: separator and */
/*   a module/construct name joined using the separator, returns a  */
/*   symbol reference to the construct name (or NULL if a construct */
/*   name cannot be extracted).                                     */
/********************************************************************/
globle SYMBOL_HN *ExtractConstructName(thePosition,theString)
  int thePosition;
  char *theString;
  {
   int theLength;
   char *newString;
   SYMBOL_HN *returnValue;
   
   if (thePosition == 0) return((SYMBOL_HN *) AddSymbol(theString));
   
   theLength = strlen(theString);
   
   if (theLength <= (thePosition + 1)) return(NULL);
   
   if (thePosition < 1) return(NULL);
   
   newString = (char *) gm2(theLength - thePosition);
   
   strncpy(newString,&theString[thePosition+1],theLength - thePosition);
   
   returnValue = (SYMBOL_HN *) AddSymbol(newString);
   
   rm(newString,theLength - thePosition);
   
   return(returnValue);
  }
  
/**************************************************************************/
/* ExtractModuleAndConstructName: Extracts both the module and construct  */
/*   name from a string. Sets the current module to the specified module. */
/**************************************************************************/
globle char *ExtractModuleAndConstructName(theName)
  char *theName;
  {
   int separatorPosition;
   SYMBOL_HN *moduleName, *shortName;
   struct defmodule *theModule;
   
   separatorPosition = FindModuleSeparator(theName);
   if (! separatorPosition) return(theName);
   
   moduleName = ExtractModuleName(separatorPosition,theName);
   if (moduleName == NULL) return(NULL);
        
   theModule = (struct defmodule *) FindDefmodule(ValueToString(moduleName));
   if (theModule == NULL) return(NULL);
        
   SetCurrentModule((VOID *) theModule);
   
   shortName = ExtractConstructName(separatorPosition,theName);
   return(ValueToString(shortName));
  }
  
/**********************************************************/
/* FindImportedConstruct:          */
/**********************************************************/
globle VOID *FindImportedConstruct(constructName,matchModule,findName,count,
                                   searchCurrent,notYetDefinedInModule)
  char *constructName;
  struct defmodule *matchModule;
  char *findName;
  int *count;
  int searchCurrent;
  struct defmodule *notYetDefinedInModule;
  {
   VOID *rv;
   struct moduleItem *theModuleItem;

   *count = 0;
      
   if (FindModuleSeparator(findName)) return(NULL);
   
   SaveCurrentModule();
   
   if ((theModuleItem = FindModuleItem(constructName)) == NULL)
     {
      RestoreCurrentModule();
      return(NULL);
     } 

   if (theModuleItem->findFunction == NULL)
     {
      RestoreCurrentModule();
      return(NULL);
     }
        
   MarkModulesAsUnvisited();
   
   rv = SearchImportedConstructModules(AddSymbol(constructName),
                                       matchModule,
                                       theModuleItem,
                                       AddSymbol(findName),
                                       count,
                                       searchCurrent,
                                       notYetDefinedInModule);
   
   RestoreCurrentModule();
   
   return(rv);
  }

/**********************************************************/
/* AmbiguousReferenceErrorMessage:          */
/**********************************************************/
globle VOID AmbiguousReferenceErrorMessage(constructName,findName)
  char *constructName;
  char *findName;
  {
   PrintCLIPS(WERROR,"Ambiguous reference to ");
   PrintCLIPS(WERROR,constructName);
   PrintCLIPS(WERROR," ");
   PrintCLIPS(WERROR,findName);
   PrintCLIPS(WERROR,".\nIt is imported from more than one module.\n");
  }

/**********************************************************/
/* MarkModulesAsUnvisited:                        */
/**********************************************************/
globle VOID MarkModulesAsUnvisited()
  {
   struct defmodule *theModule;
   
   CurrentModule->visitedFlag = CLIPS_FALSE;
   for (theModule = (struct defmodule *) GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = (struct defmodule *) GetNextDefmodule(theModule))
     { theModule->visitedFlag = CLIPS_FALSE; }
  }   
  
/********************************************************************/
/* SearchImportedConstructModules: If matchModule is non-NULL, then */
/*   we are interested if a specific construct in a specific module */
/*   is within the scope of the module which called                 */
/*   FindImportedConstruct(). Otherwise, we are interested in any   */
/*   construct of the correct name.                                 */
/********************************************************************/
static VOID *SearchImportedConstructModules(constructName,matchModule,theModuleItem,
                                            findName,count,searchCurrent,
                                            notYetDefinedInModule)
  struct symbolHashNode *constructName;
  struct defmodule *matchModule;
  struct moduleItem *theModuleItem;
  struct symbolHashNode *findName;
  int *count;
  int searchCurrent;
  struct defmodule *notYetDefinedInModule;
  {
   struct defmodule *theModule;
   struct portItem *theImportList, *theExportList;
   VOID *rv, *arv = NULL;
   int searchModule, exported;
   struct defmodule *currentModule;
   
   /*=========================================================*/
   /* Look in the current module for the specified construct. */
   /* If the construct is in the specified module, increment  */
   /* the count only if the construct actually belongs to the */
   /* module (some constructs, like the COOL system classes,  */
   /* can be found in any module, but they actually belong to */
   /* the MAIN module.                                        */
   /*=========================================================*/
   
   currentModule = ((struct defmodule *) GetCurrentModule());
   if (currentModule->visitedFlag) return(NULL);
   
   if ((searchCurrent) &&
       ((matchModule == NULL) || (currentModule == matchModule)))
     {
      rv = (*theModuleItem->findFunction)(ValueToString(findName));
      if (notYetDefinedInModule == currentModule)
        {
         (*count)++;
         arv = rv;
        }
      else if (rv != NULL)
        {  
         if (((struct constructHeader *) rv)->whichModule->theModule == currentModule)
           { (*count)++; }
         arv = rv;
        }
     }
   
   currentModule->visitedFlag = CLIPS_TRUE;
   
   /*===========================================*/
   /* Search through all of the imported items. */
   /*===========================================*/
   
   theModule = ((struct defmodule *) GetCurrentModule());
   theImportList = theModule->importList;
   
   while (theImportList != NULL)
     {
      searchModule = CLIPS_FALSE;
      
      /*====================================================================*/
      /* Determine if the module should be searched (based upon whether the */
      /* entire module, all constructs of a specific type, or specifically  */
      /* named constructs are imported.                                     */
      /*====================================================================*/
      
      if ((theImportList->constructType == NULL) ||
          (theImportList->constructType == constructName))
        {
         if ((theImportList->constructName == NULL) ||
             (theImportList->constructName == findName)) 
           { searchModule = CLIPS_TRUE; }
        }
      
      /*=================================*/
      /* Determine if the module exists. */
      /*=================================*/
      
      if (searchModule)
        {
         theModule = (struct defmodule *) FindDefmodule(ValueToString(theImportList->moduleName));
         if (theModule == NULL) searchModule = CLIPS_FALSE;
        }
        
      /*=======================================================*/
      /* Determine if the construct is exported by the module. */
      /*=======================================================*/
      
      if (searchModule)
        {
         exported = CLIPS_FALSE;
         theExportList = theModule->exportList;
         while ((theExportList != NULL) && (! exported))
           {
            if ((theExportList->constructType == NULL) ||
                (theExportList->constructType == constructName))
              {
               if ((theExportList->constructName == NULL) ||
                   (theExportList->constructName == findName)) 
                 { exported = CLIPS_TRUE; }
               }
               
            theExportList = theExportList->next;
           }
         
         if (! exported) searchModule = CLIPS_FALSE;
        }
          
      /*=================================*/
      /* Search in the specified module. */
      /*=================================*/
      
      if (searchModule)
        {
         SetCurrentModule((VOID *) theModule);
         if ((rv = SearchImportedConstructModules(constructName,matchModule,theModuleItem,findName,
                                                  count,CLIPS_TRUE,notYetDefinedInModule)) != NULL)
           { arv = rv; }
        }
        
      /*====================================*/
      /* Move on to the next imported item. */
      /*====================================*/
      
      theImportList = theImportList->next;
     }
   
   return(arv);
  }

