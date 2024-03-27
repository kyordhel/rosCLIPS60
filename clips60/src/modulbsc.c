   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFMODULE BASIC COMMANDS HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the defmodule       */
/*   construct such as clear, reset, save, ppdefmodule       */
/*   list-defmodules, and get-defmodule-list.                */
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

#define _MODULBSC_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "constrct.h"
#include "extnfunc.h"
#include "modulbin.h"
#include "prntutil.h"
#include "modulcmp.h"
#include "router.h"
#include "argacces.h"
#include "bload.h"

#include "modulbsc.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    ClearDefmodules(void);
#if DEFMODULE_CONSTRUCT
   static VOID                    SaveDefmodules(char *);
#endif
#else
   static VOID                    ClearDefmodules();
#if DEFMODULE_CONSTRUCT
   static VOID                    SaveDefmodules();
#endif
#endif

/*****************************************************************/
/* DefmoduleBasicCommands: Initializes basic defmodule commands. */
/*****************************************************************/
globle VOID DefmoduleBasicCommands()
  {
   AddClearFunction("defmodule",ClearDefmodules,2000);
   
#if DEFMODULE_CONSTRUCT
   AddSaveFunction("defmodule",SaveDefmodules,1100);
   
#if ! RUN_TIME
   DefineFunction2("get-defmodule-list",'m',PTIF GetDefmoduleList,"GetDefmoduleList","00");

#if DEBUGGING_FUNCTIONS
   DefineFunction2("list-defmodules",'v', PTIF ListDefmodulesCommand,"ListDefmodulesCommand","00");
   DefineFunction2("ppdefmodule",'v',PTIF PPDefmoduleCommand,"PPDefmoduleCommand","11w");
#endif
#endif
#endif

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)
   DefmoduleBinarySetup();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   DefmoduleCompilerSetup();
#endif
  }

/*********************************************************/
/* ClearDefmodules: Defmodule clear routine for use with */
/*   the clear command. Creates the MAIN module.         */
/*********************************************************/
static VOID ClearDefmodules()
  {
#if (BLOAD || BLOAD_AND_BSAVE || BLOAD_ONLY) && (! RUN_TIME)
   if (Bloaded() == CLIPS_TRUE) return;
#endif
#if (! RUN_TIME)
   RemoveAllDefmodules();
#endif
#if (! RUN_TIME)
   CreateMainModule();
   MainModuleRedefinable = CLIPS_TRUE;
#endif
  }  

#if DEFMODULE_CONSTRUCT

/******************************************/
/* SaveDefmodules: Defmodule save routine */
/*   for use with the save command.       */
/******************************************/
static VOID SaveDefmodules(logicalName)
  char *logicalName;
  {
   VOID *defmodulePtr;
   char *ppform;

   defmodulePtr = GetNextDefmodule(NULL);
   while (defmodulePtr != NULL)
     {
      ppform = GetDefmodulePPForm(defmodulePtr);
      if (ppform != NULL)
        {
         PrintInChunks(logicalName,ppform);
         PrintCLIPS(logicalName,"\n");
        }
      defmodulePtr = GetNextDefmodule(defmodulePtr);
     }
  }
 
/************************************************/
/* GetDefmoduleList: CLIPS and C access routine */
/*   for the get-defmodule-list function.       */
/************************************************/
globle VOID GetDefmoduleList(returnValue)
  DATA_OBJECT_PTR returnValue;
  { OldGetConstructList(returnValue,GetNextDefmodule,GetDefmoduleName); }
  
#if DEBUGGING_FUNCTIONS

/********************************************/
/* PPDefmoduleCommand: CLIPS access routine */
/*   for the ppdefmodule command.           */
/********************************************/
globle VOID PPDefmoduleCommand()
  {
   char *defmoduleName;

   defmoduleName = GetConstructName("ppdefmodule","defmodule name");
   if (defmoduleName == NULL) return;

   PPDefmodule(defmoduleName,WDISPLAY);

   return;
  }
  
/*************************************/
/* PPDefmodule: C access routine for */
/*   the ppdefmodule command.        */
/*************************************/
globle int PPDefmodule(defmoduleName,logicalName)
  char *defmoduleName, *logicalName;
  {
   VOID *defmodulePtr;

   defmodulePtr = FindDefmodule(defmoduleName);
   if (defmodulePtr == NULL)
     {
      CantFindItemErrorMessage("defmodule",defmoduleName);
      return(CLIPS_FALSE);
     }

   if (GetDefmodulePPForm(defmodulePtr) == NULL) return(CLIPS_TRUE);
   PrintInChunks(logicalName,GetDefmodulePPForm(defmodulePtr));
   return(CLIPS_TRUE);
  }

/***********************************************/
/* ListDefmodulesCommand: CLIPS access routine */
/*   for the list-defmodules command.          */
/***********************************************/
globle VOID ListDefmodulesCommand()
  {    
   if (ArgCountCheck("list-defmodules",EXACTLY,0) == -1) return;

   ListDefmodules(WDISPLAY);
  }

/****************************************/
/* ListDefmodules: C access routine for */
/*   the list-defmodules command.       */
/****************************************/
globle VOID ListDefmodules(logicalName)
  char *logicalName;
  {
   VOID *theModule;
   int count = 0;
   
   for (theModule = GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = GetNextDefmodule(theModule))
    {
     PrintCLIPS(logicalName,GetDefmoduleName(theModule));
     PrintCLIPS(logicalName,"\n");
     count++;
    }
   
   PrintTally(logicalName,count,"defmodule","defmodules");
  }

#endif /* DEBUGGING_FUNCTIONS */

#endif /* DEFMODULE_CONSTRUCT */


