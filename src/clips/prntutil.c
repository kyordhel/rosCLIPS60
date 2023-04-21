   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                PRINT UTILITY MODULE                 */
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

#define _PRNTUTIL_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "constant.h"
#include "symbol.h"
#include "utility.h"
#include "evaluatn.h"
#include "argacces.h"
#include "router.h"

#include "prntutil.h"

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle BOOLEAN              PreserveEscapedCharacters = CLIPS_FALSE;
   globle BOOLEAN              AddressesToStrings = CLIPS_FALSE;
   globle BOOLEAN              InstanceAddressesToNames = CLIPS_FALSE;

/**************************************************************/
/* PrintInChunks:  Prints a string in chunks to accomodate  */
/*   systems which have a limit on the maximum size of a      */
/*   string which can be printed.                             */
/**************************************************************/
globle VOID PrintInChunks(logicalName,str)
  char *logicalName, *str;
  {
   char tc, *s_ptr;

   s_ptr = str;

   if (s_ptr == NULL) return;

   while (strlen(s_ptr) > 500)
     {
      tc = s_ptr[500];
      s_ptr[500] = EOS;
      PrintCLIPS(logicalName,s_ptr);
      s_ptr[500] = tc;
      s_ptr += 500;
     }

   PrintCLIPS(logicalName,s_ptr);
  }

/************************************************************/
/* PrintFloat:  Controls printout of floating point numbers. */
/************************************************************/
globle VOID PrintFloat(fileid,number)
  char *fileid;
  double number;
  {
   char *theString;

   theString = FloatToString(number);
   PrintCLIPS(fileid,theString);
  }

/***************************************************/
/* PrintLongInteger:  Controls printout of integers. */
/***************************************************/
globle VOID PrintLongInteger(fileid,number)
  char *fileid;
  long int number;
  {
   char printBuffer[32];

   sprintf(printBuffer,"%ld",number);
   PrintCLIPS(fileid,printBuffer);
  }
  
/************************************************/
/* PrintAtom: */
/************************************************/
globle VOID PrintAtom(logicalName,type,value)
  char *logicalName;
  int type;
  VOID *value;
  {
   char buffer[20];

   switch (type)
     {
      case FLOAT:
        PrintFloat(logicalName,ValueToDouble(value));
        break;
      case INTEGER:
        PrintLongInteger(logicalName,ValueToLong(value));
        break;
      case SYMBOL:
        PrintCLIPS(logicalName,ValueToString(value));
        break;
      case STRING:
        if (PreserveEscapedCharacters)
          { PrintCLIPS(logicalName,StringPrintForm(ValueToString(value))); }
        else
          {
           PrintCLIPS(logicalName,"\"");
           PrintCLIPS(logicalName,ValueToString(value));
           PrintCLIPS(logicalName,"\"");
          }
        break;

      case EXTERNAL_ADDRESS:
        if (AddressesToStrings) PrintCLIPS(logicalName,"\"");
        PrintCLIPS(logicalName,"<Pointer-");
#if ANSI_COMPILER
        sprintf(buffer,"%p",value);
#else
        sprintf(buffer,"%lu",(unsigned long) value);
#endif
        PrintCLIPS(logicalName,buffer);
        PrintCLIPS(logicalName,">");
        if (AddressesToStrings) PrintCLIPS(logicalName,"\"");
        break;

#if OBJECT_SYSTEM
      case INSTANCE_NAME:
        PrintCLIPS(logicalName,"[");
        PrintCLIPS(logicalName,ValueToString(value));
        PrintCLIPS(logicalName,"]");
        break;
#endif

      default:
        if (PrimitivesArray[type] == NULL) break;
        if (PrimitivesArray[type]->longPrintFunction == NULL)
          {
           PrintCLIPS(logicalName,"<unknown atom type>");
           break;
          } 
        (*PrimitivesArray[type]->longPrintFunction)(logicalName,value);
        break;
     }
  }
  
/**********************************************************/
/* PrintTally: Prints a tally count indicating the number */
/*   of items that have been displayed. Used by functions */
/*   such as list-defrules.                               */
/**********************************************************/
globle VOID PrintTally(logicalName,count,singular,plural)
  char *logicalName;
  long count;
  char *singular, *plural;
  {
   if (count == 0) return;

   PrintCLIPS(logicalName,"For a total of ");
   PrintLongInteger(logicalName,count);
   PrintCLIPS(logicalName," ");

   if (count == 1) PrintCLIPS(logicalName,singular);
   else PrintCLIPS(logicalName,plural);

   PrintCLIPS(logicalName,".\n");
  }

/********************************************/
/* PrintErrorID: Prints the module name and */
/*   error ID for an error message.         */
/********************************************/
globle VOID PrintErrorID(module,errorID,printCR)
  char *module;
  int errorID;
  int printCR;
  {
   if (printCR) PrintCLIPS(WERROR,"\n");
   PrintCLIPS(WERROR,"[");
   PrintCLIPS(WERROR,module);
   PrintLongInteger(WERROR,(long int) errorID);
   PrintCLIPS(WERROR,"] ");
  }
  
/**********************************************/
/* PrintWarningID: Prints the module name and */
/*   warning ID for a warning message.        */
/**********************************************/
globle VOID PrintWarningID(module,warningID,printCR)
  char *module;
  int warningID;
  int printCR;
  {
   if (printCR) PrintCLIPS(WWARNING,"\n");
   PrintCLIPS(WWARNING,"[");
   PrintCLIPS(WWARNING,module);
   PrintLongInteger(WWARNING,(long int) warningID);
   PrintCLIPS(WWARNING,"] WARNING: ");
  }
  
/********************************************/
/* CantFindItemErrorMessage:          */
/********************************************/
globle VOID CantFindItemErrorMessage(itemType,itemName)
  char *itemType;
  char *itemName;
  {
   PrintErrorID("PRNTUTIL",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Unable to find ");
   PrintCLIPS(WERROR,itemType);
   PrintCLIPS(WERROR," ");
   PrintCLIPS(WERROR,itemName);
   PrintCLIPS(WERROR,".\n");
  }
  
/********************************************/
/* CantDeleteItemErrorMessage:          */
/********************************************/
globle VOID CantDeleteItemErrorMessage(itemType,itemName)
  char *itemType;
  char *itemName;
  {
   PrintErrorID("PRNTUTIL",4,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Unable to delete ");
   PrintCLIPS(WERROR,itemType);
   PrintCLIPS(WERROR," ");
   PrintCLIPS(WERROR,itemName);
   PrintCLIPS(WERROR,".\n");
  }

/********************************************/
/* AlreadyParsedErrorMessage:          */
/********************************************/
globle VOID AlreadyParsedErrorMessage(itemType,itemName)
  char *itemType;
  char *itemName;
  {
   PrintErrorID("PRNTUTIL",5,CLIPS_TRUE);
   PrintCLIPS(WERROR,"The ");
   if (itemType != NULL) PrintCLIPS(WERROR,itemType);
   if (itemName != NULL) PrintCLIPS(WERROR,itemName);
   PrintCLIPS(WERROR," has already been parsed.\n");
  }
    
/*********************************************************/
/* SyntaxErrorMessage: Generalized syntax error message. */
/*********************************************************/
globle VOID SyntaxErrorMessage(location)
  char *location;
  {
   PrintErrorID("PRNTUTIL",2,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Syntax Error");
   if (location != NULL)
     {
      PrintCLIPS(WERROR,":  Check appropriate syntax for ");
      PrintCLIPS(WERROR,location);
     }

   PrintCLIPS(WERROR,".\n");
   SetEvaluationError(CLIPS_TRUE);
  }
  
/*********************************************************/
/* LocalVariableErrorMessage: */
/*********************************************************/
globle VOID LocalVariableErrorMessage(byWhat)
  char *byWhat;
  {
   PrintErrorID("PRNTUTIL",6,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Local variables cannot be accessed by ");
   PrintCLIPS(WERROR,byWhat);
   PrintCLIPS(WERROR,".\n");
  }
   
/**************************************************************************/
/* CLIPSSystemError: Generalized error message for major internal errors. */
/**************************************************************************/
globle VOID CLIPSSystemError(module,errorID)
  char *module;
  int errorID;
  {
   PrintErrorID("PRNTUTIL",3,CLIPS_TRUE);
   PrintCLIPS(WERROR,"\n*** CLIPS SYSTEM ERROR ***\n");
   PrintCLIPS(WERROR,"ID = ");
   PrintCLIPS(WERROR,module);
   PrintLongInteger(WERROR,(long int) errorID);
   PrintCLIPS(WERROR,"\n");
   PrintCLIPS(WERROR,"CLIPS data structures are in an inconsistent or corrupted state.\n");
   PrintCLIPS(WERROR,"This error may have occurred from errors in user defined code.\n");
   PrintCLIPS(WERROR,"**************************\n");
  }  
  
/*******************************************************/
/* DivideByZeroErrorMessage: Generalized error message */
/*   for when a function attempts to divide by zero.   */
/*******************************************************/
globle VOID DivideByZeroErrorMessage(functionName)
  char *functionName;
  {
   PrintErrorID("PRNTUTIL",7,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Attempt to divide by zero in ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," function.\n");
  }

/***********************************************************/
/* FloatToString:  Converts number to CLIPS string format. */
/***********************************************************/
globle char *FloatToString(number)
  double number;
  {
   char floatString[40];
   int i;
   char x;
   VOID *thePtr;

   sprintf(floatString,"%.16g",number);

   for (i = 0; (x = floatString[i]) != '\0'; i++)
     {
      if ((x == '.') || (x == 'e'))
        {
         thePtr = AddSymbol(floatString);
         return(ValueToString(thePtr));
        }
     }

   strcat(floatString,".0");

   thePtr = AddSymbol(floatString);
   return(ValueToString(thePtr));
  }

/***********************************************************************/
/* LongIntegerToString:  Converts long integer to CLIPS string format. */
/***********************************************************************/
globle char *LongIntegerToString(number)
  long number;
  {
   char buffer[30];
   VOID *thePtr;

   sprintf(buffer,"%ld",number);

   thePtr = AddSymbol(buffer);
   return(ValueToString(thePtr));
  }

/******************************************************************/
/* ListItemsDriver: Driver routine for listing items in a module. */
/******************************************************************/
globle VOID ListItemsDriver(logicalName,theModule,singleName,pluralName,
                            nextFunction,nameFunction,printFunction,
                            doItFunction)
  char *logicalName;
  struct defmodule *theModule;
  char *singleName,*pluralName;
#if ANSI_COMPILER
  VOID *(*nextFunction)(VOID *);
  char *(*nameFunction)(VOID *);
  VOID (*printFunction)(char *,VOID *);
  int (*doItFunction)(VOID *);
#else
  VOID *(*nextFunction)();
  char *(*nameFunction)();
  VOID (*printFunction)();
  int (*doItFunction)();
#endif
  {
   VOID *constructPtr;
   char *constructName;
   long count = 0;
   int allModules = CLIPS_FALSE;
   int doIt;
   
   /*==========================*/
   /* Save the current module. */
   /*==========================*/
   
   SaveCurrentModule();
  
   /*======================*/
   /* Print out the items. */
   /*======================*/
   
   if (theModule == NULL) 
     {
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
      allModules = CLIPS_TRUE;
     }
   
   while (theModule != NULL)
     {
      if (allModules) 
        {
         PrintCLIPS(logicalName,GetDefmoduleName(theModule));
         PrintCLIPS(logicalName,":\n");
        }
        
      SetCurrentModule((VOID *) theModule);
      constructPtr = (*nextFunction)(NULL);
      while (constructPtr != NULL)
        {
         if (HaltExecution == CLIPS_TRUE) return;
  
         if (doItFunction == NULL) doIt = CLIPS_TRUE;
         else doIt = (*doItFunction)(constructPtr);
         
         if (! doIt) {}
         else if (nameFunction != NULL)
           { 
            constructName = (*nameFunction)(constructPtr);
           
            constructName = (*nameFunction)(constructPtr);
            if (constructName != NULL)
              {
               if (allModules) PrintCLIPS(logicalName,"   ");
               PrintCLIPS(logicalName,constructName);
               PrintCLIPS(logicalName,"\n");
              }
           }
         else if (printFunction != NULL)
           {
            if (allModules) PrintCLIPS(logicalName,"   ");
            (*printFunction)(logicalName,constructPtr);
            PrintCLIPS(logicalName,"\n");
           }
            
         constructPtr = (*nextFunction)(constructPtr);
         count++;
        }
        
      if (allModules) theModule = (struct defmodule *) GetNextDefmodule(theModule);
      else theModule = NULL;
     }

   /*=================================================*/
   /* Print the tally and restore the current module. */
   /*=================================================*/
   
   if (singleName != NULL) PrintTally(logicalName,count,singleName,pluralName);
   
   RestoreCurrentModule();
  }

