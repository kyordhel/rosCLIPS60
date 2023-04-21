   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                COMMAND LINE MODULE                  */
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

#define _COMMLINE_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>
#include <ctype.h>
//#include <tcl.h>
//#include <tk.h>

#include "setup.h"
#include "constant.h"
#include "commline.h"

#if ! RUN_TIME

#include "symbol.h"
#include "clipsmem.h"
#include "scanner.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"
#include "strngrtr.h"
#include "constrct.h"
#include "prcdrfun.h"
#include "prcdrpsr.h"
#include "utility.h"
#include "filecom.h"
#include "cstrcpsr.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     DoString(char *,int,int *);
   static int                     DoComment(char *,int);
   static int                     DoWhiteSpace(char *,int);
   static VOID                    DefaultGetNextEvent(void);
#else
   static int                     DoString();
   static int                     DoComment();
   static int                     DoWhiteSpace();
   static VOID                    DefaultGetNextEvent();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle int               EvaluatingTopLevelCommand = CLIPS_FALSE;

/****************************************/
/* GLOBAL EXTERNAL VARIABLE DEFINITIONS */
/****************************************/	

/*extern int TkClipsFlag;*/

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static char             *CommandString = NULL;
   static int               MaximumCharacters = 0;
   static int               ParsingTopLevelCommand = CLIPS_FALSE;
   static char             *VersionString = "         CLIPS (V6.0 05/12/93)\n";

#if ANSI_COMPILER
   static int              (*EventFunction)(void) =
                                           (int (*)(void)) DefaultGetNextEvent;
   static int              (*MemoryStatusFunction)(void) = NULL;
#else
   static int              (*EventFunction)() =
                                               (int (*)()) DefaultGetNextEvent;
   static int              (*MemoryStatusFunction)() = NULL;
#endif

/******************************************************************/
/* ExpandCommandString: Appends a character to the CommandString. */
/******************************************************************/
globle int ExpandCommandString(inchar)
  int inchar;
  {
   register int k;

   k = CLIPSInputCount;
   CommandString = ExpandStringWithChar(inchar,CommandString,&CLIPSInputCount,
                                        &MaximumCharacters,MaximumCharacters+80);
   return((CLIPSInputCount != k) ? CLIPS_TRUE : CLIPS_FALSE);
  }

/******************************************************************/
/* FlushCommandString: Empties the contents of the CommandString. */
/******************************************************************/
globle VOID FlushCommandString()
  {
   if (CommandString != NULL) rm(CommandString,MaximumCharacters);
   CommandString = NULL;
   MaximumCharacters = 0;
   CLIPSInputCount = 0;
  }

/*********************************************************************************/
/* SetCommandString: Sets the contents of the CommandString to a specific value. */
/*********************************************************************************/
globle VOID SetCommandString(str)
  char *str;
  {
   int length;

   FlushCommandString();
   length = strlen(str);
   CommandString = genrealloc(CommandString,(unsigned) MaximumCharacters,
                              (unsigned) MaximumCharacters + length + 1);

   strcpy(CommandString,str);
   MaximumCharacters += (length + 1);
   CLIPSInputCount += length;
  }

/******************************************************************************/
/* AppendCommandString: Appends a value to the contents of the CommandString. */
/******************************************************************************/
globle VOID AppendCommandString(str)
  char *str;
  {
   CommandString = AppendToString(str,CommandString,&CLIPSInputCount,&MaximumCharacters);
  }

/*****************************************************************************/
/* GetCommandString: Returns a pointer to the contents of the CommandString. */
/*****************************************************************************/
globle char *GetCommandString()
  {
   return(CommandString);
  }

/**************************************************************************/
/* CompleteCommand: Determines whether a string forms a complete command. */
/*   A complete command is either a constant, a variable, or a function   */
/*   call which is followed (at some pointer) by a carriage return. Once  */
/*   a complete command is found (not including the parenthesis),         */
/*   extraneous parenthesis and other tokens are ignored.                 */
/**************************************************************************/
globle int CompleteCommand(mstring)
  char *mstring;
  {
   int i;
   char inchar;
   int depth = 0;
   int moreThanZero = 0;
   int complete;
   int error = 0;

   if (mstring == NULL) return(0);

   i = 0;
   while ((inchar = mstring[i++]) != EOS)
     {
      switch(inchar)
        {
         case '\n' :
         case '\r' :
           if (error) return(-1);
           if (moreThanZero && (depth == 0)) return(1);
           i = DoWhiteSpace(mstring,i);
           break;

         case ' ' :
         case '\f' :
         case '\t' :
           i = DoWhiteSpace(mstring,i);
           break;

         case '"' :
           i = DoString(mstring,i,&complete);
           if ((depth == 0) && complete) moreThanZero = CLIPS_TRUE;
           break;

         case ';' :
           i = DoComment(mstring,i);
           if (moreThanZero && (depth == 0) && (mstring[i] != EOS))
             {
              if (error) return(-1);
              else return(1);
             }
           else if (mstring[i] != EOS) i++;
           break;

         case '(' :
           if ((depth > 0) || (moreThanZero == CLIPS_FALSE))
             {
              depth++;
              moreThanZero = CLIPS_TRUE;
             }
           break;

         case ')' :
           if (depth > 0) depth--;
           else if (moreThanZero == CLIPS_FALSE) error = CLIPS_TRUE;
           break;

         default:
           if (depth == 0)
             {
              if (isprint(inchar))
                {
                 while ((inchar = mstring[i++]) != EOS)
                   {
                    if ((inchar == '\n') || (inchar == '\r'))
                      {
                       if (error) return(-1);
                       else return(1);
                      }
                   }
                 return(0);
                }
             }
           break;
        }
     }

   return(0);
  }

/***********************************************************/
/* DoString: Skips over a string contained within a string */
/*   until the closing quotation mark is encountered.      */
/***********************************************************/
static int DoString(str,pos,complete)
  char *str;
  int pos;
  int *complete;
  {
   int inchar;

   inchar = str[pos];
   while (inchar  != '"')
     {
      if (inchar == '\\')
        {
         pos++;
         inchar = str[pos];
        }

      if (inchar == EOS)
        {
         *complete = CLIPS_FALSE;
         return(pos);
        }

      pos++;
      inchar = str[pos];
     }

   pos++;
   *complete = CLIPS_TRUE;
   return(pos);
  }

/*************************************************************/
/* DoComment: Skips over a comment contained within a string */
/*   until a line feed or carriage return is encountered.    */
/*************************************************************/
static int DoComment(str,pos)
  char *str;
  int pos;
  {
   int inchar;

   inchar = str[pos];
   while ((inchar != '\n') && (inchar != '\r'))
     {
      if (inchar == EOS)
        { return(pos); }

      pos++;
      inchar = str[pos];
     }

   return(pos);
  }

/**************************************************************/
/* DoWhiteSpace: Skips over white space consisting of spaces, */
/*   tabs, and form feeds that is contained within a string.  */
/**************************************************************/
static int DoWhiteSpace(str,pos)
  char *str;
  int pos;
  {
   int inchar;

   inchar = str[pos];
   while ((inchar == ' ') || (inchar == '\f') || (inchar == '\t'))
     {
      pos++;
      inchar = str[pos];
     }

   return(pos);
  }

/********************************************************************/
/* CommandLoop: Endless loop which waits for user commands and then */
/*   executes them. The command loop will bypass the EventFunction  */
/*   if there is an active batch file.                              */
/********************************************************************/
globle VOID CommandLoop()
  {
   int inchar,i;

   PrintCLIPS(WCLIPS,VersionString);
   SetHaltExecution(CLIPS_FALSE);
   SetEvaluationError(CLIPS_FALSE);
   PeriodicCleanup(CLIPS_TRUE,CLIPS_FALSE);
   PrintPrompt();
   CLIPSInputCount = 0;

/*   TkClipsFlag = 0;*/
   //while (CLIPS_TRUE && (Tk_GetNumMainWindows() > 0))
   while (CLIPS_TRUE )
     {
     
      /*===================================================*/
      /* If a batch file is active, grab the command input */
      /* directly from the batch file, otherwise call the  */
      /* event function.                                   */
      /*===================================================*/

      if (BatchActive() == CLIPS_TRUE)
        {
         inchar = LLGetcBatch("stdin",CLIPS_TRUE);
         if (inchar == EOF)
           { (*EventFunction)(); }
         else
           { ExpandCommandString((char) inchar); }
        }
      else
        { (*EventFunction)(); }

      if (GetHaltExecution() == CLIPS_TRUE)
        {
         SetHaltExecution(CLIPS_FALSE);
         SetEvaluationError(CLIPS_FALSE);
         FlushCommandString();
#if ! WINDOW_INTERFACE
         fflush(stdin);
#endif
         PrintCLIPS(WCLIPS,"\n");
         PrintPrompt();
        }

      if ((CompleteCommand(CommandString) != 0) && (CLIPSInputCount > 0))
        {
         FlushPPBuffer();
         SetPPBufferStatus(OFF);
         CLIPSInputCount = -1;
         RouteCommand(CommandString);
         FlushPPBuffer();
         SetHaltExecution(CLIPS_FALSE);
         SetEvaluationError(CLIPS_FALSE);
         FlushCommandString();
         FlushBindList();
         PeriodicCleanup(CLIPS_TRUE,CLIPS_FALSE);
         PrintPrompt();
        }
        
     }

    printf("TK-CLIPS is done!\n");
  }

/*************************************************/
/* PrintPrompt: Prints the CLIPS command prompt. */
/*************************************************/
globle VOID PrintPrompt()
   {
    PrintCLIPS(WCLIPS,"CLIPS> ");

    if (MemoryStatusFunction != NULL)
      { (*MemoryStatusFunction)(); }
   }

/********************************************************************************/
/* SetMemoryStatusFunction: Replaces the current value of MemoryStatusFunction. */
/********************************************************************************/
globle VOID SetMemoryStatusFunction(funptr)
  int (*funptr)(VOID_ARG);
  {
   MemoryStatusFunction = funptr;
  }

/************************************************/
/* RouteCommand: Processes a completed command. */
/************************************************/
globle BOOLEAN RouteCommand(command)
  char *command;
  {
   DATA_OBJECT result;
   struct expr *top;
   char *commandName;
   struct token theToken;

   if (command == NULL)
     { return(0); }

   OpenStringSource("command",command,0);

   GetToken("command",&theToken);

   if ((theToken.type == SYMBOL) || (theToken.type == STRING) ||
       (theToken.type == FLOAT) || (theToken.type == INTEGER))
     {
      CloseStringSource("command");
      PrintAtom("stdout",theToken.type,theToken.value);
      PrintCLIPS("stdout","\n");
      return(1);
     }

   if (theToken.type == GBL_VARIABLE)
     {
      CloseStringSource("command");
      top = GenConstant(theToken.type,theToken.value);
      EvaluateExpression(top,&result);
      rtn_struct(expr,top);
      PrintDataObject("stdout",&result);
      PrintCLIPS("stdout","\n");
      return(1);
     }

   if (theToken.type != LPAREN)
     {   
      PrintErrorID("COMMLINE",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Expected a '(', constant, or global variable\n");
      CloseStringSource("command");
      return(0);
     }

   GetToken("command",&theToken);
   if (theToken.type != SYMBOL)
     {
      PrintErrorID("COMMLINE",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Expected a command.\n");
      CloseStringSource("command");
      return(0);
     }

   commandName = ValueToString(theToken.value);

   /*======================*/
   /* Evaluate Constructs. */
   /*======================*/

#if (! RUN_TIME) && (! BLOAD_ONLY)
   {
    int errorFlag = CLIPS_FALSE;

    errorFlag = ParseConstruct(commandName,"command");
    if (errorFlag != -1)
      {
       CloseStringSource("command");
       if (errorFlag == 1)
         {
          PrintCLIPS(WERROR,"\nERROR:\n");
          PrintInChunks(WERROR,GetPPBuffer());
          PrintCLIPS(WERROR,"\n");
         }
       DestroyPPBuffer();
       return(errorFlag);
      }
   }
#endif

   /*======================*/
   /* Parse Function Call. */
   /*======================*/

   ParsingTopLevelCommand = CLIPS_TRUE;
   top = Function2Parse("command",commandName);
   ParsingTopLevelCommand = CLIPS_FALSE;
   ClearParsedBindNames();

   if (top == NULL)
     {
      CloseStringSource("command");
      return(0);
     }

   /*===================================*/
   /* Evaluate Top Level Function Call. */
   /*===================================*/

   EvaluatingTopLevelCommand = CLIPS_TRUE;
   EvaluateExpression(top,&result);
   EvaluatingTopLevelCommand = CLIPS_FALSE;
   ReturnExpression(top);

   if (result.type != RVOID)
     {
      PrintDataObject("stdout",&result);
      PrintCLIPS("stdout","\n");
     }

   CloseStringSource("command");
   return(1);
  }

/*****************************************************************/
/* DefaultGetNextEvent: Default event-handling function. Handles */
/*   only keyboard events by first calling GetcCLIPS to get a    */
/*   character and then calling ExpandCommandString to add the   */
/*   character to the CommandString.                             */
/*****************************************************************/
static VOID DefaultGetNextEvent()
  {
   int inchar;

   inchar = GetcCLIPS("stdin");

   if (inchar == EOF) inchar = '\n';

   ExpandCommandString((char) inchar);
  }

/******************************************************************/
/* SetEventFunction: Replaces the current value of EventFunction. */
/******************************************************************/
globle int (*SetEventFunction(theFunction))(VOID_ARG)
  int (*theFunction)(VOID_ARG);
  {
   int (*tmp_ptr)(VOID_ARG);

   tmp_ptr = EventFunction;
   EventFunction = theFunction;
   return(tmp_ptr);
  }

/***************************************************************************/
/* TopLevelCommand: Indicates whether a top-level command is being parsed. */
/***************************************************************************/
globle BOOLEAN TopLevelCommand()
  {
   return(ParsingTopLevelCommand);
  }

#endif



