   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 I/O FUNCTIONS MODULE                */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*      Gary D. Riley                                        */
/*      Bebe Ly                                              */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _IOFUN_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "router.h"
#include "strngrtr.h"
#include "filertr.h"
#include "argacces.h"
#include "extnfunc.h"
#include "scanner.h"
#include "constant.h"
#include "clipsmem.h"
#include "commline.h"
#include "sysdep.h"
#include "utility.h"

#define FORMAT_MAX 512
#define FLAG_MAX    80

/***************************************************/
/* GLOBAL AND LOCAL INTERNAL FUNCTION DEFINITIONS  */
/***************************************************/

#if ANSI_COMPILER
   VOID                    IOFunctionDefinitions(void);
#if BASIC_IO || EXT_IO
   static VOID             IllegalLogicalNameMessage(char *);
#endif
#if BASIC_IO
   VOID                    PrintoutFunction(void);
   VOID                    ReadFunction(DATA_OBJECT_PTR);
   int                     OpenFunction(void);
   int                     CloseFunction(void);
#endif
#if EXT_IO
   VOID                    ReadlineFunction(DATA_OBJECT_PTR);
   VOID                   *FormatFunction(void);
   int                     RemoveFunction(void);
   int                     RenameFunction(void);
   static char            *ControlStringCheck(int);
   static char             FindFormatFlag(char *,int *,char *,int *);
   static char            *PrintFormatFlag (char *,int,int,int);
   static char            *FillBuffer(char *,int *,int *);
#endif
#else
   VOID                    IOFunctionDefinitions();
#if BASIC_IO || EXT_IO
   static VOID             IllegalLogicalNameMessage();
#endif
#if BASIC_IO
   VOID                    PrintoutFunction();
   VOID                    ReadFunction();
   int                     OpenFunction();
   int                     CloseFunction();
#endif
#if EXT_IO
   VOID                    ReadlineFunction();
   int                     RemoveFunction();
   int                     RenameFunction();
   VOID                   *FormatFunction();
   static char            *ControlStringCheck();
   static char             FindFormatFlag();
   static char            *PrintFormatFlag ();
   static char            *FillBuffer();
#endif
#endif

#if ! RUN_TIME
/********************************************************************/
/* IOFunctionDefinitions                                            */
/********************************************************************/
globle VOID IOFunctionDefinitions()
  {
#if BASIC_IO
   DefineFunction2("printout",   'v', PTIF PrintoutFunction, "PrintoutFunction", "1*");
   DefineFunction2("read",       'u', PTIF ReadFunction,  "ReadFunction", "*1");
   DefineFunction2("open",       'b', OpenFunction,  "OpenFunction", "23*k");
   DefineFunction2("close",      'b', CloseFunction, "CloseFunction", "*1");
#endif

#if EXT_IO
   DefineFunction2("remove",       'b', RemoveFunction,  "RemoveFunction", "11k");
   DefineFunction2("rename",      'b', RenameFunction, "RenameFunction", "22k");
   DefineFunction2("format",     's', PTIF FormatFunction, "FormatFunction", "2**us");
   DefineFunction2("readline",   'k', PTIF ReadlineFunction,  "ReadlineFunction", "*1");
#endif
  }
#endif

#if BASIC_IO

/************************************************************/
/* PrintoutFunction: CLIPS function for unformatted output. */
/*   Syntax: (printout <logical-name> <expression>*)        */
/************************************************************/
globle VOID PrintoutFunction()
  {
   char *dummyid;
   int i, argc;
   DATA_OBJECT argPtr;

   /*=====================================================*/
   /* Get the logical name to which output is to be sent. */
   /*=====================================================*/

   if ((argc = ArgCountCheck("printout",AT_LEAST,1)) == -1) return;

   dummyid = GetLogicalName(1,"stdout");
   if (dummyid == NULL)
     {
      IllegalLogicalNameMessage("printout");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }

   /*============================================================*/
   /* Determine if any router recognizes the output destination. */
   /*============================================================*/
   
   if (QueryRouters(dummyid) == CLIPS_FALSE)
     {
      UnrecognizedRouterMessage(dummyid);
      return;
     }

   /*===============================================*/
   /* Print each of the arguments send to printout. */
   /*===============================================*/
   
   for (i = 2; i <= argc; i++)
     {
      RtnUnknown(i,&argPtr);
      if (HaltExecution)
        break;
      switch(GetType(argPtr))
        {
         case SYMBOL:
           if (strcmp(DOToString(argPtr),"crlf") == 0)
             { PrintCLIPS(dummyid,"\n"); }
           else if (strcmp(DOToString(argPtr),"tab") == 0)
             { PrintCLIPS(dummyid,"\t"); }
           else if (strcmp(DOToString(argPtr),"vtab") == 0)
             { PrintCLIPS(dummyid,"\v"); }
           else if (strcmp(DOToString(argPtr),"t") == 0)
             { PrintCLIPS(dummyid,"\n"); }
           else
             { PrintCLIPS(dummyid,DOToString(argPtr)); }
           break;

         case STRING:
           PrintCLIPS(dummyid,DOToString(argPtr));
           break;

         default:
           PrintDataObject(dummyid,&argPtr);
           break;
        }
     }
  }

/****************************************/
/* ReadFunction                                 */
/****************************************/
globle VOID ReadFunction(read_value)
  DATA_OBJECT_PTR read_value;
  {
   struct token theToken;
   int arg_no;
   char *dummyid = NULL;
   char *read_str;
   int max_char;
   int inchar;

   /*===============================================*/
   /* Check for an appropriate number of arguments. */
   /*===============================================*/

   if ((arg_no = ArgCountCheck("read",NO_MORE_THAN,1)) == -1)
     {
      read_value->type = STRING;
      read_value->value = (VOID *) AddSymbol("*** READ ERROR ***");
      return;
     }

   /*======================================================*/
   /* Determine the logical name from which input is read. */
   /*======================================================*/

   if (arg_no == 0)
     { dummyid = "stdin"; }
   else if (arg_no == 1)
     {
      dummyid = GetLogicalName(1,"stdin");
      if (dummyid == NULL)
        {
         IllegalLogicalNameMessage("read");
         SetHaltExecution(CLIPS_TRUE);
         SetEvaluationError(CLIPS_TRUE);
         read_value->type = STRING;
         read_value->value = (VOID *) AddSymbol("*** READ ERROR ***");
         return;
        }
     }

   if (QueryRouters(dummyid) == CLIPS_FALSE)
     {
      UnrecognizedRouterMessage(dummyid);
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      read_value->type = STRING;
      read_value->value = (VOID *) AddSymbol("*** READ ERROR ***");
      return;
     }

   /*====================================================*/
   /* Collect input into string if read source is stdin, */
   /* else just get token.                               */
   /*====================================================*/

   if (strcmp(dummyid,"stdin") == 0)
     {
      theToken.type = STOP;
      while (theToken.type == STOP)
        {
         read_str = NULL;
         max_char = CLIPSInputCount = 0;
         inchar = GetcCLIPS("stdin");
         while ((inchar != '\n') && (inchar != '\r') && (inchar != EOF) &&
                (!GetHaltExecution()))
           {
            read_str = ExpandStringWithChar(inchar,read_str,&CLIPSInputCount,
                                              &max_char,max_char + 80);
            inchar = GetcCLIPS("stdin");
           }

         OpenStringSource("read",read_str,0);
         GetToken("read",&theToken);
         CloseStringSource("read");
         if (max_char > 0) rm(read_str,max_char);

         if (GetHaltExecution())
           {
            theToken.type = STRING;
            theToken.value = (VOID *) AddSymbol("*** READ ERROR ***");
           }

         if ((theToken.type == STOP) && (inchar == EOF))
           {
            theToken.type = SYMBOL;
            theToken.value = (VOID *) AddSymbol("EOF");
           }
        }
     }
   else
     { GetToken(dummyid,&theToken); }

   CLIPSInputCount = -1;

   /*=======================*/
   /* Process return value. */
   /*=======================*/

   read_value->type = theToken.type;
   if ((theToken.type == FLOAT) || (theToken.type == STRING) ||
#if OBJECT_SYSTEM
       (theToken.type == INSTANCE_NAME) ||
#endif
       (theToken.type == SYMBOL) || (theToken.type == INTEGER))
     { read_value->value = theToken.value; }
   else if (theToken.type == STOP)
     {
      read_value->type = SYMBOL;
      read_value->value = (VOID *) AddSymbol("EOF");
     }
   else if (theToken.type == UNKNOWN)
     {
      read_value->type = STRING;
      read_value->value = (VOID *) AddSymbol("*** READ ERROR ***");
     }
   else
     {
      read_value->type = STRING;
      read_value->value = (VOID *) AddSymbol(theToken.printForm);
     }

   return;
  }

/*###################################*/
/*# FILE SYSTEM INTERFACE FUNCTIONS #*/
/*###################################*/

/**************************************************************/
/* OpenFunction: This function opens a file named by the user */
/*   and identifies it with a character string tag specified  */
/*   by the user.  This function returns a non-zero value if  */
/*   the file was successfully opened.                        */
/**************************************************************/
globle int OpenFunction()
  {
   int arg_no, status;
   char *newfilename, *newfileid, *newmode = NULL;
   DATA_OBJECT arg_ptr;

   /*======================================*/
   /* Check for valid number of arguments. */
   /*======================================*/

   if ((arg_no = ArgRangeCheck("open",2,3)) == -1) return(0);

   /*==========================*/
   /* Check for the file name. */
   /*==========================*/

   if ((newfilename = GetFileName("open",1)) == NULL) return(0);

   /*=============================*/
   /* Check for the logical name. */
   /*=============================*/

   newfileid = GetLogicalName(2,NULL);
   if (newfileid == NULL)
     {
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      IllegalLogicalNameMessage("open");
      return(0);
     }

   if (FindFile(newfileid))
     {
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      PrintErrorID("IOFUN",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Logical name ");
      PrintCLIPS(WERROR,newfileid);
      PrintCLIPS(WERROR," already in use.\n");
      return(0);
     }

   /*===================================*/
   /* Check for valid file access mode. */
   /*===================================*/

   if (arg_no == 2)
     { newmode = "r"; }
   else if (arg_no == 3)
     {
      if (ArgTypeCheck("open",3,STRING,&arg_ptr) == CLIPS_FALSE) return(0);
      newmode = DOToString(arg_ptr);
     }

   if ((strcmp(newmode,"r") != 0) &&
       (strcmp(newmode,"r+") != 0) &&
       (strcmp(newmode,"w") != 0) &&
       (strcmp(newmode,"a") != 0))
     {
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      ExpectedTypeError1("open",3,"string with value \"r\", \"r+\", \"w\", or \"a\"");
      return(0);
     }

   if (strcmp(newmode,"r") == 0) newmode = "r";
   else if (strcmp(newmode,"r+") == 0) newmode = "r+";
   
   /*======================================================*/
   /* Open named file and store it with named tag on list. */
   /*======================================================*/

   status = OpenFile(newfilename,newmode,newfileid);
   return(status);
  }


/***********************************************************************/
/* CloseFunction:  This function closes the file stream with the file  */
/*   id specified by the user, if such a stream exists.  This function */
/*   returns a non-zero value if the file was successfully closed.     */
/***********************************************************************/
globle int CloseFunction()
  {
   int arg_no;
   char *fileid;

   /*======================================================*/
   /* Check for valid number of arguments and assignments. */
   /*======================================================*/

   if ((arg_no = ArgCountCheck("close",NO_MORE_THAN,1)) == -1) return(0);

   if (arg_no == 0) return(CloseAllFiles());

   fileid = GetLogicalName(1,NULL);
   if (fileid == NULL)
     {
      IllegalLogicalNameMessage("close");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(0);
     }

   return(CloseFile(fileid));
  }


#endif

#if EXT_IO

/******************************************************/
/* RemoveFunction: CLIPS command for removing a file. */
/*   Returns TRUE if the specified file is deleted.   */
/*   Syntax: (remove <file-name>)                     */
/******************************************************/
globle int RemoveFunction()
  {
   char *theFileName;

   /*======================================*/
   /* Check for valid number of arguments. */
   /*======================================*/

   if (ArgCountCheck("remove",EXACTLY,1) == -1) return(CLIPS_FALSE);

   /*==========================*/
   /* Check for the file name. */
   /*==========================*/

   if ((theFileName = GetFileName("remove",1)) == NULL) return(CLIPS_FALSE);
   
   /*==================*/
   /* Remove the file. */
   /*==================*/
   
   return(genremove(theFileName));
  }
  
/******************************************************/
/* RenameFunction: CLIPS command for renaming a file. */
/*   Returns TRUE if the specified file is renamed.   */
/*   Syntax: (rename <old-file-name> <new-file-name>) */
/******************************************************/
globle int RenameFunction()
  {
   char *oldFileName, *newFileName;

   /*======================================*/
   /* Check for valid number of arguments. */
   /*======================================*/

   if (ArgCountCheck("rename",EXACTLY,2) == -1) return(CLIPS_FALSE);

   /*===========================*/
   /* Check for the file names. */
   /*===========================*/

   if ((oldFileName = GetFileName("rename",1)) == NULL) return(CLIPS_FALSE);
   if ((newFileName = GetFileName("rename",2)) == NULL) return(CLIPS_FALSE);
   
   /*==================*/
   /* Remove the file. */
   /*==================*/
   
   return(genrename(oldFileName,newFileName));
  }
  
/*******************************************************************/
/* FormatFunction: Implements the format function.                 */
/*   Syntax: (format <logical-name> <format-string> <parameters>*) */
/*******************************************************************/
globle VOID *FormatFunction()
  {
   int argCount, start_pos;
   char *formatString, *logicalName;
   char formatFlagType;
   int  f_cur_arg = 3;
   int form_pos = 0;
   char buffer[FORMAT_MAX];
   char percentBuffer[FLAG_MAX];
   char *fstr = NULL;
   int fmax = 0, fpos = 0;
   VOID *hptr;
   int longFound;
   char *theString;
   
   /*======================================*/
   /* Set default return value for errors. */
   /*======================================*/

   hptr = AddSymbol("");

   /*=========================================*/
   /* Format requires at least two arguments: */
   /* a logical name and a format string.     */
   /*=========================================*/

   if ((argCount = ArgCountCheck("format",AT_LEAST,2)) == -1)
     { return(hptr); }

   /*========================================*/
   /* First argument must be a logical name. */
   /*========================================*/

   if ((logicalName = GetLogicalName(1,"stdout")) == NULL)
     {
      IllegalLogicalNameMessage("format");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(hptr);
     }

   if (strcmp(logicalName,"nil") == 0)
     { /* do nothing */ }
   else if (QueryRouters(logicalName) == CLIPS_FALSE)
     {
      UnrecognizedRouterMessage(logicalName);
      return(hptr);
     }

   /*=====================================================*/
   /* Second argument must be a string.  The appropriate  */
   /* number of arguments specified by the string must be */
   /* present in the argument list.                       */
   /*=====================================================*/

   if ((formatString = ControlStringCheck (argCount)) == NULL)
     { return (hptr); }

   /*==============================================*/
   /* Locate a string of 80 character for scanning */
   /* sub_string from control_string               */
   /*==============================================*/

   /* Scanning and print the format */

   while (formatString[form_pos] != '\0')
     {
      if (formatString[form_pos] != '%')
        {
         start_pos = form_pos;
         while ((formatString[form_pos] != '%') &&
                (formatString[form_pos] != '\0') &&
                ((form_pos - start_pos) < FLAG_MAX))
           { form_pos++; }
         fstr = AppendNToString(&formatString[start_pos],fstr,form_pos-start_pos,&fpos,&fmax);
        }
      else
        {
         start_pos = form_pos;
         form_pos++;
         formatFlagType = FindFormatFlag(formatString,&form_pos,buffer,&longFound);
         if (formatFlagType != ' ')
           {
            strncpy(percentBuffer,&formatString[start_pos],form_pos-start_pos);
            percentBuffer[form_pos-start_pos] = EOS;
            if ((! longFound) &&
                ((formatFlagType == 'd') || (formatFlagType == 'o') ||
                 (formatFlagType == 'u') || (formatFlagType == 'x')))
              {
               longFound = CLIPS_TRUE;
               percentBuffer[(form_pos-start_pos) - 1] = 'l';
               percentBuffer[form_pos-start_pos] = formatFlagType;
               percentBuffer[(form_pos-start_pos) + 1] = EOS;
              }
              
            if ((theString = PrintFormatFlag(percentBuffer,f_cur_arg,formatFlagType,longFound)) == NULL)
              {
               if (fstr != NULL) rm(fstr,fmax);
               return (hptr);
              }
            fstr = AppendToString(theString,fstr,&fpos,&fmax);
            if (fstr == NULL) return(hptr);
            f_cur_arg++;
           }
         else
           {
            fstr = AppendToString(buffer,fstr,&fpos,&fmax);
            if (fstr == NULL) return(hptr);
           }
        }
     }

   if (fstr != NULL)
     {
      hptr = AddSymbol(fstr);
      if (strcmp(logicalName,"nil") != 0) PrintCLIPS(logicalName,fstr);
      rm(fstr,fmax);
     }
   else
     { hptr = AddSymbol(""); }

   return(hptr);
  }

/*********************************************************************/
/* ControlStringCheck:  Checks the 2nd parameter which is the format */
/*   control string to see if there are enough matching arguments.   */
/*********************************************************************/
static char *ControlStringCheck(argCount)
  int argCount;
  {
   DATA_OBJECT t_ptr;
   char *str_array;
   char print_buff[10];
   int longFound;

   int i,per_count;

   if (ArgTypeCheck("format",2,STRING,&t_ptr) == CLIPS_FALSE) return(NULL);

   per_count = 0;
   str_array = ValueToString(t_ptr.value);
   for (i= 0 ; str_array[i] != '\0' ; )
     {
      if (str_array[i] == '%')
        {
         i++;
         if (FindFormatFlag(str_array,&i,print_buff,&longFound) != ' ')
           { per_count++; }
        }
      else
        { i++; }
     }

   if (per_count != (argCount - 2))
     {
      ExpectedCountError("format",EXACTLY,per_count+2);
      SetEvaluationError(CLIPS_TRUE);
      return (NULL);
     }

   return(str_array);
  }

/***********************************************/
/* FindFormatFlag:  This function searches for */
/*   a format flag in the format string.       */
/***********************************************/
static char FindFormatFlag(formatString,a,formatBuffer,longFound)
  char *formatString;
  int *a;
  char *formatBuffer;
  int *longFound;
  {
   int found_type = CLIPS_FALSE;
   char inchar, formatFlagType;
   int start_pos, copy_pos = 0;

   /*===========================================================*/
   /* Set return values to the default value. A blank character */
   /* indicates that no format flag was found which requires a  */
   /* parameter. The longFound flag indicates whether the       */
   /* character 'l' was used with the float or integer flag to  */
   /* indicate a double precision float or a long integer.      */
   /*===========================================================*/
   
   formatFlagType = ' ';
   *longFound = CLIPS_FALSE;

   /*=====================================================*/
   /* The format flags for carriage returns, line feeds,  */
   /* horizontal and vertical tabs, and the percent sign, */
   /* do not require a parameter.                         */
   /*=====================================================*/
   
   if (formatString[*a] == 'n')
     {
      sprintf(formatBuffer,"\n");
      (*a)++;
      return(formatFlagType);
     }
   else if (formatString[*a] == 'r')
     {
      sprintf(formatBuffer,"\r");
      (*a)++;
      return(formatFlagType);
     }
   else if (formatString[*a] == 't')
     {
      sprintf(formatBuffer,"\t");
      (*a)++;
      return(formatFlagType);
     }
   else if (formatString[*a] == 'v')
     {
      sprintf(formatBuffer,"\v");
      (*a)++;
      return(formatFlagType);
     }
   else if (formatString[*a] == '%')
     {
      sprintf(formatBuffer,"%%");
      (*a)++;
      return(formatFlagType);
     }

   /*======================================================*/
   /* Identify the format flag which requires a parameter. */
   /*======================================================*/
   
   start_pos = *a;
   formatBuffer[copy_pos] = '\0';
   while ((formatString[*a] != '%') &&
          (formatString[*a] != '\0') &&
          ((*a - start_pos) < FLAG_MAX))
     {
      inchar = formatString[*a];
      formatBuffer[copy_pos++] = inchar;
      formatBuffer[copy_pos] = '\0';
      if ( (found_type == CLIPS_FALSE) &&
           ( (inchar == 'd') ||
             (inchar == 'o') ||
             (inchar == 'x') ||
             (inchar == 'u') ||
             (inchar == 'c') ||
             (inchar == 's') ||
             (inchar == 'e') ||
             (inchar == 'f') ||
             (inchar == 'g') ) )
        {
         found_type = CLIPS_TRUE;
         formatFlagType = inchar;
         if (formatString[(*a) - 1] == 'l') 
           { *longFound = CLIPS_TRUE; }
         (*a)++;
         return(formatFlagType);
        }
      (*a)++;
     }

   return(formatFlagType);
  }

/**********************************************************************/
/* PrintFormatFlag:  Prints out part of the total format string along */
/*   with the argument for that part of the format string.            */
/**********************************************************************/
static char *PrintFormatFlag (formatString,whichArg,formatType,longFound)
  char *formatString;
  int whichArg;
  int formatType;
  int longFound;
  {
   DATA_OBJECT theResult;
   char *theString, *printBuffer;
   int theLength;

   /*=================*/
   /* String argument */
   /*=================*/

   switch (formatType)
     {
      case 's': 
        if (ArgTypeCheck("format",whichArg,SYMBOL_OR_STRING,&theResult) == CLIPS_FALSE) return(NULL);
        theLength = strlen(formatString) + strlen(ValueToString(theResult.value)) + 200;
        printBuffer = (char *) gm2 (((int) sizeof(char) * theLength));
        sprintf(printBuffer,formatString,ValueToString(theResult.value));
        break;
        
      case 'c': 
        if (ArgTypeCheck("format",whichArg,SYMBOL_OR_STRING,&theResult) == CLIPS_FALSE) return(NULL);
        theLength = strlen(formatString) + 200;
        printBuffer = (char *) gm2 (((int) sizeof(char) * theLength));
        sprintf(printBuffer,formatString,(ValueToString(theResult.value))[0]);
        break;
        
      case 'd':
      case 'x':
      case 'o':
      case 'u':
        if (ArgTypeCheck("format",whichArg,INTEGER_OR_FLOAT,&theResult) == CLIPS_FALSE) return(NULL);
        theLength = strlen(formatString) + 200;
        printBuffer = (char *) gm2 (((int) sizeof(char) * theLength));
        if (GetType(theResult) == FLOAT)
          {
           if (longFound)
             { sprintf(printBuffer,formatString,(long) ValueToDouble(theResult.value)); }
           else
             { sprintf(printBuffer,formatString,(int) ValueToDouble(theResult.value)); }
          }
        else
          {
           if (longFound)
             { sprintf(printBuffer,formatString,(long) ValueToLong(theResult.value)); }
           else
             { sprintf(printBuffer,formatString,(int) ValueToLong(theResult.value)); }
          }
        break;
        
      case 'f':
      case 'g':
      case 'e':
        if (ArgTypeCheck("format",whichArg,INTEGER_OR_FLOAT,&theResult) == CLIPS_FALSE) return(NULL);
        theLength = strlen(formatString) + 200;
        printBuffer = (char *) gm2 (((int) sizeof(char) * theLength));
        
        if (GetType(theResult) == FLOAT)
          { sprintf(printBuffer,formatString,ValueToDouble(theResult.value)); }
        else
          { sprintf(printBuffer,formatString,(double) ValueToLong(theResult.value)); }
        break;
        
      default:
         PrintCLIPS (WERROR," Error in format, the conversion character");
         PrintCLIPS (WERROR," for formatted output is not valid\n");
         return(CLIPS_FALSE);
         break;
     }

   theString = ValueToString(AddSymbol(printBuffer));
   rm(printBuffer,(int) sizeof(char) * theLength);
   return(theString);
  }

/****************************************************************/
/* ReadlineFunction:                                                    */
/****************************************************************/
globle VOID ReadlineFunction(rlnval)
  DATA_OBJECT_PTR rlnval;
  {
   char *buffer;
   int line_max = 0;
   int num;

   char *logicalName;
   
   rlnval->type = STRING;

   if ((num = ArgCountCheck("readline",NO_MORE_THAN,1)) == -1)
     {
      rlnval->value = (VOID *) AddSymbol("*** READ ERROR ***");
      return;
     }

   if (num == 0 )
     { logicalName = "stdin"; }
   else
     {
      logicalName = GetLogicalName(1,"stdin");
      if (logicalName == NULL)
        {
         IllegalLogicalNameMessage("readline");
         SetHaltExecution(CLIPS_TRUE);
         SetEvaluationError(CLIPS_TRUE);
         rlnval->value = (VOID *) AddSymbol("*** READ ERROR ***");
         return;
        }
     }

   if (QueryRouters(logicalName) == CLIPS_FALSE)
     {
      UnrecognizedRouterMessage(logicalName);
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      rlnval->value = (VOID *) AddSymbol("*** READ ERROR ***");
      return;
     }

   CLIPSInputCount = 0;
   buffer = FillBuffer(logicalName,&CLIPSInputCount,&line_max);
   CLIPSInputCount = -1;

   if (GetHaltExecution())
     {
      rlnval->value = (VOID *) AddSymbol("*** READ ERROR ***");
      if (buffer != NULL) rm(buffer,(int) sizeof (char) * line_max);
      return;
     }

   if (buffer == NULL)
     {
      rlnval->value = (VOID *) AddSymbol("EOF");
      rlnval->type = SYMBOL;
      return;
     }

   rlnval->value = (VOID *) AddSymbol(buffer);
   rm(buffer,(int) sizeof (char) * line_max);
   return;
  }

/*****************************************************/
/* FillBuffer                                       */
/*****************************************************/
static char *FillBuffer(logicalName,buf_pos,buf_max)
  char *logicalName;
  int *buf_pos, *buf_max;
  {
   int c;
   char *buf = NULL;

   /*================================*/
   /* Read until end of line or eof. */
   /*================================*/

   c = GetcCLIPS(logicalName);

   if (c == EOF)
     { return(NULL); }

   /*==================================*/
   /* Grab characters until cr or eof. */
   /*==================================*/

   while ((c != '\n') && (c != '\r') && (c != EOF) &&
          (! GetHaltExecution()))
     {
      buf = ExpandStringWithChar(c,buf,buf_pos,buf_max,*buf_max+80);
      c = GetcCLIPS(logicalName);
     }

   /*==================*/
   /* Add closing EOS. */
   /*==================*/

   buf = ExpandStringWithChar(EOS,buf,buf_pos,buf_max,*buf_max+80);
   return (buf);
  }
  
#endif

#if BASIC_IO || EXT_IO

/*****************************************************/
/* IllegalLogicalNameMessage:            */
/*****************************************************/
static VOID IllegalLogicalNameMessage(theFunction)
  char *theFunction;
  {
   PrintErrorID("IOFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Illegal logical name used for ");
   PrintCLIPS(WERROR,theFunction);
   PrintCLIPS(WERROR," function.\n");
  }

#endif
