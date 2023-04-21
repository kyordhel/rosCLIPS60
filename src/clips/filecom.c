
   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 FILE COMMANDS MODULE                */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Bebe Ly                                              */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _FILECOM_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "clipsmem.h"
#include "argacces.h"
#include "router.h"
#include "strngrtr.h"
#include "constrct.h"
#include "extnfunc.h"
#include "cstrcpsr.h"
#include "utility.h"
#include "filecom.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bsave.h"
#include "bload.h"
#endif

#if ANSI_COMPILER
#if DEBUGGING_FUNCTIONS
   static int                     FindDribble(char *);
   static int                     GetcDribble(char *);
   static int                     UngetcDribble(int,char *);
   static int                     ExitDribble(int);
   static int                     PrintDribble(char *,char *);
   static VOID                    PutcDribbleBuffer(int);
#endif
   static int                     FindBatch(char *);
   static int                     GetcBatch(char *);
   static int                     UngetcBatch(int,char *);
   static int                     ExitBatch(int);
   static VOID                    AddBatch(int,VOID *,int,char *);
#else
#if DEBUGGING_FUNCTIONS
   static int                     PrintDribble();
   static VOID                    PutcDribbleBuffer();
   static int                     FindDribble();
   static int                     GetcDribble();
   static int                     UngetcDribble();
   static int                     ExitDribble();
#endif
   static int                     FindBatch();
   static int                     GetcBatch();
   static int                     UngetcBatch();
   static int                     ExitBatch();
   static VOID                    AddBatch();
#endif

/***********************************************************/
/* batchEntry structure:                                   */
/***********************************************************/

#define FILE_BATCH   0
#define STRING_BATCH 1

struct batchEntry
  {
   int batchType;
   VOID *inputSource;
   char *theString;
   struct batchEntry *next;
  };

/*********************************/
/* DRIBBLE AND BATCH DEFINITIONS */
/*********************************/

#define BUFFSIZE       120          /* Buffer allocation increments  */

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if DEBUGGING_FUNCTIONS
   static FILE               *DribbleFP = NULL;
   static char               *DribbleBuffer;
   static int                 DribbleCurrentPosition = 0;
   static int                 DribbleMaximumPosition = 0;
#if ANSI_COMPILER
   static int               (*DribbleStatusFunction)(int) = NULL;
#else
   static int               (*DribbleStatusFunction)() = NULL;
#endif
#endif
   static int                 BatchType;
   static VOID               *BatchSource = NULL;
   static char               *BatchBuffer;
   static int                 bat_loc = 0;
   static int                 bat_max = 0;
   static struct batchEntry  *batch_top = NULL;
   static struct batchEntry  *batch_bot = NULL;

/****************************/
/* FileFunctionDefinitions: */
/****************************/
#if ! RUN_TIME
globle VOID FileFunctionDefinitions()
  {
#if DEBUGGING_FUNCTIONS
   DefineFunction2("batch",         'b', PTIF BatchCommand,        "BatchCommand", "11k");
   DefineFunction2("dribble-on",    'b', PTIF DribbleOnCommand,    "DribbleOnCommand", "11k");
   DefineFunction2("dribble-off",   'b', PTIF DribbleOffCommand,   "DribbleOffCommand", "00");
   DefineFunction2("save",          'b', PTIF SaveCommand,         "SaveCommand", "11k");
#endif
   DefineFunction2("load",          'b', PTIF LoadCommand,         "LoadCommand", "11k");
#if BLOAD_AND_BSAVE
   DefineFunction2("bsave",'b', PTIF BsaveCommand,"BsaveCommand", "11k");
#endif
#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   DefineFunction2("bload",'b', PTIF BloadCommand,"BloadCommand", "11k");
#endif
  }
#endif

/*********************/
/* TRACING FUNCTIONS */
/*********************/

#if DEBUGGING_FUNCTIONS
/*************************************************/
/* FindDribble:                                   */
/*************************************************/
static int FindDribble(logicalName)
  char *logicalName;
  {
   if ( (strcmp(logicalName,"stdout") == 0) ||
        (strcmp(logicalName,"stdin") == 0) ||
        (strcmp(logicalName,WCLIPS) == 0) ||
        (strcmp(logicalName,WTRACE) == 0) ||
        (strcmp(logicalName,WERROR) == 0) ||
        (strcmp(logicalName,WWARNING) == 0) ||
        (strcmp(logicalName,WDISPLAY) == 0) ||
        (strcmp(logicalName,WDIALOG) == 0) )
     { return(CLIPS_TRUE); }

    return(CLIPS_FALSE);
  }

/*************************************************/
/* PrintDribble:                                  */
/*************************************************/
static int PrintDribble(logicalName,str)
  char *logicalName, *str;
  {
   int i = 0;

   /*======================================*/
   /* Send the output to the dribble file. */
   /*======================================*/

   for (i = 0 ; str[i] != EOS ; i++)
     { PutcDribbleBuffer(str[i]); }

   /*===========================================================*/
   /* Send the output to any routers interested in printing it. */
   /*===========================================================*/

   DeactivateRouter("dribble");
   PrintCLIPS(logicalName,str);
   ActivateRouter("dribble");
   
   return(1);
  }

/*************************************************/
/* GetcDribble:                                   */
/*************************************************/
static int GetcDribble(logicalName)
  char *logicalName;
  {
   int rv;

   DeactivateRouter("dribble");
   rv = GetcCLIPS(logicalName);

   ActivateRouter("dribble");

   PutcDribbleBuffer(rv);

   return(rv);
  }

/*************************************************/
/* PutcDribbleBuffer:                            */
/*************************************************/
static VOID PutcDribbleBuffer(rv)
  int rv;
  {
   if (rv == EOF)
     {
      if (DribbleCurrentPosition > 0)
        {
         fprintf(DribbleFP,"%s",DribbleBuffer);
         DribbleCurrentPosition = 0;
         DribbleBuffer[0] = EOS;
        }
     }
   else if (CLIPSInputCount < 0)
     {
      if (DribbleCurrentPosition > 0)
        {
         fprintf(DribbleFP,"%s",DribbleBuffer);
         DribbleCurrentPosition = 0;
         DribbleBuffer[0] = EOS;
        }

      fputc(rv,DribbleFP);
     }
   else
     {
      DribbleBuffer = ExpandStringWithChar(rv,DribbleBuffer,
                                           &DribbleCurrentPosition,
                                           &DribbleMaximumPosition,
                                           DribbleMaximumPosition+BUFFSIZE);
     }
  }

/*************************************************/
/* UngetcDribble:                                 */
/*************************************************/
static int UngetcDribble(ch,logicalName)
  int ch;
  char *logicalName;
  {
   int rv;

   DeactivateRouter("dribble");
   if (DribbleCurrentPosition > 0) DribbleCurrentPosition--;
   DribbleBuffer[DribbleCurrentPosition] = EOS;
   rv = UngetcCLIPS(ch,logicalName);
   ActivateRouter("dribble");

   return(rv);
  }

/*************************************************/
/* ExitDribble:                                   */
/*************************************************/
#if IBM_TBC
#pragma argsused
#endif
static int ExitDribble(num)
  int num;
  {
#if MAC_MPW
#pragma unused(num)
#endif
   if (DribbleCurrentPosition > 0)
     { fprintf(DribbleFP,"%s",DribbleBuffer); }

   if (DribbleFP != NULL) fclose(DribbleFP);
   return(1);
  }

/******************************************/
/* DribbleOnCommand: CLIPS access routine */
/*   for the dribble-on command.          */
/******************************************/
globle int DribbleOnCommand()
  {
   char *fileName;

   if (ArgCountCheck("dribble-on",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if ((fileName = GetFileName("dribble-on",1)) == NULL) return(CLIPS_FALSE);

   return (DribbleOn(fileName));
  }

/***********************************/
/* DribbleOn: C access routine for */
/*   the dribble-on command.       */
/***********************************/
globle BOOLEAN DribbleOn(fileName)
  char *fileName;
  {
   if (DribbleFP != NULL)
     { DribbleOff(); }

   DribbleFP = fopen(fileName,"w");
   if (DribbleFP == NULL)
     {
      OpenErrorMessage("dribble-on",fileName);
      return(0);
     }

   AddRouter("dribble", 40,
             FindDribble, PrintDribble,
             GetcDribble, UngetcDribble,  
             ExitDribble);  

   DribbleCurrentPosition = 0;

   if (DribbleStatusFunction != NULL)
     { (*DribbleStatusFunction)(CLIPS_TRUE); }

   return(1);
  }

/*************************************************/
/* DribbleActive:                               */
/*************************************************/
globle BOOLEAN DribbleActive()
  {
   if (DribbleFP != NULL) return(CLIPS_TRUE);

   return(CLIPS_FALSE);
  }

/*******************************************/
/* DribbleOffCommand: CLIPS access routine */
/*   for the dribble-off command.          */
/*******************************************/
globle int DribbleOffCommand()
  {
   if (ArgCountCheck("dribble-off",EXACTLY,0) == -1) return(CLIPS_FALSE);
   return(DribbleOff());
  }

/************************************/
/* DribbleOff: C access routine for */
/*   the dribble-off command.       */
/************************************/
globle BOOLEAN DribbleOff()
  {
   int rv = 0;

   if (DribbleStatusFunction != NULL)
     { (*DribbleStatusFunction)(CLIPS_FALSE); }

   if (DribbleFP != NULL)
     {
      if (DribbleCurrentPosition > 0)
        { fprintf(DribbleFP,"%s",DribbleBuffer); }
      DeleteRouter("dribble");
      if (fclose(DribbleFP) == 0) rv = 1;
     }
   else
     { rv = 1; }

   DribbleFP = NULL;

   if (DribbleBuffer != NULL)
     {
      rm(DribbleBuffer,DribbleMaximumPosition);
      DribbleBuffer = NULL;
     }

   DribbleCurrentPosition = 0;
   DribbleMaximumPosition = 0;

   return(rv);
  }

/*******************************/
/* SetDribbleStatusFunction */
/*******************************/
globle VOID SetDribbleStatusFunction(fnptr)
#if ANSI_COMPILER
  int (*fnptr)(int);
#else
  int (*fnptr)();
#endif
  {
   DribbleStatusFunction = fnptr;
  }
#endif

/*******************/
/* BATCH FUNCTIONS */
/*******************/


/*************************************************/
/* FindBatch:                                   */
/*************************************************/
static int FindBatch(logicalName)
  char *logicalName;
  {
   if (strcmp(logicalName,"stdin") == 0)
     { return(CLIPS_TRUE); }

   return(CLIPS_FALSE);
  }

/*************************************************/
/* GetcBatch:                                   */
/*************************************************/
static int GetcBatch(logicalName)
  char *logicalName;
  {
   return(LLGetcBatch(logicalName,CLIPS_FALSE));
  }

/*************************************************/
/* LLGetcBatch:                                   */
/*************************************************/
globle int LLGetcBatch(logicalName,return_on_eof)
  char *logicalName;
  int return_on_eof;
  {
   int rv = EOF, flag = 1;

   /*=================================================*/
   /* Get a character until a valid character appears */
   /* or no more batch files are left.                */
   /*=================================================*/

   while ((rv == EOF) && (flag == 1))
     {
      if (BatchType == FILE_BATCH)
        { rv = getc((FILE *) BatchSource); }
      else
        { rv = GetcCLIPS((char *) BatchSource); }

      if (rv == EOF)
        {
         if (bat_loc > 0) PrintCLIPS("stdout",(char *) BatchBuffer);
         flag = RemoveBatch();
        }
     }

   if (rv == EOF)
     {
      if (bat_loc > 0) PrintCLIPS("stdout",(char *) BatchBuffer);
      DeleteRouter("batch");
      RemoveBatch();
      if (return_on_eof == CLIPS_TRUE)
        { return (EOF); }
      else
        { return(GetcCLIPS(logicalName)); }
     }

   BatchBuffer = ExpandStringWithChar((char) rv,BatchBuffer,&bat_loc,
                                      &bat_max,bat_max+BUFFSIZE);

   if ((char) rv == '\n')
     {
      PrintCLIPS("stdout",(char *) BatchBuffer);
      bat_loc = 0;
      if ((BatchBuffer != NULL) && (bat_max > BUFFSIZE))
        {
         rm(BatchBuffer,bat_max);
         bat_max = 0;
         BatchBuffer = NULL;
        }
     }

   return(rv);
  }

/*************************************************/
/* UngetcBatch:                                  */
/*************************************************/
#if IBM_TBC
#pragma argsused
#endif
static int UngetcBatch(ch,logicalName)
  int ch;
  char *logicalName;
  {
#if MAC_MPW
#pragma unused(logicalName);
#endif
   if (bat_loc > 0) bat_loc--;
   if (BatchBuffer != NULL) BatchBuffer[bat_loc] = EOS;
   if (BatchType == FILE_BATCH)
     { return(ungetc(ch,(FILE *) BatchSource)); }

   return(UngetcCLIPS(ch,(char *) BatchSource));
  }

/*************************************************/
/* ExitBatch:                                    */
/*************************************************/
#if IBM_TBC
#pragma argsused
#endif
static int ExitBatch(num)
  int num;
  {
#if MAC_MPW
#pragma unused(num);
#endif
   CloseAllBatchSources();
   return(1);
  }

/*************************************************/
/* BatchCommand:                                 */
/*************************************************/
globle int BatchCommand()
  {
   char *fileName;

   if (ArgCountCheck("batch",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if ((fileName = GetFileName("batch",1)) == NULL) return(CLIPS_FALSE);

   return(OpenBatch(fileName,CLIPS_FALSE));
  }
  
/*************************************************/
/* Batch:                                 */
/*************************************************/
globle int Batch(fileName)
  char *fileName;
  { return(OpenBatch(fileName,CLIPS_FALSE)); }

/*************************************************/
/* OpenBatch:                                   */
/*************************************************/
globle int OpenBatch(fileName,placeAtEnd)
  char *fileName;
  int placeAtEnd;
  {
   FILE *temp_fp;

   temp_fp = fopen(fileName,"r");

   if (temp_fp == NULL)
     {
      OpenErrorMessage("batch",fileName);
      return(0);
     }

   if (batch_top == NULL)
     {
      AddRouter("batch", 20,                
                 FindBatch, NULL,              
                 GetcBatch, UngetcBatch,       
                 ExitBatch);       
     }

   AddBatch(placeAtEnd,(VOID *) temp_fp,FILE_BATCH,NULL);

   return(1);
  }

/*****************************************************************/
/* OpenStringBatch: Opens a string source for batch processing.  */
/*   The memory allocated for the argument stringName must be    */
/*   deallocated by the user. The memory allocated for theString */
/*   will be deallocated by the batch routines when batch        */
/*   processing for the  string is completed.                    */
/*****************************************************************/
globle int OpenStringBatch(stringName,theString,placeAtEnd)
  char *stringName;
  char *theString;
  int placeAtEnd;
  {
   if (OpenStringSource(stringName,theString,0) == 0)
     { return(0); }

   if (batch_top == NULL)
     {
      AddRouter("batch", 20,              
                 FindBatch, NULL,            
                 GetcBatch, UngetcBatch,       
                 ExitBatch);      
     }

   AddBatch(placeAtEnd,(VOID *) stringName,STRING_BATCH,theString);

   return(1);
  }

/***************************************************************/
/* AddBatch                                                   */
/***************************************************************/
static VOID AddBatch(placeAtEnd,theSource,type,theString)
  int placeAtEnd;
  VOID *theSource;
  int type;
  char *theString;
  {
   struct batchEntry *bptr;

   /*=========================*/
   /* Create the batch entry, */
   /*=========================*/

   bptr = get_struct(batchEntry);
   bptr->batchType = type;
   bptr->inputSource = theSource;
   bptr->theString = theString;
   bptr->next = NULL;

   if (batch_top == NULL)
     {
      batch_top = bptr;
      batch_bot = bptr;
      BatchType = type;
      BatchSource = theSource;
      bat_loc = 0;
     }
   else if (placeAtEnd == CLIPS_FALSE)
     {
      bptr->next = batch_top;
      batch_top = bptr;
      BatchType = type;
      BatchSource = theSource;
      bat_loc = 0;
     }
   else
     {
      batch_bot->next = bptr;
      batch_bot = bptr;
     }
  }

/***************************************************************/
/* RemoveBatch                                                */
/***************************************************************/
globle int RemoveBatch()
  {
   struct batchEntry *bptr;
   int rv;

   if (batch_top == NULL) return(0);

   if (batch_top->batchType == FILE_BATCH)
     { fclose((FILE *) batch_top->inputSource); }
   else
     {
      CloseStringSource((char *) batch_top->inputSource);
      rm(batch_top->theString,(int) strlen(batch_top->theString) + 1);
     }

   bptr = batch_top;
   batch_top = batch_top->next;

   rtn_struct(batchEntry,bptr);

   if (batch_top == NULL)
     {
      batch_bot = NULL;
      BatchSource = NULL;
      if (BatchBuffer != NULL)
        {
         rm(BatchBuffer,bat_max);
         BatchBuffer = NULL;
        }
      bat_loc = 0;
      bat_max = 0;
      rv = 0;
     }
   else
     {
      BatchType = batch_top->batchType;
      BatchSource = batch_top->inputSource;
      bat_loc = 0;
      rv = 1;
     }

   return(rv);
  }

/*************************************************/
/* BatchActive: */
/*************************************************/
globle BOOLEAN BatchActive()
  {
   if (batch_top != NULL) return(CLIPS_TRUE);

   return(CLIPS_FALSE);
  }

/*************************************************/
/* CloseAllBatchSources: */
/*************************************************/
globle VOID CloseAllBatchSources()
  {
   if (BatchBuffer != NULL)
     {
      if (bat_loc > 0) PrintCLIPS("stdout",(char *) BatchBuffer);
      rm(BatchBuffer,bat_max);
      BatchBuffer = NULL;
      bat_loc = 0;
      bat_max = 0;
     }

   DeleteRouter("batch");
   while (RemoveBatch());
  }

/**************************/
/* LOAD AND SAVE COMMANDS */
/**************************/

/******************************************************************/
/* LoadCommand: parses the load command and calls LoadConstructs */
/*   which will load a set of constructs from a file.             */
/*   Syntax:  (load <file-name>)                                  */
/******************************************************************/
globle int LoadCommand()
  {
#if (! BLOAD_ONLY) && (! RUN_TIME)
   char *theFileName;
   int rv = CLIPS_FALSE;

   if (ArgCountCheck("load",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if ((theFileName = GetFileName("load",1)) == NULL) return(CLIPS_FALSE);

   SetPrintWhileLoading(CLIPS_TRUE);

   if ((rv = Load(theFileName)) == CLIPS_FALSE)
     {
      SetPrintWhileLoading(CLIPS_FALSE);
      OpenErrorMessage("load",theFileName);
      return(CLIPS_FALSE);
     }

   SetPrintWhileLoading(CLIPS_FALSE);
   if (rv == -1) return(CLIPS_FALSE);
   return(CLIPS_TRUE);
#else
   PrintCLIPS(WDIALOG,"Load is not available in this environment\n");
   return(CLIPS_FALSE);
#endif
  }

#if DEBUGGING_FUNCTIONS
/**********************************************/
/* SaveCommand:  Executes the save commands. */
/*   Syntax:  (save <file-name>)              */
/**********************************************/
globle int SaveCommand()
  {
#if (! BLOAD_ONLY) && (! RUN_TIME)
   char *theFileName;

   if (ArgCountCheck("save",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if ((theFileName = GetFileName("save",1)) == NULL) return(CLIPS_FALSE);

   if (Save(theFileName) == CLIPS_FALSE)
     {
      OpenErrorMessage("save",theFileName);
      return(CLIPS_FALSE);
     }

   return(CLIPS_TRUE);
#else
   PrintCLIPS(WDIALOG,"Save is not available in this environment\n");
   return(CLIPS_FALSE);
#endif
  }
#endif



