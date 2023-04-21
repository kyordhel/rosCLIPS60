   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               SYSTEM DEPENDENT MODULE               */
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

#define _SYSDEP_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#if ANSI_COMPILER || IBM_TBC || IBM_MSC || IBM_ICB || IBM_ZTC
#include <stdlib.h>
#endif

#if IBM_ICB
#include <i32.h>
#include <stk.h>
#endif

#if MAC_MPW || MAC_TC
#include <Types.h>
#include <Files.h>
#if MAC_MPW
#include <strings.h>
#else
#include <pascal.h>
#endif
#endif

#if IBM_TBC || IBM_MSC || IBM_ICB || IBM_ZTC
#include <io.h>
#include <fcntl.h>
#include <limits.h>
#endif

#include "sysdep.h"
#include "constrct.h"
#include "filecom.h"
#include "clipsmem.h"
#include "argacces.h"
#include "utility.h"
#include "router.h"

#if DEFFACTS_CONSTRUCT
#include "dffctdef.h"
#endif

#if DEFRULE_CONSTRUCT
#include "ruledef.h"
#endif

#if DEFGENERIC_CONSTRUCT
#include "genrccom.h"
#endif

#if DEFFUNCTION_CONSTRUCT
#include "dffnxfun.h"
#endif

#if DEFGLOBAL_CONSTRUCT
#include "globldef.h"
#endif

#if DEFTEMPLATE_CONSTRUCT
#include "tmpltdef.h"
#endif

#if OBJECT_SYSTEM
#include "extobj.h"
#endif

#include "moduldef.h"

#if DEVELOPER
#include "developr.h"
#endif

#include "bmathfun.h"

#if ANSI_COMPILER
   extern VOID   SystemFunctionDefinitions(void);
   extern int    UserFunctions(void);
   extern VOID   ProceduralFunctionDefinitions(void);
   extern VOID   MiscFunctionDefinitions(void);
   extern VOID   IOFunctionDefinitions(void);
   extern VOID   PredicateFunctionDefinitions(void);
   extern VOID   WatchFunctionDefinitions(void);
   extern VOID   FileFunctionDefinitions(void);
   extern VOID   MultifieldFunctionDefinitions(void);
   extern VOID   StringFunctionDefinitions(void);
   extern VOID   ExtendedMathFunctionDefinitions(void);
   extern VOID   HelpFunctionDefinitions(void);
   extern VOID   EditorFunctionDefinition(void);
   extern VOID   ConstructsToCCommandDefinition(void);
   extern VOID   InitializeAtomTables(void);
   extern VOID   InitializeFacts(void);
   extern VOID   InitializeConstraints(void);
   extern VOID   InitExpressionData(void);
   extern VOID   InitializeConstructs(void);
   extern VOID   InstallProcedurePrimitives(void);
   static void   InitializeKeywords(void);
#else
   extern VOID   SystemFunctionDefinitions();
   extern int    UserFunctions();
   extern VOID   ProceduralFunctionDefinitions();
   extern VOID   MiscFunctionDefinitions();
   extern VOID   IOFunctionDefinitions();
   extern VOID   PredicateFunctionDefinitions();
   extern VOID   WatchFunctionDefinitions();
   extern VOID   FileFunctionDefinitions();
   extern VOID   MultifieldFunctionDefinitions();
   extern VOID   StringFunctionDefinitions();
   extern VOID   ExtendedMathFunctionDefinitions();
   extern VOID   HelpFunctionDefinitions();
   extern VOID   EditorFunctionDefinition();
   extern VOID   ConstructsToCCommandDefinition();
   extern VOID   InitializeAtomTables();
   extern VOID   InitializeFacts();
   extern VOID   InitializeConstraints();
   extern VOID   InitExpressionData();
   extern VOID   InitializeConstructs();
   extern VOID   InstallProcedurePrimitives();
   static void   InitializeKeywords();
#endif

#if ANSI_COMPILER
   static VOID                    InitializeNonportableFeatures(void);
#if   (VAX_VMS || UNIX_V || UNIX_7) && (! WINDOW_INTERFACE)
   static VOID                    CatchCtrlC(int);
#endif
#if   (IBM_TBC || IBM_MSC) && (! WINDOW_INTERFACE)
   static VOID interrupt          CatchCtrlC(void);
   static VOID                    RestoreInterruptVectors(void);
#endif
#if   IBM_ICB && (! WINDOW_INTERFACE)
#pragma interrupt (CatchCtrlC)
   static VOID                    CatchCtrlC(void);
   static VOID                    RestoreInterruptVectors(void);
#endif
#if   IBM_ZTC && (! WINDOW_INTERFACE)
   static void _cdecl             CatchCtrlC(void);
#endif
#else
   static VOID                    InitializeNonportableFeatures();
#if   (VAX_VMS || UNIX_V || UNIX_7) && (! WINDOW_INTERFACE)
   static VOID                    CatchCtrlC();
#endif
#if   (IBM_TBC || IBM_MSC) && (! WINDOW_INTERFACE)
   static VOID interrupt          CatchCtrlC();
   static VOID                    RestoreInterruptVectors();
#endif
#if   IBM_ICB && (! WINDOW_INTERFACE)
#pragma interrupt (CatchCtrlC)
   static VOID                    CatchCtrlC();
   static VOID                    RestoreInterruptVectors();
#endif
#if   IBM_ZTC && (! WINDOW_INTERFACE)
   static void _cdecl             CatchCtrlC();
#endif
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if ! WINDOW_INTERFACE
#if IBM_TBC
   static VOID interrupt  (*OldCtrlC)(void);
   static VOID interrupt  (*OldBreak)(void);
#endif
#if IBM_MSC
   static VOID  (interrupt *OldCtrlC)(void);
   static VOID  (interrupt *OldBreak)(void);
#endif
#if   IBM_ICB
#pragma interrupt (OldCtrlC,OldBreak)
   VOID         (*OldCtrlC)(void);
   VOID         (*OldBreak)(void);
#endif
#endif
#else
#if ! WINDOW_INTERFACE
#if IBM_TBC
   static VOID interrupt  (*OldCtrlC)();
   static VOID interrupt  (*OldBreak)();
#endif
#if IBM_MSC 
   static VOID  (interrupt *OldCtrlC)();
   static VOID  (interrupt *OldBreak)();
#if   IBM_ICB
#pragma interrupt (OldCtrlC,OldBreak)
   VOID         (*OldCtrlC)(void);
   VOID         (*OldBreak)(void);
#endif
#endif
#endif
#endif

#if  MAC_TC || MAC_MPW
   static short int        BinaryRefNum;
#endif

#if IBM_TBC || IBM_MSC || IBM_ICB /* || IBM_ZTC */
   static int              BinaryFileHandle;
#endif

#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) /* && (! IBM_ZTC) */
   static FILE            *BinaryFP;
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

#if ANSI_COMPILER
   globle VOID             (*RedrawScreenFunction)(void) = NULL;
   globle VOID             (*PauseEnvFunction)(void) = NULL;
   globle VOID             (*ContinueEnvFunction)(int) = NULL;
#else
   globle VOID             (*RedrawScreenFunction)() = NULL;
   globle VOID             (*PauseEnvFunction)() = NULL;
   globle VOID             (*ContinueEnvFunction)() = NULL;
#endif

/******************************************************/
/* InitializeCLIPS: Performs initialization of CLIPS. */
/******************************************************/
globle VOID InitializeCLIPS()
  {
   static BOOLEAN alreadyInitialized = CLIPS_FALSE;

   if (alreadyInitialized) return;
   
   InitializeMemory();
#if ! RUN_TIME
   InitializeAtomTables();            /* Initializes the atom hash tables.  */
#endif

   InitializeDefaultRouters();        /* Initalizes streams for I/O.        */
   InitializeNonportableFeatures();   /* System dependent initializations.  */

#if ! RUN_TIME
   SystemFunctionDefinitions();       /* Define system functions.           */
   UserFunctions();
#endif

   InitializeConstraints();           /* Initializes use of constraints.    */

#if ! RUN_TIME
   InitExpressionData();            
   InitializeConstructs();
#endif

#if DEFRULE_CONSTRUCT
   InitializeDefrules();
#endif

#if DEFFACTS_CONSTRUCT
   InitializeDeffacts();
#endif

#if DEFGENERIC_CONSTRUCT
   SetupGenericFunctions();
#endif

#if DEFFUNCTION_CONSTRUCT
   SetupDeffunctions();
#endif

#if DEFGLOBAL_CONSTRUCT
   InitializeDefglobals();
#endif

#if DEFTEMPLATE_CONSTRUCT
   InitializeDeftemplates();
#endif

#if OBJECT_SYSTEM
   SetupObjectSystem();
#endif

   InitializeDefmodules();

#if DEVELOPER
   DeveloperCommands();
#endif

   alreadyInitialized = CLIPS_TRUE;
   
   InstallProcedurePrimitives();
   
   InitializeKeywords();

   Clear();
   return;
  }

/********************************************/
/* SetRedrawFunction: Redraws the screen if */
/*   clipswin is main or does nothing.      */
/********************************************/
globle VOID SetRedrawFunction(theFunction)
  VOID (*theFunction)(VOID_ARG);
  {
   RedrawScreenFunction = theFunction;
  }

/**************************************************/
/* SetPauseEnvFunction: Puts terminal in a normal */
/*   state if clipswin is main or does nothing.   */
/**************************************************/
globle VOID SetPauseEnvFunction(theFunction)
  VOID (*theFunction)(VOID_ARG);
  {
   PauseEnvFunction = theFunction;
  }

/**************************************************************/
/* SetContinueEnvFunction: Returns terminal to special screen */
/*   interface state if clipswin is main or does nothing.     */
/**************************************************************/
globle VOID SetContinueEnvFunction(theFunction)
#if ANSI_COMPILER
   void (*theFunction)(int);
#else
   VOID (*theFunction)();
#endif
  {
   ContinueEnvFunction = theFunction;
  }

/***********************************************************/
/* RerouteStdin: Reroutes stdin to read initially from the */
/*   file specified on the command line with -f option.    */
/***********************************************************/
globle VOID RerouteStdin(argc,argv)
int argc;
char *argv[];
  {
   int i;

   /* If no arguments return */
   if (argc < 3)
     { return; }

   /* If argv was not passed then forget it */
   if (argv == NULL)
      return;

   for (i = 1 ; i < argc ; i++)
     {
      if (strcmp(argv[i],"-f") == 0)
        {
         if (i > (argc-1))
           {
            PrintErrorID("SYSDEP",1,CLIPS_FALSE);
            PrintCLIPS(WERROR,"No file found for -f option\n");
            return;
           }
         else
           OpenBatch(argv[++i],CLIPS_TRUE);
        }
     }
  }

#if ! RUN_TIME
/**************************************************/
/* SystemFunctionDefinitions: Sets up definitions */
/*   of system defined functions.                 */
/**************************************************/
globle VOID SystemFunctionDefinitions()
  {
   ProceduralFunctionDefinitions();
   MiscFunctionDefinitions();
   IOFunctionDefinitions();
   PredicateFunctionDefinitions();
   BasicMathFunctionDefinitions();
   FileFunctionDefinitions();
   
#if DEBUGGING_FUNCTIONS
   WatchFunctionDefinitions();
#endif

#if MULTIFIELD_FUNCTIONS
   MultifieldFunctionDefinitions();
#endif

#if STRING_FUNCTIONS
   StringFunctionDefinitions();
#endif

#if EX_MATH
   ExtendedMathFunctionDefinitions();
#endif

#if CLP_TEXTPRO || CLP_HELP
   HelpFunctionDefinitions();
#endif

#if CLP_EDIT
   EditorFunctionDefinition();
#endif

#if CONSTRUCT_COMPILER
   ConstructsToCCommandDefinition();
#endif
  }
#endif

/**********************************************************/
/* gentime: A function to return a floating point number  */
/*   which indicates the present time. Used internally by */
/*   CLIPS for timing rule firings and debugging.         */
/**********************************************************/

#if   VAX_VMS
#include timeb
#endif

#if   IBM_MSC || IBM_ICB
#include <sys\types.h>
#include <sys\timeb.h>
#endif

#if   IBM_TBC
#include <bios.h>
#endif

#if   IBM_ZTC
#include <time.h>
#endif

#if   UNIX_7
#include <sys/types.h>
#include <sys/timeb.h>
#endif

#if   UNIX_V
#include <sys/types.h>
#include <sys/times.h>
#endif

#if   MAC_MPW || MAC_TC
#include <Events.h>
#include <Desk.h>
#endif

globle double gentime()
  {
#if   VAX_VMS || IBM_MSC ||  UNIX_7 || IBM_ICB
   double sec, msec;
   int temp;
   struct timeb time_pointer;

/* Commented by J.Savage, 9-15-94
   ftime(&time_pointer); */
   temp = (int) time_pointer.time;
   temp = temp - ((temp/10000) * 10000);
   sec  = (double) temp;
   msec = (double) time_pointer.millitm;
   return(sec + (msec / 1000.0));
#endif

#if   UNIX_V
   long t_int;
   double t;
   struct tms buf;

   t_int = times(&buf);
   t = (double) t_int / 60.0;
   return(t);
#endif

#if   MAC_TC || MAC_MPW
   unsigned long int result;

   result = TickCount();

   return((double) result / 60.0);
#endif

#if   IBM_TBC
   unsigned long int result;

   result = biostime(0,(long int) 0);

   return((double) result / 18.2);
#endif

#if   IBM_ZTC
   return((double) time(NULL));
#endif

#if GENERIC
   return(0.0);
#endif
  }


/*****************************************************************/
/* gensystem:  This function can be called from CLIPS.  It will  */
/*   form a command string from its arguments, and pass this     */
/*   string to the operating system.  As currently defined, this */
/*   function does nothing, however, code has been included      */
/*   which should allow this function to work under VAX VMS and  */
/*   UNIX compatible systems.                                    */
/*****************************************************************/

#if   IBM_MSC || IBM_ICB || IBM_ZTC
#include <process.h>
#endif

globle VOID gensystem()
  {
   char *commandBuffer = NULL;
   int buff_index = 0;
   int buff_max = 0;
   int numa, i;
   DATA_OBJECT arg_ptr;
   char *str_ptr;

   if ((numa = ArgCountCheck("system",AT_LEAST,1)) == -1) return;

   for (i = 1 ; i <= numa; i++)
     {
      RtnUnknown(i,&arg_ptr);
      if ((GetType(arg_ptr) != STRING) &&
          (GetType(arg_ptr) != SYMBOL))
        {
         SetHaltExecution(CLIPS_TRUE);
         SetEvaluationError(CLIPS_TRUE);
         ExpectedTypeError2("system",i);
         return;
        }

     str_ptr = DOToString(arg_ptr);

     commandBuffer = AppendToString(str_ptr,commandBuffer,&buff_index,&buff_max);
    }

   if (commandBuffer == NULL) return;

#if VAX_VMS
   if (PauseEnvFunction != NULL) (*PauseEnvFunction)();
   VMSSystem(commandBuffer);
   putchar('\n');
   if (ContinueEnvFunction != NULL) (*ContinueEnvFunction)(1);
   if (RedrawScreenFunction != NULL) (*RedrawScreenFunction)();
#endif

#if   UNIX_7 || UNIX_V || IBM_MSC || IBM_TBC || IBM_ICB || IBM_ZTC
   if (PauseEnvFunction != NULL) (*PauseEnvFunction)();
   system(commandBuffer);
   if (ContinueEnvFunction != NULL) (*ContinueEnvFunction)(1);
   if (RedrawScreenFunction != NULL) (*RedrawScreenFunction)();
#else

#if ! VAX_VMS
   PrintCLIPS(WDIALOG,
            "System function not fully defined for this system.\n");
#endif

#endif

   rm(commandBuffer,buff_max);

   return;
  }


#if   VAX_VMS
#include <descrip.h>
#include <ssdef.h>
#include <stsdef.h>

extern int LIB$SPAWN();

globle VOID VMSSystem(cmd)
  char *cmd;
  {
   long status, complcode;
   struct dsc$descriptor_s cmd_desc;

   cmd_desc.dsc$w_length = strlen(cmd);
   cmd_desc.dsc$a_pointer = cmd;
   cmd_desc.dsc$b_class = DSC$K_CLASS_S;
   cmd_desc.dsc$b_dtype = DSC$K_DTYPE_T;

   status = LIB$SPAWN(&cmd_desc,0,0,0,0,0,&complcode,0,0,0);
  }
#endif

/**************************************************************/
/* The following two functions are provided to trap control-c */
/* in order to interrupt the execution of a program.          */
/**************************************************************/

#if ! WINDOW_INTERFACE

#if   VAX_VMS
#include signal
#endif

#if UNIX_V || UNIX_7
#include <signal.h>
#endif

#if IBM_TBC || IBM_MSC || IBM_ICB || IBM_ZTC
/* #include <dos.h> - already included from sysdep.h */
#endif

#if IBM_ZTC
#include <controlc.h>
#endif

#if   MAC_MPW
#include <Desk.h>
#endif

#endif

static VOID InitializeNonportableFeatures()
  {
#if ! WINDOW_INTERFACE

#if MAC_TC || MAC_MPW
   AddPeriodicFunction("systemtask",CallSystemTask,0);
#endif

#if VAX_VMS || UNIX_V || UNIX_7
   signal(SIGINT,CatchCtrlC);
#endif

#if IBM_TBC
   OldCtrlC = getvect(0x23);
   OldBreak = getvect(0x1b);
   setvect(0x23,CatchCtrlC);
   setvect(0x1b,CatchCtrlC);
   atexit(RestoreInterruptVectors);
#endif

#if IBM_MSC || IBM_ICB
   OldCtrlC = _dos_getvect(0x23);
   OldBreak = _dos_getvect(0x1b);
   _dos_setvect(0x23,CatchCtrlC);
   _dos_setvect(0x1b,CatchCtrlC);
   atexit(RestoreInterruptVectors);
#endif

#if IBM_ZTC
   _controlc_handler = CatchCtrlC;
   controlc_open();
#endif

#endif
  }

#if ! WINDOW_INTERFACE

#if MAC_TC || MAC_MPW
globle VOID CallSystemTask()
  {
   static unsigned long int lastCall;

   if (TickCount() < (lastCall + 10)) return;
   SystemTask();
   lastCall = TickCount();
   return;
  }
#endif

#if   VAX_VMS || UNIX_V || UNIX_7
static VOID CatchCtrlC(sgnl)
  int sgnl;
  {
   SetHaltExecution(CLIPS_TRUE);
   CloseAllBatchSources();
   signal(SIGINT,CatchCtrlC);
  }
#endif

#if   IBM_TBC || IBM_MSC
static VOID interrupt CatchCtrlC()
  {
   SetHaltExecution(CLIPS_TRUE);
   CloseAllBatchSources();
  }
  
static VOID RestoreInterruptVectors()
  {
#if IBM_TBC
   setvect(0x23,OldCtrlC);
   setvect(0x1b,OldBreak);
#else
   _dos_setvect(0x23,OldCtrlC);
   _dos_setvect(0x1b,OldBreak);
#endif
  }
#endif

#if IBM_ZTC
static void _cdecl CatchCtrlC()
  {
   SetHaltExecution(CLIPS_TRUE);
   CloseAllBatchSources();
  }
#endif


  
#if   IBM_ICB
static VOID CatchCtrlC()
  {
   _XSTACK *sf;                       /* Real-mode interrupt handler stack frame. */
   
   sf = (_XSTACK *) _get_stk_frame();  /* Get pointer to V86 _XSTACK frame. */
   SetHaltExecution(CLIPS_TRUE);       /* Terminate CLIPS operations and */
   CloseAllBatchSources();             /* return to CLIPS prompt.        */
   sf->opts |= _STK_NOINT;             /* Set _ST_NOINT to prevent V86 call. */
  }

static VOID RestoreInterruptVectors()
  {
   _dos_setvect(0x23,OldCtrlC);
   _dos_setvect(0x1b,OldBreak);
  }

#endif

#endif

/******************************************/
/* GENEXIT:  A generic exit function.     */
/*   Error codes:                         */
/*    -1 - Normal exit                    */
/*     1 - Out of memory exit             */
/*     2 - Arbitrary limit violation exit */
/*     3 - Memory release error exit      */
/*     4 - Rule parsing exit              */
/*     5 - Run time exit                  */
/*     6 - Rule maintenance exit          */
/******************************************/
globle VOID genexit(num)
  int num;
  {
   exit(num);
  }

/******************************************************/
/* genrand: Generic random number generator function. */
/******************************************************/
int genrand()
  {
#if ANSI_COMPILER
   return(rand());
#else
   return(0);
#endif
  }

/**********************************************************************/
/* genseed: Generic function for seeding the random number generator. */
/**********************************************************************/
globle VOID genseed(seed)
  int seed;
  {
#if ANSI_COMPILER
   srand((unsigned) seed);
#endif
  }
  
/****************************************************/
/* genremove: Generic function for removing a file. */
/****************************************************/
globle int genremove(fileName)
  char *fileName;
  {
#if ANSI_COMPILER
   if (remove(fileName)) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
#else
#if UNIX_V || UNIX_7
   if (unlink(fileName)) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
#else
   return(CLIPS_FALSE);
#endif
#endif
  }
  
/****************************************************/
/* genrename: Generic function for renaming a file. */
/****************************************************/
globle int genrename(oldFileName,newFileName)
  char *oldFileName, *newFileName;
  {
#if ANSI_COMPILER || UNIX_V || UNIX_7
   if (rename(oldFileName,newFileName)) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
#else
   return(CLIPS_FALSE);
#endif
  }

/*****************************************************************/
/* GenOpen: Generic and machine specific code for opening a file */
/*   for binary access. Only one file may be open at a time when */
/*   using this function since the file pointer is stored in a   */
/*   global variable.                                            */
/*****************************************************************/
globle int GenOpen(funcName,fileName)
  char *funcName,*fileName;
  {
#if  MAC_TC || MAC_MPW
   Str255 tempName;
   OSErr resultCode;
   Str255 volName;
   short int vRefNum;

   resultCode = GetVol(volName,&vRefNum);
   if (resultCode != noErr)
     {
      OpenErrorMessage(funcName,fileName);
      return(0);
     }
   strcpy((char *) tempName,fileName);
   c2pstr((char *) tempName);

   resultCode = FSOpen(tempName,vRefNum,&BinaryRefNum);
   if (resultCode != noErr)
     {
      OpenErrorMessage(funcName,fileName);
      return(0);
     }

#endif

#if IBM_TBC || IBM_MSC || IBM_ICB /* || IBM_ZTC */
   BinaryFileHandle = open(fileName,O_RDONLY | O_BINARY);
   if (BinaryFileHandle == -1)
     {
      OpenErrorMessage(funcName,fileName);
      return(0);
     }
#endif

#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) /* && (! IBM_ZTC) */
   if ((BinaryFP = fopen(fileName,"rb")) == NULL)
     {
      OpenErrorMessage(funcName,fileName);
      return(0);
     }
#endif

   return(1);
  }

/*****************************************/
/* GenRead: Generic and machine specific */
/*   code for reading from a file.       */
/*****************************************/
globle VOID GenRead(dataPtr,size)
  VOID *dataPtr;
  unsigned long size;
  {
#if  MAC_TC || MAC_MPW
   long dataSize;

   dataSize = size;
   FSRead(BinaryRefNum,&dataSize,dataPtr);
#endif

#if IBM_TBC || IBM_MSC || IBM_ICB /* || IBM_ZTC */
   char HUGE_ADDR *tempPtr;

   tempPtr = dataPtr;
   while (size > INT_MAX)
     {
      read(BinaryFileHandle,(VOID *) tempPtr,(unsigned int) INT_MAX);
      size -= INT_MAX;
      tempPtr = tempPtr + INT_MAX;
     }

   if (size > 0) read(BinaryFileHandle,(VOID *) tempPtr,(unsigned int) size);
#endif

#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) /* && (! IBM_ZTC) */
   unsigned int temp, number_of_reads, read_size;
 
   if (sizeof(int) == sizeof(long))
     { read_size = size; }
   else
     { read_size = (1L << (sizeof(int) * 8L)) - 1L ; }
   number_of_reads = size / read_size;
   temp = size - ((long) number_of_reads * (long) read_size);

   while (number_of_reads > 0)
     {
      fread(dataPtr,(int) read_size,1,BinaryFP);
      dataPtr = ((char *) dataPtr) + read_size;
      number_of_reads--;
     }

   fread(dataPtr,(int) temp,1,BinaryFP);
#endif
  }

/*******************************************/
/* GenSeek:  Generic and machine specific */
/*   code for closing a file.              */
/*******************************************/
globle VOID GenSeek(offset)
  long offset;
  {
#if  MAC_TC || MAC_MPW
   SetFPos(BinaryRefNum,3,offset);
#endif

#if IBM_TBC || IBM_MSC || IBM_ICB /* || IBM_ZTC */
   lseek(BinaryFileHandle,offset,SEEK_CUR);
#endif

#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) /* && (! IBM_ZTC) */
#if ANSI_COMPILER
   fseek(BinaryFP,offset,SEEK_CUR);
#else
   fseek(BinaryFP,offset,1);
#endif
#endif
  }
  
/*******************************************/
/* GenClose:  Generic and machine specific */
/*   code for closing a file.              */
/*******************************************/
globle VOID GenClose()
  {
#if  MAC_TC || MAC_MPW
   FSClose(BinaryRefNum);
#endif

#if IBM_TBC || IBM_MSC || IBM_ICB /* || IBM_ZTC */
   close(BinaryFileHandle);
#endif

#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) /* && (! IBM_ZTC) */
   fclose(BinaryFP);
#endif
  }
  
/****************************************************************/
/* InitializeKeywords: Adds CLIPS key words to the symbol table */
/*   so that they are available for command completion.         */
/****************************************************************/
static void InitializeKeywords()
  {
#if (! RUN_TIME) && WINDOW_INTERFACE
   void *ts;

   /*====================*/
   /* construct keywords */
   /*====================*/
   
   ts = AddSymbol("defrule");
   IncrementSymbolCount(ts);
   ts = AddSymbol("defglobal");
   IncrementSymbolCount(ts);
   ts = AddSymbol("deftemplate");
   IncrementSymbolCount(ts);
   ts = AddSymbol("deffacts");
   IncrementSymbolCount(ts);
   ts = AddSymbol("deffunction");
   IncrementSymbolCount(ts);
   ts = AddSymbol("defmethod");
   IncrementSymbolCount(ts);
   ts = AddSymbol("defgeneric");
   IncrementSymbolCount(ts);
   ts = AddSymbol("defclass");
   IncrementSymbolCount(ts);
   ts = AddSymbol("defmessage-handler");
   IncrementSymbolCount(ts);
   ts = AddSymbol("definstances");
   IncrementSymbolCount(ts);
   
   /*=======================*/
   /* set-strategy keywords */
   /*=======================*/

   ts = AddSymbol("depth");
   IncrementSymbolCount(ts);
   ts = AddSymbol("breadth");
   IncrementSymbolCount(ts);
   ts = AddSymbol("lex");
   IncrementSymbolCount(ts);
   ts = AddSymbol("mea");
   IncrementSymbolCount(ts);
   ts = AddSymbol("simplicity");
   IncrementSymbolCount(ts);
   ts = AddSymbol("complexity");
   IncrementSymbolCount(ts);
   ts = AddSymbol("random");
   IncrementSymbolCount(ts);

   /*==================================*/
   /* set-salience-evaluation keywords */
   /*==================================*/

   ts = AddSymbol("when-defined");
   IncrementSymbolCount(ts);
   ts = AddSymbol("when-activated");
   IncrementSymbolCount(ts);
   ts = AddSymbol("every-cycle");
   IncrementSymbolCount(ts);

   /*======================*/
   /* deftemplate keywords */
   /*======================*/

   ts = AddSymbol("field");
   IncrementSymbolCount(ts);
   ts = AddSymbol("multifield");
   IncrementSymbolCount(ts);
   ts = AddSymbol("default");
   IncrementSymbolCount(ts);
   ts = AddSymbol("type");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-symbols");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-strings");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-numbers");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-integers");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-floats");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-values");
   IncrementSymbolCount(ts);
   ts = AddSymbol("min-number-of-elements");
   IncrementSymbolCount(ts);
   ts = AddSymbol("max-number-of-elements");
   IncrementSymbolCount(ts);
   ts = AddSymbol("NONE");
   IncrementSymbolCount(ts);
   ts = AddSymbol("VARIABLE");
   IncrementSymbolCount(ts);

   /*==================*/
   /* defrule keywords */
   /*==================*/

   ts = AddSymbol("declare");
   IncrementSymbolCount(ts);
   ts = AddSymbol("salience");
   IncrementSymbolCount(ts);
   ts = AddSymbol("test");
   IncrementSymbolCount(ts);
   ts = AddSymbol("or");
   IncrementSymbolCount(ts);
   ts = AddSymbol("and");
   IncrementSymbolCount(ts);
   ts = AddSymbol("not");
   IncrementSymbolCount(ts);
   ts = AddSymbol("logical");
   IncrementSymbolCount(ts);

   /*===============*/
   /* COOL keywords */
   /*===============*/

   ts = AddSymbol("is-a");
   IncrementSymbolCount(ts);
   ts = AddSymbol("role");
   IncrementSymbolCount(ts);
   ts = AddSymbol("abstract");
   IncrementSymbolCount(ts);
   ts = AddSymbol("concrete");
   IncrementSymbolCount(ts);
   ts = AddSymbol("pattern-match");
   IncrementSymbolCount(ts);
   ts = AddSymbol("reactive");
   IncrementSymbolCount(ts);
   ts = AddSymbol("non-reactive");
   IncrementSymbolCount(ts);
   ts = AddSymbol("slot");
   IncrementSymbolCount(ts);
   ts = AddSymbol("field");
   IncrementSymbolCount(ts);
   ts = AddSymbol("multiple");
   IncrementSymbolCount(ts);
   ts = AddSymbol("single");
   IncrementSymbolCount(ts);
   ts = AddSymbol("storage");
   IncrementSymbolCount(ts);
   ts = AddSymbol("shared");
   IncrementSymbolCount(ts);
   ts = AddSymbol("local");
   IncrementSymbolCount(ts);
   ts = AddSymbol("access");
   IncrementSymbolCount(ts);
   ts = AddSymbol("read");
   IncrementSymbolCount(ts);
   ts = AddSymbol("write");
   IncrementSymbolCount(ts);
   ts = AddSymbol("read-only");
   IncrementSymbolCount(ts);
   ts = AddSymbol("read-write");
   IncrementSymbolCount(ts);
   ts = AddSymbol("initialize-only");
   IncrementSymbolCount(ts);
   ts = AddSymbol("propagation");
   IncrementSymbolCount(ts);
   ts = AddSymbol("inherit");
   IncrementSymbolCount(ts);
   ts = AddSymbol("no-inherit");
   IncrementSymbolCount(ts);
   ts = AddSymbol("source");
   IncrementSymbolCount(ts);
   ts = AddSymbol("composite");
   IncrementSymbolCount(ts);
   ts = AddSymbol("exclusive");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-lexemes");
   IncrementSymbolCount(ts);
   ts = AddSymbol("allowed-instances");
   IncrementSymbolCount(ts);
   ts = AddSymbol("around");
   IncrementSymbolCount(ts);
   ts = AddSymbol("before");
   IncrementSymbolCount(ts);
   ts = AddSymbol("primary");
   IncrementSymbolCount(ts);
   ts = AddSymbol("after");
   IncrementSymbolCount(ts);
   ts = AddSymbol("of");
   IncrementSymbolCount(ts);
   ts = AddSymbol("self");
   IncrementSymbolCount(ts);
   ts = AddSymbol("visibility");
   IncrementSymbolCount(ts);
   ts = AddSymbol("override-message");
   IncrementSymbolCount(ts);
   ts = AddSymbol("private");
   IncrementSymbolCount(ts);
   ts = AddSymbol("public");
   IncrementSymbolCount(ts);
   ts = AddSymbol("create-accessor");
   IncrementSymbolCount(ts);

   /*================*/
   /* watch keywords */
   /*================*/

   ts = AddSymbol("compilations");
   IncrementSymbolCount(ts);
   ts = AddSymbol("deffunctions");
   IncrementSymbolCount(ts);
   ts = AddSymbol("globals");
   IncrementSymbolCount(ts);
   ts = AddSymbol("rules");
   IncrementSymbolCount(ts);
   ts = AddSymbol("activations");
   IncrementSymbolCount(ts);
   ts = AddSymbol("statistics");
   IncrementSymbolCount(ts);
   ts = AddSymbol("facts");
   IncrementSymbolCount(ts);
   ts = AddSymbol("generic-functions");
   IncrementSymbolCount(ts);
   ts = AddSymbol("methods");
   IncrementSymbolCount(ts);
   ts = AddSymbol("instances");
   IncrementSymbolCount(ts);
   ts = AddSymbol("slots");
   IncrementSymbolCount(ts);
   ts = AddSymbol("messages");
   IncrementSymbolCount(ts);
   ts = AddSymbol("message-handlers");
   IncrementSymbolCount(ts);
   ts = AddSymbol("focus");
   IncrementSymbolCount(ts);
#endif
  }
  

