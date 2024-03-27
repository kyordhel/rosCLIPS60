   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*              CLIPS Version 6.00  05/12/93           */
   /*                                                     */
   /*         INSTANCE LOAD/SAVE (ASCII/BINARY) MODULE    */
   /*******************************************************/

/*************************************************************/
/* Purpose:  File load/save routines for instances           */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/
   
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if OBJECT_SYSTEM

#include "argacces.h"
#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "extnfunc.h"
#include "inscom.h"
#include "insfun.h"
#include "insmngr.h"
#include "inspsr.h"
#include "object.h"
#include "router.h"
#include "symblbin.h"
#include "sysdep.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT
#include "factmngr.h"
#endif

#define _INSFILE_SOURCE_
#include "insfile.h"

extern struct token ObjectParseToken;

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define MAX_BLOCK_SIZE 10240

/* =========================================
   *****************************************
               MACROS AND TYPES
   =========================================
   ***************************************** */
struct bsaveSlotValue
  {
   long slotName;
   int valueCount;
  };
   
struct bsaveSlotValueAtom
  {
   int type;
   long value;
  };
    
/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static long InstancesSaveCommandParser(char *,long (*)(char *,int,
                                                   EXPRESSION *,BOOLEAN));
static DATA_OBJECT *ProcessSaveClassList(char *,EXPRESSION *,int,BOOLEAN);
static VOID ReturnSaveClassList(DATA_OBJECT *);
static long SaveOrMarkInstances(VOID *,int,DATA_OBJECT *,BOOLEAN,BOOLEAN,
                                         VOID (*)(VOID *,INSTANCE_TYPE *));
static long SaveOrMarkInstancesOfClass(VOID *,struct defmodule *,int,DEFCLASS *,
                                                BOOLEAN,int,VOID (*)(VOID *,INSTANCE_TYPE *));
static VOID SaveSingleInstanceText(VOID *,INSTANCE_TYPE *);
static VOID ProcessFileErrorMessage(char *,char *);
#if BSAVE_INSTANCES
static VOID WriteBinaryHeader(FILE *);
static VOID MarkSingleInstance(VOID *,INSTANCE_TYPE *);
static VOID MarkNeededAtom(int,VOID *);
static VOID SaveSingleInstanceBinary(VOID *,INSTANCE_TYPE *);
static VOID SaveAtomBinary(int,VOID *,FILE *);
#endif

static long LoadOrRestoreInstances(char *,int);

#if BLOAD_INSTANCES
static BOOLEAN VerifyBinaryHeader(char *);
static BOOLEAN LoadSingleBinaryInstance(void);
static VOID BinaryLoadInstanceError(SYMBOL_HN *,DEFCLASS *);
static VOID CreateSlotValue(DATA_OBJECT *,struct bsaveSlotValueAtom *,int);
static VOID *GetBinaryAtomValue(struct bsaveSlotValueAtom *);
static VOID BufferedRead(VOID *,unsigned long);
static VOID FreeReadBuffer(void);
#endif

#else

static long InstancesSaveCommandParser();
static DATA_OBJECT *ProcessSaveClassList();
static VOID ReturnSaveClassList();
static long SaveOrMarkInstances();
static long SaveOrMarkInstancesOfClass();
static VOID SaveSingleInstanceText();
static VOID ProcessFileErrorMessage();

#if BSAVE_INSTANCES
static VOID WriteBinaryHeader();
static VOID MarkSingleInstance();
static VOID MarkNeededAtom();
static VOID SaveSingleInstanceBinary();
static VOID SaveAtomBinary();
#endif

static long LoadOrRestoreInstances();

#if BLOAD_INSTANCES
static BOOLEAN VerifyBinaryHeader();
static BOOLEAN LoadSingleBinaryInstance();
static VOID BinaryLoadInstanceError();
static VOID CreateSlotValue();
static VOID *GetBinaryAtomValue();
static VOID BufferedRead();
static VOID FreeReadBuffer();
#endif

#endif

/* =========================================
   *****************************************
       INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
   
#if BLOAD_INSTANCES || BSAVE_INSTANCES

static char *InstanceBinaryPrefixID = "\5\6\7CLIPS";
static char *InstanceBinaryVersionID = "V6.00";
static unsigned long BinaryInstanceFileSize;

#if BLOAD_INSTANCES
static unsigned long BinaryInstanceFileOffset;
static char HUGE_ADDR *CurrentReadBuffer = NULL;
static unsigned long CurrentReadBufferSize = 0L;
#endif

#endif

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if (! RUN_TIME)

/***************************************************
  NAME         : SetupInstanceFileCommands
  DESCRIPTION  : Defines function interfaces for
                 saving instances to files
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Functions defined to CLIPS
  NOTES        : None
 ***************************************************/
globle VOID SetupInstanceFileCommands()
  {
   DefineFunction2("save-instances",'l',PTIF SaveInstancesCommand,
                   "SaveInstancesCommand","1*wk");
   DefineFunction2("load-instances",'l',PTIF LoadInstancesCommand,
                   "LoadInstancesCommand","11k");
   DefineFunction2("restore-instances",'l',PTIF RestoreInstancesCommand,
                   "RestoreInstancesCommand","11k");
                   
#if BSAVE_INSTANCES
   DefineFunction2("bsave-instances",'l',PTIF BinarySaveInstancesCommand,
                   "BinarySaveInstancesCommand","1*wk");
#endif
#if BLOAD_INSTANCES
   DefineFunction2("bload-instances",'l',PTIF BinaryLoadInstancesCommand,
                   "BinaryLoadInstancesCommand","11k");
#endif
  }

#endif

/****************************************************************************
  NAME         : SaveInstancesCommand
  DESCRIPTION  : CLIPS interface for saving
                   current instances to a file
  INPUTS       : None
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Instances saved to named file
  NOTES        : CLIPS Syntax : 
                 (save-instances <file> [local|visible [[inherit] <class>+]])
 ****************************************************************************/
globle long SaveInstancesCommand()
  {
   return(InstancesSaveCommandParser("save-instances",SaveInstances));
  }

/******************************************************
  NAME         : LoadInstancesCommand
  DESCRIPTION  : CLIPS interface for loading
                   instances from a file
  INPUTS       : None
  RETURNS      : The number of instances loaded
  SIDE EFFECTS : Instances loaded from named file
  NOTES        : CLIPS Syntax : (load-instances <file>)
 ******************************************************/
globle long LoadInstancesCommand()
  {
   char *fileFound;
   DATA_OBJECT temp;
   long instanceCount;
   
   if (ArgTypeCheck("load-instances",1,SYMBOL_OR_STRING,&temp) == CLIPS_FALSE)
     return(0L);

   fileFound = DOToString(temp);

   instanceCount = LoadInstances(fileFound);
   if (EvaluationError)
     ProcessFileErrorMessage("load-instances",fileFound);
   return(instanceCount);
  }

/***************************************************
  NAME         : LoadInstances
  DESCRIPTION  : Loads instances from named file
  INPUTS       : The name of the input file
  RETURNS      : The number of instances loaded
  SIDE EFFECTS : Instances loaded from file
  NOTES        : None
 ***************************************************/
globle long LoadInstances(file)
  char *file;
  {
   return(LoadOrRestoreInstances(file,CLIPS_TRUE));
  }

/*********************************************************
  NAME         : RestoreInstancesCommand
  DESCRIPTION  : CLIPS interface for loading
                   instances from a file w/o messages
  INPUTS       : None
  RETURNS      : The number of instances restored
  SIDE EFFECTS : Instances loaded from named file
  NOTES        : CLIPS Syntax : (restore-instances <file>)
 *********************************************************/
globle long RestoreInstancesCommand()
  {
   char *fileFound;
   DATA_OBJECT temp;
   long instanceCount;
   
   if (ArgTypeCheck("restore-instances",1,SYMBOL_OR_STRING,&temp) == CLIPS_FALSE)
     return(0L);

   fileFound = DOToString(temp);

   instanceCount = RestoreInstances(fileFound);
   if (EvaluationError) 
     ProcessFileErrorMessage("restore-instances",fileFound);
   return(instanceCount);
  }

/***************************************************
  NAME         : RestoreInstances
  DESCRIPTION  : Restores instances from named file
  INPUTS       : The name of the input file
  RETURNS      : The number of instances restored
  SIDE EFFECTS : Instances restored from file
  NOTES        : None
 ***************************************************/
globle long RestoreInstances(file)
  char *file;
  {
   return(LoadOrRestoreInstances(file,CLIPS_FALSE));
  }

#if BLOAD_INSTANCES

/*******************************************************
  NAME         : BinaryLoadInstancesCommand
  DESCRIPTION  : CLIPS interface for loading
                   instances from a binary file
  INPUTS       : None
  RETURNS      : The number of instances loaded
  SIDE EFFECTS : Instances loaded from named binary file
  NOTES        : CLIPS Syntax : (bload-instances <file>)
 *******************************************************/
globle long BinaryLoadInstancesCommand()
  {
   char *fileFound;
   DATA_OBJECT temp;
   long instanceCount;
   
   if (ArgTypeCheck("bload-instances",1,SYMBOL_OR_STRING,&temp) == CLIPS_FALSE)
     return(0L);

   fileFound = DOToString(temp);

   instanceCount = BinaryLoadInstances(fileFound);
   if (EvaluationError) 
     ProcessFileErrorMessage("bload-instances",fileFound);
   return(instanceCount);
  }

/****************************************************
  NAME         : BinaryLoadInstances
  DESCRIPTION  : Loads instances quickly from a
                 binary file
  INPUTS       : The file name
  RETURNS      : The number of instances loaded
  SIDE EFFECTS : Instances loaded w/o message-passing
  NOTES        : None
 ****************************************************/
globle long BinaryLoadInstances(theFile)
  char *theFile;
  {
   long i,instanceCount;
   
   if (GenOpen("bload-instances",theFile) == 0)
     {
      SetEvaluationError(CLIPS_TRUE);
      return(-1L);
     }
   if (VerifyBinaryHeader(theFile) == CLIPS_FALSE)
     {
      GenClose();
      SetEvaluationError(CLIPS_TRUE);
      return(-1L);
     }
   ReadNeededAtomicValues();
   
   BinaryInstanceFileOffset = 0L;

   GenRead((VOID *) &BinaryInstanceFileSize,(unsigned long) sizeof(unsigned long));
   GenRead((VOID *) &instanceCount,(unsigned long) sizeof(long));
   for (i = 0L ; i < instanceCount ; i++)
     {
      if (LoadSingleBinaryInstance() == CLIPS_FALSE)
        {
         FreeReadBuffer();
         FreeAtomicValueStorage();
         GenClose();
         SetEvaluationError(CLIPS_TRUE);
         return(i);
        }
     }
   FreeReadBuffer();
   FreeAtomicValueStorage();
   GenClose();
   return(instanceCount);
  }

#endif

/*******************************************************
  NAME         : SaveInstances
  DESCRIPTION  : Saves current instances to named file
  INPUTS       : 1) The name of the output file
                 2) A flag indicating whether to
                    save local (current module only)
                    or visible instances
                    LOCAL_SAVE or VISIBLE_SAVE
                 3) A list of expressions containing
                    the names of classes for which
                    instances are to be saved
                 4) A flag indicating if the subclasses
                    of specified classes shoudl also
                    be processed
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Instances saved to file
  NOTES        : None
 *******************************************************/
globle long SaveInstances(file,saveCode,classExpressionList,inheritFlag)
  char *file;
  int saveCode;
  EXPRESSION *classExpressionList;
  BOOLEAN inheritFlag;
  {
   FILE *sfile = NULL;
   int oldPEC,oldATS,oldIAN;
   DATA_OBJECT *classList;
   long instanceCount;
   
   classList = ProcessSaveClassList("save-instances",classExpressionList,
                                    saveCode,inheritFlag);
   if ((classList == NULL) && (classExpressionList != NULL))
     return(0L);

   instanceCount = SaveOrMarkInstances((VOID *) sfile,saveCode,classList,
                                       inheritFlag,CLIPS_TRUE,NULL);

   if ((sfile = fopen(file,"w")) == NULL)
     {
      OpenErrorMessage("save-instances",file);
      ReturnSaveClassList(classList);
      SetEvaluationError(CLIPS_TRUE);
      return(0L);
     }
     
   oldPEC = PreserveEscapedCharacters;
   PreserveEscapedCharacters = CLIPS_TRUE;
   oldATS = AddressesToStrings;
   AddressesToStrings = CLIPS_TRUE;
   oldIAN = InstanceAddressesToNames;
   InstanceAddressesToNames = CLIPS_TRUE;
   
   SetFastSave(sfile);
   instanceCount = SaveOrMarkInstances((VOID *) sfile,saveCode,classList,
                                       inheritFlag,CLIPS_TRUE,SaveSingleInstanceText);
   fclose(sfile);
   SetFastSave(NULL);
   
   PreserveEscapedCharacters = oldPEC;
   AddressesToStrings = oldATS;
   InstanceAddressesToNames = oldIAN;
   ReturnSaveClassList(classList);
   return(instanceCount);
  }
  
#if BSAVE_INSTANCES

/****************************************************************************
  NAME         : BinarySaveInstancesCommand
  DESCRIPTION  : CLIPS interface for saving
                   current instances to a binary file
  INPUTS       : None
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Instances saved (in binary format) to named file
  NOTES        : CLIPS Syntax : 
                 (bsave-instances <file> [local|visible [[inherit] <class>+]])
 *****************************************************************************/
globle long BinarySaveInstancesCommand()
  {
   return(InstancesSaveCommandParser("bsave-instances",BinarySaveInstances));
  }

/*******************************************************
  NAME         : BinarySaveInstances
  DESCRIPTION  : Saves current instances to binary file
  INPUTS       : 1) The name of the output file
                 2) A flag indicating whether to
                    save local (current module only)
                    or visible instances
                    LOCAL_SAVE or VISIBLE_SAVE
                 3) A list of expressions containing
                    the names of classes for which
                    instances are to be saved
                 4) A flag indicating if the subclasses
                    of specified classes shoudl also
                    be processed
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Instances saved to file
  NOTES        : None
 *******************************************************/
globle long BinarySaveInstances(file,saveCode,classExpressionList,inheritFlag)
  char *file;
  int saveCode;
  EXPRESSION *classExpressionList;
  BOOLEAN inheritFlag;
  {
   DATA_OBJECT *classList;
   FILE *bsaveFP;
   long instanceCount;
   
   classList = ProcessSaveClassList("bsave-instances",classExpressionList,
                                    saveCode,inheritFlag);
   if ((classList == NULL) && (classExpressionList != NULL))
     return(0L);
   
   BinaryInstanceFileSize = 0L;
   InitAtomicValueNeededFlags();
   instanceCount = SaveOrMarkInstances(NULL,saveCode,classList,inheritFlag,
                                       CLIPS_FALSE,MarkSingleInstance);
                                            
   if ((bsaveFP = fopen(file,"wb")) == NULL)
     {
      OpenErrorMessage("bsave-instances",file);
      ReturnSaveClassList(classList);
      SetEvaluationError(CLIPS_TRUE);
      return(0L);
     }
   WriteBinaryHeader(bsaveFP);     
   WriteNeededAtomicValues(bsaveFP);
   
   fwrite((VOID *) &BinaryInstanceFileSize,(int) sizeof(unsigned long),1,bsaveFP);
   fwrite((VOID *) &instanceCount,(int) sizeof(long),1,bsaveFP);
   
   SetAtomicValueIndices(CLIPS_FALSE);
   SaveOrMarkInstances((VOID *) bsaveFP,saveCode,classList,
                       inheritFlag,CLIPS_FALSE,SaveSingleInstanceBinary);
   RestoreAtomicValueBuckets();
   fclose(bsaveFP);
   ReturnSaveClassList(classList);
   return(instanceCount);
  }
  
#endif

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/******************************************************
  NAME         : InstancesSaveCommandParser
  DESCRIPTION  : Argument parser for save-instances
                 and bsave-instances
  INPUTS       : 1) The name of the calling function
                 2) A pointer to the support
                    function to call for the save/bsave
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Instances saved/bsaved
  NOTES        : None
 ******************************************************/
static long InstancesSaveCommandParser(functionName,saveFunction)
  char *functionName;
#if ANSI_COMPILER
   long (*saveFunction)(char *,int,EXPRESSION *,BOOLEAN);
#else
   long (*saveFunction)();
#endif
  {
   char *fileFound;
   DATA_OBJECT temp;
   int argCount,saveCode = LOCAL_SAVE;
   EXPRESSION *classList = NULL;
   BOOLEAN inheritFlag = CLIPS_FALSE;
   
   if (ArgTypeCheck(functionName,1,SYMBOL_OR_STRING,&temp) == CLIPS_FALSE)
     return(0L);
   fileFound = DOToString(temp);

   argCount = RtnArgCount();
   if (argCount > 1)
     {
      if (ArgTypeCheck(functionName,2,SYMBOL,&temp) == CLIPS_FALSE)
        {
         ExpectedTypeError1(functionName,2,"symbol \"local\" or \"visible\"");
         SetEvaluationError(CLIPS_TRUE);
         return(0L);
        }
      if (strcmp(DOToString(temp),"local") == 0)
        saveCode = LOCAL_SAVE;
      else if (strcmp(DOToString(temp),"visible") == 0)
        saveCode = VISIBLE_SAVE;
      else
        {
         ExpectedTypeError1(functionName,2,"symbol \"local\" or \"visible\"");
         SetEvaluationError(CLIPS_TRUE);
         return(0L);
        }
      classList = GetFirstArgument()->nextArg->nextArg;
      
      /* ===========================
         Check for "inherit" keyword
         Must be at least one class
         name following
         =========================== */
      if ((classList != NULL) ? (classList->nextArg != NULL) : CLIPS_FALSE)
        {
         if ((classList->type != SYMBOL) ? CLIPS_FALSE :
             (strcmp(ValueToString(classList->value),"inherit") == 0))
           {
            inheritFlag = CLIPS_TRUE;
            classList = classList->nextArg;
           }
        }
     }

   return((*saveFunction)(fileFound,saveCode,classList,inheritFlag));
  }

/****************************************************
  NAME         : ProcessSaveClassList
  DESCRIPTION  : Evaluates a list of class name
                 expressions and stores them in a
                 data object list
  INPUTS       : 1) The name of the calling function
                 2) The class expression list
                 3) A flag indicating if only local
                    or all visible instances are
                    being saved
                 4) A flag indicating if inheritance
                    relationships should be checked
                    between classes
  RETURNS      : The evaluated class pointer data
                 objects - NULL on errors
  SIDE EFFECTS : Data objects allocated and
                 classes validated
  NOTES        : None
 ****************************************************/
static DATA_OBJECT *ProcessSaveClassList(functionName,classExps,saveCode,inheritFlag)
  char *functionName;
  EXPRESSION *classExps;
  int saveCode;
  BOOLEAN inheritFlag;
  {
   DATA_OBJECT *head = NULL,*prv,*new,tmp;
   DEFCLASS *theDefclass;
   struct defmodule *currentModule;
   int argIndex = inheritFlag ? 4 : 3;
   
   currentModule = ((struct defmodule *) GetCurrentModule());
   while (classExps != NULL)
     {
      if (EvaluateExpression(classExps,&tmp))
        goto ProcessClassListError;
      if (tmp.type != SYMBOL)
        goto ProcessClassListError;
      if (saveCode == LOCAL_SAVE)
        theDefclass = LookupDefclassAnywhere(currentModule,DOToString(tmp));
      else
        theDefclass = LookupDefclassInScope(DOToString(tmp));
      if (theDefclass == NULL)
        goto ProcessClassListError;
      else if (theDefclass->abstract && (inheritFlag == CLIPS_FALSE))
        goto ProcessClassListError;
      prv = new = head;
      while (new != NULL)
        {
         if (new->value == (VOID *) theDefclass)
           goto ProcessClassListError;
         else if (inheritFlag)
           {
            if (HasSuperclass((DEFCLASS *) new->value,theDefclass) ||
                HasSuperclass(theDefclass,(DEFCLASS *) new->value))
             goto ProcessClassListError;
           }
         prv = new;
         new = new->next;
        }
      new = get_struct(dataObject);
      new->type = DEFCLASS_PTR;
      new->value = (VOID *) theDefclass;
      new->next = NULL;
      if (prv == NULL)
        head = new;
      else
        prv->next = new;
      prv = new;
      argIndex++;
      classExps = classExps->nextArg;
     }
   return(head);
   
ProcessClassListError:
   ExpectedTypeError1(functionName,argIndex,
                      inheritFlag ? "valid class name" : "valid concrete class name");
   ReturnSaveClassList(head);
   SetEvaluationError(CLIPS_TRUE);
   return(NULL);
  }

/****************************************************
  NAME         : ReturnSaveClassList
  DESCRIPTION  : Deallocates the class data object
                 list created by ProcessSaveClassList
  INPUTS       : The class data object list
  RETURNS      : Nothing useful
  SIDE EFFECTS : Class data object returned
  NOTES        : None
 ****************************************************/
static VOID ReturnSaveClassList(classList)
  DATA_OBJECT *classList;
  {
   DATA_OBJECT *tmp;
   
   while (classList != NULL)
     {
      tmp = classList;
      classList = classList->next;
      rtn_struct(dataObject,tmp);
     }
  }

/***************************************************
  NAME         : SaveOrMarkInstances
  DESCRIPTION  : Iterates through all specified
                 instances either marking needed
                 atoms or writing instances in
                 binary/text format
  INPUTS       : 1) NULL (for marking),
                    logical name (for text saves)
                    file pointer (for binary saves)
                 2) A cope flag indicating LOCAL
                    or VISIBLE saves only
                 3) A list of data objects
                    containing the names of classes
                    of instances to be saved
                 4) A flag indicating whether to
                    include subclasses of arg #3
                 5) A flag indicating if the
                    iteration can be interrupted
                    or not
                 6) The access function to mark
                    or save an instance (can be NULL
                    if only counting instances)
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Instances amrked or saved
  NOTES        : None
 ***************************************************/
static long SaveOrMarkInstances(theOutput,saveCode,classList,
                                inheritFlag,interruptOK,saveInstanceFunc)
  VOID *theOutput;
  int saveCode;
  DATA_OBJECT *classList;
  BOOLEAN inheritFlag,interruptOK;
#if ANSI_COMPILER
  VOID (*saveInstanceFunc)(VOID *,INSTANCE_TYPE *);
#else
  VOID (*saveInstanceFunc)();
#endif
  {   
   struct defmodule *currentModule;
   int traversalID;
   DATA_OBJECT *tmp;
   INSTANCE_TYPE *ins;
   long instanceCount = 0L;
   
   currentModule = ((struct defmodule *) GetCurrentModule());
   if (classList != NULL)
     {
      traversalID = GetTraversalID();
      if (traversalID != -1)
        {
         for (tmp = classList ;
              (! ((tmp == NULL) || (HaltExecution && interruptOK))) ;
              tmp = tmp->next)
           instanceCount += SaveOrMarkInstancesOfClass(theOutput,currentModule,saveCode,
                                                       (DEFCLASS *) tmp->value,inheritFlag,
                                                       traversalID,saveInstanceFunc);
         ReleaseTraversalID();
        }
     }
   else
     {
      for (ins = (INSTANCE_TYPE *) GetNextInstanceInScope(NULL) ;
           (ins != NULL) && (HaltExecution != CLIPS_TRUE) ; 
           ins = (INSTANCE_TYPE *) GetNextInstanceInScope((VOID *) ins))
        {
         if ((saveCode == VISIBLE_SAVE) ? CLIPS_TRUE :
             (ins->cls->header.whichModule->theModule == currentModule))
           {
            if (saveInstanceFunc != NULL)
              (*saveInstanceFunc)(theOutput,ins);
            instanceCount++;
           }
        }
     }
   return(instanceCount);
  }
     
/***************************************************
  NAME         : SaveOrMarkInstancesOfClass
  DESCRIPTION  : Saves off the direct (and indirect)
                 instance of the specified class
  INPUTS       : 1) The logical name of the output
                    (or file pointer for binary
                     output)
                 2) The current module
                 3) A flag indicating local
                    or visible saves
                 4) The defclass
                 5) A flag indicating whether to
                    save subclass instances or not
                 6) A traversal id for marking
                    visited classes
                 7) A pointer to the instance
                    manipulation function to call
                    (can be NULL for only counting
                     instances)
  RETURNS      : The number of instances saved
  SIDE EFFECTS : Appropriate instances saved
  NOTES        : None
 ***************************************************/
static long SaveOrMarkInstancesOfClass(theOutput,currentModule,saveCode,
                                       theDefclass,inheritFlag,traversalID,
                                       saveInstanceFunc)
  VOID *theOutput;
  struct defmodule *currentModule;
  int saveCode;
  DEFCLASS *theDefclass;
  BOOLEAN inheritFlag;
  int traversalID;
#if ANSI_COMPILER
  VOID (*saveInstanceFunc)(VOID *,INSTANCE_TYPE *);
#else
  VOID (*saveInstanceFunc)();
#endif
  {
   INSTANCE_TYPE *theInstance;
   DEFCLASS *subclass;
   register unsigned i;
   long instanceCount = 0L;
   
   if (TestTraversalID(theDefclass->traversalRecord,traversalID))
     return(instanceCount);
   SetTraversalID(theDefclass->traversalRecord,traversalID);
   if (((saveCode == LOCAL_SAVE) &&
        (theDefclass->header.whichModule->theModule == currentModule)) ||
       ((saveCode == VISIBLE_SAVE) &&
        DefclassInScope(theDefclass,currentModule)))
     {
      for (theInstance = (INSTANCE_TYPE *) 
             GetNextInstanceInClass((VOID *) theDefclass,NULL) ;
           theInstance != NULL ;
           theInstance = (INSTANCE_TYPE *)
           GetNextInstanceInClass((VOID *) theDefclass,(VOID *) theInstance))
        {
         if (saveInstanceFunc != NULL)
           (*saveInstanceFunc)(theOutput,theInstance);
         instanceCount++;
        }
     }
   if (inheritFlag)
     {
      for (i = 0 ; i < theDefclass->directSubclasses.classCount ; i++)
        {
         subclass = theDefclass->directSubclasses.classArray[i];
           instanceCount += SaveOrMarkInstancesOfClass(theOutput,currentModule,saveCode,
                                                       subclass,CLIPS_TRUE,traversalID,
                                                       saveInstanceFunc);
        }
     }
   return(instanceCount);
  }
  
/***************************************************
  NAME         : SaveSingleInstanceText
  DESCRIPTION  : Writes given instance to file
  INPUTS       : 1) The logical name of the output
                 2) The instance to save
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance written
  NOTES        : None
 ***************************************************/
static VOID SaveSingleInstanceText(vLogicalName,theInstance)
  VOID *vLogicalName;
  INSTANCE_TYPE *theInstance;
  {
   register unsigned i;
   INSTANCE_SLOT *sp;
   char *logicalName = (char *) vLogicalName;
   
   PrintCLIPS(logicalName,"([");
   PrintCLIPS(logicalName,ValueToString(theInstance->name));
   PrintCLIPS(logicalName,"] of ");
   PrintCLIPS(logicalName,ValueToString(theInstance->cls->header.name));
   for (i = 0 ; i < theInstance->cls->instanceSlotCount ; i++)
     {
      sp = theInstance->slotAddresses[i];
      PrintCLIPS(logicalName,"\n   (");
      PrintCLIPS(logicalName,ValueToString(sp->desc->slotName->name));
      if (sp->type != MULTIFIELD)
        {
         PrintCLIPS(logicalName," ");
         PrintAtom(logicalName,(int) sp->type,sp->value);
        }
      else if (GetInstanceSlotLength(sp) != 0)
        {
         PrintCLIPS(logicalName," ");
         PrintMultifield(logicalName,(MULTIFIELD_PTR) sp->value,0,
                         GetInstanceSlotLength(sp) - 1,CLIPS_FALSE);
        }
      PrintCLIPS(logicalName,")");
     }
   PrintCLIPS(logicalName,")\n\n");
  }

#if BSAVE_INSTANCES

/***************************************************
  NAME         : WriteBinaryHeader
  DESCRIPTION  : Writes identifying string to
                 instance binary file to assist in
                 later verification
  INPUTS       : The binary file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Binary prefix headers written
  NOTES        : None
 ***************************************************/
static VOID WriteBinaryHeader(bsaveFP)
  FILE *bsaveFP;
  {   
   fwrite((VOID *) InstanceBinaryPrefixID,
          (int) (strlen(InstanceBinaryPrefixID) + 1),1,bsaveFP);
   fwrite((VOID *) InstanceBinaryVersionID,
          (int) (strlen(InstanceBinaryVersionID) + 1),1,bsaveFP);
  }

/***************************************************
  NAME         : MarkSingleInstance
  DESCRIPTION  : Marks all the atoms needed in
                 the slot values of an instance
  INPUTS       : 1) The output (ignored)
                 2) The instance
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance slot value atoms marked
  NOTES        : None
 ***************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID MarkSingleInstance(theOutput,theInstance)
  VOID *theOutput;
  INSTANCE_TYPE *theInstance;
  {
#if MAC_MPW
#pragma unused(theOutput)
#endif
   INSTANCE_SLOT *sp;
   register unsigned i,j;
   
   BinaryInstanceFileSize += (unsigned long) (sizeof(long) * 2);
   theInstance->name->neededSymbol = CLIPS_TRUE;
   theInstance->cls->header.name->neededSymbol = CLIPS_TRUE;
   BinaryInstanceFileSize += 
       (unsigned long) ((sizeof(long) * 2) +
                        (sizeof(struct bsaveSlotValue) * 
                         theInstance->cls->instanceSlotCount) +
                        sizeof(unsigned long) +
                        sizeof(unsigned));
   for (i = 0 ; i < theInstance->cls->instanceSlotCount ; i++)
     {
      sp = theInstance->slotAddresses[i];
      sp->desc->slotName->name->neededSymbol = CLIPS_TRUE;
      if (sp->desc->multiple)
        {
         for (j = 1 ; j <= GetInstanceSlotLength(sp) ; j++)
           MarkNeededAtom(GetMFType(sp->value,j),GetMFValue(sp->value,j));
        }
      else
        MarkNeededAtom((int) sp->type,sp->value);
     }
  }

/***************************************************
  NAME         : MarkNeededAtom
  DESCRIPTION  : Marks an integer/float/symbol as
                 being need by a set of instances
  INPUTS       : 1) The type of atom
                 2) The value of the atom
  RETURNS      : Nothing useful
  SIDE EFFECTS : Atom marked for saving
  NOTES        : None
 ***************************************************/
static VOID MarkNeededAtom(type,value)
  int type;
  VOID *value;
  {
   BinaryInstanceFileSize += (unsigned long) sizeof(struct bsaveSlotValueAtom);
  
   /* =====================================
      Assumes slot value atoms  can only be
      floats, integers, symbols, strings,
      instance-names, instance-addresses,
      fact-addresses or external-addresses
      ===================================== */
   switch (type)
     {
      case SYMBOL:
      case STRING:
      case INSTANCE_NAME:
         ((SYMBOL_HN *) value)->neededSymbol = CLIPS_TRUE;
         break;
      case FLOAT:
         ((FLOAT_HN *) value)->neededFloat = CLIPS_TRUE;
         break;
      case INTEGER:
         ((INTEGER_HN *) value)->neededInteger = CLIPS_TRUE;
         break;
      case INSTANCE_ADDRESS:
         GetFullInstanceName((INSTANCE_TYPE *) value)->neededSymbol = CLIPS_TRUE;
         break;
     }
  }
    
/****************************************************
  NAME         : SaveSingleInstanceBinary
  DESCRIPTION  : Writes given instance to binary file
  INPUTS       : 1) Binary file pointer
                 2) The instance to save
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance written
  NOTES        : None
 ****************************************************/
static VOID SaveSingleInstanceBinary(vBsaveFP,theInstance)
  VOID *vBsaveFP;
  INSTANCE_TYPE *theInstance;
  {
   long nameIndex;
   register unsigned i,j;
   INSTANCE_SLOT *sp;
   FILE *bsaveFP = (FILE *) vBsaveFP;
   struct bsaveSlotValue bs;
   unsigned long totalValueCount = 0L;
   int slotLen;
   
   /* ===========================
      Write out the instance name
      =========================== */
   nameIndex = (long) theInstance->name->bucket;
   fwrite((VOID *) &nameIndex,(int) sizeof(long),1,bsaveFP);
   
   /* ========================
      Write out the class name
      ======================== */
   nameIndex = (long) theInstance->cls->header.name->bucket;
   fwrite((VOID *) &nameIndex,(int) sizeof(long),1,bsaveFP);
   
   /* ======================================
      Write out the number of slot-overrides
      ====================================== */
   fwrite((VOID *) &theInstance->cls->instanceSlotCount,
          (int) sizeof(unsigned),1,bsaveFP);
   
   /* =========================================
      Write out the slot names and value counts
      ========================================= */
   for (i = 0 ; i < theInstance->cls->instanceSlotCount ; i++)
     {
      sp = theInstance->slotAddresses[i];
      
      /* ===============================================
         Write out the number of atoms in the slot value
         =============================================== */
      bs.slotName = (long) sp->desc->slotName->name->bucket;
      bs.valueCount = sp->desc->multiple ? GetInstanceSlotLength(sp) : 1;
      fwrite((VOID *) &bs,(int) sizeof(struct bsaveSlotValue),1,bsaveFP);
      totalValueCount += (unsigned long) bs.valueCount;
     }
   
   /* ==================================
      Write out the number of slot value
      atoms for the whole instance
      ================================== */
   if (totalValueCount != 0L)
     fwrite((VOID *) &totalValueCount,(int) sizeof(unsigned long),1,bsaveFP);
      
   /* ==============================
      Write out the slot value atoms
      ============================== */
   for (i = 0 ; i < theInstance->cls->instanceSlotCount ; i++)
     {
      sp = theInstance->slotAddresses[i];
      slotLen = sp->desc->multiple ? GetInstanceSlotLength(sp) : 1;
     
      /* =========================================
         Write out the type and index of each atom
         ========================================= */
      if (sp->desc->multiple)
        {
         for (j = 1 ; j <= slotLen ; j++)
           SaveAtomBinary(GetMFType(sp->value,j),GetMFValue(sp->value,j),bsaveFP);
        }
      else
        SaveAtomBinary((int) sp->type,sp->value,bsaveFP);
     }
  }

/***************************************************
  NAME         : SaveAtomBinary
  DESCRIPTION  : Writes out an instance slot value
                 atom to the binary file
  INPUTS       : 1) The atom type
                 2) The atom value
                 3) The binary file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : atom written
  NOTES        : 
 ***************************************************/
static VOID SaveAtomBinary(type,value,bsaveFP)
  int type;
  VOID *value;
  FILE *bsaveFP;
  {
   struct bsaveSlotValueAtom bsa;
   
   /* =====================================
      Assumes slot value atoms  can only be
      floats, integers, symbols, strings,
      instance-names, instance-addresses,
      fact-addresses or external-addresses
      ===================================== */
   bsa.type = type;
   switch (type)
     {
      case SYMBOL:
      case STRING:
      case INSTANCE_NAME:
         bsa.value = (long) ((SYMBOL_HN *) value)->bucket;
         break;
      case FLOAT:
         bsa.value = (long) ((FLOAT_HN *) value)->bucket;
         break;
      case INTEGER:
         bsa.value = (long) ((INTEGER_HN *) value)->bucket;
         break;
      case INSTANCE_ADDRESS:
         bsa.type = INSTANCE_NAME;
         bsa.value = (long) GetFullInstanceName((INSTANCE_TYPE *) value)->bucket;
         break;
      default:
         bsa.value = -1L;
     }
   fwrite((VOID *) &bsa,(int) sizeof(struct bsaveSlotValueAtom),1,bsaveFP);
  }

#endif
  
/**********************************************************************
  NAME         : LoadOrRestoreInstances
  DESCRIPTION  : Loads instances from named file
  INPUTS       : 1) The name of the input file
                 2) An integer flag indicating whether or
                    not to use message-passing to create
                    the new instances and delete old versions
  RETURNS      : The number of instances loaded/restored
  SIDE EFFECTS : Instances loaded from file
  NOTES        : None
 **********************************************************************/
static long LoadOrRestoreInstances(file,usemsgs)
  char *file;
  int usemsgs;
  {
   DATA_OBJECT temp;
   FILE *sfile,*svload;
   char *ilog;
   EXPRESSION *top;
   int svoverride;
   long instanceCount = 0L;
   
   if ((sfile = fopen(file,"r")) == NULL)
     {
      SetEvaluationError(CLIPS_TRUE);
      return(-1L);
     }
   svload = GetFastLoad();
   ilog = (char *) sfile;
   SetFastLoad(sfile);
   top = GenConstant(FCALL,(VOID *) FindFunction("make-instance"));
   GetToken(ilog,&ObjectParseToken);
   svoverride = MkInsMsgPass;
   MkInsMsgPass = usemsgs;
   while ((GetType(ObjectParseToken) != STOP) && (HaltExecution != CLIPS_TRUE))
     {
      if (GetType(ObjectParseToken) != LPAREN)
        {
         SyntaxErrorMessage("instance definition");
         rtn_struct(expr,top);
         fclose(sfile);
         SetFastLoad(svload);
         SetEvaluationError(CLIPS_TRUE);
         MkInsMsgPass = svoverride;
         return(instanceCount);
        }
      if (ParseSimpleInstance(top,ilog) == NULL)
        {
         fclose(sfile);
         SetFastLoad(svload);
         MkInsMsgPass = svoverride;
         SetEvaluationError(CLIPS_TRUE);
         return(instanceCount);
        }
      EvaluateExpression(top,&temp);
      if (! EvaluationError)
        instanceCount++;
      ReturnExpression(top->argList);
      GetToken(ilog,&ObjectParseToken);
     }
   rtn_struct(expr,top);
   fclose(sfile);
   SetFastLoad(svload);
   MkInsMsgPass = svoverride;
   return(instanceCount);
  }
  
/***************************************************
  NAME         : ProcessFileErrorMessage
  DESCRIPTION  : Prints an error message when a
                 file containing text or binary
                 instances cannot be processed.
  INPUTS       : The name of the input file and the
                 function which opened it.
  RETURNS      : No value
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static VOID ProcessFileErrorMessage(functionName,fileName)
  char *functionName, *fileName;
  {
   PrintErrorID("INSFILE",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," could not completely process file ");
   PrintCLIPS(WERROR,fileName);
   PrintCLIPS(WERROR,".\n");
  }
  
#if BLOAD_INSTANCES

/*******************************************************
  NAME         : VerifyBinaryHeader
  DESCRIPTION  : Reads the prefix and version headers
                 from a file to verify that the
                 input is a valid binary instances file
  INPUTS       : The name of the file
  RETURNS      : CLIPS_TRUE if OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Input prefix and version read
  NOTES        : Assumes file already open with GenOpen
 *******************************************************/
static BOOLEAN VerifyBinaryHeader(theFile)
  char *theFile;
  {
   char buf[20];
   
   GenRead((VOID *) buf,(unsigned long) (strlen(InstanceBinaryPrefixID) + 1));
   if (strcmp(buf,InstanceBinaryPrefixID) != 0)
     {
      PrintErrorID("INSFILE",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,theFile);
      PrintCLIPS(WERROR," file is not a binary instances file.\n");
      return(CLIPS_FALSE);
     }
   GenRead((VOID *) buf,(unsigned long) (strlen(InstanceBinaryVersionID) + 1));
   if (strcmp(buf,InstanceBinaryVersionID) != 0)
     {
      PrintErrorID("INSFILE",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,theFile);
      PrintCLIPS(WERROR," file is not a compatible binary instances file.\n");
      return(CLIPS_FALSE);
     }
   return(CLIPS_TRUE);
  }

/***************************************************
  NAME         : LoadSingleBinaryInstance
  DESCRIPTION  : Reads the binary data for a new
                 instance and its slot values and
                 creates/initializes the instance
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if all OK,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : Binary data read and instance
                 created
  NOTES        : Uses global GenRead()
 ***************************************************/
static BOOLEAN LoadSingleBinaryInstance()
  {
   SYMBOL_HN *instanceName,
             *className;
   unsigned slotCount;
   DEFCLASS *theDefclass;
   INSTANCE_TYPE *newInstance;
   struct bsaveSlotValue *bsArray;
   struct bsaveSlotValueAtom HUGE_ADDR *bsaArray;
   long nameIndex;
   unsigned long totalValueCount;
   register unsigned i;
   unsigned long j;
   INSTANCE_SLOT *sp;
   DATA_OBJECT slotValue;
   
   /* =====================
      Get the instance name
      ===================== */
   BufferedRead((VOID *) &nameIndex,(unsigned long) sizeof(long));
   instanceName = SymbolPointer(nameIndex);
   
   /* ==================
      Get the class name
      ================== */
   BufferedRead((VOID *) &nameIndex,(unsigned long) sizeof(long));
   className = SymbolPointer(nameIndex);
   
   /* ==================
      Get the slot count
      ================== */
   BufferedRead((VOID *) &slotCount,(unsigned long) sizeof(unsigned));

   /* =============================
      Make sure the defclass exists
      and check the slot count
      ============================= */
   theDefclass = LookupDefclassInScope(ValueToString(className));
   if (theDefclass == NULL)
     {
      ClassExistError("bload-instances",ValueToString(className));
      return(CLIPS_FALSE);
     }
   if (theDefclass->instanceSlotCount != slotCount)
     {
      BinaryLoadInstanceError(instanceName,theDefclass);
      return(CLIPS_FALSE);
     }
   
   /* ===================================
      Create the new unitialized instance
      =================================== */
   newInstance = BuildInstance(instanceName,theDefclass,CLIPS_FALSE);
   if (newInstance == NULL)
     {
      BinaryLoadInstanceError(instanceName,theDefclass);
      return(CLIPS_FALSE);
     }
   if (slotCount == 0)
     return(CLIPS_TRUE);
     
   /* ====================================
      Read all slot override info and slot
      value atoms into big arrays
      ==================================== */
   bsArray = (struct bsaveSlotValue *) gm2((int) (sizeof(struct bsaveSlotValue) * slotCount));
   BufferedRead((VOID *) bsArray,(unsigned long) (sizeof(struct bsaveSlotValue) * slotCount));
   
   BufferedRead((VOID *) &totalValueCount,(unsigned long) sizeof(unsigned long));
   
   if (totalValueCount != 0L)
     {
      bsaArray = (struct bsaveSlotValueAtom HUGE_ADDR *) 
                  gm3((long) (totalValueCount * sizeof(struct bsaveSlotValueAtom)));
      BufferedRead((VOID *) bsaArray,
                   (unsigned long) (totalValueCount * sizeof(struct bsaveSlotValueAtom)));
     }
     
   /* =========================
      Insert the values for the
      slot overrides
      ========================= */            
   for (i = 0 , j = 0L ; i < slotCount ; i++)
     {
      /* ===========================================================
         Here is another check for the validity of the binary file -
         the order of the slots in the file should match the
         order in the class definition
         =========================================================== */
      sp = newInstance->slotAddresses[i];
      if (sp->desc->slotName->name != SymbolPointer(bsArray[i].slotName))
        goto LoadError;
      CreateSlotValue(&slotValue,(struct bsaveSlotValueAtom *) &bsaArray[j],
                      bsArray[i].valueCount);
      if (PutSlotValue(newInstance,sp,&slotValue,"bload-instances") == CLIPS_FALSE)
        goto LoadError;
      j += (unsigned long) bsArray[i].valueCount;
     }
     
   rm((VOID *) bsArray,(int) (sizeof(struct bsaveSlotValue) * slotCount));
   
   if (totalValueCount != 0L)
     rm3((VOID *) bsaArray,
         (long) (totalValueCount * sizeof(struct bsaveSlotValueAtom)));
   
   return(CLIPS_TRUE);
   
LoadError:
   BinaryLoadInstanceError(instanceName,theDefclass);
   QuashInstance(newInstance);
   rm((VOID *) bsArray,(int) (sizeof(struct bsaveSlotValue) * slotCount));
   rm3((VOID *) bsaArray,
       (long) (totalValueCount * sizeof(struct bsaveSlotValueAtom)));
   return(CLIPS_FALSE);
  }

/***************************************************
  NAME         : BinaryLoadInstanceError
  DESCRIPTION  : Prints out an error message when
                 an instance could not be
                 successfully loaded from a
                 binary file
  INPUTS       : 1) The instance name
                 2) The defclass
  RETURNS      : Nothing useful
  SIDE EFFECTS : Error message printed
  NOTES        : None
 ***************************************************/
static VOID BinaryLoadInstanceError(instanceName,theDefclass)
  SYMBOL_HN *instanceName;
  DEFCLASS *theDefclass;
  {
   PrintErrorID("INSFILE",4,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function bload-instances unable to load instance [");
   PrintCLIPS(WERROR,ValueToString(instanceName));
   PrintCLIPS(WERROR,"] of class ");
   PrintClassName(WERROR,theDefclass,CLIPS_TRUE);
  }

/***************************************************
  NAME         : CreateSlotValue
  DESCRIPTION  : Creates a data object value from
                 the binary slot value atom data
  INPUTS       : 1) A data object buffer
                 2) The slot value atoms array
                 3) The number of values to put
                    in the data object
  RETURNS      : Nothing useful
  SIDE EFFECTS : Data object initialized
                 (if more than one value, a
                 multifield is created)
  NOTES        : None
 ***************************************************/
static VOID CreateSlotValue(result,bsaValues,valueCount)
  DATA_OBJECT *result;
  struct bsaveSlotValueAtom *bsaValues;
  int valueCount;
  {
   register unsigned i;
   
   if (valueCount == 0)
     {
      result->type = MULTIFIELD;
      result->value = CreateMultifield(0);
      result->begin = 0;
      result->end = -1;
     }
   else if (valueCount == 1)
     {
      result->type = bsaValues[0].type;
      result->value = GetBinaryAtomValue(&bsaValues[0]);
     }
   else
     {
      result->type = MULTIFIELD;
      result->value = CreateMultifield(valueCount);
      result->begin = 0;
      result->end = valueCount - 1;
      for (i = 1 ; i <= valueCount ; i++)
        {
         SetMFType(result->value,i,bsaValues[i-1].type);
         SetMFValue(result->value,i,GetBinaryAtomValue(&bsaValues[i-1]));
        }
     }
  }

/***************************************************
  NAME         : GetBinaryAtomValue
  DESCRIPTION  : Uses the binary index of an atom
                 to find the ephemeris value
  INPUTS       : The binary type and index
  RETURNS      : The symbol/etc. pointer
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static VOID *GetBinaryAtomValue(ba)
  struct bsaveSlotValueAtom *ba;
  {
   switch (ba->type)
     {
      case SYMBOL:
      case STRING:
      case INSTANCE_NAME:
         return((VOID *) SymbolPointer(ba->value));
      case FLOAT:
         return((VOID *) FloatPointer(ba->value));
      case INTEGER:
         return((VOID *) IntegerPointer(ba->value));
      case FACT_ADDRESS:
#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT
         return((VOID *) &DummyFact);
#else
         return(NULL);
#endif
      case EXTERNAL_ADDRESS:
        return(NULL);
      default:
        {
         CLIPSSystemError("INSFILE",1);
         ExitCLIPS(2);
        }
     }
   return(NULL);
  }

/***************************************************
  NAME         : BufferedRead
  DESCRIPTION  : Reads data from binary file
                 (Larger blocks than requested size
                  may be read and buffered)
  INPUTS       : 1) The buffer
                 2) The buffer size
  RETURNS      : Nothing useful
  SIDE EFFECTS : Data stored in buffer
  NOTES        : None
 ***************************************************/
static VOID BufferedRead(buf,bufsz)
  VOID *buf;
  unsigned long bufsz;
  {
   static unsigned long CurrentReadBufferOffset = 0L;
   unsigned long i,amountLeftToRead;

   if (CurrentReadBuffer != NULL)
     {
      amountLeftToRead = CurrentReadBufferSize - CurrentReadBufferOffset;
      if (bufsz <= amountLeftToRead)
        {
         for (i = 0L ; i < bufsz ; i++)
           ((char HUGE_ADDR *) buf)[i] = CurrentReadBuffer[i + CurrentReadBufferOffset];
         CurrentReadBufferOffset += bufsz;
         if (CurrentReadBufferOffset == CurrentReadBufferSize)
           FreeReadBuffer();
        }
      else
        {
         if (CurrentReadBufferOffset < CurrentReadBufferSize)
           {
            for (i = 0L ; i < amountLeftToRead ; i++)
              ((char HUGE_ADDR *) buf)[i] = CurrentReadBuffer[i + CurrentReadBufferOffset];
            bufsz -= amountLeftToRead;
            buf = (VOID *) (((char HUGE_ADDR *) buf) + amountLeftToRead);
           }
         FreeReadBuffer();
         BufferedRead(buf,bufsz);
        }
     }
   else
     {
      if (bufsz > MAX_BLOCK_SIZE)
        {
         CurrentReadBufferSize = bufsz;
         if (bufsz > (BinaryInstanceFileSize - BinaryInstanceFileOffset))
           {
            CLIPSSystemError("INSFILE",2);
            ExitCLIPS(2);
           }
        }
      else if (MAX_BLOCK_SIZE > 
              (BinaryInstanceFileSize - BinaryInstanceFileOffset))
        CurrentReadBufferSize = BinaryInstanceFileSize - BinaryInstanceFileOffset;
      else
        CurrentReadBufferSize = (unsigned long) MAX_BLOCK_SIZE;
      CurrentReadBuffer = (char HUGE_ADDR *) genlongalloc(CurrentReadBufferSize);
      GenRead((VOID *) CurrentReadBuffer,CurrentReadBufferSize);
      for (i = 0L ; i < bufsz ; i++)
        ((char HUGE_ADDR *) buf)[i] = CurrentReadBuffer[i];
      CurrentReadBufferOffset = bufsz;
      BinaryInstanceFileOffset += CurrentReadBufferSize;
     }
  }
  
/*****************************************************
  NAME         : FreeReadBuffer
  DESCRIPTION  : Deallocates buffer for binary reads
  INPUTS       : None
  RETURNS      : Nothing usefu
  SIDE EFFECTS : Binary global read buffer deallocated
  NOTES        : None
 *****************************************************/
static VOID FreeReadBuffer()
  {
   if (CurrentReadBufferSize != 0L)
     {
      genlongfree((VOID *) CurrentReadBuffer,CurrentReadBufferSize);
      CurrentReadBuffer = NULL;
      CurrentReadBufferSize = 0L;
     }
  }
  
#endif
    
#endif
 
/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/


