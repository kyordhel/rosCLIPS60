   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                    BLOAD MODULE                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _BLOAD_SOURCE_

#include "setup.h"

#include "clipsmem.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"
#include "constrct.h"
#include "bsave.h"
#include "cstrnbin.h"
#include "utility.h"

#include "bload.h"

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static struct FunctionDefinition * HUGE_ADDR *ReadNeededFunctions(long *,int *);
   static struct FunctionDefinition  *FastFindFunction(char *,struct FunctionDefinition *);
   static int                         ClearBload(void);
   static VOID                        AbortBload(void);
   static int                         BloadOutOfMemoryFunction(unsigned long);
#else
   static struct FunctionDefinition * HUGE_ADDR *ReadNeededFunctions();
   static struct FunctionDefinition  *FastFindFunction();
   static int                         ClearBload();
   static VOID                        AbortBload();
   static int                         BloadOutOfMemoryFunction();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static int                                    BloadActive = CLIPS_FALSE;
   static struct callFunctionItem               *BeforeBloadFunctions = NULL;
   static struct callFunctionItem               *AfterBloadFunctions = NULL;
   static struct callFunctionItem               *BloadReadyFunctions = NULL;
   static struct callFunctionItem               *ClearBloadReadyFunctions = NULL;
   static struct callFunctionItem               *AbortBloadFunctions = NULL;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle char                                  *BinaryPrefixID = "\1\2\3\4CLIPS";
   globle char                                  *BinaryVersionID = "V6.00";
   globle struct FunctionDefinition * HUGE_ADDR *FunctionArray;

/*******************************************************/
/* Bload: Loads the binary representation of the CLIPS */
/*   environment from a file.                          */
/*******************************************************/
globle int Bload(fileName)
  char *fileName;
  {
   long numberOfFunctions;
   unsigned long space;
   int error,ready;
   char IDbuffer[20];
   char constructBuffer[CONSTRUCT_HEADER_SIZE];
   struct BinaryItem *biPtr;
   struct callFunctionItem *bfPtr;

   /*================*/
   /* Open the file. */
   /*================*/

   if (GenOpen("bload",fileName) == 0) return(CLIPS_FALSE);

   /*=====================================*/
   /* Determine if this is a binary file. */
   /*=====================================*/

   GenRead(IDbuffer,(unsigned long) strlen(BinaryPrefixID) + 1);
   if (strcmp(IDbuffer,BinaryPrefixID) != 0)
     {
      PrintErrorID("BLOAD",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"File ");
      PrintCLIPS(WERROR,fileName);
      PrintCLIPS(WERROR," is not a binary construct file.\n");
      GenClose();
      return(CLIPS_FALSE);
     }

   GenRead(IDbuffer,(unsigned long) strlen(BinaryVersionID) + 1);
   if (strcmp(IDbuffer,BinaryVersionID) != 0)
     {
      PrintErrorID("BLOAD",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,"File ");
      PrintCLIPS(WERROR,fileName);
      PrintCLIPS(WERROR," is an incompatible binary construct file.\n");
      GenClose();
      return(CLIPS_FALSE);
     }

   /*====================*/
   /* Clear environment. */
   /*====================*/

   if (BloadActive)
     {
      if (ClearBload() == CLIPS_FALSE)
        {
         GenClose();
         return(CLIPS_FALSE);
        }
     }

   if (ClearReady() == CLIPS_FALSE)
     {
      GenClose();
      PrintCLIPS(WERROR,"The CLIPS environment could not be cleared.\n");
      PrintCLIPS(WERROR,"Binary load cannot continue.\n");
      return(CLIPS_FALSE);
     }
     
   bfPtr = BeforeBloadFunctions;
   while (bfPtr != NULL)
     {
      (*bfPtr->func)();
      bfPtr = bfPtr->next;
     }

   /*========================================*/
   /* Make sure it's safe to do a bload now. */
   /* This section isn't really necessary now. */
   /*========================================*/

   bfPtr = BloadReadyFunctions;
   error = CLIPS_FALSE;
   while (bfPtr != NULL)
     {
      ready = (* ((int (*)(VOID_ARG)) bfPtr->func))();
      if (ready == CLIPS_FALSE)
        {
         if (! error)
           { 
            PrintErrorID("BLOAD",5,CLIPS_FALSE);
            PrintCLIPS(WERROR,"Some constructs are still in use by the current binary image:\n"); 
           }
         PrintCLIPS(WERROR,"   ");
         PrintCLIPS(WERROR,bfPtr->name);
         PrintCLIPS(WERROR,"\n");
         error = CLIPS_TRUE;
        }
      bfPtr = bfPtr->next;
     }

   if (error == CLIPS_TRUE)
     {
      PrintCLIPS(WERROR,"Binary load cannot continue.\n");
      GenClose();
      AbortBload();
      return(CLIPS_FALSE);
     }

   /*====================================================*/
   /* Read in the functions needed by this binary image. */
   /*====================================================*/

   FunctionArray = ReadNeededFunctions(&numberOfFunctions,&error);
   if (error)
     {
      GenClose();
      AbortBload();
      return(CLIPS_FALSE);
     }

   /*================================================*/
   /* Read in the atoms needed by this binary image. */
   /*================================================*/

   ReadNeededAtomicValues();
   
   /*=================================================*/
   /* Determine the number of expressions to be read  */
   /* and allocate the appropriate space              */
   /*=================================================*/
   
   AllocateExpressions();
   
   /*==========================================================*/
   /* Read in the memory requirements of the constructs stored */
   /* in this binary image and allocate the necessary space    */
   /*==========================================================*/
   
   GenRead(constructBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE);
   while (strncmp(constructBuffer,BinaryPrefixID,CONSTRUCT_HEADER_SIZE) != 0)
     {
      BOOLEAN found;
      
      found = CLIPS_FALSE;
      biPtr = ListOfBinaryItems;
      while (biPtr != NULL)
        {
         if (strncmp(biPtr->name,constructBuffer,CONSTRUCT_HEADER_SIZE) == 0)
           {
            if (biPtr->bloadStorageFunction != NULL)
              {
               (*biPtr->bloadStorageFunction)();
               found = CLIPS_TRUE;
              }
            biPtr = NULL;
           }
         else biPtr = biPtr->next;
        }
      if (! found)
        {
         GenRead(&space,(unsigned long) sizeof(unsigned long));
         GenSeek((long) space);
         if (space != 0)
           {
            PrintCLIPS(WDIALOG,"\nSkipping ");
            PrintCLIPS(WDIALOG,constructBuffer);
            PrintCLIPS(WDIALOG," constructs because of unavailibility\n");
           }
        }
      GenRead(constructBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE);
     }

   /*===============================*/
   /* Set pointers in expressions.  */
   /*===============================*/
   
   RefreshExpressions();
   
   /*==========================*/
   /* Read in the constraints. */
   /*==========================*/
   
   ReadNeededConstraints();
   
   /*======================================================*/
   /* Read in the constructs stored in this binary image.  */
   /*======================================================*/

   GenRead(constructBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE);
   while (strncmp(constructBuffer,BinaryPrefixID,CONSTRUCT_HEADER_SIZE) != 0)
     {
      BOOLEAN found;
      
      found = CLIPS_FALSE;
      biPtr = ListOfBinaryItems;
      while (biPtr != NULL)
        {
         if (strncmp(biPtr->name,constructBuffer,CONSTRUCT_HEADER_SIZE) == 0)
           {
            if (biPtr->bloadFunction != NULL)
              {
               (*biPtr->bloadFunction)();
               found = CLIPS_TRUE;
              }
            biPtr = NULL;
           }
         else biPtr = biPtr->next;
        }
        
      if (! found)
        {
         GenRead(&space,(unsigned long) sizeof(unsigned long));
         GenSeek((long) space);
        }
      GenRead(constructBuffer,(unsigned long) CONSTRUCT_HEADER_SIZE);
     }

   /*=================*/
   /* Close the file. */
   /*=================*/
   
   GenClose();

   /*==================*/
   /* Free up storage. */
   /*==================*/

   if (FunctionArray != NULL)
     genlongfree((VOID *) FunctionArray,(unsigned long) sizeof(struct FunctionDefinition *) * numberOfFunctions);
   FreeAtomicValueStorage();
    
   /*=============================*/
   /* Do any reset functionality. */
   /*=============================*/

   bfPtr = AfterBloadFunctions;
   while (bfPtr != NULL)
     {
      (*bfPtr->func)();
      bfPtr = bfPtr->next;
     }

   /*============================================*/
   /* Add a clear function to remove binary load */
   /* when a clear command is issued.            */
   /*============================================*/

   BloadActive = CLIPS_TRUE;
   AddClearFunction("bload",(VOID (*)(VOID_ARG)) ClearBload,10000);

   /*================================================*/
   /* Return CLIPS_TRUE to indicate load successful. */
   /*================================================*/

   return(CLIPS_TRUE);
  }

/************************************************************
  NAME         : BloadandRefresh
  DESCRIPTION  : Loads and refreshes objects - will bload
                 all objects at once, if possible, but
                 will aslo work in increments if memory is
                 restricted
  INPUTS       : 1) the number of objects to bload and update
                 2) the size of one object
                 3) An update function which takes a bloaded
                    object buffer and the index of the object
                    to refresh as arguments
  RETURNS      : Nothing useful
  SIDE EFFECTS : Objects bloaded and updated
  NOTES        : Assumes binary file pointer is positioned
                 for bloads of the objects
 ************************************************************/
globle VOID BloadandRefresh(objcnt,objsz,objupdate)
  long objcnt;
  unsigned objsz;
#if ANSI_COMPILER
  VOID (*objupdate)(VOID *,long);
#else
  VOID (*objupdate)();
#endif
  {
   register long i,bi;
   char HUGE_ADDR *buf;
   long objsmaxread,objsread;
   unsigned long space;
#if ANSI_COMPILER
   int (*oldOutOfMemoryFunction)(unsigned long);
#else
   int (*oldOutOfMemoryFunction)();
#endif
   
   if (objcnt == 0L)
     return;
   oldOutOfMemoryFunction = SetOutOfMemoryFunction(BloadOutOfMemoryFunction);
   objsmaxread = objcnt;
   buf = NULL;
   do
     {
      space = objsmaxread * objsz;
      buf = (char HUGE_ADDR *) genlongalloc(space);
      if (buf == NULL)
        {
         if ((objsmaxread / 2) == 0)
           {
            if ((*oldOutOfMemoryFunction)(space) == CLIPS_TRUE)
              {
               SetOutOfMemoryFunction(oldOutOfMemoryFunction);
               return;
              }
           }
         else
           objsmaxread /= 2;
        }
     }
   while (buf == NULL);
   SetOutOfMemoryFunction(oldOutOfMemoryFunction);
   
   i = 0L;
   do
     {
      objsread = (objsmaxread > (objcnt - i)) ? (objcnt - i) : objsmaxread;
      GenRead((VOID *) buf,objsread * objsz);
      for (bi = 0L ; bi < objsread ; bi++ , i++)
        (*objupdate)(buf + objsz * bi,i);
     }
   while (i < objcnt);
   genlongfree((VOID *) buf,space);
  }

/********************************************************/
/* ReadNeededFunctions: Reads in the names of functions */
/*   needed by the binary image.                        */
/********************************************************/
static struct FunctionDefinition * HUGE_ADDR *ReadNeededFunctions(numberOfFunctions,error)
  long int *numberOfFunctions;
  int *error;
  {
   char *functionNames, *namePtr;
   unsigned long int space,temp;
   long i;
   struct FunctionDefinition * HUGE_ADDR *newFunctionArray, *functionPtr;
   int functionsNotFound = 0;

   /*===================================================*/
   /* Determine the number of function names to be read */
   /* and the space required for them.                  */
   /*===================================================*/

   GenRead(numberOfFunctions,(unsigned long) sizeof(long int));
   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   if (*numberOfFunctions == 0)
     {
      *error = CLIPS_FALSE;
      return(NULL);
     }

   /*=======================================*/
   /* Allocate area for strings to be read. */
   /*=======================================*/

   functionNames = (char *) genlongalloc(space);
   GenRead((VOID *) functionNames,space);

   /*====================================================*/
   /* Store the function pointers in the function array. */
   /*====================================================*/

   temp = (unsigned long) sizeof(struct FunctionDefinition *) * *numberOfFunctions;
   newFunctionArray = (struct FunctionDefinition **) genlongalloc(temp);
   namePtr = functionNames;
   functionPtr = NULL;
   for (i = 0; i < *numberOfFunctions; i++)
     {
      if ((functionPtr = FastFindFunction(namePtr,functionPtr)) == NULL)
        {
         if (! functionsNotFound)
           {
            PrintErrorID("BLOAD",6,CLIPS_FALSE);
            PrintCLIPS(WERROR,"The following undefined functions are ");
            PrintCLIPS(WERROR,"referenced by this binary image:\n");
           }

         PrintCLIPS(WERROR,"   ");
         PrintCLIPS(WERROR,namePtr);
         PrintCLIPS(WERROR,"\n");
         functionsNotFound = 1;
        }

      newFunctionArray[i] = functionPtr;
      namePtr += strlen(namePtr) + 1;
     }

   /*==========================================*/
   /* Free the memory used by the name buffer. */
   /*==========================================*/

   genlongfree((VOID *) functionNames,space);

   /*==================================================*/
   /* If any of the required functions were not found, */
   /* then free the memory used by the function array. */
   /*==================================================*/

   if (functionsNotFound)
     {
      genlongfree((VOID *) newFunctionArray,temp);
      newFunctionArray = NULL;
     }

   /*===================================*/
   /* Set globals to appropriate values */
   /* and return the function array.    */
   /*===================================*/

   *error = functionsNotFound;
   return(newFunctionArray);
  }

/*************************************************/
/* FastFindFunction: Search the CLIPS function */
/*   list for a specific function.               */
/*************************************************/
static struct FunctionDefinition *FastFindFunction(fun_name,end_function)
  char *fun_name;
  struct FunctionDefinition *end_function;
  {
   struct FunctionDefinition *fun_list, *fun_ptr;

   fun_list = GetFunctionList();
   if (fun_list == NULL) { return(NULL); }

   if (end_function != NULL)
     { fun_ptr = end_function->next; }
   else
     { fun_ptr = fun_list; }

   while (strcmp(fun_name,ValueToString(fun_ptr->callFunctionName)) != 0)
     {
      fun_ptr = fun_ptr->next;
      if (fun_ptr == end_function) return(NULL);
      if (fun_ptr == NULL) fun_ptr = fun_list;
     }

   return(fun_ptr);
  }



/***********************************************************/
/* BLOADED: Returns CLIPS_TRUE if the current environment is the */
/*   result of a bload command, otherwise returns CLIPS_FALSE.   */
/***********************************************************/
globle BOOLEAN Bloaded()
  {
   return(BloadActive);
  }

/*****************************************************************/
/* ClearBload: Clears a binary image from the CLIPS environment. */
/*****************************************************************/
static int ClearBload()
  {
   struct BinaryItem *biPtr;
   struct callFunctionItem *bfPtr;
   int ready,error;

   /* ===========================================
      Make sure it's safe to do a clear-bload now
      =========================================== */
   bfPtr = ClearBloadReadyFunctions;
   error = CLIPS_FALSE;
   while (bfPtr != NULL)
     {
      ready = (* ((int (*)(VOID_ARG)) bfPtr->func))();
      if (ready == CLIPS_FALSE)
        {
         if (! error)
           {
            PrintErrorID("BLOAD",5,CLIPS_FALSE); 
            PrintCLIPS(WERROR,"Some constructs are still in use by the current binary image:\n");
           }
         PrintCLIPS(WERROR,"   ");
         PrintCLIPS(WERROR,bfPtr->name);
         PrintCLIPS(WERROR,"\n");
         error = CLIPS_TRUE;
        }
      bfPtr = bfPtr->next;
     }

   if (error == CLIPS_TRUE)
     {
      PrintCLIPS(WERROR,"Binary clear cannot continue.\n");
      return(CLIPS_FALSE);
     }

   /*=============================*/
   /* Call bload clear functions. */
   /*=============================*/

   biPtr = ListOfBinaryItems;
   while (biPtr != NULL)
     {
      if (biPtr->clearFunction != NULL) (*biPtr->clearFunction)();
      biPtr = biPtr->next;
     }
     
   /*===========================*/
   /* Free bloaded expressions. */
   /*===========================*/
   
   ClearBloadedExpressions();
   
   /*===========================*/
   /* Free bloaded constraints. */
   /*===========================*/
   
   ClearBloadedConstraints();
   
   /*==================================*/
   /* Remove the bload clear function. */
   /*==================================*/

   BloadActive = CLIPS_FALSE;
   RemoveClearFunction("bload");
   return(CLIPS_TRUE);
  }

/*******************************************************************************/
/* AbortBload: Cleans up effects of before-bload functions in event of failure */
/*******************************************************************************/
static VOID AbortBload()
  {
   struct callFunctionItem *bfPtr;

   bfPtr = AbortBloadFunctions;
   while (bfPtr != NULL)
     {
      (*bfPtr->func)();
      bfPtr = bfPtr->next;
     }
  }

/*****************************/
/* AddBeforeBloadFunction:   */
/*****************************/
globle VOID AddBeforeBloadFunction(name,func,priority)
  char *name;
  VOID (*func)(VOID_ARG);
  int priority;
  {
   BeforeBloadFunctions = AddFunctionToCallList(name,priority,func,BeforeBloadFunctions);
  }

/*****************************/
/* AddAfterBloadFunction:   */
/*****************************/
globle VOID AddAfterBloadFunction(name,func,priority)
  char *name;
  VOID (*func)(VOID_ARG);
  int priority;
  {
   AfterBloadFunctions = AddFunctionToCallList(name,priority,func,AfterBloadFunctions);
  }

/*****************************/
/* AddBloadReadyFunction:   */
/*****************************/
globle VOID AddBloadReadyFunction(name,func,priority)
  char *name;
  int (*func)(VOID_ARG);
  int priority;
  {
   BloadReadyFunctions = AddFunctionToCallList(name,priority,
                         (VOID (*)(VOID_ARG)) func,BloadReadyFunctions);
  }

/*********************************/
/* AddClearBloadReadyFunction:   */
/*********************************/
globle VOID AddClearBloadReadyFunction(name,func,priority)
  char *name;
  int (*func)(VOID_ARG);
  int priority;
  {
   ClearBloadReadyFunctions = AddFunctionToCallList(name,priority,
                            (VOID (*)(VOID_ARG)) func,ClearBloadReadyFunctions);
  }

/*****************************/
/* AddAbortBloadFunction:   */
/*****************************/
globle VOID AddAbortBloadFunction(name,func,priority)
  char *name;
  VOID (*func)(VOID_ARG);
  int priority;
  {
   AbortBloadFunctions = AddFunctionToCallList(name,priority,func,AbortBloadFunctions);
  }

/*******************************************************
  NAME         : BloadOutOfMemoryFunction
  DESCRIPTION  : Memory function used by bload to
                   force CLIPS not to exit when out
                   of memory - used by BloadandRefresh
  INPUTS       : The memory request size (unused)
  RETURNS      : CLIPS_TRUE (indicates a failure and for
                 the CLIPS memory functions to simply
                 return a NULL pointer)
  SIDE EFFECTS : None
  NOTES        : None
 *******************************************************/
#if IBM_TBC
#pragma argsused
#endif
static int BloadOutOfMemoryFunction(size)
  unsigned long size;
  {
#if MAC_MPW
#pragma unused(size)
#endif
   return(CLIPS_TRUE);
  }
  
/*****************************/
/* CannotLoadWithBloadMessage:   */
/*****************************/
globle VOID CannotLoadWithBloadMessage(constructName)
  char *constructName;
  {      
   PrintErrorID("BLOAD",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Cannot load ");
   PrintCLIPS(WERROR,constructName);
   PrintCLIPS(WERROR," construct with binary load in effect.\n");
  }

#endif

/********************************************************/
/* BloadCommand: Performs the top level bload command. */
/********************************************************/
globle int BloadCommand()
  {
#if (! RUN_TIME) && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)
   char *fileName;

   if (ArgCountCheck("bload",EXACTLY,1) == -1) return(CLIPS_FALSE);
   fileName = GetFileName("bload",1);
   if (fileName != NULL) return(Bload(fileName));
#endif
   return(CLIPS_FALSE);
  }
