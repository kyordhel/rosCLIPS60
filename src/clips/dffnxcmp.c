   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose: Generic Function Construct Compiler Code         */
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

#if DEFFUNCTION_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME)

#include "conscomp.h"

#define _DFFNXCMP_SOURCE_
#include "dffnxcmp.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
   
/* =========================================
   *****************************************
               MACROS AND TYPES
   =========================================
   ***************************************** */
   
/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */   
#if ANSI_COMPILER

static VOID ReadyDeffunctionsForCode(void);
static int DeffunctionsToCode(char *,int,FILE *,int,int);
static VOID CloseDeffunctionFiles(FILE *,FILE *,int);  
static VOID DeffunctionModuleToCode(FILE *,struct defmodule *,int,int);
static VOID SingleDeffunctionToCode(FILE *,DEFFUNCTION *,int,int,int);

#else

static VOID ReadyDeffunctionsForCode();
static int DeffunctionsToCode();
static VOID CloseDeffunctionFiles();  
static VOID DeffunctionModuleToCode();
static VOID SingleDeffunctionToCode();

#endif
      
/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static struct CodeGeneratorItem *DeffunctionCodeItem;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***************************************************
  NAME         : SetupDeffunctionCompiler
  DESCRIPTION  : Initializes the construct compiler
                   item for deffunctions
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Code generator item initialized
  NOTES        : None
 ***************************************************/
globle VOID SetupDeffunctionCompiler()
  {
   DeffunctionCodeItem = AddCodeGeneratorItem("deffunctions",0,ReadyDeffunctionsForCode,
                                              NULL,DeffunctionsToCode,2);
  }
  
   
/***************************************************
  NAME         : PrintDeffunctionReference
  DESCRIPTION  : Prints a reference to the run-time
                 deffunction array for the construct
                 compiler
  INPUTS       : 1) The file output destination
                 2) A pointer to the deffunction
                 3) The id of the run-time image
                 4) The maximum number of indices
                    in any array
  RETURNS      : Nothing useful
  SIDE EFFECTS : Reference printed
  NOTES        : None
 ***************************************************/
globle VOID PrintDeffunctionReference(fp,dfPtr,imageID,maxIndices)
  FILE *fp;
  DEFFUNCTION *dfPtr;
  int imageID;
  int maxIndices;
  {
   if (dfPtr == NULL)
     fprintf(fp,"NULL");
   else
     fprintf(fp,"&%s%d_%d[%d]",ConstructPrefix(DeffunctionCodeItem),imageID,
                               (int) ((dfPtr->header.bsaveID / maxIndices) + 1),
                               (int) (dfPtr->header.bsaveID % maxIndices)); 
  }

/****************************************************
  NAME         : DeffunctionCModuleReference
  DESCRIPTION  : Prints out a reference to a
                 deffunction module
  INPUTS       : 1) The output file
                 2) The id of the module item
                 3) The id of the image
                 4) The maximum number of elements
                    allowed in an array
  RETURNS      : Nothing useful
  SIDE EFFECTS : Deffunction module reference printed
  NOTES        : None
 ****************************************************/
globle VOID DeffunctionCModuleReference(theFile,count,imageID,maxIndices)
  FILE *theFile;
  int count;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"MIHS &%s%d_%d[%d]",
                      ModulePrefix(DeffunctionCodeItem),
                      imageID,
                      (count / maxIndices) + 1,
                      (count % maxIndices));
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***************************************************
  NAME         : ReadyDeffunctionsForCode
  DESCRIPTION  : Sets index of deffunctions
                   for use in compiled expressions
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : BsaveIndices set
  NOTES        : None
 ***************************************************/
static VOID ReadyDeffunctionsForCode()
  {
   MarkConstructBsaveIDs(DeffunctionModuleIndex);
  }

/*******************************************************
  NAME         : DeffunctionsToCode
  DESCRIPTION  : Writes out static array code for
                   deffunctions
  INPUTS       : 1) The base name of the construct set
                 2) The base id for this construct
                 3) The file pointer for the header file
                 4) The base id for the construct set
                 5) The max number of indices allowed
                    in an array
  RETURNS      : -1 if no deffunctions, 0 on errors,
                  1 if deffunctions written
  SIDE EFFECTS : Code written to files
  NOTES        : None
 *******************************************************/
static int DeffunctionsToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   int fileCount = 1;
   struct defmodule *theModule;
   DEFFUNCTION *theDeffunction;
   int moduleCount = 0, moduleArrayCount = 0, moduleArrayVersion = 1;  
   int deffunctionArrayCount = 0, deffunctionArrayVersion = 1;
   FILE *moduleFile = NULL, *deffunctionFile = NULL;
  
   /* ===============================================
      Include the appropriate deffunction header file
      =============================================== */
   fprintf(headerFP,"#include \"dffnxfun.h\"\n");

   /* =============================================================
      Loop through all the modules and all the deffunctions writing
      their C code representation to the file as they are traversed
      ============================================================= */
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {           
      SetCurrentModule((VOID *) theModule);
            
      moduleFile = OpenFileIfNeeded(moduleFile,fileName,fileID,imageID,&fileCount,
                                    moduleArrayVersion,headerFP,
                                    "DEFFUNCTION_MODULE",ModulePrefix(DeffunctionCodeItem),
                                    CLIPS_FALSE,NULL);
                                    
      if (moduleFile == NULL)
        {
         CloseDeffunctionFiles(moduleFile,deffunctionFile,maxIndices);
         return(0);
        }
        
      DeffunctionModuleToCode(moduleFile,theModule,imageID,maxIndices);
      moduleFile = CloseFileIfNeeded(moduleFile,&moduleArrayCount,&moduleArrayVersion,
                                     maxIndices,NULL,NULL);

      theDeffunction = (DEFFUNCTION *) GetNextDeffunction(NULL);

      while (theDeffunction != NULL)
        {
         deffunctionFile = OpenFileIfNeeded(deffunctionFile,fileName,fileID,imageID,&fileCount,
                                            deffunctionArrayVersion,headerFP,
                                            "DEFFUNCTION",ConstructPrefix(DeffunctionCodeItem),
                                            CLIPS_FALSE,NULL);
         if (deffunctionFile == NULL)
           {
            CloseDeffunctionFiles(moduleFile,deffunctionFile,maxIndices);
            return(0);
           }
           
         SingleDeffunctionToCode(deffunctionFile,theDeffunction,imageID,
                                 maxIndices,moduleCount);
         deffunctionArrayCount++;
         deffunctionFile = CloseFileIfNeeded(deffunctionFile,&deffunctionArrayCount,
                                             &deffunctionArrayVersion,maxIndices,NULL,NULL);
                                         
         theDeffunction = (DEFFUNCTION *) GetNextDeffunction(theDeffunction);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
      moduleCount++;
      moduleArrayCount++;
     }
        
   CloseDeffunctionFiles(moduleFile,deffunctionFile,maxIndices);
     
   return(1);
  }

/***************************************************
  NAME         : CloseDeffunctionFiles
  DESCRIPTION  : Closes construct compiler files
                  for deffunction structures
  INPUTS       : 1) The deffunction module file
                 2) The deffunction structure file
                 3) The maximum number of indices
                    allowed in an array
  RETURNS      : Nothing useful
  SIDE EFFECTS : Files closed
  NOTES        : None
 ***************************************************/
static VOID CloseDeffunctionFiles(moduleFile,deffunctionFile,maxIndices)
  FILE *moduleFile, *deffunctionFile; 
  int maxIndices;  
  {
   int count = maxIndices;
   int arrayVersion = 0;
     
   if (deffunctionFile != NULL) 
     {
      count = maxIndices;
      deffunctionFile = CloseFileIfNeeded(deffunctionFile,&count,&arrayVersion,
                                          maxIndices,NULL,NULL);
     }
     
   if (moduleFile != NULL) 
     {
      count = maxIndices;
      moduleFile = CloseFileIfNeeded(moduleFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
  }

/***************************************************
  NAME         : DeffunctionModuleToCode
  DESCRIPTION  : Writes out the C values for a
                 deffunction module item
  INPUTS       : 1) The output file
                 2) The module for the deffunctions
                 3) The compile image id
                 4) The maximum number of elements
                    in an array
  RETURNS      : Nothing useful
  SIDE EFFECTS : Deffunction module item written
  NOTES        : None
 ***************************************************/
static VOID DeffunctionModuleToCode(theFile,theModule,imageID,maxIndices)
  FILE *theFile;
  struct defmodule *theModule;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"{"); 
   ConstructModuleToCode(theFile,theModule,imageID,maxIndices,
                         DeffunctionModuleIndex,ConstructPrefix(DeffunctionCodeItem));
   fprintf(theFile,"}"); 
  }
  
/***************************************************
  NAME         : SingleDeffunctionToCode
  DESCRIPTION  : Writes out a single deffunction's
                 data to the file
  INPUTS       : 1) The output file
                 2) The deffunction
                 3) The compile image id
                 4) The maximum number of
                    elements in an array
                 5) The module index
  RETURNS      : Nothing useful
  SIDE EFFECTS : Deffunction data written
  NOTES        : None
 ***************************************************/
static VOID SingleDeffunctionToCode(theFile,theDeffunction,imageID,maxIndices,moduleCount)
  FILE *theFile;
  DEFFUNCTION *theDeffunction;
  int imageID;
  int maxIndices;
  int moduleCount;
  {   
   /* ==================
      Deffunction Header
      ================== */
   
   fprintf(theFile,"{");
   ConstructHeaderToCode(theFile,&theDeffunction->header,imageID,maxIndices,moduleCount,
                         ModulePrefix(DeffunctionCodeItem),
                         ConstructPrefix(DeffunctionCodeItem));
     
   /* =========================
      Deffunction specific data
      ========================= */
   fprintf(theFile,",0,0,0,"); 
   ExpressionToCode(theFile,theDeffunction->code);
   fprintf(theFile,",%d,%d,%d",
           theDeffunction->minNumberOfParameters,
           theDeffunction->maxNumberOfParameters,
           theDeffunction->numberOfLocalVars);
   
   fprintf(theFile,"}");
  }

#endif

/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
