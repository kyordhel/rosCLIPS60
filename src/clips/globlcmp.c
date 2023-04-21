   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFGLOBAL CONSTRUCTS-TO-C MODULE         */
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

#define _GLOBLCMP_SOURCE_

#include "setup.h"

#if DEFGLOBAL_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "conscomp.h"
#include "globldef.h"

#include "globlcmp.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     ConstructToCode(char *,int,FILE *,int,int);
   static VOID                    DefglobalToCode(FILE *,struct defglobal *,
                                                 int,int,int);
   static VOID                    DefglobalModuleToCode(FILE *,struct defmodule *,int,int,int);
   static VOID                    CloseDefglobalFiles(FILE *,FILE *,int);
   static VOID                    BeforeDefglobalsToCode(void);
   static VOID                    InitDefglobalsCode(FILE *,int,int);
#else
   static int                     ConstructToCode();
   static VOID                    DefglobalToCode();
   static VOID                    DefglobalModuleToCode();
   static VOID                    CloseDefglobalFiles();
   static VOID                    BeforeDefglobalsToCode();
   static VOID                    InitDefglobalsCode();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/
   static struct CodeGeneratorItem *DefglobalCodeItem;

/***************************************************************/
/* DefglobalCompilerSetup: Initializes the defglobal construct */
/*    for use with the constructs-to-c command.                */
/***************************************************************/
globle VOID DefglobalCompilerSetup()
  {
   DefglobalCodeItem = AddCodeGeneratorItem("defglobal",0,BeforeDefglobalsToCode,
                                            InitDefglobalsCode,ConstructToCode,2);
  }
  
/**************************************************************/
/* BeforeDefglobalsToCode: Assigns each defglobal a unique ID */
/*   which will be used for pointer references when the data  */
/*   structures are written to a file as C code               */
/**************************************************************/ 
static VOID BeforeDefglobalsToCode()
  {
   MarkConstructBsaveIDs(DefglobalModuleIndex);
  }

/**************************************************************/
/* InitDefglobalsCode:   */
/**************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID InitDefglobalsCode(initFP,imageID,maxIndices)
  FILE *initFP;
  int imageID;
  int maxIndices;
  {
#if MAC_MPW
#pragma unused(maxIndices)
#pragma unused(imageID)
#endif
   fprintf(initFP,"   ResetDefglobals();\n");
  }
  
/***********************************************************/
/* ConstructToCode: Produces defglobal code for a run-time */
/*   module created using the constructs-to-c function.    */
/***********************************************************/
static int ConstructToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   int fileCount = 1;
   struct defmodule *theModule;
   struct defglobal *theDefglobal;
   int moduleCount = 0, moduleArrayCount = 0, moduleArrayVersion = 1;  
   int defglobalCount = 0, defglobalArrayCount = 0, defglobalArrayVersion = 1;
   FILE *moduleFile = NULL, *defglobalFile = NULL;
  
   /*================================================*/
   /* Include the appropriate defglobal header file. */
   /*================================================*/
   
   fprintf(headerFP,"#include \"globldef.h\"\n");

   /*===================================================================*/
   /* Loop through all the modules and all the defglobals writing their */
   /*  C code representation to the file as they are traversed.         */
   /*===================================================================*/
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {           
      SetCurrentModule((VOID *) theModule);
            
      moduleFile = OpenFileIfNeeded(moduleFile,fileName,fileID,imageID,&fileCount,
                                    moduleArrayVersion,headerFP,
                                    "struct defglobalModule",ModulePrefix(DefglobalCodeItem),
                                    CLIPS_FALSE,NULL);
                                    
      if (moduleFile == NULL)
        {
         CloseDefglobalFiles(moduleFile,defglobalFile,maxIndices);
         return(0);
        }
        
      DefglobalModuleToCode(moduleFile,theModule,imageID,maxIndices,moduleCount);
      moduleFile = CloseFileIfNeeded(moduleFile,&moduleArrayCount,&moduleArrayVersion,
                                     maxIndices,NULL,NULL);

      theDefglobal = (struct defglobal *) GetNextDefglobal(NULL);

      while (theDefglobal != NULL)
        {
         defglobalFile = OpenFileIfNeeded(defglobalFile,fileName,fileID,imageID,&fileCount,
                                         defglobalArrayVersion,headerFP,
                                         "struct defglobal",ConstructPrefix(DefglobalCodeItem),
                                         CLIPS_FALSE,NULL);
         if (defglobalFile == NULL)
           {
            CloseDefglobalFiles(moduleFile,defglobalFile,maxIndices);
            return(0);
           }
           
         DefglobalToCode(defglobalFile,theDefglobal,imageID,maxIndices,moduleCount);
         defglobalCount++;
         defglobalArrayCount++;
         defglobalFile = CloseFileIfNeeded(defglobalFile,&defglobalArrayCount,
                                           &defglobalArrayVersion,maxIndices,NULL,NULL);
                                         
         theDefglobal = (struct defglobal *) GetNextDefglobal(theDefglobal);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
      moduleCount++;
      moduleArrayCount++;
     }
        
   CloseDefglobalFiles(moduleFile,defglobalFile,maxIndices);
     
   return(1);
  }
  
/**********************************************************/
/* CloseDefglobalFiles: Closes all of the C files created */
/*   for defglobals. Called when an error occurs or when  */
/*   the defglobals have all been written to the files.   */
/**********************************************************/
static VOID CloseDefglobalFiles(moduleFile,defglobalFile,maxIndices)
  FILE *moduleFile, *defglobalFile; 
  int maxIndices;  
  {
   int count = maxIndices;
   int arrayVersion = 0;
     
   if (defglobalFile != NULL) 
     {
      count = maxIndices;
      defglobalFile = CloseFileIfNeeded(defglobalFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
     
   if (moduleFile != NULL) 
     {
      count = maxIndices;
      moduleFile = CloseFileIfNeeded(moduleFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
  }

/***********************************************************/
/* DefglobalModuleToCode: Writes the C code representation */
/*   of a single defglobal module to the specified file.   */
/***********************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID DefglobalModuleToCode(theFile,theModule,imageID,maxIndices,moduleCount)
  FILE *theFile;
  struct defmodule *theModule;
  int imageID;
  int maxIndices;
  int moduleCount;
  {
#if MAC_MPW
#pragma unused(moduleCount)
#endif

   fprintf(theFile,"{"); 
   
   ConstructModuleToCode(theFile,theModule,imageID,maxIndices,
                                  DefglobalModuleIndex,ConstructPrefix(DefglobalCodeItem));
      
   fprintf(theFile,"}"); 
  }
  
/**********************************************************/
/* DefglobalToCode: Writes the C code representation of a */
/*   single defglobal construct to the specified file.    */
/**********************************************************/
static VOID DefglobalToCode(theFile,theDefglobal,imageID,maxIndices,moduleCount)
  FILE *theFile;
  struct defglobal *theDefglobal;
  int imageID;
  int maxIndices;
  int moduleCount;
  {
   /*==================*/
   /* Defglobal Header */
   /*==================*/
   
   fprintf(theFile,"{"); 
            
   ConstructHeaderToCode(theFile,&theDefglobal->header,imageID,maxIndices,
                         moduleCount,ModulePrefix(DefglobalCodeItem),
                         ConstructPrefix(DefglobalCodeItem));
                                  
   fprintf(theFile,","); 
     
   /*============================================*/
   /* Watch Flag, In Scope Flag, and Busy Count. */
   /*============================================*/
   
   fprintf(theFile,"0,0,%ld,",theDefglobal->busyCount);

   /*================*/
   /* Current Value. */
   /*================*/ 
   
   fprintf(theFile,"{NULL,RVOID}"); 
   
   /*=====================*/
   /* Initial Expression. */
   /*=====================*/
            
   fprintf(theFile,","); 
   PrintHashedExpressionReference(theFile,theDefglobal->initial,imageID,maxIndices);

   fprintf(theFile,"}");
  }
    
/*********************************************************/
/* DefglobalCModuleReference:    */
/*********************************************************/
globle VOID DefglobalCModuleReference(theFile,count,imageID,maxIndices)
  FILE *theFile;
  int count;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"MIHS &%s%d_%d[%d]",
                      ModulePrefix(DefglobalCodeItem),
                      imageID,
                      (count / maxIndices) + 1,
                      (count % maxIndices));
  }

/*********************************************************/
/* DefglobalCConstructReference:    */
/*********************************************************/
globle VOID DefglobalCConstructReference(theFile,vTheGlobal,imageID,maxIndices)
  FILE *theFile;
  VOID *vTheGlobal;
  int imageID;
  int maxIndices;
  {  
   struct defglobal *theGlobal = (struct defglobal *) vTheGlobal;

   if (theGlobal == NULL)
     { fprintf(theFile,"NULL"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld]",ConstructPrefix(DefglobalCodeItem),
                      imageID,
                      (theGlobal->header.bsaveID / maxIndices) + 1,
                      theGlobal->header.bsaveID % maxIndices);
     }

  } 
         
#endif /* DEFGLOBAL_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME) */


