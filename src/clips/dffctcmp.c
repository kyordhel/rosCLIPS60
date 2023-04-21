   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFFACTS CONSTRUCTS-TO-C MODULE          */
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

#define _DFFCTCMP_SOURCE_

#include "setup.h"

#if DEFFACTS_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "conscomp.h"
#include "dffctdef.h"

#include "dffctcmp.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     ConstructToCode(char *,int,FILE *,int,int);
   static VOID                    DeffactsToCode(FILE *,struct deffacts *,
                                                 int,int,int);
   static VOID                    DeffactsModuleToCode(FILE *,struct defmodule *,int,int,int);
   static VOID                    CloseDeffactsFiles(FILE *,FILE *,int);
   static VOID                    BeforeDeffactsToCode(void);  
#else
   static int                     ConstructToCode();
   static VOID                    DeffactsToCode();
   static VOID                    DeffactsModuleToCode();
   static VOID                    CloseDeffactsFiles();
   static VOID                    BeforeDeffactsToCode();  
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/
   static struct CodeGeneratorItem *DeffactsCodeItem;

/*************************************************************/
/* DeffactsCompilerSetup: Initializes the deffacts construct */
/*    for use with the constructs-to-c command.              */
/*************************************************************/
globle VOID DeffactsCompilerSetup()
  {
   DeffactsCodeItem = AddCodeGeneratorItem("deffacts",0,BeforeDeffactsToCode,
                                           NULL,ConstructToCode,2);
  }
  
/*************************************************************/
/* BeforeDeffactsToCode: Assigns each deffacts a unique ID   */
/*   which will be used for pointer references when the data */
/*   structures are written to a file as C code              */
/*************************************************************/ 
static VOID BeforeDeffactsToCode()
  {
   MarkConstructBsaveIDs(DeffactsModuleIndex);
  }
  
/**********************************************************/
/* ConstructToCode: Produces deffacts code for a run-time */
/*   module created using the constructs-to-c function.   */
/**********************************************************/
static int ConstructToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   int fileCount = 1;
   struct defmodule *theModule;
   struct deffacts *theDeffacts;
   int moduleCount = 0, moduleArrayCount = 0, moduleArrayVersion = 1;  
   int deffactsCount = 0, deffactsArrayCount = 0, deffactsArrayVersion = 1;
   FILE *moduleFile = NULL, *deffactsFile = NULL;
  
   /*==================================================*/
   /* Include the appropriate deftemplate header file. */
   /*==================================================*/
   
   fprintf(headerFP,"#include \"dffctdef.h\"\n");

   /*=================================================================*/
   /* Loop through all the modules and all the deffacts writing their */
   /*  C code representation to the file as they are traversed.       */
   /*=================================================================*/
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {           
      SetCurrentModule((VOID *) theModule);
            
      moduleFile = OpenFileIfNeeded(moduleFile,fileName,fileID,imageID,&fileCount,
                                    moduleArrayVersion,headerFP,
                                    "struct deffactsModule",ModulePrefix(DeffactsCodeItem),
                                    CLIPS_FALSE,NULL);
                                    
      if (moduleFile == NULL)
        {
         CloseDeffactsFiles(moduleFile,deffactsFile,maxIndices);
         return(0);
        }
        
      DeffactsModuleToCode(moduleFile,theModule,imageID,maxIndices,moduleCount);
      moduleFile = CloseFileIfNeeded(moduleFile,&moduleArrayCount,&moduleArrayVersion,
                                     maxIndices,NULL,NULL);

      theDeffacts = (struct deffacts *) GetNextDeffacts(NULL);

      while (theDeffacts != NULL)
        {
         deffactsFile = OpenFileIfNeeded(deffactsFile,fileName,fileID,imageID,&fileCount,
                                         deffactsArrayVersion,headerFP,
                                         "struct deffacts",ConstructPrefix(DeffactsCodeItem),
                                         CLIPS_FALSE,NULL);
         if (deffactsFile == NULL)
           {
            CloseDeffactsFiles(moduleFile,deffactsFile,maxIndices);
            return(0);
           }
           
         DeffactsToCode(deffactsFile,theDeffacts,imageID,maxIndices,moduleCount);
         deffactsCount++;
         deffactsArrayCount++;
         deffactsFile = CloseFileIfNeeded(deffactsFile,&deffactsArrayCount,
                                          &deffactsArrayVersion,maxIndices,NULL,NULL);
                                         
         theDeffacts = (struct deffacts *) GetNextDeffacts(theDeffacts);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
      moduleCount++;
      moduleArrayCount++;
     }
        
   CloseDeffactsFiles(moduleFile,deffactsFile,maxIndices);
     
   return(1);
  }
  
/*********************************************************/
/* CloseDeffactsFiles: Closes all of the C files created */
/*   for deffacts. Called when an error occurs or when   */
/*   the deffacts have all been written to the files.    */
/*********************************************************/
static VOID CloseDeffactsFiles(moduleFile,deffactsFile,maxIndices)
  FILE *moduleFile, *deffactsFile; 
  int maxIndices;  
  {
   int count = maxIndices;
   int arrayVersion = 0;
     
   if (deffactsFile != NULL) 
     {
      count = maxIndices;
      deffactsFile = CloseFileIfNeeded(deffactsFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
     
   if (moduleFile != NULL) 
     {
      count = maxIndices;
      moduleFile = CloseFileIfNeeded(moduleFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
  }

/**********************************************************/
/* DeffactsModuleToCode: Writes the C code representation */
/*   of a single deffacts module to the specified file.   */
/**********************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID DeffactsModuleToCode(theFile,theModule,imageID,maxIndices,moduleCount)
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
                                  DeffactsModuleIndex,ConstructPrefix(DeffactsCodeItem));
      
   fprintf(theFile,"}"); 
  }
  
/*********************************************************/
/* DeffactsToCode: Writes the C code representation of a */
/*   single deffacts construct to the specified file.    */
/*********************************************************/
static VOID DeffactsToCode(theFile,theDeffacts,imageID,maxIndices,moduleCount)
  FILE *theFile;
  struct deffacts *theDeffacts;
  int imageID;
  int maxIndices;
  int moduleCount;
  {
   /*=================*/
   /* Deffacts Header */
   /*=================*/
   
   fprintf(theFile,"{");
            
   ConstructHeaderToCode(theFile,&theDeffacts->header,imageID,maxIndices,
                         moduleCount,ModulePrefix(DeffactsCodeItem),
                         ConstructPrefix(DeffactsCodeItem));
                                  
   fprintf(theFile,","); 
     
   /*=============*/
   /* Assert List */
   /*=============*/

   ExpressionToCode(theFile,theDeffacts->assertList);
   fprintf(theFile,"}");
  }
    
/*********************************************************/
/* DeffactsCModuleReference:    */
/*********************************************************/
globle VOID DeffactsCModuleReference(theFile,count,imageID,maxIndices)
  FILE *theFile;
  int count;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"MIHS &%s%d_%d[%d]",
                      ModulePrefix(DeffactsCodeItem),
                      imageID,
                      (count / maxIndices) + 1,
                      (count % maxIndices));
  }
                      
#endif /* DEFFACTS_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME) */


