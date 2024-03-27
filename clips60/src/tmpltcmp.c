   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          DEFTEMPLATE CONSTRUCTS-TO-C MODULE         */
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

#define _TMPLTCMP_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME)

#define SlotPrefix() ArbitraryPrefix(DeftemplateCodeItem,2)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "conscomp.h"
#include "factcmp.h"
#include "cstrncmp.h"
#include "tmpltdef.h"

#include "tmpltcmp.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     ConstructToCode(char *,int,FILE *,int,int);
   static VOID                    SlotToCode(FILE *,struct templateSlot *,int,int,int);
   static VOID                    DeftemplateModuleToCode(FILE *,struct defmodule *,int,int,int);
   static VOID                    DeftemplateToCode(FILE *,struct deftemplate *,
                                                 int,int,int,int);
   static VOID                    CloseDeftemplateFiles(FILE *,FILE *,FILE *,int);
#else
   static int                     ConstructToCode();
   static VOID                    SlotToCode();
   static VOID                    DeftemplateModuleToCode();
   static VOID                    DeftemplateToCode();
   static VOID                    CloseDeftemplateFiles();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct CodeGeneratorItem *DeftemplateCodeItem;

/*********************************************************/
/* DeftemplateCompilerSetup: Initializes the deftemplate */
/*   construct for use with the constructs-to-c command. */
/*********************************************************/
globle VOID DeftemplateCompilerSetup()
  {
   DeftemplateCodeItem = AddCodeGeneratorItem("deftemplate",0,NULL,NULL,ConstructToCode,3);
  }
  
/***************************************************************/
/* ConstructToCode:  Produces all the deftemplate definitions. */
/***************************************************************/
static int ConstructToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   int fileCount = 1;
   struct defmodule *theModule;
   struct deftemplate *theTemplate;
   struct templateSlot *slotPtr;
   int slotCount = 0, slotArrayCount = 0, slotArrayVersion = 1;
   int moduleCount = 0, moduleArrayCount = 0, moduleArrayVersion = 1;  
   int templateCount = 0, templateArrayCount = 0, templateArrayVersion = 1;
   FILE *slotFile = NULL, *moduleFile = NULL, *templateFile = NULL;

   /*==================================================*/
   /* Include the appropriate deftemplate header file. */
   /*==================================================*/
   
   fprintf(headerFP,"#include \"tmpltdef.h\"\n");
   
   /*=============================================================*/
   /* Loop through all the modules, all the deftemplates, and all */
   /* the deftemplate slots writing their C code representation   */
   /* to the file as they are traversed.                          */
   /*=============================================================*/
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {           
      SetCurrentModule((VOID *) theModule);
            
      moduleFile = OpenFileIfNeeded(moduleFile,fileName,fileID,imageID,&fileCount,
                                    moduleArrayVersion,headerFP,
                                    "struct deftemplateModule",ModulePrefix(DeftemplateCodeItem),
                                    CLIPS_FALSE,NULL);
                                    
      if (moduleFile == NULL)
        {
         CloseDeftemplateFiles(moduleFile,templateFile,slotFile,maxIndices);
         return(0);
        }
        
      DeftemplateModuleToCode(moduleFile,theModule,imageID,maxIndices,moduleCount);
      moduleFile = CloseFileIfNeeded(moduleFile,&moduleArrayCount,&moduleArrayVersion,
                                     maxIndices,NULL,NULL);

      theTemplate = (struct deftemplate *) GetNextDeftemplate(NULL);

      while (theTemplate != NULL)
        {
         templateFile = OpenFileIfNeeded(templateFile,fileName,fileID,imageID,&fileCount,
                                         templateArrayVersion,headerFP,
                                         "struct deftemplate",ConstructPrefix(DeftemplateCodeItem),
                                         CLIPS_FALSE,NULL);
         if (templateFile == NULL)
           {
            CloseDeftemplateFiles(moduleFile,templateFile,slotFile,maxIndices);
            return(0);
           }
           
         DeftemplateToCode(templateFile,theTemplate,imageID,maxIndices,
                        moduleCount,slotCount);
         templateCount++;
         templateArrayCount++;
         templateFile = CloseFileIfNeeded(templateFile,&templateArrayCount,&templateArrayVersion,
                                          maxIndices,NULL,NULL);
                              
         slotPtr = theTemplate->slotList;
         while (slotPtr != NULL)
           {
            slotFile = OpenFileIfNeeded(slotFile,fileName,fileID,imageID,&fileCount,
                                        slotArrayVersion,headerFP,
                                       "struct templateSlot",SlotPrefix(),CLIPS_FALSE,NULL);
            if (slotFile == NULL) 
              {
               CloseDeftemplateFiles(moduleFile,templateFile,slotFile,maxIndices);
               return(0);
              }
              
            SlotToCode(slotFile,slotPtr,imageID,maxIndices,slotCount);
            slotCount++;
            slotArrayCount++;
            slotFile = CloseFileIfNeeded(slotFile,&slotArrayCount,&slotArrayVersion,
                                         maxIndices,NULL,NULL);
            slotPtr = slotPtr->next;
           }
           
         theTemplate = (struct deftemplate *) GetNextDeftemplate(theTemplate);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
      moduleCount++;
      moduleArrayCount++;

     }
        
   CloseDeftemplateFiles(moduleFile,templateFile,slotFile,maxIndices);
     
   return(1);
  }

/************************************************************/
/* CloseDeftemplateFiles: Closes all of the C files created */
/*   for deftemplates. Called when an error occurs or when  */
/*   the deftemplates have all been written to the files.   */
/************************************************************/
static VOID CloseDeftemplateFiles(moduleFile,templateFile,slotFile,maxIndices)
  FILE *moduleFile, *templateFile, *slotFile; 
  int maxIndices;  
  {
   int count = maxIndices;
   int arrayVersion = 0;
   
   if (slotFile != NULL) 
     {
      count = maxIndices;
      slotFile = CloseFileIfNeeded(slotFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
     
   if (templateFile != NULL) 
     {
      count = maxIndices;
      templateFile = CloseFileIfNeeded(templateFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
     
   if (moduleFile != NULL) 
     {
      count = maxIndices;
      moduleFile = CloseFileIfNeeded(moduleFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
  }
    
/*************************************************************/
/* DeftemplateModuleToCode: Writes the C code representation */
/*   of a single deftemplate module to the specified file.   */
/*************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID DeftemplateModuleToCode(theFile,theModule,imageID,maxIndices,moduleCount)
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
                                  DeftemplateModuleIndex,ConstructPrefix(DeftemplateCodeItem));
      
   fprintf(theFile,"}"); 
  }
  
/************************************************************/
/* DeftemplateToCode: Writes the C code representation of a */
/*   single deftemplate construct to the specified file.    */
/************************************************************/
static VOID DeftemplateToCode(theFile,theTemplate,imageID,maxIndices,
                           moduleCount,slotCount)
  FILE *theFile;
  struct deftemplate *theTemplate;
  int imageID;
  int maxIndices;
  int moduleCount;
  int slotCount;
  {
   /*====================*/
   /* Deftemplate Header */
   /*====================*/
   
   fprintf(theFile,"{");
            
   ConstructHeaderToCode(theFile,&theTemplate->header,imageID,maxIndices,
                                  moduleCount,ModulePrefix(DeftemplateCodeItem),
                                  ConstructPrefix(DeftemplateCodeItem));
   fprintf(theFile,","); 
    
   /*===========*/
   /* Slot List */
   /*===========*/

   if (theTemplate->slotList == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%d[%d],",SlotPrefix(),
                 imageID,
                 (slotCount / maxIndices) + 1,
                 slotCount % maxIndices);
     }

   /*==========================================*/
   /* Implied Flag, Watch Flag, In Scope Flag, */
   /* Number of Slots, and Busy Count.         */
   /*==========================================*/
   
   fprintf(theFile,"%d,0,0,%d,%ld,",theTemplate->implied,theTemplate->numberOfSlots,theTemplate->busyCount);

   /*=================*/
   /* Pattern Network */
   /*=================*/
   
   if (theTemplate->patternNetwork == NULL)
     { fprintf(theFile,"NULL"); }
   else
     { FactPatternNodeReference(theTemplate->patternNetwork,theFile,imageID,maxIndices); }
   
   fprintf(theFile,"}");
  }
  
/*****************************************************/
/* SlotToCode: Writes the C code representation of a */
/*   single deftemplate slot to the specified file.  */
/*****************************************************/
static VOID SlotToCode(theFile,theSlot,imageID,maxIndices,slotCount)
  FILE *theFile;
  struct templateSlot *theSlot;
  int imageID;
  int maxIndices;
  int slotCount;
  {
   /*===========*/
   /* Slot Name */
   /*===========*/
   
   fprintf(theFile,"{");
   PrintSymbolReference(theFile,theSlot->slotName);
   
   /*=============================*/
   /* Multislot and Default Flags */
   /*=============================*/
   
   fprintf(theFile,",%d,%d,%d,%d,",theSlot->multislot,theSlot->noDefault,
                                   theSlot->defaultPresent,theSlot->defaultDynamic);
   
   /*=============*/
   /* Constraints */
   /*=============*/
               
   PrintConstraintReference(theFile,theSlot->constraints,imageID,maxIndices);
      
   /*===============*/
   /* Default Value */
   /*===============*/
   
   fprintf(theFile,",");
   PrintHashedExpressionReference(theFile,theSlot->defaultList,imageID,maxIndices);
   fprintf(theFile,",");

   /*===========*/
   /* Next Slot */
   /*===========*/
   
   if (theSlot->next == NULL)
     { fprintf(theFile,"NULL}"); }
   else
     {
      fprintf(theFile,"&%s%d_%d[%d]}",SlotPrefix(),imageID,
                               ((slotCount+1) / maxIndices) + 1,
                                (slotCount+1) % maxIndices);
     }
  }
        
/*********************************************************/
/* DeftemplateCModuleReference:    */
/*********************************************************/
globle VOID DeftemplateCModuleReference(theFile,count,imageID,maxIndices)
  FILE *theFile;
  int count;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"MIHS &%s%d_%d[%d]",ModulePrefix(DeftemplateCodeItem),
                      imageID,
                      (count / maxIndices) + 1,
                      (count % maxIndices));
  }

/*********************************************************/
/* DeftemplateCConstructReference:    */
/*********************************************************/
globle VOID DeftemplateCConstructReference(theFile,vTheTemplate,imageID,maxIndices)
  FILE *theFile;
  VOID *vTheTemplate;
  int imageID;
  int maxIndices;
  {  
   struct deftemplate *theTemplate = (struct deftemplate *) vTheTemplate;

   if (theTemplate == NULL)
     { fprintf(theFile,"NULL"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld]",ConstructPrefix(DeftemplateCodeItem),
                      imageID,
                      (theTemplate->header.bsaveID / maxIndices) + 1,
                      theTemplate->header.bsaveID % maxIndices);
     }

  } 
#endif


