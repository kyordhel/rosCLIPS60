
   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFRULE CONSTRUCTS-TO-C MODULE           */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Barry Cameron                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _RULECMP_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT && (! RUN_TIME) && CONSTRUCT_COMPILER

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "factbld.h" 
#include "reteutil.h"

#include "rulecmp.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     ConstructToCode(char *,int,FILE *,int,int);
   static VOID                    JoinToCode(FILE *,struct joinNode *,int,int);
   static VOID                    DefruleModuleToCode(FILE *,struct defmodule *,int,int,int);
   static VOID                    DefruleToCode(FILE *,struct defrule *,int,int,int);
   static VOID                    CloseDefruleFiles(FILE *,FILE *,FILE *,int);
   static VOID                    BeforeDefrulesCode(void);
#else
   static int                     ConstructToCode();
   static VOID                    JoinToCode();
   static VOID                    DefruleModuleToCode();
   static VOID                    DefruleToCode();
   static VOID                    CloseDefruleFiles();
   static VOID                    BeforeDefrulesCode();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/
   globle struct CodeGeneratorItem *DefruleCodeItem;

/***********************************************************/
/* DefruleCompilerSetup: Initializes the defrule construct */
/*   for use with the constructs-to-c command.             */
/***********************************************************/
globle VOID DefruleCompilerSetup()
  {
   DefruleCodeItem = AddCodeGeneratorItem("defrules",0,BeforeDefrulesCode,
                                          NULL,ConstructToCode,3);
  }

/**********************************************************/
/* BeforeDefrulesCode:    */
/**********************************************************/
static VOID BeforeDefrulesCode()
  {
   long int moduleCount, ruleCount, joinCount;

   TagRuleNetwork(&moduleCount,&ruleCount,&joinCount);
  }

/***********************************************************/
/* ConstructToCode:  Produces all the defrule definitions. */
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
   struct defrule *theDefrule;
   struct joinNode *theJoin;
   int joinCount = 0, joinArrayCount = 0, joinArrayVersion = 1;
   int moduleCount = 0, moduleArrayCount = 0, moduleArrayVersion = 1;  
   int defruleArrayCount = 0, defruleArrayVersion = 1;
   FILE *joinFile = NULL, *moduleFile = NULL, *defruleFile = NULL;

   /*==============================================*/
   /* Include the appropriate defrule header file. */
   /*==============================================*/
   
   fprintf(headerFP,"#include \"ruledef.h\"\n");

   /*=========================================================*/
   /* Loop through all the modules, all the defrules, and all */
   /* the join nodes writing their C code representation to   */
   /* the file as they are traversed.                         */
   /*=========================================================*/
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {           
      SetCurrentModule((VOID *) theModule);
            
      moduleFile = OpenFileIfNeeded(moduleFile,fileName,fileID,imageID,&fileCount,
                                    moduleArrayVersion,headerFP,
                                    "struct defruleModule",ModulePrefix(DefruleCodeItem),
                                    CLIPS_FALSE,NULL);
                                    
      if (moduleFile == NULL)
        {
         CloseDefruleFiles(moduleFile,defruleFile,joinFile,maxIndices);
         return(0);
        }
        
      DefruleModuleToCode(moduleFile,theModule,imageID,maxIndices,moduleCount);
      moduleFile = CloseFileIfNeeded(moduleFile,&moduleArrayCount,&moduleArrayVersion,
                                     maxIndices,NULL,NULL);

      theDefrule = (struct defrule *) GetNextDefrule(NULL);

      while (theDefrule != NULL)
        {
         defruleFile = OpenFileIfNeeded(defruleFile,fileName,fileID,imageID,&fileCount,
                                        defruleArrayVersion,headerFP,
                                        "struct defrule",ConstructPrefix(DefruleCodeItem),
                                        CLIPS_FALSE,NULL);
         if (defruleFile == NULL)
           {
            CloseDefruleFiles(moduleFile,defruleFile,joinFile,maxIndices);
            return(0);
           }
           
         DefruleToCode(defruleFile,theDefrule,imageID,maxIndices,
                        moduleCount);
         defruleArrayCount++;
         defruleFile = CloseFileIfNeeded(defruleFile,&defruleArrayCount,&defruleArrayVersion,
                                         maxIndices,NULL,NULL);
                                 
         theJoin = theDefrule->lastJoin;
         while (theJoin != NULL)
           {  
            if (theJoin->marked)
              {
               joinFile = OpenFileIfNeeded(joinFile,fileName,fileID,imageID,&fileCount,
                                        joinArrayVersion,headerFP,
                                       "struct joinNode",JoinPrefix(),CLIPS_FALSE,NULL);
               if (joinFile == NULL) 
                 {
                  CloseDefruleFiles(moduleFile,defruleFile,joinFile,maxIndices);
                  return(0);
                 }
              
               JoinToCode(joinFile,theJoin,imageID,maxIndices);
               joinCount++;
               joinArrayCount++;
               joinFile = CloseFileIfNeeded(joinFile,&joinArrayCount,&joinArrayVersion,
                                            maxIndices,NULL,NULL);
              }
              
            theJoin = GetPreviousJoin(theJoin);
           }
           
         if (theDefrule->disjunct != NULL) theDefrule = theDefrule->disjunct;
         else theDefrule = (struct defrule *) GetNextDefrule(theDefrule);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
      moduleCount++;
      moduleArrayCount++;
     }
        
   CloseDefruleFiles(moduleFile,defruleFile,joinFile,maxIndices);
     
   return(1);
  }

/********************************************************/
/* CloseDefruleFiles: Closes all of the C files created */
/*   for defrule. Called when an error occurs or when   */
/*   the defrules have all been written to the files.   */
/********************************************************/
static VOID CloseDefruleFiles(moduleFile,defruleFile,joinFile,maxIndices)
  FILE *moduleFile, *defruleFile, *joinFile; 
  int maxIndices;  
  {
   int count = maxIndices;
   int arrayVersion = 0;
   
   if (joinFile != NULL) 
     {
      count = maxIndices;
      joinFile = CloseFileIfNeeded(joinFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
     
   if (defruleFile != NULL) 
     {
      count = maxIndices;
      defruleFile = CloseFileIfNeeded(defruleFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
     
   if (moduleFile != NULL) 
     {
      count = maxIndices;
      moduleFile = CloseFileIfNeeded(moduleFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
  }
  
/*********************************************************/
/* DefruleModuleToCode: Writes the C code representation */
/*   of a single defrule module to the specified file.   */
/*********************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID DefruleModuleToCode(theFile,theModule,imageID,maxIndices,moduleCount)
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
                                  DefruleModuleIndex,ConstructPrefix(DefruleCodeItem));
      
   fprintf(theFile,",NULL}"); 
  }
  
/********************************************************/
/* DefruleToCode: Writes the C code representation of a */
/*   single defrule construct to the specified file.    */
/********************************************************/
static VOID DefruleToCode(theFile,theDefrule,imageID,maxIndices,moduleCount)
  FILE *theFile;
  struct defrule *theDefrule;
  int imageID;
  int maxIndices;
  int moduleCount;
  {
   /*====================*/
   /* Deftemplate Header */
   /*====================*/
   
   fprintf(theFile,"{");
            
   ConstructHeaderToCode(theFile,&theDefrule->header,imageID,maxIndices,
                                  moduleCount,ModulePrefix(DefruleCodeItem),
                                  ConstructPrefix(DefruleCodeItem));

   /*==========================*/
   /* Flags and Integer Values */
   /*==========================*/
      
   fprintf(theFile,",%d,%d,%d,%d,%d,%d,%d,%d,",
                   theDefrule->salience,theDefrule->localVarCnt,
                   theDefrule->complexity,theDefrule->afterBreakpoint,
                   theDefrule->watchActivation,theDefrule->watchFiring,
                   theDefrule->autoFocus,theDefrule->executing);

   /*==================*/
   /* Dynamic Salience */
   /*==================*/
        
#if DYNAMIC_SALIENCE
      ExpressionToCode(theFile,theDefrule->dynamicSalience);
      fprintf(theFile,",");
#endif

   /*=============*/
   /* RHS Actions */
   /*=============*/
   
   ExpressionToCode(theFile,theDefrule->actions);
   fprintf(theFile,",");
      
   /*=========================*/
   /* Logical Dependency Join */
   /*=========================*/
   
#if LOGICAL_DEPENDENCIES
   if (theDefrule->logicalJoin != NULL)
     {
      fprintf(theFile,",&%s%d_%ld[%ld],",JoinPrefix(),
                     imageID,(theDefrule->logicalJoin->bsaveID / maxIndices) + 1,
                             theDefrule->logicalJoin->bsaveID % maxIndices);
     }
   else
     { fprintf(theFile,"NULL,"); }
#endif

   /*===============*/
   /* Next Disjunct */
   /*===============*/
   
   if (theDefrule->disjunct != NULL)
     {
      fprintf(theFile,",&%s%d_%ld[%ld]}",ConstructPrefix(DefruleCodeItem),
                     imageID,(theDefrule->disjunct->header.bsaveID / maxIndices) + 1,
                             theDefrule->disjunct->header.bsaveID % maxIndices);
     }
   else
     { fprintf(theFile,"NULL}"); }
  } 
   
/***************************************************/
/* JoinToCode: Writes the C code representation of */
/*   a single join node to the specified file.     */
/***************************************************/
static VOID JoinToCode(theFile,theJoin,imageID,maxIndices)
  FILE *theFile;
  struct joinNode *theJoin;
  int imageID;
  int maxIndices;
  {
   struct patternParser *theParser;
   
   /*===========================*/
   /* Mark the join as visited. */
   /*===========================*/
   
   theJoin->marked = 0;
   
   /*===========================*/
   /* Flags and Integer Values. */
   /*===========================*/
   
   fprintf(theFile,"{%d,%d,%d,%d,0,0,%d,%d,0,",
                   theJoin->firstJoin,theJoin->logicalJoin,
                   theJoin->joinFromTheRight,theJoin->patternIsNegated,
                   theJoin->rhsType,theJoin->depth);

   /*==============*/
   /* Beta Memory. */
   /*==============*/
   
   fprintf(theFile,"NULL,");
   
   /*====================*/
   /* Network Expression */
   /*====================*/
   
   PrintHashedExpressionReference(theFile,theJoin->networkTest,imageID,maxIndices);
   fprintf(theFile,",");

   /*============================*/
   /* Right Side Entry Structure */
   /*============================*/
   
   if (theJoin->rightSideEntryStructure == NULL)
     { fprintf(theFile,"NULL,"); }
   else if (theJoin->joinFromTheRight == CLIPS_FALSE)
     {
      theParser = GetPatternParser((int) theJoin->rhsType);
      if (theParser->codeReferenceFunction == NULL) fprintf(theFile,"NULL,");
      else
        {
         fprintf(theFile,"VS ");
         (*theParser->codeReferenceFunction)(theJoin->rightSideEntryStructure,
                                             theFile,imageID,maxIndices);
         fprintf(theFile,",");
        }
     }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",JoinPrefix(),
              imageID,(((struct joinNode *) theJoin->rightSideEntryStructure)->bsaveID / maxIndices) + 1,
                      ((struct joinNode *) theJoin->rightSideEntryStructure)->bsaveID % maxIndices);
     }

   /*=================*/
   /* Next Join Level */
   /*=================*/
 
   if (theJoin->nextLevel == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",JoinPrefix(),
                    imageID,(theJoin->nextLevel->bsaveID / maxIndices) + 1,
                            theJoin->nextLevel->bsaveID % maxIndices);
     }

   /*=================*/
   /* Last Join Level */
   /*=================*/
 
   if (theJoin->lastLevel == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",JoinPrefix(),
                    imageID,(theJoin->lastLevel->bsaveID / maxIndices) + 1,
                            theJoin->lastLevel->bsaveID % maxIndices);
     }

   /*==================*/
   /* Right Drive Node */
   /*==================*/
 
   if (theJoin->rightDriveNode == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",JoinPrefix(),
                    imageID,(theJoin->rightDriveNode->bsaveID / maxIndices) + 1,
                            theJoin->rightDriveNode->bsaveID % maxIndices);
     }

   /*==================*/
   /* Right Match Node */
   /*==================*/
 
   if (theJoin->rightMatchNode == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",JoinPrefix(),
                    imageID,(theJoin->rightMatchNode->bsaveID / maxIndices) + 1,
                            theJoin->rightMatchNode->bsaveID % maxIndices);
     }

   /*==================*/
   /* Rule to Activate */
   /*==================*/

   if (theJoin->ruleToActivate == NULL)
     { fprintf(theFile,"NULL}"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld]}",ConstructPrefix(DefruleCodeItem),imageID,
                                    (theJoin->ruleToActivate->header.bsaveID / maxIndices) + 1,
                                    theJoin->ruleToActivate->header.bsaveID % maxIndices);
     }
  }

/*********************************************************/
/* DefruleCModuleReference:    */
/*********************************************************/
globle VOID DefruleCModuleReference(theFile,count,imageID,maxIndices)
  FILE *theFile;
  int count;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"MIHS &%s%d_%d[%d]",ModulePrefix(DefruleCodeItem),
                      imageID,
                      (count / maxIndices) + 1,
                      (count % maxIndices));
  }

#endif


