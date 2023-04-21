
   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*     FACT PATTERN NETWORK CONSTRUCTS-TO-C MODULE     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _FACTCMP_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT && (! RUN_TIME) && DEFTEMPLATE_CONSTRUCT && CONSTRUCT_COMPILER

#define FactPrefix() ArbitraryPrefix(FactCodeItem,0)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "factbld.h" 
#include "conscomp.h"
#include "factcmp.h"
#include "tmpltdef.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     PatternNetworkToCode(char *,int,FILE *,int,int);
   static VOID                    BeforePatternNetworkToCode(VOID);
   static struct factPatternNode *GetNextPatternNode(struct factPatternNode *);
   static VOID                    CloseNetworkFiles(FILE *,int);  
   static VOID                    PatternNodeToCode(FILE *,struct factPatternNode *,int,int);
#else
   static int                     PatternNetworkToCode();
   static VOID                    BeforePatternNetworkToCode();
   static struct factPatternNode *GetNextPatternNode();
   static VOID                    CloseNetworkFiles();  
   static VOID                    PatternNodeToCode();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/
   static struct CodeGeneratorItem *FactCodeItem;

/*****************************************************/
/* FactPatternsCompilerSetup: Initializes the use of */
/*   the construct compiler for fact pattern nodes.  */
/*****************************************************/
globle VOID FactPatternsCompilerSetup()
  {
   FactCodeItem = AddCodeGeneratorItem("facts",0,BeforePatternNetworkToCode,
                                       NULL,PatternNetworkToCode,1);
  }

/****************************************************************/
/* BeforePatternNetworkToCode: Assigns each pattern node with a */
/*   unique ID which will be used for pointer references when   */
/*   the data structures are written to a file as C code        */
/****************************************************************/ 
static VOID BeforePatternNetworkToCode()
  {
   int whichPattern = 0;
   int whichDeftemplate = 0;
   struct defmodule *theModule;
   struct deftemplate *theDeftemplate;
   struct factPatternNode *thePattern = NULL;
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         theDeftemplate->header.bsaveID = whichDeftemplate++;
         thePattern = theDeftemplate->patternNetwork;
         while (thePattern != NULL)
           {
            thePattern->bsaveID = whichPattern++;
            thePattern = GetNextPatternNode(thePattern);
           }
     
         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
     
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }

/***************************************************************/
/* GetNextPatternNode                                      */
/***************************************************************/
static struct factPatternNode *GetNextPatternNode(thePattern)
  struct factPatternNode *thePattern;
  {
   if (thePattern->nextLevel != NULL) return(thePattern->nextLevel);
        
   while (thePattern->rightNode == NULL)
     {
      thePattern = thePattern->lastLevel;
      if (thePattern == NULL) return(NULL);
     }
     
   return(thePattern->rightNode);
  }
  
/***************************************************************/
/* PatternNetworkToCode:   */
/***************************************************************/
static int PatternNetworkToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   int fileCount = 1;
   struct defmodule *theModule;
   struct deftemplate *theTemplate;
   struct factPatternNode *thePatternNode;
   int networkCount = 0, networkArrayCount = 0, networkArrayVersion = 1;
   FILE *networkFile = NULL;

   /*===========================================================*/
   /* Include the appropriate fact pattern network header file. */
   /*===========================================================*/
   
   fprintf(headerFP,"#include \"factbld.h\"\n");
      
   /*=============================================================*/
   /* Loop through all the modules, all the deftemplates, and all */
   /* the deftemplate slots writing their C code representation   */
   /* to the file as they are traversed.                          */
   /*=============================================================*/
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   
   while (theModule != NULL)
     {           
      SetCurrentModule((VOID *) theModule);

      theTemplate = (struct deftemplate *) GetNextDeftemplate(NULL);

      while (theTemplate != NULL)
        {
         thePatternNode = theTemplate->patternNetwork;
         while (thePatternNode != NULL)
           {
            networkFile = OpenFileIfNeeded(networkFile,fileName,fileID,imageID,&fileCount,
                                         networkArrayVersion,headerFP,
                                         "struct factPatternNode",FactPrefix(),CLIPS_FALSE,NULL);
            if (networkFile == NULL)
              {
               CloseNetworkFiles(networkFile,maxIndices);
               return(0);
              }
           
            PatternNodeToCode(networkFile,thePatternNode,imageID,maxIndices);
            networkCount++;
            networkArrayCount++;
            networkFile = CloseFileIfNeeded(networkFile,&networkArrayCount,
                                            &networkArrayVersion,maxIndices,NULL,NULL);
                
            thePatternNode = GetNextPatternNode(thePatternNode);    
           }          
           
         theTemplate = (struct deftemplate *) GetNextDeftemplate(theTemplate);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
        
   CloseNetworkFiles(networkFile,maxIndices);
     
   return(1);
  }
  
/****************************************************************/
/* CloseNetworkFiles: Closes all of the C files created for the */
/*   fact pattern network. Called when an error occurs or when  */
/*   the deftemplates have all been written to the files.       */
/****************************************************************/
static VOID CloseNetworkFiles(networkFile,maxIndices)
  FILE *networkFile; 
  int maxIndices;  
  {
   int count = maxIndices;
   int arrayVersion = 0;
   
   if (networkFile != NULL) 
     {
      count = maxIndices;
      networkFile = CloseFileIfNeeded(networkFile,&count,&arrayVersion,maxIndices,NULL,NULL);
     }
  }
  
/************************************************************/
/* PatternNodeToCode: Writes the C code representation of a */
/*   single fact pattern node slot to the specified file.   */
/************************************************************/
static VOID PatternNodeToCode(theFile,thePatternNode,imageID,maxIndices)
  FILE *theFile;
  struct factPatternNode *thePatternNode;
  int imageID;
  int maxIndices;
  {
   fprintf(theFile,"{");
      
   PatternNodeHeaderToCode(theFile,&thePatternNode->header,imageID,maxIndices);

   fprintf(theFile,",0,%d,%d,%d,",thePatternNode->whichField,
                             thePatternNode->whichSlot,
                             thePatternNode->leaveFields);
      
   PrintHashedExpressionReference(theFile,thePatternNode->networkTest,imageID,maxIndices);

   if (thePatternNode->nextLevel == NULL)
     { fprintf(theFile,",NULL,"); }
   else
     {
      fprintf(theFile,",&%s%d_%ld[%ld],",FactPrefix(),
                 imageID,(thePatternNode->nextLevel->bsaveID / maxIndices) + 1,
                         thePatternNode->nextLevel->bsaveID % maxIndices);
     }

   if (thePatternNode->lastLevel == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",FactPrefix(),
              imageID,(thePatternNode->lastLevel->bsaveID / maxIndices) + 1,
                   thePatternNode->lastLevel->bsaveID % maxIndices);
     }

   if (thePatternNode->leftNode == NULL)
     { fprintf(theFile,"NULL,"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld],",FactPrefix(),
             imageID,(thePatternNode->leftNode->bsaveID / maxIndices) + 1,
                  thePatternNode->leftNode->bsaveID % maxIndices);
     }

   if (thePatternNode->rightNode == NULL)
     { fprintf(theFile,"NULL}"); }
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld]}",FactPrefix(),
            imageID,(thePatternNode->rightNode->bsaveID / maxIndices) + 1,
                thePatternNode->rightNode->bsaveID % maxIndices);
     }
  } 

/**********************************************************/
/* FactPatternNodeReference: Prints C code representation */
/*   of a fact pattern node data structure reference.     */
/**********************************************************/
globle VOID FactPatternNodeReference(theVPattern,theFile,imageID,maxIndices)
  VOID *theVPattern;
  FILE *theFile;
  int imageID;
  int maxIndices;
  {
   struct factPatternNode *thePattern = (struct factPatternNode *) theVPattern;
   
   if (thePattern == NULL) fprintf(theFile,"NULL");
   else
     {
      fprintf(theFile,"&%s%d_%ld[%ld]",FactPrefix(),
                    imageID,(thePattern->bsaveID / maxIndices) + 1,
                            thePattern->bsaveID % maxIndices);
     }
  }
                            
#endif


