   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           DEFMODULE CONSTRUCTS-TO-C MODULE          */
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

#define _MODULCMP_SOURCE_

#include "setup.h"

#if CONSTRUCT_COMPILER && (! RUN_TIME)

#define ItemPrefix()      ArbitraryPrefix(DefmoduleCodeItem,0)
#define DefmodulePrefix() ArbitraryPrefix(DefmoduleCodeItem,1)
#define PortPrefix()      ArbitraryPrefix(DefmoduleCodeItem,2)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "conscomp.h"
#include "moduldef.h"

#include "modulcmp.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     DefmodulesToCode(char *,int,FILE *,int,int);
   static VOID                    InitDefmoduleCode(FILE *,int,int);
   static struct portItem        *GetNextPortItem(struct defmodule **,struct portItem **,
                                                  int *,int *);
   static int                     PortItemsToCode(char *,int,FILE *,int,int,int *);
#else
   static int                     DefmodulesToCode();
   static VOID                    InitDefmoduleCode();
   static struct portItem        *GetNextPortItem();
   static int                     PortItemsToCode();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/
   static struct CodeGeneratorItem *DefmoduleCodeItem;

/***************************************************************/
/* DefmoduleCompilerSetup: Initializes the defmodule construct */
/*    for use with the constructs-to-c command.                */
/***************************************************************/
globle VOID DefmoduleCompilerSetup()
  {
   DefmoduleCodeItem = AddCodeGeneratorItem("defmodule",200,NULL,InitDefmoduleCode,
                                            DefmodulesToCode,3);
  }

/************************************************/
/* PrintDefmoduleReference:  */
/************************************************/
globle VOID PrintDefmoduleReference(theFile,theModule)
  FILE *theFile;
  struct defmodule *theModule;
  {
   if (theModule == NULL) fprintf(theFile,"NULL");
   else fprintf(theFile,"&%s%d_%ld[%ld]",DefmodulePrefix(),ImageID,
                                    (long) ((theModule->bsaveID / MaxIndices) + 1),
                                    (long) (theModule->bsaveID % MaxIndices));
  }
  
/************************************************/
/* InitDefmoduleCode: Writes out initialization */
/*   code for defmodules for a run-time module. */
/************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID InitDefmoduleCode(initFP,imageID,maxIndices)
  FILE *initFP;
  int imageID;
  int maxIndices;
  {
#if MAC_MPW
#pragma unused(maxIndices)
#endif
   if (GetNextDefmodule(NULL) != NULL)
     { fprintf(initFP,"   SetListOfDefmodules((VOID *) %s%d_1);\n",DefmodulePrefix(),imageID); }
   else
     { fprintf(initFP,"   SetListOfDefmodules(NULL);\n"); }
   fprintf(initFP,"   SetCurrentModule((VOID *) GetNextDefmodule(NULL));\n");
  }
    
/************************************************************/
/* DefmodulesToCode: Produces defmodule code for a run-time */
/*   module created using the constructs-to-c function.     */
/************************************************************/
static int DefmodulesToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   struct defmodule *theConstruct;
   FILE *fp, *itemsFile;
   int version = 1;
   int i = 1;
   int newHeader = CLIPS_TRUE;
   int portItemCount = 0;
   struct portItem *portItemPtr;
   int mihCount = 0;
   int j;
   struct moduleItem *theItem;

   /*==================*/ 
   /* Create the file. */
   /*==================*/

   fprintf(headerFP,"#include \"moduldef.h\"\n");
   if (GetNextDefmodule(NULL) == NULL) return(-1);
   
   if ((itemsFile = NewCFile(fileName,fileID,version,CLIPS_FALSE)) == NULL) return(0);
   fprintf(itemsFile,"struct defmoduleItemHeader *%s%d_%d[] = {\n",ItemPrefix(),imageID,version);
   fprintf(headerFP,"extern struct defmoduleItemHeader *%s%d_%d[];\n",ItemPrefix(),imageID,version);
   
   version++;
   if ((fp = NewCFile(fileName,fileID,version,CLIPS_FALSE)) == NULL) return(0);
   
   theConstruct = (struct defmodule *) GetNextDefmodule(NULL);
   while (theConstruct != NULL)
     {
      if (newHeader)
        {
         fprintf(fp,"struct defmodule %s%d_%d[] = {\n",DefmodulePrefix(),imageID,version - 1);
         fprintf(headerFP,"extern struct defmodule %s%d_%d[];\n",DefmodulePrefix(),imageID,version - 1);
         newHeader = CLIPS_FALSE;
        }
      
      fprintf(fp,"{");
      PrintSymbolReference(fp,theConstruct->name); /* name */
      fprintf(fp,",NULL,");                        /* ppform */
   
      fprintf(fp,"&%s%d_1[%d],",ItemPrefix(),imageID,mihCount);            /* itemsArray */
    
      for (j = 0, theItem = GetListOfModuleItems();
           (j < GetNumberOfModuleItems()) && (theItem != NULL) ; 
           j++, theItem = theItem->next)
        { 
         mihCount++;
         if (theItem->constructsToCModuleReference == NULL)
           { fprintf(itemsFile,"NULL"); }
         else
           { (*theItem->constructsToCModuleReference)(itemsFile,(int) theConstruct->bsaveID,imageID,maxIndices); }
         
         if ((j + 1) < GetNumberOfModuleItems()) fprintf(itemsFile,",");
         else if (theConstruct->next != NULL) fprintf(itemsFile,",\n");
        } 

      /*=================================*/
      /* Write the importList reference. */
      /*=================================*/
      
      if (theConstruct->importList == NULL)
        { fprintf(fp,"NULL,"); }
      else
        {
         fprintf(fp,"&%s%d_%d[%d],",PortPrefix(),imageID,
                                     (portItemCount / maxIndices) + 1,
                                     portItemCount % maxIndices);
         portItemPtr = theConstruct->importList;
         while (portItemPtr != NULL)
           {
            portItemCount++;
            portItemPtr = portItemPtr->next;
           }
        }
        
      /*=================================*/
      /* Write the exportList reference. */
      /*=================================*/
      
      if (theConstruct->exportList == NULL)
        { fprintf(fp,"NULL,"); }
      else
        {
         fprintf(fp,"&%s%d_%d[%d],",PortPrefix(),imageID,
                                     (portItemCount / maxIndices) + 1,
                                     portItemCount % maxIndices);
         portItemPtr = theConstruct->exportList;
         while (portItemPtr != NULL)
           {
            portItemCount++;
            portItemPtr = portItemPtr->next;
           }
        }

      fprintf(fp,"0,%ld,",theConstruct->bsaveID);
      
      if (theConstruct->next == NULL)
        { fprintf(fp,"NULL"); }
      else
        {
         if ((i + 1) > maxIndices)
           { fprintf(fp,"&%s%d_%d[%d]",DefmodulePrefix(),imageID,version ,0); }
         else
           { fprintf(fp,"&%s%d_%d[%d]",DefmodulePrefix(),imageID,version - 1,i); }
        }

      i++;
      theConstruct = theConstruct->next;
      if ((i > maxIndices) || (theConstruct == NULL))
        {
         fprintf(fp,"}};\n");
         fclose(fp);
         i = 1;
         version++;
         if (theConstruct != NULL)
           {
            if ((fp = NewCFile(fileName,fileID,version,CLIPS_FALSE)) == NULL)
              {
               fclose(itemsFile);
               return(0);
              }
            newHeader = CLIPS_TRUE;
           }
        }
      else
        { fprintf(fp,"},\n"); }
     }
   
   fprintf(itemsFile,"};\n");
   fclose(itemsFile);
   
   if (portItemCount == 0) return(1);
   
   return(PortItemsToCode(fileName,fileID,headerFP,imageID,maxIndices,&version));
  }
  
/************************************************************/
/* PortItemsToCode:     */
/************************************************************/
static int PortItemsToCode(fileName,fileID,headerFP,imageID,maxIndices,version)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  int *version;
  {
   FILE *fp;
   int i = 1;
   int newHeader = CLIPS_TRUE;
   struct defmodule *theDefmodule = NULL;
   struct portItem *thePortItem = NULL;
   int portItemCount, arrayVersion;
   int importChecked = CLIPS_FALSE;
   int exportChecked = CLIPS_FALSE;
   
   if ((fp = NewCFile(fileName,fileID,*version,CLIPS_FALSE)) == NULL) return(0);
   newHeader = CLIPS_TRUE;

   portItemCount = 0;
   arrayVersion = 1;
   i = 1;

   thePortItem = GetNextPortItem(&theDefmodule,&thePortItem,&importChecked,&exportChecked);

   while (thePortItem != NULL)
     {
      if (newHeader)
        {
         fprintf(fp,"struct portItem %s%d_%d[] = {\n",PortPrefix(),imageID,arrayVersion);
         fprintf(headerFP,"extern struct portItem %s%d_%d[];\n",PortPrefix(),imageID,arrayVersion);
         newHeader = CLIPS_FALSE;
        }

      fprintf(fp,"{");
      PrintSymbolReference(fp,thePortItem->moduleName);
      fprintf(fp,",");
      PrintSymbolReference(fp,thePortItem->constructType);
      fprintf(fp,",");
      PrintSymbolReference(fp,thePortItem->constructName);
      fprintf(fp,",");
   
      if (thePortItem->next == NULL)
        { fprintf(fp,"NULL}"); }
      else
        {
         fprintf(fp,"&%s%d_%d[%d]}",PortPrefix(),imageID,
                                  ((portItemCount+1) / maxIndices) + 1,
                                   (portItemCount+1) % maxIndices);
        }

      portItemCount++;
      i++;
      
      thePortItem = GetNextPortItem(&theDefmodule,&thePortItem,&importChecked,&exportChecked);

      if ((i == maxIndices) || (thePortItem == NULL))
        {
         fprintf(fp,"};\n");
         fclose(fp);
         i = 1;
         (*version)++;
         arrayVersion++;
         if (thePortItem != NULL)
           {
            if ((fp = NewCFile(fileName,fileID,*version,CLIPS_FALSE)) == NULL) return(0);
            newHeader = CLIPS_TRUE;
           }
        }
      else
        { fprintf(fp,",\n"); }
     }
   return(1);
  }
     
/**********************************/
/* GetNextPortItem: */
/**********************************/
static struct portItem *GetNextPortItem(theDefmodule,thePortItem,importChecked,exportChecked)
  struct defmodule **theDefmodule;
  struct portItem **thePortItem;
  int *importChecked;
  int *exportChecked;
  {
   if (*theDefmodule == NULL) 
     {
      *theDefmodule = (struct defmodule *) GetNextDefmodule(NULL);
      *thePortItem = NULL;
      *importChecked = CLIPS_FALSE;
      *exportChecked = CLIPS_FALSE;
     }
     
   while (*theDefmodule != NULL)
     {
      if (*thePortItem != NULL) *thePortItem = (*thePortItem)->next;
      
      if (*thePortItem == NULL)
        {
         if (*importChecked && ! (*exportChecked)) 
           {
            *exportChecked = CLIPS_TRUE;
            *thePortItem = (*theDefmodule)->exportList;
           }
         else if (! (*importChecked))
           {
            *thePortItem = (*theDefmodule)->importList;
            *importChecked = CLIPS_TRUE;
            if (*thePortItem == NULL)
              {
               *thePortItem = (*theDefmodule)->exportList;
               *exportChecked = CLIPS_TRUE;
              }
           }
        }
        
      if (*thePortItem != NULL) return(*thePortItem);
     
      *theDefmodule = (struct defmodule *) GetNextDefmodule(*theDefmodule);
      *importChecked = CLIPS_FALSE;
      *exportChecked = CLIPS_FALSE;
     }
     
   return(NULL);
  }
     
#endif /* CONSTRUCT_COMPILER && (! RUN_TIME) */


