   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          CONSTRAINT CONSTRUCTS-TO-C MODULE          */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Support functions for the constructs-to-c of         */
/*      constraint records used by various constructs.       */
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

#define _CSTRNCMP_SOURCE_

#include "setup.h"

#if CONSTRUCT_COMPILER && (! RUN_TIME)

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "conscomp.h"

#include "cstrncmp.h"
  
/**************************************************************/
/* ConstraintsToCode: Loops through the constraint table to   */
/*   produce the code for each constraint entry in the table. */
/**************************************************************/
globle int ConstraintsToCode(fileName,fileID,headerFP,imageID,maxIndices)
  char *fileName;
  int fileID;
  FILE *headerFP;
  int imageID;
  int maxIndices;
  {
   int i, j, count;
   int newHeader = CLIPS_TRUE;
   FILE *fp;
   int version = 1;
   int arrayVersion = 1;
   unsigned long int numberOfConstraints = 0;
   CONSTRAINT_RECORD *tmpPtr;

   /*====================================*/
   /* Count the total number of entries. */
   /*====================================*/
   
   for (i = 0 ; i < SIZE_CONSTRAINT_HASH; i++)
     {
      tmpPtr = ConstraintHashtable[i];
      while (tmpPtr != NULL)
        {
         tmpPtr->bsaveIndex = numberOfConstraints++;
         tmpPtr = tmpPtr->next;
        }
     }

   if ((! GetDynamicConstraintChecking()) && (numberOfConstraints != 0))
     {
      numberOfConstraints = 0;
      PrintWarningID("CSTRNCMP",1,CLIPS_FALSE);
      PrintCLIPS(WWARNING,"Constraints are not saved with a constructs-to-c image\n");
      PrintCLIPS(WWARNING,"  when dynamic constraint checking is disabled.\n");
     }

   if (numberOfConstraints == 0) return(-1);

   for (i = 1; i <= (numberOfConstraints / maxIndices) + 1 ; i++)
     { fprintf(headerFP,"extern CONSTRAINT_RECORD C%d_%d[];\n",imageID,i); }
     
   /*=============================================*/
   /* If dynamic constraint checking is disabled, */
   /* then no constraints are saved.              */
   /*=============================================*/
   

   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,fileID,version,CLIPS_FALSE)) == NULL) return(-1);

   /*===================*/
   /* List the entries. */
   /*===================*/

   j = 0;
   count = 0;

   for (i = 0; i < SIZE_CONSTRAINT_HASH; i++)
     {
      tmpPtr = ConstraintHashtable[i];
      while (tmpPtr != NULL)
        {
         if (newHeader)
           {
            fprintf(fp,"CONSTRAINT_RECORD C%d_%d[] = {\n",imageID,arrayVersion);
            newHeader = CLIPS_FALSE;
           }
           
         fprintf(fp,"{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                 tmpPtr->anyAllowed,
                 tmpPtr->symbolsAllowed,
                 tmpPtr->stringsAllowed,
                 tmpPtr->floatsAllowed,
                 tmpPtr->integersAllowed,
                 tmpPtr->instanceNamesAllowed,
                 tmpPtr->instanceAddressesAllowed,
                 tmpPtr->externalAddressesAllowed,
                 tmpPtr->multifieldsAllowed,  
                 tmpPtr->factAddressesAllowed,  
                 tmpPtr->anyRestriction,
                 tmpPtr->symbolRestriction,
                 tmpPtr->stringRestriction,
                 tmpPtr->floatRestriction,
                 tmpPtr->integerRestriction,
                 tmpPtr->instanceNameRestriction);

         fprintf(fp,",0,"); /* bsaveIndex */
   
         PrintHashedExpressionReference(fp,tmpPtr->restrictionList,imageID,maxIndices);
         fprintf(fp,",");
         PrintHashedExpressionReference(fp,tmpPtr->minValue,imageID,maxIndices);
         fprintf(fp,",");
         PrintHashedExpressionReference(fp,tmpPtr->maxValue,imageID,maxIndices);
         fprintf(fp,",");
         PrintHashedExpressionReference(fp,tmpPtr->minFields,imageID,maxIndices);
         fprintf(fp,",");
         PrintHashedExpressionReference(fp,tmpPtr->maxFields,imageID,maxIndices);
         
         if (tmpPtr->next == NULL)
           { fprintf(fp,",NULL,"); }
         else
           {
            if ((j + 1) >= maxIndices)
              { fprintf(fp,",&C%d_%d[%d],",imageID,arrayVersion + 1,0); }
            else
              { fprintf(fp,",&C%d_%d[%d],",imageID,arrayVersion,j + 1); }
           }
   
         fprintf(fp,"%d,%d",tmpPtr->bucket,tmpPtr->count + 1);
   
         count++;
         j++;

         if ((count == numberOfConstraints) || (j >= maxIndices))
           {
            fprintf(fp,"}};\n");
            fclose(fp);
            j = 0;
            version++;
            arrayVersion++;
            if (count < numberOfConstraints)
              {
               if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(0);
               newHeader = CLIPS_TRUE;
              }
           }
         else
           { fprintf(fp,"},\n"); }

         tmpPtr = tmpPtr->next;
        }
     }

   return(version);
  }
  
/************************************************************/
/* PrintConstraintReference:                                */
/************************************************************/
globle VOID PrintConstraintReference(fp,cPtr,imageID,maxIndices)
  FILE *fp;
  CONSTRAINT_RECORD *cPtr;
  int imageID, maxIndices;
  {
   if ((cPtr == NULL) || (! GetDynamicConstraintChecking()))
     { fprintf(fp,"NULL"); }
   else fprintf(fp,"&C%d_%d[%d]",imageID,
                                    (cPtr->bsaveIndex / maxIndices) + 1,
                                    cPtr->bsaveIndex % maxIndices);
  }
  
#endif



