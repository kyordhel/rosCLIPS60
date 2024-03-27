   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           SYMBOL CONSTRUCT COMPILER MODULE          */
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

#define _SYMBLCMP_SOURCE_

#include "setup.h"

#if CONSTRUCT_COMPILER && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "symbol.h"
#include "clipsmem.h"
#include "constant.h"
#include "exprnpsr.h"
#include "cstrccom.h"
#include "constrct.h"
#include "argacces.h"
#include "cstrncmp.h"
#include "router.h"
#include "conscomp.h"
#include "utility.h"

#include "symblcmp.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                         SymbolsToCode(char *,int);
   static int                         BitMapHashNodesToCode(char *,int);
   static int                         BitMapsToCode(char *,int);
   static int                         FloatsToCode(char *,int);
   static int                         IntegersToCode(char *,int);
   static int                         HashTablesToCode(char *);
   static VOID                        PrintCString(FILE *,char *);
#else
   static int                         SymbolsToCode();
   static int                         BitMapHashNodesToCode();
   static int                         BitMapsToCode();
   static int                         FloatsToCode();
   static int                         IntegersToCode();
   static int                         HashTablesToCode();
   static VOID                        PrintCString();
#endif

/********************************************************/
/* AtomicValuesToCode:                                       */
/********************************************************/
globle VOID AtomicValuesToCode(fileName)
  char *fileName;
  {
   int version;

   SetAtomicValueIndices(CLIPS_TRUE);

   HashTablesToCode(fileName);

   version = SymbolsToCode(fileName,5);
   version = FloatsToCode(fileName,version);
   version = IntegersToCode(fileName,version);
   version = BitMapHashNodesToCode(fileName,version);
   version = BitMapsToCode(fileName,version);
  }

/**********************************************************************/
/* SymbolsToCode: Loops through the hash table calling enum_entries to */
/*   produce the definitions for each of the hash table entries.      */
/**********************************************************************/
static int SymbolsToCode(fileName,version)
  char *fileName;
  int version;
  {
   int i, j;
   struct symbolHashNode *hashPtr;
   int count;
   int numberOfEntries;
   struct symbolHashNode **symbolTable;
   int newHeader = CLIPS_TRUE;
   int arrayVersion = 1;
   FILE *fp;

   /*====================================*/
   /* Count the total number of entries. */
   /*====================================*/

   symbolTable = GetSymbolTable();
   count = numberOfEntries = 0;

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      hashPtr = symbolTable[i];
      while (hashPtr != NULL)
        {
         numberOfEntries++;
         hashPtr = hashPtr->next;
        }
     }

   if (numberOfEntries == 0) return(version);

   for (i = 1; i <= (numberOfEntries / MaxIndices) + 1 ; i++)
     { fprintf(HeaderFP,"extern struct symbolHashNode S%d_%d[];\n",ImageID,i); }

   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(-1);

   /*===================*/
   /* List the entries. */
   /*===================*/

   j = 0;

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      hashPtr = symbolTable[i];
      while (hashPtr != NULL)
        {
         if (newHeader)
           {
            fprintf(fp,"struct symbolHashNode S%d_%d[] = {\n",ImageID,arrayVersion);
            newHeader = CLIPS_FALSE;
           }

         if (hashPtr->next == NULL)
           { fprintf(fp,"{NULL,"); }
         else
           {
            if ((j + 1) >= MaxIndices)
              { fprintf(fp,"{&S%d_%d[%d],",ImageID,arrayVersion + 1,0); }
            else
              { fprintf(fp,"{&S%d_%d[%d],",ImageID,arrayVersion,j + 1); }
           }

         fprintf(fp,"%d,0,0,0,%d,",hashPtr->count + 1,i);
         PrintCString(fp,hashPtr->contents);

         count++;
         j++;

         if ((count == numberOfEntries) || (j >= MaxIndices))
           {
            fprintf(fp,"}};\n");
            fclose(fp);
            j = 0;
            arrayVersion++;
            version++;
            if (count < numberOfEntries)
              {
               if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(0);
               newHeader = CLIPS_TRUE;
              }
           }
         else
           { fprintf(fp,"},\n"); }

         hashPtr = hashPtr->next;
        }
     }

   return(version);
  }
  
/***********************************************************************/
/* BitMapHashNodesToCode:       */
/***********************************************************************/
static int BitMapHashNodesToCode(fileName,version)
  char *fileName;
  int version;
  {
   int i, j;
   struct bitMapHashNode *hashPtr;
   int count;
   int numberOfEntries;
   struct bitMapHashNode **bitMapTable;
   int newHeader = CLIPS_TRUE;
   int arrayVersion = 1;
   FILE *fp;
   int longsReqdPartition = 1,longsReqdPartitionCount = 0;
   
   /*====================================*/
   /* Count the total number of entries. */
   /*====================================*/

   bitMapTable = GetBitMapTable();
   count = numberOfEntries = 0;
   
   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      hashPtr = bitMapTable[i];
      while (hashPtr != NULL)
        {
         numberOfEntries++;
         hashPtr = hashPtr->next;
        }
     }

   if (numberOfEntries == 0) return(version);

   for (i = 1; i <= (numberOfEntries / MaxIndices) + 1 ; i++)
     { fprintf(HeaderFP,"extern struct bitMapHashNode B%d_%d[];\n",ImageID,i); }
     
   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(-1);

   /*===================*/
   /* List the entries. */
   /*===================*/

   j = 0;

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      hashPtr = bitMapTable[i];
      while (hashPtr != NULL)
        {
         if (newHeader)
           {
            fprintf(fp,"struct bitMapHashNode B%d_%d[] = {\n",ImageID,arrayVersion);
            newHeader = CLIPS_FALSE;
           }

         if (hashPtr->next == NULL)
           { fprintf(fp,"{NULL,"); }
         else
           {
            if ((j + 1) >= MaxIndices)
              { fprintf(fp,"{&B%d_%d[%d],",ImageID,arrayVersion + 1,0); }
            else
              { fprintf(fp,"{&B%d_%d[%d],",ImageID,arrayVersion,j + 1); }
           }

         fprintf(fp,"%d,0,0,0,%d,(char *) &L%d_%d[%d],%d",
                     hashPtr->count + 1,i,
                     ImageID,longsReqdPartition,longsReqdPartitionCount,
                     hashPtr->size);
                              
         longsReqdPartitionCount += (int) (hashPtr->size / sizeof(unsigned long));
         if ((hashPtr->size % sizeof(unsigned long)) != 0)
           longsReqdPartitionCount++;
         if (longsReqdPartitionCount >= MaxIndices)
           {
            longsReqdPartitionCount = 0;
            longsReqdPartition++;
           }
                    
         count++;
         j++;

         if ((count == numberOfEntries) || (j >= MaxIndices))
           {
            fprintf(fp,"}};\n");
            fclose(fp);
            j = 0;
            arrayVersion++;
            version++;
            if (count < numberOfEntries)
              {
               if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(0);
               newHeader = CLIPS_TRUE;
              }
           }
         else
           { fprintf(fp,"},\n"); }

         hashPtr = hashPtr->next;
        }
     }

   return(version);
  }

/***********************************************************************/
/* BitMapsToCode:       */
/***********************************************************************/
static int BitMapsToCode(fileName,version)
  char *fileName;
  int version;
  {
   int i, j, k, l;
   struct bitMapHashNode *hashPtr;
   int count;
   int numberOfEntries;
   struct bitMapHashNode **bitMapTable;
   int newHeader = CLIPS_TRUE;
   int arrayVersion = 1;
   FILE *fp;
   unsigned long tmpLong;
   int longsReqd;
   
   /*====================================*/
   /* Count the total number of entries. */
   /*====================================*/

   bitMapTable = GetBitMapTable();
   count = numberOfEntries = 0;
   
   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      hashPtr = bitMapTable[i];
      while (hashPtr != NULL)
        {
         numberOfEntries += (int) (hashPtr->size / sizeof(unsigned long));
         if ((hashPtr->size % sizeof(unsigned long)) != 0)
           numberOfEntries++;
         hashPtr = hashPtr->next;
        }
     }

   if (numberOfEntries == 0) return(version);
     
   for (i = 1; i <= (numberOfEntries / MaxIndices) + 1 ; i++)
     { fprintf(HeaderFP,"extern unsigned long L%d_%d[];\n",ImageID,i); }

   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(-1);

   /*===================*/
   /* List the entries. */
   /*===================*/

   j = 0;

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      hashPtr = bitMapTable[i];
      while (hashPtr != NULL)
        {
         if (newHeader)
           {
            fprintf(fp,"unsigned long L%d_%d[] = {\n",ImageID,arrayVersion);
            newHeader = CLIPS_FALSE;
           }
         
         longsReqd = (int) (hashPtr->size / sizeof(unsigned long));
         if ((hashPtr->size % sizeof(unsigned long)) != 0)
           longsReqd++;
           
         for (k = 0 ; k < longsReqd ; k++)
           {
            if (k > 0)
              fprintf(fp,",");
            tmpLong = 0L;
            for (l = 0 ; 
                 ((l < sizeof(unsigned long)) && 
                 (((k * sizeof(unsigned long)) + l) < hashPtr->size)) ;
                 l++)
              ((char *) &tmpLong)[l] = hashPtr->contents[(k * sizeof(unsigned long)) + l];
            fprintf(fp,"0x%lxL",tmpLong);
           }
           
         count += longsReqd;   
         j += longsReqd;

         if ((count == numberOfEntries) || (j >= MaxIndices))
           {
            fprintf(fp,"};\n");
            fclose(fp);
            j = 0;
            arrayVersion++;
            version++;
            if (count < numberOfEntries)
              {
               if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(0);
               newHeader = CLIPS_TRUE;
              }
           }
         else
           { fprintf(fp,",\n"); }

         hashPtr = hashPtr->next;
        }
     }

   return(version);
  }

/**********************************************************************/
/* FloatsToCode: Loops through the hash table calling enum_entries to */
/*   produce the definitions for each of the hash table entries.      */
/**********************************************************************/
static int FloatsToCode(fileName,version)
  char *fileName;
  int version;
  {
   int i, j;
   struct floatHashNode *hashPtr;
   int count;
   int numberOfEntries;
   struct floatHashNode **floatTable;
   int newHeader = CLIPS_TRUE;
   FILE *fp;
   int arrayVersion = 1;


   /*====================================*/
   /* Count the total number of entries. */
   /*====================================*/

   floatTable = GetFloatTable();
   count = numberOfEntries = 0;

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
     {
      hashPtr = floatTable[i];
      while (hashPtr != NULL)
        {
         numberOfEntries++;
         hashPtr = hashPtr->next;
        }
     }

   if (numberOfEntries == 0) return(version);

   for (i = 1; i <= (numberOfEntries / MaxIndices) + 1 ; i++)
     { fprintf(HeaderFP,"extern struct floatHashNode F%d_%d[];\n",ImageID,i); }

   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(-1);

   /*===================*/
   /* List the entries. */
   /*===================*/

   j = 0;

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
     {
      hashPtr = floatTable[i];
      while (hashPtr != NULL)
        {
         if (newHeader)
           {
            fprintf(fp,"struct floatHashNode F%d_%d[] = {\n",ImageID,arrayVersion);
            newHeader = CLIPS_FALSE;
           }

         if (hashPtr->next == NULL)
           { fprintf(fp,"{NULL,"); }
         else
           {
            if ((j + 1) >= MaxIndices)
              { fprintf(fp,"{&F%d_%d[%d],",ImageID,arrayVersion + 1,0); }
            else
              { fprintf(fp,"{&F%d_%d[%d],",ImageID,arrayVersion,j + 1); }
           }

         fprintf(fp,"%d,0,0,0,%d,",hashPtr->count + 1,i);
         fprintf(fp,"%s",FloatToString(hashPtr->contents));

         count++;
         j++;

         if ((count == numberOfEntries) || (j >= MaxIndices))
           {
            fprintf(fp,"}};\n");
            fclose(fp);
            j = 0;
            version++;
            arrayVersion++;
            if (count < numberOfEntries)
              {
               if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(0);
               newHeader = CLIPS_TRUE;
              }
           }
         else
           { fprintf(fp,"},\n"); }

         hashPtr = hashPtr->next;
        }
     }

   return(version);
  }

/**********************************************************************/
/* IntegersToCode: Loops through the hash table calling enum_entries to */
/*   produce the definitions for each of the hash table entries.      */
/**********************************************************************/
static int IntegersToCode(fileName,version)
  char *fileName;
  int version;
  {
   int i, j;
   struct integerHashNode *hashPtr;
   int count;
   int numberOfEntries;
   struct integerHashNode **integerTable;
   int newHeader = CLIPS_TRUE;
   FILE *fp;
   int arrayVersion = 1;


   /*====================================*/
   /* Count the total number of entries. */
   /*====================================*/

   integerTable = GetIntegerTable();
   count = numberOfEntries = 0;

   for (i = 0; i < INTEGER_HASH_SIZE; i++)
     {
      hashPtr = integerTable[i];
      while (hashPtr != NULL)
        {
         numberOfEntries++;
         hashPtr = hashPtr->next;
        }
     }

   if (numberOfEntries == 0) return(version);

   for (i = 1; i <= (numberOfEntries / MaxIndices) + 1 ; i++)
     { fprintf(HeaderFP,"extern struct integerHashNode I%d_%d[];\n",ImageID,i); }

   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(-1);

   /*===================*/
   /* List the entries. */
   /*===================*/

   j = 0;

   for (i = 0; i < INTEGER_HASH_SIZE; i++)
     {
      hashPtr = integerTable[i];
      while (hashPtr != NULL)
        {
         if (newHeader)
           {
            fprintf(fp,"struct integerHashNode I%d_%d[] = {\n",ImageID,arrayVersion);
            newHeader = CLIPS_FALSE;
           }

         if (hashPtr->next == NULL)
           { fprintf(fp,"{NULL,"); }
         else
           {
            if ((j + 1) >= MaxIndices)
              { fprintf(fp,"{&I%d_%d[%d],",ImageID,arrayVersion + 1,0); }
            else
              { fprintf(fp,"{&I%d_%d[%d],",ImageID,arrayVersion,j + 1); }
           }

         fprintf(fp,"%d,0,0,0,%d,",hashPtr->count + 1,i);
         fprintf(fp,"%ld",hashPtr->contents);

         count++;
         j++;

         if ((count == numberOfEntries) || (j >= MaxIndices))
           {
            fprintf(fp,"}};\n");
            fclose(fp);
            j = 0;
            version++;
            arrayVersion++;
            if (count < numberOfEntries)
              {
               if ((fp = NewCFile(fileName,1,version,CLIPS_FALSE)) == NULL) return(0);
               newHeader = CLIPS_TRUE;
              }
           }
         else
           { fprintf(fp,"},\n"); }

         hashPtr = hashPtr->next;
        }
     }

   return(version);
  }
  
/****************************************************************/
/* HashTablesToCode: Produces the definition of the hash tables */
/*   for symbols, floats, and integers.                         */
/****************************************************************/
static int HashTablesToCode(fileName)
  char *fileName;
  {
   int i;
   FILE *fp;
   struct symbolHashNode **symbolTable;
   struct floatHashNode **floatTable;
   struct integerHashNode **integerTable;
   struct bitMapHashNode **bitMapTable;

   /*========================*/
   /* Dump the symbol table. */
   /*========================*/

   symbolTable = GetSymbolTable();

   if ((fp = NewCFile(fileName,1,1,CLIPS_FALSE)) == NULL) return(0);

   fprintf(HeaderFP,"extern struct symbolHashNode *sht%d[];\n",ImageID);
   fprintf(fp,"struct symbolHashNode *sht%d[%d] = {\n",ImageID,SYMBOL_HASH_SIZE);

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
      {
       PrintSymbolReference(fp,symbolTable[i]);

       if (i + 1 != SYMBOL_HASH_SIZE) fprintf(fp,",\n");
      }

    fprintf(fp,"};\n");

    fclose(fp);

   /*========================*/
   /* Dump the float table. */
   /*========================*/

   floatTable = GetFloatTable();

   if ((fp = NewCFile(fileName,1,2,CLIPS_FALSE)) == NULL) return(0);

   fprintf(HeaderFP,"extern struct floatHashNode *fht%d[];\n",ImageID);
   fprintf(fp,"struct floatHashNode *fht%d[%d] = {\n",ImageID,FLOAT_HASH_SIZE);

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
      {
       if (floatTable[i] == NULL) { fprintf(fp,"NULL"); }
       else PrintFloatReference(fp,floatTable[i]);

       if (i + 1 != FLOAT_HASH_SIZE) fprintf(fp,",\n");
      }

    fprintf(fp,"};\n");

    fclose(fp);

   /*=========================*/
   /* Dump the integer table. */
   /*=========================*/

   integerTable = GetIntegerTable();

   if ((fp = NewCFile(fileName,1,3,CLIPS_FALSE)) == NULL) return(0);

   fprintf(HeaderFP,"extern struct integerHashNode *iht%d[];\n",ImageID);
   fprintf(fp,"struct integerHashNode *iht%d[%d] = {\n",ImageID,INTEGER_HASH_SIZE);

   for (i = 0; i < INTEGER_HASH_SIZE; i++)
      {
       if (integerTable[i] == NULL) { fprintf(fp,"NULL"); }
       else PrintIntegerReference(fp,integerTable[i]);

       if (i + 1 != INTEGER_HASH_SIZE) fprintf(fp,",\n");
      }

    fprintf(fp,"};\n");

    fclose(fp);

   /*========================*/
   /* Dump the bitmap table. */
   /*========================*/

   bitMapTable = GetBitMapTable();

   if ((fp = NewCFile(fileName,1,4,CLIPS_FALSE)) == NULL) return(0);

   fprintf(HeaderFP,"extern struct bitMapHashNode *bmht%d[];\n",ImageID);
   fprintf(fp,"struct bitMapHashNode *bmht%d[%d] = {\n",ImageID,BITMAP_HASH_SIZE);

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
      {
       PrintBitMapReference(fp,bitMapTable[i]);

       if (i + 1 != BITMAP_HASH_SIZE) fprintf(fp,",\n");
      }

    fprintf(fp,"};\n");

    fclose(fp);

    return(1);
   }
   
/************************************************************/
/* PrintSymbolReference:                          */
/************************************************************/
globle VOID PrintSymbolReference(fp,sPtr)
  FILE *fp;
  struct symbolHashNode *sPtr;
  {
   if (sPtr == NULL) fprintf(fp,"NULL");
   else fprintf(fp,"&S%d_%d[%d]",ImageID,
                                    (sPtr->bucket / MaxIndices) + 1,
                                    sPtr->bucket % MaxIndices);
  }

/************************************************************/
/* PrintFloatReference:                          */
/************************************************************/
globle VOID PrintFloatReference(fp,sPtr)
  FILE *fp;
  struct floatHashNode *sPtr;
  {
   fprintf(fp,"&F%d_%d[%d]",ImageID,
                                 (sPtr->bucket / MaxIndices) + 1,
                                 sPtr->bucket % MaxIndices);
  }

/************************************************************/
/* PrintIntegerReference:                          */
/************************************************************/
globle VOID PrintIntegerReference(fp,sPtr)
  FILE *fp;
  struct integerHashNode *sPtr;
  {
   fprintf(fp,"&I%d_%d[%d]",ImageID,
                              (sPtr->bucket / MaxIndices) + 1,
                              sPtr->bucket % MaxIndices);
  }
  
/************************************************************/
/* PrintBitMapReference:                          */
/************************************************************/
globle VOID PrintBitMapReference(fp,bmPtr)
  FILE *fp;
  struct bitMapHashNode *bmPtr;
  {
   if (bmPtr == NULL) fprintf(fp,"NULL");
   else fprintf(fp,"&B%d_%d[%d]",ImageID,
                                    (bmPtr->bucket / MaxIndices) + 1,
                                    bmPtr->bucket % MaxIndices);
  }
  

  
/************************************************************/
/* PrintCString:        */
/************************************************************/
static VOID PrintCString(file_ptr,str)
  FILE *file_ptr;
  char *str;
  {
   int i, slen;

   fprintf(file_ptr,"\"");
   slen = strlen(str);
   for (i = 0 ; i < slen ; i++)
     {
      if ((str[i] == '"') || (str[i] == '\\'))
        {
         fputc('\\',file_ptr);
         fputc(str[i],file_ptr);
        }
      else if (str[i] == '\n')
        {
         fputc('\\',file_ptr);
         fputc('n',file_ptr);
        }
      else
        { fputc(str[i],file_ptr); }
     }

   fprintf(file_ptr,"\"");
  }

#endif
