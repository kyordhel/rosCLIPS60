   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                     BSAVE MODULE                    */
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

#define _BSAVE_SOURCE_

#include "setup.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE || BLOAD_INSTANCES || BSAVE_INSTANCES

#include "clipsmem.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"
#include "cstrnbin.h"
#include "moduldef.h"
#include "bload.h"

#include "bsave.h"

#include "symblbin.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long                                   NumberOfSymbols = 0;
   static long                                   NumberOfFloats = 0;
   static long                                   NumberOfIntegers = 0;
   static long                                   NumberOfBitMaps = 0;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle SYMBOL_HN * HUGE_ADDR                 *SymbolArray;
   globle struct floatHashNode * HUGE_ADDR      *FloatArray;
   globle INTEGER_HN * HUGE_ADDR                *IntegerArray;
   globle BITMAP_HN * HUGE_ADDR                 *BitMapArray;
   
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                        ReadNeededSymbols(void);
   static VOID                        ReadNeededFloats(void);
   static VOID                        ReadNeededIntegers(void);
   static VOID                        ReadNeededBitMaps(void);
#if BLOAD_AND_BSAVE || BSAVE_INSTANCES
   static VOID                        WriteNeededSymbols(FILE *);
   static VOID                        WriteNeededFloats(FILE *);
   static VOID                        WriteNeededIntegers(FILE *);
   static VOID                        WriteNeededBitMaps(FILE *);
#endif
#else
   static VOID                        ReadNeededSymbols();
   static VOID                        ReadNeededFloats();
   static VOID                        ReadNeededIntegers();
   static VOID                        ReadNeededBitMaps();
#if BLOAD_AND_BSAVE || BSAVE_INSTANCES
   static VOID                        WriteNeededSymbols();
   static VOID                        WriteNeededFloats();
   static VOID                        WriteNeededIntegers();
   static VOID                        WriteNeededBitMaps();
#endif
#endif

#if BLOAD_AND_BSAVE || BSAVE_INSTANCES

/****************************************************************************/
/* WriteNeededAtomicValues:                                                 */
/****************************************************************************/
globle VOID WriteNeededAtomicValues(fp)
  FILE *fp;
  {
   WriteNeededSymbols(fp);
   WriteNeededFloats(fp);
   WriteNeededIntegers(fp);
   WriteNeededBitMaps(fp);
  }
  
/****************************************************************************/
/* InitAtomicValueNeededFlags:                                                  */
/****************************************************************************/
globle VOID InitAtomicValueNeededFlags()
  {
   int i;
   SYMBOL_HN *symbolPtr, **symbolArray;
   FLOAT_HN *floatPtr, **floatArray;
   INTEGER_HN *integerPtr, **integerArray;
   BITMAP_HN *bitMapPtr, **bitMapArray;

   /*===============================*/
   /* Initialize the symbol tables. */
   /*===============================*/

   symbolArray = GetSymbolTable();

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      symbolPtr = symbolArray[i];
      while (symbolPtr != NULL)
        {
         symbolPtr->neededSymbol = CLIPS_FALSE;
         symbolPtr = symbolPtr->next;
        }
     }

   floatArray = GetFloatTable();

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
     {
      floatPtr = floatArray[i];
      while (floatPtr != NULL)
        {
         floatPtr->neededFloat = CLIPS_FALSE;
         floatPtr = floatPtr->next;
        }
     }

   integerArray = GetIntegerTable();

   for (i = 0; i < INTEGER_HASH_SIZE; i++)
     {
      integerPtr = integerArray[i];
      while (integerPtr != NULL)
        {
         integerPtr->neededInteger = CLIPS_FALSE;
         integerPtr = integerPtr->next;
        }
     }

   bitMapArray = GetBitMapTable();

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      bitMapPtr = bitMapArray[i];
      while (bitMapPtr != NULL)
        {
         bitMapPtr->neededBitMap = CLIPS_FALSE;
         bitMapPtr = bitMapPtr->next;
        }
     }
  }

/*****************************************************************/
/* WriteNeededSymbols: Stores all of the symbols in the symbol   */
/*   table needed for this binary image in the binary save file. */
/*****************************************************************/
static VOID WriteNeededSymbols(fp)
  FILE *fp;
  {
   int i, length;
   SYMBOL_HN **symbolArray;
   SYMBOL_HN *symbolPtr;
   unsigned long int numberOfUsedSymbols = 0, size = 0;

   /*=================================*/
   /* Get a copy of the symbol table. */
   /*=================================*/

   symbolArray = GetSymbolTable();

   /*======================================================*/
   /* Get the number of symbols and the total string size. */
   /*======================================================*/

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      symbolPtr = symbolArray[i];
      while (symbolPtr != NULL)
        {
         if (symbolPtr->neededSymbol)
           {
            numberOfUsedSymbols++;
            size += strlen(symbolPtr->contents) + 1;
           }
         symbolPtr = symbolPtr->next;
        }
     }

   /*=============================================*/
   /* Write out the symbols and the string sizes. */
   /*=============================================*/

   GenWrite((VOID *) &numberOfUsedSymbols,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite((VOID *) &size,(unsigned long) sizeof(unsigned long int),fp);

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      symbolPtr = symbolArray[i];
      while (symbolPtr != NULL)
        {
         if (symbolPtr->neededSymbol)
           {
            length = strlen(symbolPtr->contents) + 1;
            GenWrite((VOID *) symbolPtr->contents,(unsigned long) length,fp);
           }
         symbolPtr = symbolPtr->next;
        }
     }
  }

/*****************************************************************/
/* WriteNeededFloats: Stores all of the floats in the float   */
/*   table needed for this binary image in the binary save file. */
/*****************************************************************/
static VOID WriteNeededFloats(fp)
  FILE *fp;
  {
   int i;
   FLOAT_HN **floatArray;
   FLOAT_HN *floatPtr;
   unsigned long int numberOfUsedFloats = 0;

   /*=================================*/
   /* Get a copy of the symbol table. */
   /*=================================*/

   floatArray = GetFloatTable();

   /*======================================================*/
   /* Get the number of symbols and the total string size. */
   /*======================================================*/

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
     {
      floatPtr = floatArray[i];
      while (floatPtr != NULL)
        {
         if (floatPtr->neededFloat) numberOfUsedFloats++;
         floatPtr = floatPtr->next;
        }
     }

   /*======================================================*/
   /* Write out the number of floats and the float values. */
   /*======================================================*/

   GenWrite(&numberOfUsedFloats,(unsigned long) sizeof(unsigned long int),fp);

   for (i = 0 ; i < FLOAT_HASH_SIZE; i++)
     {
      floatPtr = floatArray[i];
      while (floatPtr != NULL)
        {
         if (floatPtr->neededFloat)
           { GenWrite(&floatPtr->contents,
                      (unsigned long) sizeof(floatPtr->contents),fp); }
         floatPtr = floatPtr->next;
        }
     }
  }

/*****************************************************************/
/* WriteNeededIntegers: Stores all of the integers in the integer   */
/*   table needed for this binary image in the binary save file. */
/*****************************************************************/
static VOID WriteNeededIntegers(fp)
  FILE *fp;
  {
   int i;
   INTEGER_HN **integerArray;
   INTEGER_HN *integerPtr;
   unsigned long int numberOfUsedIntegers = 0;

   /*=================================*/
   /* Get a copy of the symbol table. */
   /*=================================*/

   integerArray = GetIntegerTable();

   /*======================================================*/
   /* Get the number of symbols and the total string size. */
   /*======================================================*/

   for (i = 0 ; i < INTEGER_HASH_SIZE; i++)
     {
      integerPtr = integerArray[i];
      while (integerPtr != NULL)
        {
         if (integerPtr->neededInteger) numberOfUsedIntegers++;
         integerPtr = integerPtr->next;
        }
     }

   /*======================================================*/
   /* Write out the number of integers and the integer values. */
   /*======================================================*/

   GenWrite(&numberOfUsedIntegers,(unsigned long) sizeof(unsigned long int),fp);

   for (i = 0 ; i < INTEGER_HASH_SIZE; i++)
     {
      integerPtr = integerArray[i];
      while (integerPtr != NULL)
        {
         if (integerPtr->neededInteger)
           {
            GenWrite(&integerPtr->contents,
                     (unsigned long) sizeof(integerPtr->contents),fp);
           }
         integerPtr = integerPtr->next;
        }
     }
  }
  
/*****************************************************************/
/* WriteNeededBitMaps: */
/*****************************************************************/
static VOID WriteNeededBitMaps(fp)
  FILE *fp;
  {
   int i;
   BITMAP_HN **bitMapArray;
   BITMAP_HN *bitMapPtr;
   unsigned long int numberOfUsedBitMaps = 0, size = 0;
   char tempSize;

   /*=================================*/
   /* Get a copy of the symbol table. */
   /*=================================*/

   bitMapArray = GetBitMapTable();

   /*======================================================*/
   /* Get the number of bitmaps and the total bitmap size. */
   /*======================================================*/

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      bitMapPtr = bitMapArray[i];
      while (bitMapPtr != NULL)
        {
         if (bitMapPtr->neededBitMap)
           {
            numberOfUsedBitMaps++;
            size += bitMapPtr->size + 1;
           }
         bitMapPtr = bitMapPtr->next;
        }
     }
     
   /*========================================*/
   /* Write out the bitmaps and their sizes. */
   /*========================================*/

   GenWrite((VOID *) &numberOfUsedBitMaps,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite((VOID *) &size,(unsigned long) sizeof(unsigned long int),fp);

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      bitMapPtr = bitMapArray[i];
      while (bitMapPtr != NULL)
        {
         if (bitMapPtr->neededBitMap)
           {
            tempSize = bitMapPtr->size;
            GenWrite((VOID *) &tempSize,(unsigned long) sizeof(char),fp);
            GenWrite((VOID *) bitMapPtr->contents,(unsigned long) bitMapPtr->size,fp);
           }
         bitMapPtr = bitMapPtr->next;
        }
     }
  }
  

#endif

/************************************************/
/* ReadNeededAtomicValues:                      */
/************************************************/
globle VOID ReadNeededAtomicValues()
  {
   ReadNeededSymbols();
   ReadNeededFloats();
   ReadNeededIntegers();
   ReadNeededBitMaps();
  }

/************************************************/
/* ReadNeededSymbols: Reads in the symbols used */
/*   by the binary image.                       */
/************************************************/
static VOID ReadNeededSymbols()
  {
   char *symbolNames, *namePtr;
   unsigned long space;
   long i;

   /*=================================================*/
   /* Determine the number of symbol names to be read */
   /* and space required for them.                    */
   /*=================================================*/

   GenRead((VOID *) &NumberOfSymbols,(unsigned long) sizeof(long int));
   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   if (NumberOfSymbols == 0)
     {
      SymbolArray = NULL;
      return;
     }

   /*=======================================*/
   /* Allocate area for strings to be read. */
   /*=======================================*/

   symbolNames = (char *) gm3((long) space);
   GenRead((VOID *) symbolNames,space);

   /*================================================*/
   /* Store the symbol pointers in the symbol array. */
   /*================================================*/

   SymbolArray = (SYMBOL_HN **)
                 gm3((long) sizeof(SYMBOL_HN *) *  NumberOfSymbols);
   namePtr = symbolNames;
   for (i = 0; i < NumberOfSymbols; i++)
     {
      SymbolArray[i] = (SYMBOL_HN *) AddSymbol(namePtr);
      namePtr += strlen(namePtr) + 1;
     }

   /*=======================*/
   /* Free the name buffer. */
   /*=======================*/

   rm3((VOID *) symbolNames,(long) space);
  }

/**********************************************/
/* ReadNeededFloats: Reads in the floats used */
/*   by the binary image.                     */
/**********************************************/
static VOID ReadNeededFloats()
  {
   double HUGE_ADDR *floatValues;
   long i;

   /*============================================*/
   /* Determine the number of floats to be read. */
   /*============================================*/

   GenRead(&NumberOfFloats,(unsigned long) sizeof(long int));
   if (NumberOfFloats == 0)
     {
      FloatArray = NULL;
      return;
     }

   /*===============================*/
   /* Allocate area for the floats. */
   /*===============================*/

   floatValues = (double *) gm3((long) sizeof(double) * NumberOfFloats);
   GenRead((VOID *) floatValues,(unsigned long) (sizeof(double) * NumberOfFloats));

   /*======================================*/
   /* Store the floats in the float array. */
   /*======================================*/

   FloatArray = (FLOAT_HN **)
               gm3((long) sizeof(FLOAT_HN *) * NumberOfFloats);
   for (i = 0; i < NumberOfFloats; i++)
     { FloatArray[i] = (FLOAT_HN *) AddDouble(floatValues[i]); }

   /*========================*/
   /* Free the float buffer. */
   /*========================*/

   rm3((VOID *) floatValues,(long) (sizeof(double) * NumberOfFloats));
  }

/**********************************************/
/* ReadNeededIntegers: Reads in the integers used */
/*   by the binary image.                     */
/**********************************************/
static VOID ReadNeededIntegers()
  {
   long int HUGE_ADDR *integerValues;
   long i;

   /*==============================================*/
   /* Determine the number of integers to be read. */
   /*==============================================*/

   GenRead(&NumberOfIntegers,(unsigned long) sizeof(unsigned long int));
   if (NumberOfIntegers == 0)
     {
      IntegerArray = NULL;
      return;
     }
     
   /*=================================*/
   /* Allocate area for the integers. */
   /*=================================*/

   integerValues = (long *) gm3((long) (sizeof(long) * NumberOfIntegers));
   GenRead((VOID *) integerValues,(unsigned long) (sizeof(long) * NumberOfIntegers));

   /*==========================================*/
   /* Store the integers in the integer array. */
   /*==========================================*/

   IntegerArray = (INTEGER_HN **)
           gm3((long) (sizeof(INTEGER_HN *) * NumberOfIntegers));
   for (i = 0; i < NumberOfIntegers; i++)
     { IntegerArray[i] = (INTEGER_HN *) AddLong(integerValues[i]); }

   /*==========================*/
   /* Free the integer buffer. */
   /*==========================*/

   rm3((VOID *) integerValues,(long) (sizeof(long int) * NumberOfIntegers));
  }
  
/************************************************/
/* ReadNeededBitMaps: Reads in the bitmaps used */
/*   by the binary image.                       */
/************************************************/
static VOID ReadNeededBitMaps()
  {
   char *bitMapStorage, *bitMapPtr;
   unsigned long space;
   long i;

   /*=================================================*/
   /* Determine the number of symbol names to be read */
   /* and space required for them.                    */
   /*=================================================*/

   GenRead((VOID *) &NumberOfBitMaps,(unsigned long) sizeof(long int));
   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   if (NumberOfBitMaps == 0)
     {
      BitMapArray = NULL;
      return;
     }
     
   /*=======================================*/
   /* Allocate area for strings to be read. */
   /*=======================================*/

   bitMapStorage = (char *) gm3((long) space);
   GenRead((VOID *) bitMapStorage,space);

   /*================================================*/
   /* Store the bitMap pointers in the bitmap array. */
   /*================================================*/

   BitMapArray = (BITMAP_HN **)
                 gm3((long) sizeof(BITMAP_HN *) *  NumberOfBitMaps);
   bitMapPtr = bitMapStorage;
   for (i = 0; i < NumberOfBitMaps; i++)
     {
      BitMapArray[i] = (BITMAP_HN *) AddBitMap(bitMapPtr+1,(int) *bitMapPtr);
      bitMapPtr += *bitMapPtr + 1;
     }

   /*=======================*/
   /* Free the name buffer. */
   /*=======================*/

   rm3((VOID *) bitMapStorage,(long) space);
  }
  
/************************************************/
/* FreeAtomicValueStorage:                     */
/************************************************/
globle VOID FreeAtomicValueStorage()
  {
   if (SymbolArray != NULL)
     rm3((VOID *) SymbolArray,(long) sizeof(SYMBOL_HN *) * NumberOfSymbols);
   if (FloatArray != NULL)
     rm3((VOID *) FloatArray,(long) sizeof(FLOAT_HN *) * NumberOfFloats);
   if (IntegerArray != NULL)
     rm3((VOID *) IntegerArray,(long) sizeof(INTEGER_HN *) * NumberOfIntegers);
   if (BitMapArray != NULL)
     rm3((VOID *) BitMapArray,(long) sizeof(BITMAP_HN *) * NumberOfBitMaps);
  }
  
#endif
