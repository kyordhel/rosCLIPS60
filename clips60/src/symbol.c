   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    SYMBOL MODULE                    */
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

#define _SYMBOL_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"
#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "utility.h"
#include "argacces.h"
#include "symbol.h"

#define AVERAGE_STRING_SIZE 10
#define AVERAGE_BITMAP_SIZE sizeof(long)
#define NUMBER_OF_LONGS_FOR_HASH 25

/**********************************************************/
/* EPHEMERON STRUCTURE: Data structure used to keep track */
/*   of ephemeral symbols, floats, and integers.          */
/*                                                        */
/*   associatedValue: Contains a pointer to the storage   */
/*   structure for the symbol, float, or integer which is */
/*   ephemeral.                                           */
/*                                                        */
/*   next: Contains a pointer to the next ephemeral item  */
/*   in a list of ephemeral items.                        */
/**********************************************************/
struct ephemeron
  {
   GENERIC_HN *associatedValue;
   struct ephemeron *next;
  };

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    RemoveHashNode(GENERIC_HN *,GENERIC_HN **,int,int);
   static VOID                    AddEphemeralHashNode(GENERIC_HN *,struct ephemeron **,
                                                       int,int);
   static VOID                    RemoveEphemeralHashNodes(struct ephemeron **,
                                                           GENERIC_HN **,
                                                           int,int,int);
#else
   static VOID                    RemoveHashNode();
   static VOID                    AddEphemeralHashNode();
   static VOID                    RemoveEphemeralHashNodes();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle VOID               *CLIPSTrueSymbol;
   globle VOID               *CLIPSFalseSymbol;
   globle VOID               *PositiveInfinity;
   globle VOID               *NegativeInfinity;
   globle VOID               *Zero;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static SYMBOL_HN         **SymbolTable;
   static FLOAT_HN          **FloatTable;
   static INTEGER_HN        **IntegerTable;
   static BITMAP_HN         **BitMapTable;
   static struct ephemeron   *EphemeralSymbolList = NULL;
   static struct ephemeron   *EphemeralFloatList = NULL;
   static struct ephemeron   *EphemeralIntegerList = NULL;
   static struct ephemeron   *EphemeralBitMapList = NULL;
   static char               *FalseSymbol = "FALSE";
   static char               *TrueSymbol = "TRUE";

/********************************************************************/
/* AddSymbol:  Searches for the string in the symbol table. If the  */
/*   string is already in the symbol table, then the address of the */
/*   string's location in the symbol table is returned.  Otherwise, */
/*   the string is added to the symbol table and then the address   */
/*   of the string's location in the symbol table is returned.      */
/********************************************************************/
globle VOID *AddSymbol(str)
   char *str;
   {
    int tally, length;
    SYMBOL_HN *past = NULL, *peek;

    /*====================================*/
    /* Get the hash value for the string. */
    /*====================================*/

    if (str == NULL)
      {
       CLIPSSystemError("SYMBOL",1);
       ExitCLIPS(5);
      }

    tally = HashSymbol(str,SYMBOL_HASH_SIZE);
    peek = SymbolTable[tally];

    /*==================================================*/
    /* Search for the string in the list of entries for */
    /* this symbol table location.  If the string is    */
    /* found, then return the address of the string.    */
    /*==================================================*/

    while (peek != NULL)
      {
       if (strcmp(str,peek->contents) == 0)
         { return((VOID *) peek); }
       past = peek;
       peek = peek->next;
      }

    /*==================================================*/
    /* Add the string at the end of the list of entries */
    /* for this symbol table location.  Return the      */
    /* address of the string.                           */
    /*==================================================*/
       
    peek = get_struct(symbolHashNode);
    if (past == NULL) SymbolTable[tally] = peek;
    else past->next = peek;

    length = strlen(str) + 1;
    peek->contents = (char *) gm2(length);
    peek->next = NULL;
    peek->bucket = tally;
    peek->count = 0;
    strcpy(peek->contents,str);

    AddEphemeralHashNode((GENERIC_HN *) peek,&EphemeralSymbolList,
                         sizeof(SYMBOL_HN),AVERAGE_STRING_SIZE);
    peek->depth = CurrentEvaluationDepth;
    return((VOID *) peek);
   }

/****************************************************************/
/* FindSymbol:  Searches for the string in the symbol table and */
/*   returns a pointer to it if found, otherwise returns NULL.  */
/****************************************************************/
globle SYMBOL_HN *FindSymbol(str)
   char *str;
   {
    int tally;
    SYMBOL_HN *peek;

    tally = HashSymbol(str,SYMBOL_HASH_SIZE);
    peek = SymbolTable[tally];

    while (peek != NULL)
      {
       if (strcmp(str,peek->contents) == 0) return(peek);
       peek = peek->next;
      }

    return(NULL);
   }

/*******************************************************************/
/* AddDouble:  Searches for the double in the hash table. If the   */
/*   double is already in the hash table, then the address of the  */
/*   double is returned.  Otherwise, the double is hashed into the */
/*   table and the address of the double is also returned.         */
/*******************************************************************/
globle VOID *AddDouble(number)
   double number;
   {
    int tally;
    FLOAT_HN *past = NULL, *peek;

    /*====================================*/
    /* Get the hash value for the double. */
    /*====================================*/

    tally = HashFloat(number,FLOAT_HASH_SIZE);
    peek = FloatTable[tally];

    /*==================================================*/
    /* Search for the double in the list of entries for */
    /* this hash location.  If the double is found,     */
    /* then return the address of the double.           */
    /*==================================================*/

    while (peek != NULL)
      {
       if (number == peek->contents)
         { return((VOID *) peek); }
       past = peek;
       peek = peek->next;
      }

    /*==================================================*/
    /* Add the double at the end of the list of entries */
    /* for this hash location.  Return the address of   */
    /* the double.                                      */
    /*==================================================*/

    peek = get_struct(floatHashNode);
    if (past == NULL) FloatTable[tally] = peek;
    else past->next = peek;
    
    peek->contents = number;
    peek->next = NULL;
    peek->bucket = tally;
    peek->count = 0;

    AddEphemeralHashNode((GENERIC_HN *) peek,&EphemeralFloatList,
                         sizeof(FLOAT_HN),0);
    peek->depth = CurrentEvaluationDepth;
    return((VOID *) peek);
   }

/****************************************************************/
/* AddLong:  Searches for the long in the hash table. If the    */
/*   long is already in the hash table, then the address of the */
/*   long is returned.  Otherwise, the long is hashed into the  */
/*   table and the address of the long is also returned.        */
/****************************************************************/
globle VOID *AddLong(number)
   long int number;
   {
    int tally;
    INTEGER_HN *past = NULL, *peek;

    /*==================================*/
    /* Get the hash value for the long. */
    /*==================================*/

    tally = HashInteger(number,INTEGER_HASH_SIZE);
    peek = IntegerTable[tally];

    /*================================================*/
    /* Search for the long in the list of entries for */
    /* this hash location.  If the long is found,     */
    /* then return the address of the long.           */
    /*================================================*/

    while (peek != NULL)
      {
       if (number == peek->contents)
         { return((VOID *) peek); }
       past = peek;
       peek = peek->next;
      }

    /*================================================*/
    /* Add the long at the end of the list of entries */
    /* for this hash location.  Return the address of */
    /* the long.                                      */
    /*================================================*/

    peek = get_struct(integerHashNode);
    if (past == NULL) IntegerTable[tally] = peek;
    else past->next = peek;
    
    peek->contents = number;
    peek->next = NULL;
    peek->bucket = tally;
    peek->count = 0;

    AddEphemeralHashNode((GENERIC_HN *) peek,&EphemeralIntegerList,
                         sizeof(INTEGER_HN),0);
    peek->depth = CurrentEvaluationDepth;
    return((VOID *) peek);
   }
   
/****************************************************************/
/* FindLong:   */
/****************************************************************/
globle INTEGER_HN *FindLong(theLong)
   long int theLong;
   {
    int tally;
    INTEGER_HN *peek;

    tally = HashInteger(theLong,INTEGER_HASH_SIZE);
    peek = IntegerTable[tally];

    while (peek != NULL)
      {
       if (peek->contents == theLong) return(peek);
       peek = peek->next;
      }

    return(NULL);
   }

/********************************************************************/
/* AddBitMap:      */
/********************************************************************/
globle VOID *AddBitMap(vTheBitMap,size)
   VOID *vTheBitMap;
   int size;
   {
    char *theBitMap = vTheBitMap;
    int tally, i;
    BITMAP_HN *past = NULL, *peek;

    /*====================================*/
    /* Get the hash value for the string. */
    /*====================================*/

    if (theBitMap == NULL)
      {
       CLIPSSystemError("SYMBOL",2);
       ExitCLIPS(5);
      }

    tally = HashBitMap(theBitMap,BITMAP_HASH_SIZE,size);
    peek = BitMapTable[tally];

    /*==================================================*/
    /* Search for the string in the list of entries for */
    /* this symbol table location.  If the string is    */
    /* found, then return the address of the string.    */
    /*==================================================*/

    while (peek != NULL)
      {
       if (peek->size == size)
         {
          for (i = 0; i < size ; i++)
            { if (peek->contents[i] != theBitMap[i]) break; }
            
          if (i == size) return((VOID *) peek);
         }
         
       past = peek;
       peek = peek->next;
      }

    /*==================================================*/
    /* Add the string at the end of the list of entries */
    /* for this symbol table location.  Return the      */
    /* address of the string.                           */
    /*==================================================*/
       
    peek = get_struct(bitMapHashNode);
    if (past == NULL) BitMapTable[tally] = peek;
    else past->next = peek;

    peek->contents = (char *) gm2(size);
    peek->next = NULL;
    peek->bucket = tally;
    peek->count = 0;
    peek->size = size;
    
    for (i = 0; i < size ; i++) peek->contents[i] = theBitMap[i];

    AddEphemeralHashNode((GENERIC_HN *) peek,&EphemeralBitMapList,
                         sizeof(BITMAP_HN),sizeof(long));
    peek->depth = CurrentEvaluationDepth;
    return((VOID *) peek);
   }  
   
/********************************************************************/
/* InitializeAtomTables: Initializes the SymbolTable, IntegerTable, */
/*   and FloatTable. It also initializes the CLIPSTrueSymbol and    */
/*   CLIPSFalseSymbol.                                              */
/********************************************************************/
globle VOID InitializeAtomTables()
   {
    int i;

    SymbolTable = (SYMBOL_HN **)
                   gm2((int) sizeof (SYMBOL_HN *) * SYMBOL_HASH_SIZE);

    FloatTable = (FLOAT_HN **)
                   gm2((int) sizeof (FLOAT_HN *) * FLOAT_HASH_SIZE);

    IntegerTable = (INTEGER_HN **)
                    gm2((int) sizeof (INTEGER_HN *) * INTEGER_HASH_SIZE);
                    
    BitMapTable = (BITMAP_HN **)
                    gm2((int) sizeof (BITMAP_HN *) * BITMAP_HASH_SIZE);

    for (i = 0; i < SYMBOL_HASH_SIZE; i++) SymbolTable[i] = NULL;
    for (i = 0; i < FLOAT_HASH_SIZE; i++) FloatTable[i] = NULL;
    for (i = 0; i < INTEGER_HASH_SIZE; i++) IntegerTable[i] = NULL;
    for (i = 0; i < BITMAP_HASH_SIZE; i++) BitMapTable[i] = NULL;

    CLIPSTrueSymbol = AddSymbol(TrueSymbol);
    IncrementSymbolCount(CLIPSTrueSymbol);
    CLIPSFalseSymbol = AddSymbol(FalseSymbol);
    IncrementSymbolCount(CLIPSFalseSymbol);
    PositiveInfinity = AddSymbol("+oo");
    IncrementSymbolCount(PositiveInfinity);
    NegativeInfinity = AddSymbol("-oo");
    IncrementSymbolCount(NegativeInfinity);
    Zero = AddLong(0L);
    IncrementIntegerCount(Zero);
   }

/***************************************************/
/* HashSymbol: Computes a hash value for a symbol. */
/***************************************************/
globle int HashSymbol(word,range)
  char *word;
  int range;
  {
   register int k,j,i;
   register int length;
   int tally;
   unsigned long count = 0L,tmpLong;
   char *tmpPtr;

   tmpPtr = (char *) &tmpLong;

   /*===============================================*/
   /* Count the number of characters in the symbol. */
   /*===============================================*/

   for (length = 0; word[length]; length++);

   /*================================================================ */
   /* Add up the first part of the word as unsigned long int values.  */
   /*================================================================ */

   length = length / sizeof(unsigned long);
   for (i = 0 , j = 0 ; i < length; i++)
     {
      for (k = 0 ; k < sizeof(unsigned long) ; k++ , j++)
        tmpPtr[k] = word[j];
      count += tmpLong;
     }

   /*============================================*/
   /* Add the remaining characters to the count. */
   /*============================================*/

   tmpLong = 0L;
   for (word = (char *) &word[j], k = 0; 
        *word; 
        word++, k++) 
     {
      tmpPtr[k] = *word;
      /* count += (unsigned long) *word; */
     }
     
   count += tmpLong;

   /*========================*/
   /* Return the hash value. */
   /*========================*/

   tally = (int) (count % range);
   if (tally < 0) return(-tally);

   return(tally);
  }

/*************************************************/
/* HashFloat: Computes a hash value for a float. */
/*************************************************/
globle int HashFloat(number,range)
  double number;
  int range;
  {
   union
     {
      double fv;
      unsigned long int liv;
     } fis;
   unsigned long count = 0;
   int tally;

   fis.liv = 0;
   fis.fv = number;
   count = fis.liv;

   tally = (int) (count % range);

   if (tally < 0) return(-tally);

   return(tally);
  }

/******************************************************/
/* HashInteger: Computes a hash value for an integer. */
/******************************************************/
globle int HashInteger(number,range)
  long int number;
  int range;
  {
   int tally;

   tally = (int) (number % range);

   if (tally < 0) return(-tally);

   return(tally);
  }
  
/***************************************************/
/* HashBitMap: Computes a hash value for a bitmap. */
/***************************************************/
globle int HashBitMap(word,range,length)
  char *word;
  int range, length;
  {
   register int k,j,i;
   int tally;
   int longLength;
   unsigned long count = 0L,tmpLong;
   char *tmpPtr;

   tmpPtr = (char *) &tmpLong;

   /*================================================================ */
   /* Add up the first part of the word as unsigned long int values.  */
   /*================================================================ */

   longLength = length / sizeof(unsigned long);
   for (i = 0 , j = 0 ; i < longLength; i++)
     {
      for (k = 0 ; k < sizeof(unsigned long) ; k++ , j++)
        tmpPtr[k] = word[j];
      count += tmpLong;
     }

   /*============================================*/
   /* Add the remaining characters to the count. */
   /*============================================*/

   for (; j < length; j++) count += (unsigned long) word[j];

   /*========================*/
   /* Return the hash value. */
   /*========================*/

   tally = (int) (count % range);
   if (tally < 0) return(-tally);

   return(tally);
  }
  
/*****************************************************************************/
/* DecrementSymbolCount: Decrements the count value for a SymbolTable entry. */
/*   Adds the symbol to the EphemeralSymbolList if the count becomes zero.   */
/*****************************************************************************/
globle VOID DecrementSymbolCount(theValue)
  SYMBOL_HN *theValue;
  {
   if (theValue->count < 0)
     {
      CLIPSSystemError("SYMBOL",3);
      ExitCLIPS(5);
     }

   if (theValue->count == 0)
     {
      CLIPSSystemError("SYMBOL",4);
      ExitCLIPS(5);
     }

   theValue->count--;

   if (theValue->count != 0) return;

   if (theValue->markedEphemeral == CLIPS_FALSE) 
     {  
      AddEphemeralHashNode((GENERIC_HN *) theValue,&EphemeralSymbolList,
                           sizeof(SYMBOL_HN),AVERAGE_STRING_SIZE);
     }

   return;
  }

/***************************************************************************/
/* DecrementFloatCount: Decrements the count value for a FloatTable entry. */
/*   Adds the float to the EphemeralFloatList if the count becomes zero.   */
/***************************************************************************/
globle VOID DecrementFloatCount(theValue)
  FLOAT_HN *theValue;
  {
   if (theValue->count <= 0)
     {
      CLIPSSystemError("SYMBOL",5);
      ExitCLIPS(5);
     }

   theValue->count--;

   if (theValue->count != 0) return;

   if (theValue->markedEphemeral == CLIPS_FALSE)
     {  
      AddEphemeralHashNode((GENERIC_HN *) theValue,&EphemeralFloatList,
                           sizeof(FLOAT_HN),0);
     }

   return;
  }

/*******************************************************************************/
/* DecrementIntegerCount: Decrements the count value for a IntegerTable entry. */
/*   Adds the integer to the EphemeralIntegerList if the count becomes zero.   */
/*******************************************************************************/
globle VOID DecrementIntegerCount(theValue)
  INTEGER_HN *theValue;
  {
   if (theValue->count <= 0)
     {
      CLIPSSystemError("SYMBOL",6);
      ExitCLIPS(5);
     }

   theValue->count--;

   if (theValue->count != 0) return;

   if (theValue->markedEphemeral == CLIPS_FALSE) 
     {  
      AddEphemeralHashNode((GENERIC_HN *) theValue,&EphemeralIntegerList,
                           sizeof(INTEGER_HN),0);
     }

   return;
  }
  
/*****************************************************************************/
/* DecrementBitMapCount:   */
/*****************************************************************************/
globle VOID DecrementBitMapCount(theValue)
  BITMAP_HN *theValue;
  {
   if (theValue->count < 0)
     {
      CLIPSSystemError("SYMBOL",7);
      ExitCLIPS(5);
     }

   if (theValue->count == 0)
     {
      CLIPSSystemError("SYMBOL",8);
      ExitCLIPS(5);
     }

   theValue->count--;

   if (theValue->count != 0) return;

   if (theValue->markedEphemeral == CLIPS_FALSE) 
     {  
      AddEphemeralHashNode((GENERIC_HN *) theValue,&EphemeralBitMapList,
                           sizeof(BITMAP_HN),sizeof(long));
     }

   return;
  }  
  
/*************************************************************/
/* RemoveHashNode: Removes a hash node from the SymbolTable, */
/*   FloatTable, IntegerTable, or BitMapTable.               */
/*************************************************************/
static VOID RemoveHashNode(theValue,theTable,size,type)
  GENERIC_HN *theValue, **theTable;
  int size, type;
  {
   GENERIC_HN *previousNode, *currentNode;

   previousNode = NULL;
   currentNode = theTable[theValue->bucket];

   while (currentNode != theValue)
     {
      previousNode = currentNode;
      currentNode = currentNode->next;

      if (currentNode == NULL)
        {
         CLIPSSystemError("SYMBOL",9);
         ExitCLIPS(5);
        }
     }

   if (previousNode == NULL)
     { theTable[theValue->bucket] = theValue->next; }
   else
     { previousNode->next = currentNode->next; }

   if (type == SYMBOL) 
     { 
      rm(((SYMBOL_HN *) theValue)->contents,
         (int) strlen(((SYMBOL_HN *) theValue)->contents) + 1);
     }
   else if (type == BITMAPARRAY)
     { 
      rm(((BITMAP_HN *) theValue)->contents,
         (int) ((BITMAP_HN *) theValue)->size);
     }
     
   rtn_sized_struct(size,theValue);
  }

/*************************************************************/
/* AddEphemeralHashNode: Adds a SymbolTable entry to the list  */
/*   of ephemeral symbols. These locations have a zero count */
/*   indicating that no structure is using the symbol.       */
/*************************************************************/
static VOID AddEphemeralHashNode(theHashNode,theEphemeralList,hashNodeSize,averageContentsSize)
  GENERIC_HN *theHashNode;
  struct ephemeron **theEphemeralList;
  int hashNodeSize, averageContentsSize;
  {
   struct ephemeron *temp;

   if (theHashNode->count != 0)
     {
      CLIPSSystemError("SYMBOL",10);
      ExitCLIPS(5);
     }

   theHashNode->markedEphemeral = CLIPS_TRUE;

   temp = get_struct(ephemeron);
   temp->associatedValue = theHashNode;
   temp->next = *theEphemeralList;
   *theEphemeralList = temp;

   EphemeralItemCount++;
   EphemeralItemSize += sizeof(struct ephemeron) + hashNodeSize +
                        averageContentsSize;
  }

/*************************************************************/
/* RemoveEphemeralAtoms: Causes the removal of all ephemeral */
/*   symbols, integers, and floats, that still have a count  */
/*   value of zero, from their respective storage tables.    */
/*************************************************************/
globle VOID RemoveEphemeralAtoms()
  {
   RemoveEphemeralHashNodes(&EphemeralSymbolList,(GENERIC_HN **) SymbolTable,
                            sizeof(SYMBOL_HN),SYMBOL,AVERAGE_STRING_SIZE);
   RemoveEphemeralHashNodes(&EphemeralFloatList,(GENERIC_HN **) FloatTable,
                            sizeof(FLOAT_HN),FLOAT,0);
   RemoveEphemeralHashNodes(&EphemeralIntegerList,(GENERIC_HN **) IntegerTable,
                            sizeof(INTEGER_HN),INTEGER,0);
   RemoveEphemeralHashNodes(&EphemeralBitMapList,(GENERIC_HN **) BitMapTable,
                            sizeof(BITMAP_HN),BITMAPARRAY,AVERAGE_BITMAP_SIZE);
  }

/**************************************************************/
/* RemoveEphemeralHashNodes: Removes symbols from the ephemeral */
/*   symbol list that have a count of zero and were placed on */
/*   the list at a higher level than the current evaluation   */
/*   depth. Since symbols are ordered in the list in          */
/*   descending order, the removal process can end when a     */
/*   depth is reached less than the current evaluation depth. */
/*   Because ephemeral symbols can be "pulled" up through an  */
/*   evaluation depth, this routine needs to check through    */
/*   both the previous and current evaluation depth.          */
/**************************************************************/
static VOID RemoveEphemeralHashNodes(theEphemeralList,theTable,
                                     hashNodeSize,hashNodeType,averageContentsSize)
  struct ephemeron **theEphemeralList;
  GENERIC_HN **theTable;
  int hashNodeSize, hashNodeType, averageContentsSize;
  {
   struct ephemeron *edPtr, *lastPtr = NULL, *nextPtr;

   edPtr = *theEphemeralList;
   while (edPtr != NULL)
     {
      /*======================================================*/
      /* Check through previous and current evaluation depth  */
      /* because these symbols can be interspersed, otherwise */
      /* symbols are stored in descending evaluation depth.   */
      /*======================================================*/

      nextPtr = edPtr->next;

      /*==================================================*/
      /* Remove any symbols that have a count of zero and */
      /* were added to the ephemeral list at a higher     */
      /* evaluation depth.                                */
      /*==================================================*/

      if ((edPtr->associatedValue->count == 0) &&
          (edPtr->associatedValue->depth > CurrentEvaluationDepth))
        {
         RemoveHashNode(edPtr->associatedValue,theTable,hashNodeSize,hashNodeType);
         rtn_struct(ephemeron,edPtr);
         if (lastPtr == NULL) *theEphemeralList = nextPtr;
         else lastPtr->next = nextPtr;
         EphemeralItemCount--;
         EphemeralItemSize -= sizeof(struct ephemeron) + hashNodeSize +
                              averageContentsSize;
        }

      /*=======================================*/
      /* Remove ephemeral status of any symbol */
      /* with a count greater than zero.       */
      /*=======================================*/

      else if (edPtr->associatedValue->count > 0)
        {
         edPtr->associatedValue->markedEphemeral = CLIPS_FALSE;
         rtn_struct(ephemeron,edPtr);
         if (lastPtr == NULL) *theEphemeralList = nextPtr;
         else lastPtr->next = nextPtr;
         EphemeralItemCount--;
         EphemeralItemSize -= sizeof(struct ephemeron) + hashNodeSize +
                              averageContentsSize;
        }

      /*==================================================*/
      /* Otherwise keep the symbol in the ephemeral list. */
      /*==================================================*/

      else
        { lastPtr = edPtr; }

      edPtr = nextPtr;
     }
  }

/*********************************************************/
/* GetSymbolTable: Returns a pointer to the SymbolTable. */
/*********************************************************/
globle SYMBOL_HN **GetSymbolTable()
  {
   return(SymbolTable);
  }

/******************************************************/
/* SetSymbolTable: Sets the value of the SymbolTable. */
/******************************************************/
globle VOID SetSymbolTable(value)
  SYMBOL_HN **value;
  {
   SymbolTable = value;
  }

/*******************************************************/
/* GetFloatTable: Returns a pointer to the FloatTable. */
/*******************************************************/
globle FLOAT_HN **GetFloatTable()
  {
   return(FloatTable);
  }

/****************************************************/
/* SetFloatTable: Sets the value of the FloatTable. */
/****************************************************/
globle VOID SetFloatTable(value)
  FLOAT_HN **value;
  {
   FloatTable = value;
  }

/***********************************************************/
/* GetIntegerTable: Returns a pointer to the IntegerTable. */
/***********************************************************/
globle INTEGER_HN **GetIntegerTable()
  {
   return(IntegerTable);
  }

/********************************************************/
/* SetIntegerTable: Sets the value of the IntegerTable. */
/********************************************************/
globle VOID SetIntegerTable(value)
  INTEGER_HN **value;
  {
   IntegerTable = value;
  }
  
/*********************************************************/
/* GetBitMapTable: Returns a pointer to the BitMapTable. */
/*********************************************************/
globle BITMAP_HN **GetBitMapTable()
  {
   return(BitMapTable);
  }

/******************************************************/
/* SetBitMapTable: Sets the value of the BitMapTable. */
/******************************************************/
globle VOID SetBitMapTable(value)
  BITMAP_HN **value;
  {
   BitMapTable = value;
  }
  
/***************************************************/
/* RefreshBooleanSymbols: Resets the values of the */
/*    CLIPSTrueSymbol and the CLIPSFalseSymbol.    */
/***************************************************/
globle VOID RefreshBooleanSymbols()
  {
   CLIPSTrueSymbol = (VOID *) FindSymbol(TrueSymbol);
   CLIPSFalseSymbol = (VOID *) FindSymbol(FalseSymbol);
   PositiveInfinity = (VOID *) FindSymbol("+infinity");
   NegativeInfinity = (VOID *) FindSymbol("-infinity");
   Zero = (VOID *) FindLong(0L);
  }

/*****************************************************************/
/* FindSymbolMatches: Finds all symbols in the SymbolTable which */
/*   begin with a specified symbol. This function is used to     */
/*   implement the command completion feature found in some of   */
/*   the CLIPS machine specific interfaces.                      */
/*****************************************************************/
globle struct symbolMatch *FindSymbolMatches(searchString,numberOfMatches)
  char *searchString;
  int *numberOfMatches;
  {
   struct symbolMatch *reply = NULL, *temp;
   struct symbolHashNode *hashPtr = NULL;
   int searchLength;

   searchLength = strlen(searchString);
   *numberOfMatches = 0;

   while ((hashPtr = GetNextSymbolMatch(searchString,searchLength,hashPtr,CLIPS_FALSE)) != NULL)
     {
      *numberOfMatches = *numberOfMatches + 1;
      temp = get_struct(symbolMatch);
      temp->match = hashPtr;
      temp->next = reply;
      reply = temp;
     }

   return(reply);
  }

/*********************************************************/
/* ReturnSymbolMatches: Returns a set of symbol matches. */
/*********************************************************/
globle VOID ReturnSymbolMatches(listOfMatches)
  struct symbolMatch *listOfMatches;
  {
   struct symbolMatch *temp;

   while (listOfMatches != NULL)
     {
      temp = listOfMatches->next;
      rtn_struct(symbolMatch,listOfMatches);
      listOfMatches = temp;
     }
  }
  
/*********************************************************/
/* ClearBitString:  */
/*********************************************************/
globle VOID ClearBitString(vTheBitMap,length)
  VOID *vTheBitMap;
  int length;
  {
   char *theBitMap = vTheBitMap;
   int i;
   
   for (i = 0; i < length; i++) theBitMap[i] = '\0';
  }
  
/*****************************************************************/
/* GetNextSymbolMatch: Finds the next symbol in the SymbolTable  */
/*   which begins with a specified symbol. This function is used */
/*   to implement the command completion feature found in some   */
/*   of the CLIPS machine specific interfaces.                   */
/*****************************************************************/
globle SYMBOL_HN *GetNextSymbolMatch(searchString,searchLength,prevSymbol,anywhere)
  char *searchString;
  int searchLength;
  SYMBOL_HN *prevSymbol;
  int anywhere;
  {
   register int i;
   SYMBOL_HN *hashPtr;
   int flag = CLIPS_TRUE;

   if (prevSymbol == NULL)
     {
      i = 0;
      hashPtr = SymbolTable[0];
     }
   else
     {
      i = prevSymbol->bucket;
      hashPtr = prevSymbol->next;
     }

   while (flag)
     {
      while (hashPtr != NULL)
        {
         if ((hashPtr->contents[0] != '(') && (! hashPtr->markedEphemeral))
           {
#if ANSI_COMPILER
            if (! anywhere)
#endif
              {
               if (strncmp(searchString,hashPtr->contents,searchLength) == 0)
                 { return(hashPtr); }
              }
#if ANSI_COMPILER
            else
              {
               if (strstr(hashPtr->contents,searchString) != NULL)
                 { return(hashPtr); }
              }
#endif
           }
         hashPtr = hashPtr->next;
        }
      if (++i >= SYMBOL_HASH_SIZE) flag = CLIPS_FALSE;
      else hashPtr = SymbolTable[i];
     }
     
   return(NULL);
  }

#if BLOAD_AND_BSAVE || CONSTRUCT_COMPILER || BSAVE_INSTANCES

/****************************************************************************/
/* SetAtomicValueIndices:                                                   */
/****************************************************************************/
globle VOID SetAtomicValueIndices(setAll)
  int setAll;
  {
   unsigned int count = 0;
   int i;
   SYMBOL_HN *symbolPtr, **symbolArray;
   FLOAT_HN *floatPtr, **floatArray;
   INTEGER_HN *integerPtr, **integerArray;
   BITMAP_HN *bitMapPtr, **bitMapArray;

   symbolArray = GetSymbolTable();

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      symbolPtr = symbolArray[i];
      while (symbolPtr != NULL)
        {
         if ((symbolPtr->neededSymbol == CLIPS_TRUE) || setAll)
           { symbolPtr->bucket = count++; }
         symbolPtr = symbolPtr->next;
        }
     }

   count = 0;
   floatArray = GetFloatTable();

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
     {
      floatPtr = floatArray[i];
      while (floatPtr != NULL)
        {
         if ((floatPtr->neededFloat == CLIPS_TRUE) || setAll)
           { floatPtr->bucket = count++; }
         floatPtr = floatPtr->next;
        }
     }

   count = 0;
   integerArray = GetIntegerTable();

   for (i = 0; i < INTEGER_HASH_SIZE; i++)
     {
      integerPtr = integerArray[i];
      while (integerPtr != NULL)
        {
         if ((integerPtr->neededInteger == CLIPS_TRUE) || setAll)
           { integerPtr->bucket = count++; }
         integerPtr = integerPtr->next;
        }
     }

   count = 0;
   bitMapArray = GetBitMapTable();

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      bitMapPtr = bitMapArray[i];
      while (bitMapPtr != NULL)
        {
         if ((bitMapPtr->neededBitMap == CLIPS_TRUE) || setAll)
           { bitMapPtr->bucket = count++; }
         bitMapPtr = bitMapPtr->next;
        }
     }
  }

/****************************************************************************/
/* RestoreAtomicValueBuckets:                                          */
/****************************************************************************/
globle VOID RestoreAtomicValueBuckets()
  {
   int i;
   SYMBOL_HN *symbolPtr, **symbolArray;
   FLOAT_HN *floatPtr, **floatArray;
   INTEGER_HN *integerPtr, **integerArray;
   BITMAP_HN *bitMapPtr, **bitMapArray;

   symbolArray = GetSymbolTable();

   for (i = 0; i < SYMBOL_HASH_SIZE; i++)
     {
      symbolPtr = symbolArray[i];
      while (symbolPtr != NULL)
        {
         symbolPtr->bucket = i;
         symbolPtr = symbolPtr->next;
        }
     }

   floatArray = GetFloatTable();

   for (i = 0; i < FLOAT_HASH_SIZE; i++)
     {
      floatPtr = floatArray[i];
      while (floatPtr != NULL)
        {
         floatPtr->bucket = i;
         floatPtr = floatPtr->next;
        }
     }

   integerArray = GetIntegerTable();

   for (i = 0; i < INTEGER_HASH_SIZE; i++)
     {
      integerPtr = integerArray[i];
      while (integerPtr != NULL)
        {
         integerPtr->bucket = i;
         integerPtr = integerPtr->next;
        }
     }   
     
   bitMapArray = GetBitMapTable();

   for (i = 0; i < BITMAP_HASH_SIZE; i++)
     {
      bitMapPtr = bitMapArray[i];
      while (bitMapPtr != NULL)
        {
         bitMapPtr->bucket = i;
         bitMapPtr = bitMapPtr->next;
        }
     }
  }

#endif


