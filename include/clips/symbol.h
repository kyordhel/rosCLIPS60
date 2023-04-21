   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 SYMBOL HEADER FILE                  */
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

#ifndef _H_symbol
#define _H_symbol

struct symbolHashNode;
struct floatHashNode;
struct integerHashNode;
struct bitMapHashNode;
struct genericHashNode;
struct symbolMatch;

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _SYMBOL_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#define SYMBOL_HASH_SIZE        1013
#define FLOAT_HASH_SIZE          503 
#define INTEGER_HASH_SIZE        167 
#define BITMAP_HASH_SIZE        167

/************************************************************/
/* symbolHashNode STRUCTURE:                                */
/************************************************************/
struct symbolHashNode
  {
   struct symbolHashNode *next;
   int count;
   int depth;
   unsigned int markedEphemeral : 1;
   unsigned int neededSymbol : 1;
   unsigned int bucket : 14;
   char *contents;
  };
  
/************************************************************/
/* floatHashNode STRUCTURE:                                  */
/************************************************************/
struct floatHashNode
  {
   struct floatHashNode *next;
   int count;
   int depth;
   unsigned int markedEphemeral : 1;
   unsigned int neededFloat : 1;
   unsigned int bucket : 14;
   double contents;
  };

/************************************************************/
/* integerHashNode STRUCTURE:                               */
/************************************************************/
struct integerHashNode
  {
   struct integerHashNode *next;
   int count;
   int depth;
   unsigned int markedEphemeral : 1;
   unsigned int neededInteger : 1;
   unsigned int bucket : 14;
   long int contents;
  };
  
/************************************************************/
/* bitMapHashNode STRUCTURE:                                */
/************************************************************/
struct bitMapHashNode
  {
   struct bitMapHashNode *next;
   int count;
   int depth;
   unsigned int markedEphemeral : 1;
   unsigned int neededBitMap : 1;
   unsigned int bucket : 14;
   char *contents;
   unsigned short size;
  };
  
/************************************************************/
/* genericHashNode STRUCTURE:                               */
/************************************************************/
struct genericHashNode
  {
   struct genericHashNode *next;
   int count;
   int depth;
   unsigned int markedEphemeral : 1;
   unsigned int needed : 1;
   unsigned int bucket : 14;
  };
  
/************************************************************/
/* symbolMatch STRUCTURE:                               */
/************************************************************/
struct symbolMatch
  {
   struct symbolHashNode *match;
   struct symbolMatch *next;
  };
  
typedef struct symbolHashNode SYMBOL_HN;
typedef struct floatHashNode FLOAT_HN;
typedef struct integerHashNode INTEGER_HN;
typedef struct bitMapHashNode BITMAP_HN;
typedef struct genericHashNode GENERIC_HN;

#define ValueToString(target) (((struct symbolHashNode *) (target))->contents)
#define ValueToDouble(target) (((struct floatHashNode *) (target))->contents)
#define ValueToLong(target) (((struct integerHashNode *) (target))->contents)
#define ValueToInteger(target) ((int) (((struct integerHashNode *) (target))->contents))
#define ValueToBitMap(target) ((VOID *) ((struct bitMapHashNode *) (target))->contents)

#define IncrementSymbolCount(theValue) (((SYMBOL_HN *) theValue)->count++)
#define IncrementFloatCount(theValue) (((FLOAT_HN *) theValue)->count++)
#define IncrementIntegerCount(theValue) (((INTEGER_HN *) theValue)->count++)
#define IncrementBitMapCount(theValue) (((BITMAP_HN *) theValue)->count++)

#if ANSI_COMPILER
   LOCALE VOID                          *AddSymbol(char *);
   LOCALE SYMBOL_HN                     *FindSymbol(char *);
   LOCALE VOID                          *AddDouble(double);
   LOCALE VOID                          *AddLong(long int);
   LOCALE VOID                          *AddBitMap(VOID *,int);
   LOCALE INTEGER_HN                    *FindLong(long int);
   LOCALE VOID                           InitializeAtomTables(void);
   LOCALE int                            HashSymbol(char *,int);
   LOCALE int                            HashFloat(double,int);
   LOCALE int                            HashInteger(long int,int);
   LOCALE int                            HashBitMap(char *,int,int);
   LOCALE VOID                           DecrementSymbolCount(struct symbolHashNode *);
   LOCALE VOID                           DecrementFloatCount(struct floatHashNode *);
   LOCALE VOID                           DecrementIntegerCount(struct integerHashNode *);
   LOCALE VOID                           DecrementBitMapCount(struct bitMapHashNode *);
   LOCALE VOID                           RemoveEphemeralAtoms(void); 
   LOCALE struct symbolHashNode        **GetSymbolTable(void);
   LOCALE VOID                           SetSymbolTable(struct symbolHashNode **);
   LOCALE struct floatHashNode          **GetFloatTable(void);
   LOCALE VOID                           SetFloatTable(struct floatHashNode **);
   LOCALE struct integerHashNode       **GetIntegerTable(void);
   LOCALE VOID                           SetIntegerTable(struct integerHashNode **);
   LOCALE struct bitMapHashNode        **GetBitMapTable(void);
   LOCALE VOID                           SetBitMapTable(struct bitMapHashNode **);
   LOCALE VOID                           RefreshBooleanSymbols(void);
   LOCALE struct symbolMatch            *FindSymbolMatches(char *,int *);
   LOCALE VOID                           ReturnSymbolMatches(struct symbolMatch *);
   LOCALE SYMBOL_HN                     *GetNextSymbolMatch(char *,int,SYMBOL_HN *,int);
   LOCALE VOID                           ClearBitString(VOID *,int);
   LOCALE VOID                           SetAtomicValueIndices(int);
   LOCALE VOID                           RestoreAtomicValueBuckets(void);
#else
   LOCALE VOID                          *AddSymbol();
   LOCALE SYMBOL_HN                     *FindSymbol();
   LOCALE VOID                          *AddDouble();
   LOCALE VOID                          *AddLong();
   LOCALE VOID                          *AddBitMap();
   LOCALE INTEGER_HN                    *FindLong();
   LOCALE VOID                           InitializeAtomTables();
   LOCALE int                            HashSymbol();
   LOCALE int                            HashFloat();
   LOCALE int                            HashInteger();
   LOCALE int                            HashBitMap();
   LOCALE VOID                           DecrementSymbolCount();
   LOCALE VOID                           DecrementFloatCount();
   LOCALE VOID                           DecrementIntegerCount();
   LOCALE VOID                           DecrementBitMapCount();
   LOCALE VOID                           RemoveEphemeralAtoms(); 
   LOCALE struct symbolHashNode        **GetSymbolTable();
   LOCALE VOID                           SetSymbolTable();
   LOCALE struct floatHashNode          **GetFloatTable();
   LOCALE VOID                           SetFloatTable();
   LOCALE struct integerHashNode       **GetIntegerTable();
   LOCALE VOID                           SetIntegerTable();
   LOCALE struct bitMapHashNode        **GetBitMapTable();
   LOCALE VOID                           SetBitMapTable();
   LOCALE VOID                           RefreshBooleanSymbols();
   LOCALE struct symbolMatch            *FindSymbolMatches();
   LOCALE VOID                           ReturnSymbolMatches();
   LOCALE SYMBOL_HN                     *GetNextSymbolMatch();
   LOCALE VOID                           ClearBitString();
   LOCALE VOID                           SetAtomicValueIndices();
   LOCALE VOID                           RestoreAtomicValueBuckets();
#endif


#ifndef _SYMBOL_SOURCE
   extern VOID                   *CLIPSTrueSymbol;
   extern VOID                   *CLIPSFalseSymbol;
   extern VOID                   *NegativeInfinity;
   extern VOID                   *PositiveInfinity;
   extern VOID                   *Zero;
#endif

#endif



