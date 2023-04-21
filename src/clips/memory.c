   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    MEMORY MODULE                    */
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

#define _MEMORY_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"
#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "utility.h"

#if ANSI_COMPILER
#include <stdlib.h>
#define CLIPS_STD_SIZE size_t
#else
#if (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB)
extern char *malloc();
#endif
#define CLIPS_STD_SIZE int
#endif

#if IBM_TBC
#include <alloc.h>
#endif
#if IBM_MSC || IBM_ICB
#include <malloc.h>
#endif
#if IBM_ZTC
#include <dos.h>
#endif

#define STRICT_ALIGN_SIZE sizeof(double)

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if BLOCK_MEMORY

#if ANSI_COMPILER
   static int                     InitializeBlockMemory(unsigned int);
   static int                     AllocateBlock(struct blockInfo *,unsigned int);
   static VOID                    AllocateChunk(struct blockInfo *,struct chunkInfo *,unsigned int);
#else
   static int                     InitializeBlockMemory();
   static int                     AllocateBlock();
   static VOID                    AllocateChunk();
#endif

#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long int                MemoryAmount = 0;
   static long int                MemoryCalls = 0;
   static BOOLEAN                 ConserveMemory = CLIPS_FALSE;
#if ANSI_COMPILER
   static int                   (*OutOfMemoryFunction)(unsigned long)
                                       = DefaultOutOfMemoryFunction;
#else
   static int                   (*OutOfMemoryFunction)()
                                       = DefaultOutOfMemoryFunction;
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct memoryPtr       *TempMemoryPtr;
   globle struct memoryPtr      **MemoryTable;
   globle unsigned int            TempSize;
   globle unsigned long           TempSize2;

/********************************************/
/* InitializeMemory: Sets up memory tables. */
/********************************************/
globle VOID InitializeMemory()
  {
   int i;
   
   MemoryTable = (struct memoryPtr **) 
                 malloc((CLIPS_STD_SIZE) (sizeof(struct memoryPtr *) * MEM_TABLE_SIZE));

   if (MemoryTable == NULL)
     {
      PrintErrorID("MEMORY",1,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Out of memory.\n");
      ExitCLIPS(1);
     }

   for (i = 0; i < MEM_TABLE_SIZE; i++) MemoryTable[i] = NULL; 
  }

/***************************************************/
/* genalloc: A generic memory allocation function. */
/***************************************************/
globle VOID *genalloc(size)
  unsigned int size;
  {
   char *memPtr;

#if   BLOCK_MEMORY
   memPtr = RequestChunk(size);
   if (memPtr == NULL)
     {
      ReleaseMem((long) ((size * 5 > 4096) ? size * 5 : 4096),CLIPS_FALSE);
      memPtr = RequestChunk(size);
      if (memPtr == NULL)
        {
         PeriodicCleanup(CLIPS_FALSE,CLIPS_FALSE);
         ReleaseMem(-1L,CLIPS_TRUE);
         memPtr = RequestChunk(size);
         while (memPtr == NULL)
           {
            if ((*OutOfMemoryFunction)((unsigned long) size))
              return(NULL);
            memPtr = RequestChunk(size);
           }
        }
     }
#else
   memPtr = malloc((CLIPS_STD_SIZE) size);
   if (memPtr == NULL)
     {
      ReleaseMem((long) ((size * 5 > 4096) ? size * 5 : 4096),CLIPS_FALSE);
      memPtr = malloc((CLIPS_STD_SIZE) size);
      if (memPtr == NULL)
        {
         PeriodicCleanup(CLIPS_FALSE,CLIPS_FALSE);
         ReleaseMem(-1L,CLIPS_TRUE);
         memPtr = malloc((CLIPS_STD_SIZE) size);
         while (memPtr == NULL)
           {
            if ((*OutOfMemoryFunction)((unsigned long) size))
              return(NULL);
            memPtr = malloc((CLIPS_STD_SIZE) size);
           }
        }
     }
#endif

   MemoryAmount += size;
   MemoryCalls++;

   return((VOID *) memPtr);
  }

/****************************************************/
/* DefaultOutOfMemoryFunction: Function called when */
/*   CLIPS runs out of memory.                      */
/****************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle int DefaultOutOfMemoryFunction(size)
  unsigned long size;
  {
#if MAC_MPW
#pragma unused(size)
#endif
   PrintErrorID("MEMORY",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Out of memory.\n");
   ExitCLIPS(1);
   return(CLIPS_TRUE);
  }

/*********************************************************/
/* SetOutOfMemoryFunction: Allows the function which is  */
/*   called when CLIPS runs out of memory to be changed. */
/*********************************************************/
#if ANSI_COMPILER
globle int (*SetOutOfMemoryFunction(functionPtr))(unsigned long)
  int (*functionPtr)(unsigned long);
  {
   int (*tmpPtr)(unsigned long);
#else
globle int (*SetOutOfMemoryFunction(functionPtr))()
  int (*functionPtr)();
  {
   int (*tmpPtr)();
#endif

   tmpPtr = OutOfMemoryFunction;
   OutOfMemoryFunction = functionPtr;
   return(tmpPtr);
  }

/****************************************************/
/* genfree: A generic memory deallocation function. */
/****************************************************/
globle int genfree(waste,size)
  VOID *waste;
  unsigned size;
  {

#if BLOCK_MEMORY
   if (ReturnChunk(waste,size) == -1)
     {
      PrintErrorID("MEMORY",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Release error in genfree.\n");
      return(-1);
     }
#else
   free(waste);
#endif

   MemoryAmount -= size;
   MemoryCalls--;

   return(0);
  }

/******************************************************/
/* genrealloc: Simple (i.e. dumb) version of realloc. */
/*    Should be reimplemented later.                  */
/******************************************************/
globle VOID *genrealloc(oldaddr,oldsz,newsz)
  VOID *oldaddr;
  unsigned oldsz, newsz;
  {
   char *newaddr;
   int i, limit;

   newaddr = ((newsz != 0) ? (char *) gm2((int) newsz) : NULL);

   if (oldaddr != NULL)
     {
      limit = (oldsz < newsz) ? oldsz : newsz;
      for (i = 0 ; i < limit ; i++)
        { newaddr[i] = ((char *) oldaddr)[i]; }
      rm((VOID *) oldaddr,(int) oldsz);
     }

   return((VOID *) newaddr);
  }

#if MAC_TC || MAC_MPW || IBM_ICB
#define SpecialMalloc(sz) malloc((CLIPS_STD_SIZE) sz)
#define SpecialFree(ptr) free(ptr)
#endif
#if IBM_TBC
#define SpecialMalloc(sz) farmalloc(sz)
#define SpecialFree(ptr) farfree(ptr)
#endif
#if IBM_ZTC
#ifdef __SMALL__
#define SpecialMalloc(sz) malloc((CLIPS_STD_SIZE) sz)
#define SpecialFree(ptr) free(ptr)
#else
#define SpecialMalloc(sz) farmalloc(sz)
#define SpecialFree(ptr) farfree(ptr)
#endif
#endif
#if IBM_MSC
#define SpecialMalloc(sz) halloc(sz,1)
#define SpecialFree(ptr) hfree(ptr)
#endif

/************************************************/
/* genlongalloc: Allocates blocks of memory for */
/*   sizes expressed using long integers.       */
/************************************************/
globle VOID *genlongalloc(size)
  unsigned long size;
  {
#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) && (! IBM_ZTC)
   unsigned int test;
#else
   VOID *memPtr;
#endif

   if (sizeof(int) == sizeof(long))
     { return(genalloc((unsigned) size)); }

#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) && (! IBM_ZTC)
   test = (unsigned int) size;
   if (test != size)
     {
      PrintErrorID("MEMORY",3,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Unable to allocate memory block > 32K.\n");
      ExitCLIPS(1);
     }
   return((VOID *) genalloc((unsigned) test));
#else

   memPtr = (VOID *) SpecialMalloc(size);
   if (memPtr == NULL)
     {
      ReleaseMem((long) ((size * 5 > 4096) ? size * 5 : 4096),CLIPS_FALSE);
      memPtr = (VOID *) SpecialMalloc(size);
      if (memPtr == NULL)
        {
         PeriodicCleanup(CLIPS_FALSE,CLIPS_FALSE);
         ReleaseMem(-1L,CLIPS_TRUE);
         memPtr = (VOID *) SpecialMalloc(size);
         while (memPtr == NULL)
           {
            if ((*OutOfMemoryFunction)(size))
              return(NULL);
            memPtr = (VOID *) SpecialMalloc(size);
           }
        }
     }
   MemoryAmount += size;
   MemoryCalls++;
   return(memPtr);
#endif
  }

/*********************************************/
/* genlongfree: Returns blocks of memory for */
/*   sizes expressed using long integers.    */
/*********************************************/
globle int genlongfree(ptr,size)
  VOID *ptr;
  unsigned long size;
  {
#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) && (! IBM_ZTC)
   unsigned int test;
#endif

   if (sizeof(unsigned int) == sizeof(unsigned long))
     { return(genfree((VOID *) ptr,(unsigned) size)); }

#if MAC_TC || MAC_MPW || IBM_ICB
   MemoryAmount -= size;
   MemoryCalls--;
   SpecialFree(ptr);
   return(0);
#endif

#if IBM_TBC || IBM_ZTC
   MemoryAmount -= size;
   MemoryCalls--;
   SpecialFree(ptr);
   return(0);
#endif

#if IBM_MSC
   MemoryAmount -= size;
   MemoryCalls--;
   return(SpecialFree(ptr));
#endif
 
#if (! MAC_TC) && (! MAC_MPW) && (! IBM_TBC) && (! IBM_MSC) && (! IBM_ICB) && (! IBM_ZTC)
   test = (unsigned int) size;
   if (test != size) return(-1);

   return(genfree((VOID *) ptr,(unsigned) test));
#endif
  }

/*********************************/
/* MemUsed: C access routine for */
/*   the mem-requests command.   */
/*********************************/
globle long int MemUsed()
  {
   return(MemoryAmount);
  }

/*************************************/
/* MemRequests: C access routine for */
/*   the mem-requests command.       */
/*************************************/
globle long int MemRequests()
  {
   return(MemoryCalls);
  }

/******************************************************************************/
/* UpdateMemoryUsed: Allows the amount of memory used by CLIPS to be updated. */
/******************************************************************************/
globle long int UpdateMemoryUsed(value)
  long int value;
  {
   MemoryAmount += value;
   return(MemoryAmount);
  }

/**********************************************/
/* UpdateMemoryRequests: Allows the number of */
/*   memory requests to CLIPS to be updated.  */
/**********************************************/
globle long int UpdateMemoryRequests(value)
  long int value;
  {
   MemoryCalls += value;
   return(MemoryCalls);
  }

/************************************/
/* ReleaseMem: C access routine for */
/*   the release-mem command.       */
/************************************/
globle long int ReleaseMem(maximum,printMessage)
  long int maximum;
  int printMessage;
  {
   struct memoryPtr *tmpPtr, *memPtr;
   int i;
   long int amount = 0;

   if (printMessage == CLIPS_TRUE)
     { PrintCLIPS(WDIALOG,"\n*** DEALLOCATING MEMORY ***\n"); }

   for (i = (MEM_TABLE_SIZE - 1) ; i >= sizeof(char *) ; i--)
     {
      memPtr = MemoryTable[i];
      while (memPtr != NULL)
        {
         tmpPtr = memPtr->next;
         genfree((VOID *) memPtr,(unsigned) i);
         memPtr = tmpPtr;
         amount += i;
        }
      MemoryTable[i] = NULL;
      if ((amount > maximum) && (maximum > 0))
        {
         if (printMessage == CLIPS_TRUE)
           { PrintCLIPS(WDIALOG,"*** MEMORY  DEALLOCATED ***\n"); }
         return(amount);
        }
     }

   if (printMessage == CLIPS_TRUE)
     { PrintCLIPS(WDIALOG,"*** MEMORY  DEALLOCATED ***\n"); }

   return(amount);
  }

/*****************************************************/
/* gm1: Allocates memory and sets all bytes to zero. */
/*****************************************************/
globle VOID *gm1(size)
  int size;
  {
   struct memoryPtr *memPtr;
   char *tmpPtr;
   int i;

   if (size < sizeof(char *)) size = sizeof(char *);

   if (size >= MEM_TABLE_SIZE) return(genalloc((unsigned) size));

   memPtr = (struct memoryPtr *) MemoryTable[size];
   if (memPtr == NULL)
     {
      tmpPtr = (char *) genalloc((unsigned) size);
      for (i = 0 ; i < size ; i++)
        { tmpPtr[i] = '\0'; }
      return((VOID *) tmpPtr);
     }

   MemoryTable[size] = memPtr->next;

   tmpPtr = (char *) memPtr;
   for (i = 0 ; i < size ; i++)
     { tmpPtr[i] = '\0'; }

   return ((VOID *) tmpPtr);
  }

/*****************************************************/
/* gm2: Allocates memory and does not initialize it. */
/*****************************************************/
globle VOID *gm2(size)
  int size;
  {
   struct memoryPtr *memPtr;

   if (size < sizeof(char *)) size = sizeof(char *);

   if (size >= MEM_TABLE_SIZE) return(genalloc((unsigned) size));

   memPtr = (struct memoryPtr *) MemoryTable[size];
   if (memPtr == NULL)
     {
      return(genalloc((unsigned) size));
     }

   MemoryTable[size] = memPtr->next;

   return ((VOID *) memPtr);
  }

/*****************************************************/
/* gm3: Allocates memory and does not initialize it. */
/*****************************************************/
globle VOID *gm3(size)
  long size;
  {
   struct memoryPtr *memPtr;

   if (size < sizeof(char *)) size = sizeof(char *);

   if (size >= MEM_TABLE_SIZE) return(genlongalloc((unsigned long) size));

   memPtr = (struct memoryPtr *) MemoryTable[(int) size];
   if (memPtr == NULL)
     {
      return(genalloc((unsigned int) size));
     }

   MemoryTable[(int) size] = memPtr->next;

   return ((VOID *) memPtr);
  }

/******************************************************************************/
/* rm: Returns a block of memory to the CLIPS maintained pool of free memory. */
/******************************************************************************/
globle int rm(str,size)
  VOID *str;
  int size;
  {
   struct memoryPtr *memPtr;

   if (size == 0)
     {
      CLIPSSystemError("MEMORY",1);
      ExitCLIPS(3);
     }

   if (size < sizeof(char *)) size = sizeof(char *);

   if (size >= MEM_TABLE_SIZE) return(genfree((VOID *) str,(unsigned) size));

   memPtr = (struct memoryPtr *) str;
   memPtr->next = MemoryTable[size];
   MemoryTable[size] = memPtr;
   return(1);
  }

/******************************************************************/
/* rm3: Returns a block of memory to the CLIPS maintained pool of */
/*   free memory that's size is indicated with a long integer.    */
/******************************************************************/
globle int rm3(str,size)
  VOID *str;
  long size;
  {
   struct memoryPtr *memPtr;

   if (size == 0)
     {
      CLIPSSystemError("MEMORY",1);
      ExitCLIPS(3);
     }

   if (size < sizeof(char *)) size = sizeof(char *);

   if (size >= MEM_TABLE_SIZE) return(genlongfree((VOID *) str,(unsigned long) size));

   memPtr = (struct memoryPtr *) str;
   memPtr->next = MemoryTable[(int) size];
   MemoryTable[(int) size] = memPtr;
   return(1);
  }

/*********************************************************/
/* PoolSize: Returns number of bytes in CLIPS free pool. */
/*********************************************************/
globle unsigned long PoolSize()
  {
   register int i;
   struct memoryPtr *memPtr;
   unsigned long cnt = 0;

   for (i = sizeof(char *) ; i < MEM_TABLE_SIZE ; i++)
     {
      memPtr = MemoryTable[i];
      while (memPtr != NULL)
        {
         cnt += (unsigned long) i;
         memPtr = memPtr->next;
        }
     }
   return(cnt);
  }

/***************************************************************/
/* ActualPoolSize : Returns number of bytes DOS requires to    */
/*   store the CLIPS free pool.  This routine is functionally  */
/*   equivalent to pool_size on anything other than the IBM-PC */
/***************************************************************/
globle unsigned long ActualPoolSize()
  {
#if IBM_TBC
   register int i;
   struct memoryPtr *memPtr;
   unsigned long cnt = 0;

   for (i = sizeof(char *) ; i < MEM_TABLE_SIZE ; i++)
     {
      memPtr = MemoryTable[i];
      while (memPtr != NULL)
        {
         /*==============================================================*/
         /* For a block of size n, the Turbo-C Library routines require  */
         /* a header of size 8 bytes and further require that all memory */
         /* allotments be paragraph (16-bytes) aligned.                  */
         /*==============================================================*/

         cnt += (((unsigned long) i) + 19L) & 0xfffffff0L;
         memPtr = memPtr->next;
        }
     }
   return(cnt);
#else
   return(PoolSize());
#endif
  }

/********************************************/
/* SetConserveMemory: Allows the setting of */
/*    the memory conservation flag.         */
/********************************************/
globle BOOLEAN SetConserveMemory(value)
  BOOLEAN value;
  {
   int ov;

   ov = ConserveMemory;
   ConserveMemory = value;
   return(ov);
  }

/*******************************************/
/* GetConserveMemory: Returns the value of */
/*    the memory conservation flag.        */
/*******************************************/
globle BOOLEAN GetConserveMemory()
  {
   return(ConserveMemory);
  }

#if ! ANSI_COMPILER

/**************************/
/* genmemcpy:             */
/**************************/
globle VOID genmemcpy(dst,src,size)
  char *dst,*src;
  unsigned long size;
  {
   unsigned long i;
   
   for (i = 0L ; i < size ; i++)
     dst[i] = src[i];
  }
  
#endif


/**************************/
/* BLOCK MEMORY FUNCTIONS */
/**************************/

#if BLOCK_MEMORY

static struct blockInfo  *TopMemoryBlock;
static int                BlockInfoSize;
static int                ChunkInfoSize;
static int                BlockMemoryInitialized = CLIPS_FALSE;

/***************************************************/
/* InitializeBlockMemory: Initializes block memory */
/*   management and allocates the first block.     */
/***************************************************/
static int InitializeBlockMemory(requestSize)
  unsigned int requestSize;
  {
   struct chunkInfo *chunkPtr;
   unsigned int initialBlockSize, usableBlockSize;

   if (sizeof(char) != 1)
     {
      fprintf(stdout, "Size of character data is not 1\n");
      fprintf(stdout, "Memory allocation functions may not work\n");
      return(0);
     }

   ChunkInfoSize = sizeof(struct chunkInfo);
   ChunkInfoSize = (((ChunkInfoSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;

   BlockInfoSize = sizeof(struct blockInfo);
   BlockInfoSize = (((BlockInfoSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;

   initialBlockSize = (INITBLOCKSIZE > requestSize ? INITBLOCKSIZE : requestSize);
   initialBlockSize += ChunkInfoSize * 2 + BlockInfoSize;
   initialBlockSize = (((initialBlockSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;

   usableBlockSize = initialBlockSize - (2 * ChunkInfoSize) - BlockInfoSize;

   /* make sure we get a buffer big enough to be usable */
   if ((requestSize < INITBLOCKSIZE) &&
       (usableBlockSize <= requestSize + ChunkInfoSize))
     {
      initialBlockSize = requestSize + ChunkInfoSize * 2 + BlockInfoSize;
      initialBlockSize = (((initialBlockSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;
      usableBlockSize = initialBlockSize - (2 * ChunkInfoSize) - BlockInfoSize;
     }

   TopMemoryBlock = (struct blockInfo *) malloc((CLIPS_STD_SIZE) initialBlockSize);

   if (TopMemoryBlock == NULL)
     {
      fprintf(stdout, "Unable to allocate initial memory pool\n");
      return(0);
     }

   TopMemoryBlock->nextBlock = NULL;
   TopMemoryBlock->prevBlock = NULL;
   TopMemoryBlock->nextFree = (struct chunkInfo *) (((char *) TopMemoryBlock) + BlockInfoSize);
   TopMemoryBlock->size = usableBlockSize;

   chunkPtr = (struct chunkInfo *) (((char *) TopMemoryBlock) + BlockInfoSize + ChunkInfoSize + usableBlockSize);
   chunkPtr->nextFree = NULL;
   chunkPtr->lastFree = NULL;
   chunkPtr->prevChunk = TopMemoryBlock->nextFree;
   chunkPtr->size = 0;

   TopMemoryBlock->nextFree->nextFree = NULL;
   TopMemoryBlock->nextFree->lastFree = NULL;
   TopMemoryBlock->nextFree->prevChunk = NULL;
   TopMemoryBlock->nextFree->size = usableBlockSize;

   BlockMemoryInitialized = CLIPS_TRUE;
   return(1);
  }

/***************************************************************************/
/* AllocateBlock: Adds a new block of memory to the list of memory blocks. */
/***************************************************************************/
static int AllocateBlock(blockPtr,requestSize)
  struct blockInfo *blockPtr;
  unsigned int requestSize;
  {
   unsigned int blockSize, usableBlockSize;
   struct blockInfo *newBlock;
   struct chunkInfo *newTopChunk;

   blockSize = (BLOCKSIZE > requestSize ? BLOCKSIZE : requestSize);
   blockSize += BlockInfoSize + ChunkInfoSize * 2;
   blockSize = (((blockSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;

   usableBlockSize = blockSize - BlockInfoSize - (2 * ChunkInfoSize);

   /* make sure we get a buffer big enough to be usable */
   if ((requestSize < BLOCKSIZE) &&
       (usableBlockSize <= requestSize + ChunkInfoSize))
     {
      blockSize = requestSize + ChunkInfoSize * 2 + BlockInfoSize;
      blockSize = (((blockSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;
      usableBlockSize = blockSize - (2 * ChunkInfoSize) - BlockInfoSize;
     }

   newBlock = (struct blockInfo *) malloc((CLIPS_STD_SIZE) blockSize);

   if (newBlock == NULL) return(0);

   newBlock->nextBlock = NULL;
   newBlock->prevBlock = blockPtr;
   newBlock->nextFree = (struct chunkInfo *) (((char *) newBlock) + BlockInfoSize);
   newBlock->size = usableBlockSize;
   blockPtr->nextBlock = newBlock;

   newTopChunk = (struct chunkInfo *) (((char *) newBlock) + BlockInfoSize + ChunkInfoSize + usableBlockSize);
   newTopChunk->nextFree = NULL;
   newTopChunk->lastFree = NULL;
   newTopChunk->size = 0;
   newTopChunk->prevChunk = newBlock->nextFree;

   newBlock->nextFree->nextFree = NULL;
   newBlock->nextFree->lastFree = NULL;
   newBlock->nextFree->prevChunk = NULL;
   newBlock->nextFree->size = usableBlockSize;

   return(1);
  }

/*******************************************************/
/* RequestChunk: Allocates memory by returning a chunk */
/*   of memory from a larger block of memory.          */
/*******************************************************/
globle VOID *RequestChunk(requestSize)
  unsigned int requestSize;
  {
   struct chunkInfo *chunkPtr;
   struct blockInfo *blockPtr;
   
   /*==================================================*/
   /* Allocate initial memory pool block if it has not */
   /* already been allocated.                          */
   /*==================================================*/

   if (BlockMemoryInitialized == CLIPS_FALSE)
      {
       if (InitializeBlockMemory(requestSize) == 0) return(NULL);
      }

   /*====================================================*/
   /* Make sure that the amount of memory requested will */
   /* fall on a boundary of strictest alignment          */
   /*====================================================*/

   requestSize = (((requestSize - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;

   /*=====================================================*/
   /* Search through the list of free memory for a block  */
   /* of the appropriate size.  If a block is found, then */
   /* allocate and return a pointer to it.                */
   /*=====================================================*/

   blockPtr = TopMemoryBlock;

   while (blockPtr != NULL)
     {
      chunkPtr = blockPtr->nextFree;

      while (chunkPtr != NULL)
        {
         if ((chunkPtr->size == requestSize) ||
             (chunkPtr->size > (requestSize + ChunkInfoSize)))
           {
            AllocateChunk(blockPtr,chunkPtr,requestSize);

            return((VOID *) (((char *) chunkPtr) + ChunkInfoSize));
           }
         chunkPtr = chunkPtr->nextFree;
        }

      if (blockPtr->nextBlock == NULL)
        {
         if (AllocateBlock(blockPtr,requestSize) == 0)  /* get another block */
           { return(NULL); }
        }
      blockPtr = blockPtr->nextBlock;
     }

   CLIPSSystemError("MEMORY",2);
   ExitCLIPS(1);
   return(NULL); /* Unreachable, but prevents warning. */
  }

/********************************************/
/* AllocateChunk: Allocates a chunk from an */
/*   existing chunk in a block of memory.   */
/********************************************/
static VOID AllocateChunk(parentBlock,chunkPtr,requestSize)
  struct blockInfo *parentBlock;
  struct chunkInfo *chunkPtr;
  unsigned int requestSize;
  {
   struct chunkInfo *splitChunk, *nextChunk;

   /*=============================================================*/
   /* If the size of the memory chunk is an exact match for the   */
   /* requested amount of memory, then the chunk can be allocated */
   /* without splitting it.                                       */
   /*=============================================================*/

   if (requestSize == chunkPtr->size)
     {
      chunkPtr->size = - (long int) requestSize;
      if (chunkPtr->lastFree == NULL)
        {
         if (chunkPtr->nextFree != NULL)
           { parentBlock->nextFree = chunkPtr->nextFree; }
         else
           { parentBlock->nextFree = NULL; }
        }
      else
        { chunkPtr->lastFree->nextFree = chunkPtr->nextFree; }

      if (chunkPtr->nextFree != NULL)
        { chunkPtr->nextFree->lastFree = chunkPtr->lastFree; }

      chunkPtr->lastFree = NULL;
      chunkPtr->nextFree = NULL;
      return;
     }

   /*===========================================================*/
   /* If the size of the memory chunk is larger than the memory */
   /* request, then split the chunk into two pieces.            */
   /*===========================================================*/

   nextChunk = (struct chunkInfo *)
              (((char *) chunkPtr) + ChunkInfoSize + chunkPtr->size);

   splitChunk = (struct chunkInfo *)
                  (((char *) chunkPtr) + (ChunkInfoSize + requestSize));

   splitChunk->size = chunkPtr->size - (requestSize + ChunkInfoSize);
   splitChunk->prevChunk = chunkPtr;

   splitChunk->nextFree = chunkPtr->nextFree;
   splitChunk->lastFree = chunkPtr->lastFree;

   nextChunk->prevChunk = splitChunk;

   if (splitChunk->lastFree == NULL)
     { parentBlock->nextFree = splitChunk; }
   else
     { splitChunk->lastFree->nextFree = splitChunk; }

   if (splitChunk->nextFree != NULL)
     { splitChunk->nextFree->lastFree = splitChunk; }

   chunkPtr->size = - (long int) requestSize;
   chunkPtr->lastFree = NULL;
   chunkPtr->nextFree = NULL;

   return;
  }

/***********************************************************/
/* ReturnChunk: Frees memory allocated using RequestChunk. */
/***********************************************************/
globle int ReturnChunk(memPtr,size)
  VOID *memPtr;
  unsigned int size;
  {
   struct chunkInfo *chunkPtr, *lastChunk, *nextChunk, *topChunk;
   struct blockInfo *blockPtr;

   /*=====================================================*/
   /* Determine if the expected size of the chunk matches */
   /* the size stored in the chunk's information record.  */
   /*=====================================================*/

   size = (((size - 1) / STRICT_ALIGN_SIZE) + 1) * STRICT_ALIGN_SIZE;

   chunkPtr = (struct chunkInfo *) (((char *) memPtr) - ChunkInfoSize);

   if (chunkPtr == NULL)
     { return(-1); }

   if (chunkPtr->size >= 0)
     { return(-1); }

   if (chunkPtr->size != - (long int) size)
     { return(-1); }

   chunkPtr->size = - chunkPtr->size;

   /*=============================================*/
   /* Determine in which block the chunk resides. */
   /*=============================================*/

   topChunk = chunkPtr;
   while (topChunk->prevChunk != NULL)
     { topChunk = topChunk->prevChunk; }
   blockPtr = (struct blockInfo *) (((char *) topChunk) - BlockInfoSize);

   /*===========================================*/
   /* Determine the chunks physically preceding */
   /* and following the returned chunk.         */
   /*===========================================*/

   lastChunk = chunkPtr->prevChunk;
   nextChunk = (struct chunkInfo *) (((char *) memPtr) + size);

   /*=========================================================*/
   /* Add the chunk to the list of free chunks for the block. */
   /*=========================================================*/

   if (blockPtr->nextFree != NULL)
     { blockPtr->nextFree->lastFree = chunkPtr; }

   chunkPtr->nextFree = blockPtr->nextFree;
   chunkPtr->lastFree = NULL;

   blockPtr->nextFree = chunkPtr;

   /*=====================================================*/
   /* Combine this chunk with previous chunk if possible. */
   /*=====================================================*/

   if (lastChunk != NULL)
     {
      if (lastChunk->size > 0)
        {
         lastChunk->size += (ChunkInfoSize + chunkPtr->size);

         if (nextChunk != NULL)
           { nextChunk->prevChunk = lastChunk; }
         else
           { return(-1); }

         if (lastChunk->lastFree != NULL)
           { lastChunk->lastFree->nextFree = lastChunk->nextFree; }

         if (lastChunk->nextFree != NULL)
           { lastChunk->nextFree->lastFree = lastChunk->lastFree; }

         lastChunk->nextFree = chunkPtr->nextFree;
         if (chunkPtr->nextFree != NULL)
           { chunkPtr->nextFree->lastFree = lastChunk; }
         lastChunk->lastFree = NULL;

         blockPtr->nextFree = lastChunk;
         chunkPtr->lastFree = NULL;
         chunkPtr->nextFree = NULL;
         chunkPtr = lastChunk;
        }
     }

   /*=====================================================*/
   /* Combine this chunk with the next chunk if possible. */
   /*=====================================================*/

   if (nextChunk == NULL) return(-1);
   if (chunkPtr == NULL) return(-1);

   if (nextChunk->size > 0)
     {
      chunkPtr->size += (ChunkInfoSize + nextChunk->size);

      topChunk = (struct chunkInfo *) (((char *) nextChunk) + nextChunk->size + ChunkInfoSize);
      if (topChunk != NULL)
        { topChunk->prevChunk = chunkPtr; }
      else
        { return(-1); }

      if (nextChunk->lastFree != NULL)
        { nextChunk->lastFree->nextFree = nextChunk->nextFree; }

      if (nextChunk->nextFree != NULL)
        { nextChunk->nextFree->lastFree = nextChunk->lastFree; }

     }

   /*===========================================*/
   /* Free the buffer if we can, but don't free */
   /* the first buffer if it's the only one.    */
   /*===========================================*/

   if ((chunkPtr->prevChunk == NULL) &&
       (chunkPtr->size == blockPtr->size))
     {
      if (blockPtr->prevBlock != NULL)
        {
         blockPtr->prevBlock->nextBlock = blockPtr->nextBlock;
         if (blockPtr->nextBlock != NULL)
           { blockPtr->nextBlock->prevBlock = blockPtr->prevBlock; }
         free((char *) blockPtr);
        }
      else
        {
         if (blockPtr->nextBlock != NULL)
           {
            blockPtr->nextBlock->prevBlock = NULL;
            TopMemoryBlock = blockPtr->nextBlock;
            free((char *) blockPtr);
           }
        }
     }

   return(1);
  }

#endif
