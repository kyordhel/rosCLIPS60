   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              CLIPS MEMORY HEADER FILE               */
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

#ifndef _H_clipsmem

#if ANSI_COMPILER 
#include <string.h>
#endif

#define _H_clipsmem

struct chunkInfo;
struct blockInfo;
struct memoryPtr;

#define MEM_TABLE_SIZE 300

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MEMORY_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

struct chunkInfo
  {
   struct chunkInfo *prevChunk;
   struct chunkInfo *nextFree;
   struct chunkInfo *lastFree;
   long int size;
  };

struct blockInfo
  {
   struct blockInfo *nextBlock;
   struct blockInfo *prevBlock;
   struct chunkInfo *nextFree;
   long int size;
  };
  
struct memoryPtr
  {
   struct memoryPtr *next;
  };
                                                                     
#define get_struct(type) \
  ((MemoryTable[sizeof(struct type)] == NULL) ? \
   ((struct type *) genalloc((unsigned) sizeof(struct type))) :\
   ((TempMemoryPtr = MemoryTable[sizeof(struct type)]),\
    MemoryTable[sizeof(struct type)] = TempMemoryPtr->next,\
    ((struct type *) TempMemoryPtr)))
    
#define rtn_struct(type,struct_ptr) \
  (TempMemoryPtr = (struct memoryPtr *) struct_ptr,\
   TempMemoryPtr->next = MemoryTable[sizeof(struct type)], \
   MemoryTable[sizeof(struct type)] = TempMemoryPtr)
   
#define rtn_sized_struct(size,struct_ptr) \
  (TempMemoryPtr = (struct memoryPtr *) struct_ptr,\
   TempMemoryPtr->next = MemoryTable[size], \
   MemoryTable[size] = TempMemoryPtr)
                                                     
#define get_var_struct(type,vsize) \
  ((((sizeof(struct type) + vsize) <  MEM_TABLE_SIZE) ? \
    (MemoryTable[sizeof(struct type) + vsize] == NULL) : 1) ? \
   ((struct type *) genalloc((unsigned) sizeof(struct type) + vsize)) :\
   ((TempMemoryPtr = MemoryTable[sizeof(struct type) + vsize]),\
    MemoryTable[sizeof(struct type) + vsize] = TempMemoryPtr->next,\
    ((struct type *) TempMemoryPtr)))
  
#define rtn_var_struct(type,vsize,struct_ptr) \
  (((sizeof(struct type) + vsize) <  MEM_TABLE_SIZE) ? \
   (TempMemoryPtr = (struct memoryPtr *) struct_ptr,\
    TempSize = sizeof(struct type) + vsize, \
    TempMemoryPtr->next = MemoryTable[TempSize], \
    MemoryTable[TempSize] =  TempMemoryPtr) : \
   (genfree((VOID *) struct_ptr,(unsigned) TempSize),(struct memoryPtr *) struct_ptr))

#define get_var_struct2(type,vsize) \
  ((((sizeof(struct type) + vsize) <  (unsigned long) MEM_TABLE_SIZE) ? \
    (MemoryTable[sizeof(struct type) + vsize] == NULL) : 1) ? \
   ((struct type *) gm3((long) sizeof(struct type) + vsize)) :\
   ((TempMemoryPtr = MemoryTable[sizeof(struct type) + vsize]),\
    MemoryTable[sizeof(struct type) + vsize] = TempMemoryPtr->next,\
    ((struct type *) TempMemoryPtr)))
  
#define rtn_var_struct2(type,vsize,struct_ptr) \
  (((sizeof(struct type) + vsize) <  (unsigned long) MEM_TABLE_SIZE) ? \
   (TempMemoryPtr = (struct memoryPtr *) struct_ptr,\
    TempSize2 = sizeof(struct type) + vsize, \
    TempMemoryPtr->next = MemoryTable[TempSize2], \
    MemoryTable[TempSize2] =  TempMemoryPtr) : \
   (rm3((VOID *) struct_ptr,(long) sizeof(struct type) + vsize),(struct memoryPtr *) struct_ptr))

#if ANSI_COMPILER

#define CopyMemory(type,cnt,dst,src) \
   memcpy((void *) (dst),(void *) (src),sizeof(type) * (size_t) (cnt))

#else

#define CopyMemory(type,cnt,dst,src) \
   genmemcpy((char *) (dst),(char *) (src),((unsigned long) sizeof(type) * (unsigned long) (cnt)))
  
#endif
   
#if ANSI_COMPILER  
   LOCALE VOID                          *genalloc(unsigned int);
   LOCALE int                            DefaultOutOfMemoryFunction(unsigned long);
   LOCALE int                          (*SetOutOfMemoryFunction(int (*)(unsigned long)))(unsigned long);
   LOCALE int                            genfree(VOID *,unsigned int);
   LOCALE VOID                          *genrealloc(VOID *,unsigned int,unsigned int);
   LOCALE long                           MemUsed(void);
   LOCALE long                           MemRequests(void);
   LOCALE long                           UpdateMemoryUsed(long int);
   LOCALE long                           UpdateMemoryRequests(long int);
   LOCALE long                           ReleaseMem(long,int);
   LOCALE VOID                          *gm1(int);
   LOCALE VOID                          *gm2(int);
   LOCALE VOID                          *gm3(long);
   LOCALE int                            rm(VOID *,int);
   LOCALE int                            rm3(VOID *,long);
   LOCALE unsigned long                  PoolSize(void);
   LOCALE unsigned long                  ActualPoolSize(void);
   LOCALE VOID                          *RequestChunk(unsigned int);
   LOCALE int                            ReturnChunk(VOID *,unsigned int);
   LOCALE VOID                          *genlongalloc(unsigned long);
   LOCALE int                            genlongfree(VOID *,unsigned long);
   LOCALE BOOLEAN                        SetConserveMemory(BOOLEAN);
   LOCALE BOOLEAN                        GetConserveMemory(void);
   LOCALE VOID                           InitializeMemory(void);
#else
   LOCALE VOID                          *genalloc();
   LOCALE int                            DefaultOutOfMemoryFunction();
   LOCALE int                          (*SetOutOfMemoryFunction())();
   LOCALE int                            genfree();
   LOCALE VOID                          *genrealloc();
   LOCALE long                           MemUsed();
   LOCALE long                           MemRequests();
   LOCALE long                           UpdateMemoryUsed();
   LOCALE long                           UpdateMemoryRequests();
   LOCALE long                           ReleaseMem();
   LOCALE VOID                          *gm1();
   LOCALE VOID                          *gm2();
   LOCALE VOID                          *gm3();
   LOCALE int                            rm();
   LOCALE int                            rm3();
   LOCALE unsigned long                  PoolSize();
   LOCALE unsigned long                  ActualPoolSize();
   LOCALE VOID                          *RequestChunk();
   LOCALE int                            ReturnChunk();
   LOCALE VOID                          *genlongalloc();
   LOCALE int                            genlongfree();
   LOCALE BOOLEAN                        SetConserveMemory();
   LOCALE BOOLEAN                        GetConserveMemory();
   LOCALE VOID                           genmemcpy();
   LOCALE VOID                           InitializeMemory();
#endif
   
#ifndef _MEMORY_SOURCE_
   extern struct memoryPtr                      *TempMemoryPtr;
   extern unsigned int                           TempSize;
   extern unsigned long                          TempSize2;
   extern struct memoryPtr                     **MemoryTable;
#endif

#endif






