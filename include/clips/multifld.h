   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                MULTIFIELD HEADER FILE               */
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

#ifndef _H_multifld

#define _H_multifld

struct field;
struct multifield;

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

struct field
  {
   short int type;
   VOID *value;
  };
    
struct multifield
  {
   unsigned busyCount;
   short depth;
   unsigned short multifieldLength;
   struct multifield *next;
   struct field theFields[1];
  };

typedef struct multifield SEGMENT;
typedef struct multifield * SEGMENT_PTR;
typedef struct multifield * MULTIFIELD_PTR;
typedef struct field FIELD;
typedef struct field * FIELD_PTR;

#define GetMFLength(target)     (((struct multifield *) (target))->multifieldLength) 
#define GetMFPtr(target,index)  (&(((struct multifield *) (target))->theFields[index-1])) 
#define SetMFType(target,index,value)  (((struct multifield *) (target))->theFields[index-1].type = (value)) 
#define SetMFValue(target,index,val)  (((struct multifield *) (target))->theFields[index-1].value = (VOID *) (val))  
#define GetMFType(target,index)  (((struct multifield *) (target))->theFields[index-1].type) 
#define GetMFValue(target,index)  (((struct multifield *) (target))->theFields[index-1].value) 

#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _MULTIFLD_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                          *CreateMultifield2(int);
   LOCALE VOID                           ReturnMultifield(struct multifield *);
   LOCALE VOID                           MultifieldInstall(struct multifield *);
   LOCALE VOID                           MultifieldDeinstall(struct multifield *);
   LOCALE struct multifield             *StringToMultifield(char *);
   LOCALE VOID                          *CreateMultifield(int);
   LOCALE VOID                           AddToMultifieldList(struct multifield *);
   LOCALE VOID                           FlushMultifields(void);
   LOCALE VOID                           DuplicateMultifield(struct dataObject *,struct dataObject *);
   LOCALE VOID                           PrintMultifield(char *,SEGMENT_PTR,int,int,int);
   LOCALE BOOLEAN                        MultifieldDOsEqual(DATA_OBJECT_PTR,DATA_OBJECT_PTR);
   LOCALE VOID                           StoreInMultifield(DATA_OBJECT *,EXPRESSION *,int);
   LOCALE VOID                          *CopyMultifield(struct multifield *);
   LOCALE BOOLEAN                        MultifieldsEqual(struct multifield *,struct multifield *);
   LOCALE VOID                          *DOToMultifield(DATA_OBJECT *);
#else
   LOCALE VOID                          *CreateMultifield2();
   LOCALE VOID                           ReturnMultifield();
   LOCALE VOID                           MultifieldInstall();
   LOCALE VOID                           MultifieldDeinstall();
   LOCALE struct multifield             *StringToMultifield();
   LOCALE VOID                          *CreateMultifield();
   LOCALE VOID                           AddToMultifieldList();
   LOCALE VOID                           FlushMultifields();
   LOCALE VOID                           DuplicateMultifield();
   LOCALE VOID                           PrintMultifield();
   LOCALE BOOLEAN                        MultifieldDOsEqual();
   LOCALE VOID                           StoreInMultifield();
   LOCALE VOID                          *CopyMultifield();
   LOCALE BOOLEAN                        MultifieldsEqual();
   LOCALE VOID                          *DOToMultifield();
#endif

#endif





