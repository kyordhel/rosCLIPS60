   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                DEFMODULE HEADER FILE                */
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

#ifndef _H_moduldef
#define _H_moduldef

struct defmodule;
struct portItem;
struct defmoduleItemHeader;
struct moduleItem;

#ifndef _CLIPS_STDIO_
#include <stdio.h>
#define _CLIPS_STDIO_
#endif

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_constrct
#include "constrct.h"
#endif

/**********************************************************************/
/* defmodule                                                          */
/* ----------                                                         */
/* name: The name of the defmodule (stored as a reference in the      */
/*   table).                                                          */
/*                                                                    */
/* ppForm: The pretty print representation of the defmodule (used by  */
/*   the save and ppdefmodule commands).                              */
/*                                                                    */
/* itemsArray: An array of pointers to the module specific data used  */
/*   by each construct specified with the RegisterModuleItem          */
/*   function. The data pointer stored in the array is allocated by   */
/*   the allocateFunction in moduleItem data structure.               */
/*                                                                    */
/* importList: The list of items which are being imported by this     */
/*   module from other modules.                                       */
/*                                                                    */
/* next: A pointer to the next defmodule data structure.              */
/**********************************************************************/
struct defmodule
  {
   struct symbolHashNode *name;
   char *ppForm;
   struct defmoduleItemHeader **itemsArray;     
   struct portItem *importList;
   struct portItem *exportList;
   unsigned visitedFlag;
   long bsaveID;        
   struct defmodule *next; 
  };
  
struct portItem
  {
   struct symbolHashNode *moduleName;
   struct symbolHashNode *constructType;
   struct symbolHashNode *constructName;
   struct portItem *next; 
  };
    
struct defmoduleItemHeader
  {
   struct defmodule *theModule;
   struct constructHeader *firstItem;
   struct constructHeader *lastItem;
  };

#define MIHS (struct defmoduleItemHeader *)

/**********************************************************************/
/* moduleItem                                                         */
/* ----------                                                         */
/* name: The name of the construct which can be placed in a module.   */
/*   For example, "defrule".                                          */
/*                                                                    */
/* allocateFunction: Used to allocate a data structure containing all */
/*   pertinent information related to a specific construct for a      */
/*   given module. For example, the deffacts construct stores a       */
/*   pointer to the first and last deffacts for each each module.     */
/*                                                                    */
/* freeFunction: Used to deallocate a data structure allocated by     */
/*   the allocateFunction. In addition, the freeFunction deletes      */
/*   all constructs of the specified type in the given module.        */
/*                                                                    */
/* bloadModuleReference: Used during a binary load to establish a     */
/*   link between the defmodule data structure and the data structure */
/*   containing all pertinent module information for a specific       */
/*   construct.                                                       */
/*                                                                    */
/* findFunction: Used to determine if a specified construct is in a   */
/*   specific module. The name is the specific construct is passed as */
/*   a string and the function returns a pointer to the specified     */
/*   construct if it exists.                                          */
/*                                                                    */
/* exportable: If TRUE, then the specified construct type can be      */
/*   exported (and hence imported). If FALSE, it can't be exported.   */
/*                                                                    */
/* next: A pointer to the next moduleItem data structure.             */
/**********************************************************************/

struct moduleItem
  {
   char *name;
   int moduleIndex;
#if ANSI_COMPILER
   VOID *(*allocateFunction)(VOID);
   VOID  (*freeFunction)(VOID *);
   VOID *(*bloadModuleReference)(int);
   VOID  (*constructsToCModuleReference)(FILE *,int,int,int);
   VOID *(*findFunction)(char *);
#else
   VOID *(*allocateFunction)();
   VOID (*freeFunction)();
   VOID *(*bloadModuleReference)();
   VOID  (*constructsToCModuleReference)();
   VOID *(*findFunction)();
#endif
   struct moduleItem *next;
  };
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MODULDEF_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           InitializeDefmodules(void);
   LOCALE VOID                          *FindDefmodule(char *);
   LOCALE char                          *GetDefmoduleName(void *);
   LOCALE char                          *GetDefmodulePPForm(void *);
   LOCALE VOID                          *GetNextDefmodule(void *);
   LOCALE VOID                           RemoveAllDefmodules(void);
   LOCALE int                            AllocateModuleStorage(void);
   LOCALE int                            RegisterModuleItem(char *,
                                                            VOID *(*)(void),
                                                            VOID (*)(VOID *),
                                                            VOID *(*)(int),
                                                            VOID (*)(FILE *,int,int,int),
                                                            VOID *(*)(char *));
   LOCALE VOID                          *GetModuleItem(struct defmodule *,int);
   LOCALE VOID                           SetModuleItem(struct defmodule *,int,VOID *);
   LOCALE VOID                          *GetCurrentModule(VOID);
   LOCALE VOID                          *SetCurrentModule(VOID *);
   LOCALE SYMBOL_HN                     *GetCurrentModuleCommand(void);
   LOCALE SYMBOL_HN                     *SetCurrentModuleCommand(void);
   LOCALE int                            GetNumberOfModuleItems(void);
   LOCALE VOID                           CreateMainModule(void);
   LOCALE VOID                           SetListOfDefmodules(void *);
   LOCALE struct moduleItem             *GetListOfModuleItems(void);
   LOCALE struct moduleItem             *FindModuleItem(char *);
   LOCALE VOID                           SaveCurrentModule(void);
   LOCALE VOID                           RestoreCurrentModule(void);
   LOCALE VOID                           AddAfterModuleChangeFunction(char *,VOID (*)(void),int);
   LOCALE VOID                           IllegalModuleSpecifierMessage(void);
#else
   LOCALE VOID                           InitializeDefmodules();
   LOCALE VOID                          *FindDefmodule();
   LOCALE char                          *GetDefmoduleName();
   LOCALE char                          *GetDefmodulePPForm();
   LOCALE VOID                          *GetNextDefmodule();
   LOCALE VOID                           RemoveAllDefmodules();
   LOCALE int                            AllocateModuleStorage();
   LOCALE int                            RegisterModuleItem();
   LOCALE VOID                          *GetModuleItem();
   LOCALE VOID                           SetModuleItem();
   LOCALE VOID                          *GetCurrentModule();
   LOCALE VOID                          *SetCurrentModule();
   LOCALE SYMBOL_HN                     *GetCurrentModuleCommand();
   LOCALE SYMBOL_HN                     *SetCurrentModuleCommand();
   LOCALE int                            GetNumberOfModuleItems();
   LOCALE VOID                           CreateMainModule();
   LOCALE VOID                           SetListOfDefmodules();
   LOCALE struct moduleItem             *GetListOfModuleItems();
   LOCALE struct moduleItem             *FindModuleItem();
   LOCALE VOID                           SaveCurrentModule();
   LOCALE VOID                           RestoreCurrentModule();
   LOCALE VOID                           AddAfterModuleChangeFunction();
   LOCALE VOID                           IllegalModuleSpecifierMessage();
#endif

#ifndef _MODULDEF_SOURCE_
   extern struct defmodule              *ListOfDefmodules;
   extern struct defmodule              *CurrentModule;
   extern struct defmodule              *LastDefmodule;
   extern int                            NumberOfModuleItems;
   extern struct moduleItem             *ListOfModuleItems;
   extern long                           ModuleChangeIndex;
   extern int                            MainModuleRedefinable;
#endif
   
#endif



