   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*        CLASS INFO PROGRAMMATIC ACCESS MODULE        */
   /*******************************************************/

/**************************************************************/
/* Purpose: Class Information Interface Support Routines      */
/*                                                            */
/* Principal Programmer(s):                                   */
/*      Brian L. Donnell                                      */
/*                                                            */
/* Contributing Programmer(s):                                */
/*                                                            */
/* Revision History:                                          */
/*                                                            */
/**************************************************************/
   
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if OBJECT_SYSTEM

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#if ANSI_COMPILER
#include <string.h>
#endif

#include "argacces.h"
#include "classcom.h"
#include "classexm.h"
#include "classfun.h"
#include "classini.h"
#include "clipsmem.h"
#include "insfun.h"
#include "msgfun.h"
#include "multifld.h"
#include "prntutil.h"

#define _CLASSINF_SOURCE_
#include "classinf.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static VOID SlotInfoSupportFunction(DATA_OBJECT *,char *,VOID (*)(VOID *,char *,DATA_OBJECT *));
static int CountSubclasses(DEFCLASS *,int,int);
static int StoreSubclasses(VOID *,int,DEFCLASS *,int,int);
static SLOT_DESC *SlotInfoSlot(DATA_OBJECT *,DEFCLASS *,char *,char *);

#else

static VOID SlotInfoSupportFunction();
static int CountSubclasses();
static int StoreSubclasses();
static SLOT_DESC *SlotInfoSlot();

#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/*********************************************************************
  NAME         : ClassAbstractPCommand
  DESCRIPTION  : Determines if direct instances of a class can be made
  INPUTS       : None
  RETURNS      : CLIPS_TRUE (1) if class is abstract, CLIPS_FALSE (0) if concrete
  SIDE EFFECTS : None
  NOTES        : Syntax: (class-abstractp <class>)
 *********************************************************************/
globle int ClassAbstractPCommand()
  {
   DATA_OBJECT tmp;
   DEFCLASS *cls;
   
   if (ArgTypeCheck("class-abstractp",1,SYMBOL,&tmp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   cls = LookupDefclassByMdlOrScope(DOToString(tmp));
   if (cls == NULL)
     {
      ClassExistError("class-abstractp",ValueToString(tmp.value));
      return(CLIPS_FALSE);
     }
   return(ClassAbstractP((VOID *) cls));
  }

#if INSTANCE_PATTERN_MATCHING

/*****************************************************************
  NAME         : ClassReactivePCommand
  DESCRIPTION  : Determines if instances of a class can match rule
                 patterns
  INPUTS       : None
  RETURNS      : CLIPS_TRUE (1) if class is reactive, CLIPS_FALSE (0) 
                 if non-reactive
  SIDE EFFECTS : None
  NOTES        : Syntax: (class-reactivep <class>)
 *****************************************************************/
globle int ClassReactivePCommand()
  {
   DATA_OBJECT tmp;
   DEFCLASS *cls;
   
   if (ArgTypeCheck("class-reactivep",1,SYMBOL,&tmp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   cls = LookupDefclassByMdlOrScope(DOToString(tmp));
   if (cls == NULL)
     {
      ClassExistError("class-reactivep",ValueToString(tmp.value));
      return(CLIPS_FALSE);
     }
   return(ClassReactiveP((VOID *) cls));
  }

#endif 

/***********************************************************
  NAME         : ClassInfoFnxArgs
  DESCRIPTION  : Examines arguments for:
                   class-slots, get-defmessage-handler-list,
                   class-superclasses and class-subclasses
  INPUTS       : 1) Name of function
                 2) A buffer to hold a flag indicating if
                    the inherit keyword was specified
  RETURNS      : Pointer to the class on success,
                   NULL on errors 
  SIDE EFFECTS : inhp flag set
                 error flag set
  NOTES        : None
 ***********************************************************/
globle VOID *ClassInfoFnxArgs(fnx,inhp)
  char *fnx;
  int *inhp;
  {
   VOID *clsptr;
   DATA_OBJECT tmp;

   *inhp = 0;
   if (RtnArgCount() == 0)
     {
      ExpectedCountError(fnx,AT_LEAST,1);
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
   if (ArgTypeCheck(fnx,1,SYMBOL,&tmp) == CLIPS_FALSE)
     return(NULL);
   clsptr = (VOID *) LookupDefclassByMdlOrScope(DOToString(tmp));
   if (clsptr == NULL)
     {
      ClassExistError(fnx,ValueToString(tmp.value));
      return(NULL);
     }
   if (RtnArgCount() == 2)
     {
      if (ArgTypeCheck(fnx,2,SYMBOL,&tmp) == CLIPS_FALSE)
        return(NULL);
      if (strcmp(ValueToString(tmp.value),"inherit") == 0)
        *inhp = 1;
      else
        {
         SyntaxErrorMessage(fnx);
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
     }
   return(clsptr);
  }

/********************************************************************
  NAME         : ClassSlotsCommand
  DESCRIPTION  : Groups slot info for a class into a multifield value
                   for dynamic perusal
  INPUTS       : Data object buffer to hold the slots of the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names of
                    the slots of the class
  NOTES        : Syntax: (class-slots <class> [inherit])
 ********************************************************************/
globle VOID ClassSlotsCommand(result)
  DATA_OBJECT *result;
  {
   int inhp;
   VOID *clsptr;
   
   clsptr = ClassInfoFnxArgs("class-slots",&inhp);
   if (clsptr == NULL)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   ClassSlots(clsptr,result,inhp);
  }
  
/************************************************************************
  NAME         : ClassSuperclassesCommand
  DESCRIPTION  : Groups superclasses for a class into a multifield value
                   for dynamic perusal
  INPUTS       : Data object buffer to hold the superclasses of the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names of
                    the superclasses of the class
  NOTES        : Syntax: (class-superclasses <class> [inherit])
 ************************************************************************/
globle VOID ClassSuperclassesCommand(result)
  DATA_OBJECT *result;
  {
   int inhp;
   VOID *clsptr;
   
   clsptr = ClassInfoFnxArgs("class-superclasses",&inhp);
   if (clsptr == NULL)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   ClassSuperclasses(clsptr,result,inhp);
  }
  
/************************************************************************
  NAME         : ClassSubclassesCommand
  DESCRIPTION  : Groups subclasses for a class into a multifield value
                   for dynamic perusal
  INPUTS       : Data object buffer to hold the subclasses of the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names of
                    the subclasses of the class
  NOTES        : Syntax: (class-subclasses <class> [inherit])
 ************************************************************************/
globle VOID ClassSubclassesCommand(result)
  DATA_OBJECT *result;
  {
   int inhp;
   VOID *clsptr;
   
   clsptr = ClassInfoFnxArgs("class-subclasses",&inhp);
   if (clsptr == NULL)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   ClassSubclasses(clsptr,result,inhp);
  }
  
/***********************************************************************
  NAME         : GetDefmessageHandlersListCmd
  DESCRIPTION  : Groups message-handlers for a class into a multifield
                   value for dynamic perusal
  INPUTS       : Data object buffer to hold the handlers of the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names of
                    the message-handlers of the class
  NOTES        : Syntax: (get-defmessage-handler-list <class> [inherit])
 ***********************************************************************/
globle VOID GetDefmessageHandlersListCmd(result)
  DATA_OBJECT *result;
  {
   int inhp;
   VOID *clsptr;
   
   if (RtnArgCount () == 0)
      GetDefmessageHandlerList(NULL,result,0);
   else
     {
      clsptr = ClassInfoFnxArgs("get-defmessage-handler-list",&inhp);
      if (clsptr == NULL)
        {
         SetMultifieldErrorValue(result);
         return;
        }
      GetDefmessageHandlerList(clsptr,result,inhp);
     }
  }
  
/*********************************
 Slot Information Access Functions
 *********************************/
globle VOID SlotFacetsCommand(result)
  DATA_OBJECT *result;
  {
   SlotInfoSupportFunction(result,"slot-facets",SlotFacets);
  }

globle VOID SlotSourcesCommand(result)
  DATA_OBJECT *result;
  {
   SlotInfoSupportFunction(result,"slot-sources",SlotSources);
  }
  
globle VOID SlotTypesCommand(result)
  DATA_OBJECT *result;
  {
   SlotInfoSupportFunction(result,"slot-types",SlotTypes);
  }
  
globle VOID SlotAllowedValuesCommand(result)
  DATA_OBJECT *result;
  {
   SlotInfoSupportFunction(result,"slot-allowed-values",SlotAllowedValues);
  }
  
globle VOID SlotRangeCommand(result)
  DATA_OBJECT *result;
  {
   SlotInfoSupportFunction(result,"slot-range",SlotRange);
  }
  
globle VOID SlotCardinalityCommand(result)
  DATA_OBJECT *result;
  {
   SlotInfoSupportFunction(result,"slot-cardinality",SlotCardinality);
  }

/********************************************************************
  NAME         : ClassAbstractP
  DESCRIPTION  : Determines if a class is abstract or not
  INPUTS       : Generic pointer to class
  RETURNS      : 1 if class is abstract, 0 otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************************/
globle BOOLEAN ClassAbstractP(clsptr)
  VOID *clsptr;
  {
   return(((DEFCLASS *) clsptr)->abstract);
  }

#if INSTANCE_PATTERN_MATCHING

/********************************************************************
  NAME         : ClassReactiveP
  DESCRIPTION  : Determines if a class is reactive or not
  INPUTS       : Generic pointer to class
  RETURNS      : 1 if class is reactive, 0 otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************************/
globle BOOLEAN ClassReactiveP(clsptr)
  VOID *clsptr;
  {
   return(((DEFCLASS *) clsptr)->reactive);
  }

#endif

/********************************************************************
  NAME         : ClassSlots
  DESCRIPTION  : Groups slot info for a class into a multifield value
                   for dynamic perusal
  INPUTS       : 1) Generic pointer to class
                 2) Data object buffer to hold the slots of the class
                 3) Include (1) or exclude (0) inherited slots
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names of
                    the slots of the class
  NOTES        : None
 ********************************************************************/
globle VOID ClassSlots(clsptr,result,inhp)
  VOID *clsptr;
  DATA_OBJECT *result;
  int inhp;
  {
   int size;
   register DEFCLASS *cls;
   register int i;
   
   cls = (DEFCLASS *) clsptr;
   size = inhp ? cls->instanceSlotCount : cls->slotCount;
   result->type = MULTIFIELD;
   result->begin = 0;
   result->end = size - 1;
   result->value = (VOID *) CreateMultifield(size);
   if (size == 0)
     return;
   if (inhp)
     {
      for (i = 0 ; i < cls->instanceSlotCount ; i++)
        {
         SetMFType(result->value,i+1,SYMBOL);
         SetMFValue(result->value,i+1,cls->instanceTemplate[i]->slotName->name);
        }
     }
   else
     {
      for (i = 0 ; i < cls->slotCount ; i++)
        {
         SetMFType(result->value,i+1,SYMBOL);
         SetMFValue(result->value,i+1,cls->slots[i].slotName->name);
        }
     }
  }
  
/************************************************************************
  NAME         : GetDefmessageHandlerList
  DESCRIPTION  : Groups handler info for a class into a multifield value
                   for dynamic perusal
  INPUTS       : 1) Generic pointer to class (NULL to get handlers for
                    all classes)
                 2) Data object buffer to hold the handlers of the class
                 3) Include (1) or exclude (0) inherited handlers
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names and types of
                    the message-handlers of the class
  NOTES        : None
 ************************************************************************/
globle VOID GetDefmessageHandlerList(clsptr,result,inhp)
  VOID *clsptr;
  DATA_OBJECT *result;
  int inhp;
  {
   DEFCLASS *cls,*svcls,*svnxt,*supcls;
   register int i,j,classi,classiLimit;
   int len,sublen;
   
   if (clsptr == NULL)
     {
      inhp = 0;
      cls = (DEFCLASS *) GetNextDefclass(NULL);
      svnxt = (DEFCLASS *) GetNextDefclass((VOID *) cls);
     }
   else
     {
      cls = (DEFCLASS *) clsptr;
      svnxt = (DEFCLASS *) GetNextDefclass((VOID *) cls);
      SetNextDefclass((VOID *) cls,NULL);
     }
   for (svcls = cls , i = 0 ; 
        cls != NULL ;
        cls = (DEFCLASS *) GetNextDefclass((VOID *) cls))
     {
      classiLimit = inhp ? cls->allSuperclasses.classCount : 1;
      for (classi = 0 ; classi < classiLimit ; classi++)
        i += cls->allSuperclasses.classArray[classi]->handlerCount;
     }
   len = i * 3;
   result->type = MULTIFIELD;
   result->begin = 0;
   result->end = len - 1;
   result->value = (VOID *) CreateMultifield(len);
   for (cls = svcls , sublen = 0 ;
        cls != NULL ;
        cls = (DEFCLASS *) GetNextDefclass((VOID *) cls))
     {
      classiLimit = inhp ? cls->allSuperclasses.classCount : 1;
      for (classi = 0 ; classi < classiLimit ; classi++)
        {
         supcls = cls->allSuperclasses.classArray[classi];
         if (inhp == 0)
           i = sublen + 1;
         else
           i = len - (supcls->handlerCount * 3) - sublen + 1;
         for (j = 0 ; j < supcls->handlerCount ; j++)
           {
            SetMFType(result->value,i,SYMBOL);
            SetMFValue(result->value,i++,GetDefclassNamePointer((VOID *) supcls));
            SetMFType(result->value,i,SYMBOL);
            SetMFValue(result->value,i++,supcls->handlers[j].name);
            SetMFType(result->value,i,SYMBOL);
            SetMFValue(result->value,i++,AddSymbol(hndquals[supcls->handlers[j].type]));
           }
         sublen += supcls->handlerCount * 3;
        }
     }
   if (svcls != NULL)
     SetNextDefclass((VOID *) svcls,(VOID *) svnxt);
  }
  
/***************************************************************************
  NAME         : ClassSuperclasses
  DESCRIPTION  : Groups the names of superclasses into a multifield
                   value for dynamic perusal
  INPUTS       : 1) Generic pointer to class
                 2) Data object buffer to hold the superclasses of the class
                 3) Include (1) or exclude (0) indirect superclasses
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names of
                    the superclasses of the class
  NOTES        : None
 ***************************************************************************/
globle VOID ClassSuperclasses(clsptr,result,inhp)
  VOID *clsptr;
  DATA_OBJECT *result;
  int inhp;
  {
   PACKED_CLASS_LINKS *plinks;
   int offset;
   register unsigned i,j;
   
   if (inhp)
     {
      plinks = &((DEFCLASS *) clsptr)->allSuperclasses;
      offset = 1;
     }
   else
     {
      plinks = &((DEFCLASS *) clsptr)->directSuperclasses;
      offset = 0;
     }
   result->type = MULTIFIELD;
   result->begin = 0;
   result->end = plinks->classCount - offset - 1;
   result->value = (VOID *) CreateMultifield(result->end + 1);
   if (result->end == -1)
     return;
   for (i = offset , j = 1 ; i < plinks->classCount ; i++ , j++)
     {
      SetMFType(result->value,j,SYMBOL);
      SetMFValue(result->value,j,GetDefclassNamePointer((VOID *) plinks->classArray[i]));
     }
  }
  
/**************************************************************************
  NAME         : ClassSubclasses
  DESCRIPTION  : Groups the names of subclasses for a class into a
                   multifield value for dynamic perusal
  INPUTS       : 1) Generic pointer to class
                 2) Data object buffer to hold the sublclasses of the class
                 3) Include (1) or exclude (0) indirect subclasses
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the names
                    the subclasses of the class
  NOTES        : None
 **************************************************************************/
globle VOID ClassSubclasses(clsptr,result,inhp)
  VOID *clsptr;
  DATA_OBJECT *result;
  int inhp;
  {
   register int i,id;
   
   if ((id = GetTraversalID()) == -1)
     return;
   i = CountSubclasses((DEFCLASS *) clsptr,inhp,id);
   ReleaseTraversalID();
   result->type = MULTIFIELD;
   result->begin = 0;
   result->end = i - 1;
   result->value = (VOID *) CreateMultifield(i);
   if (i == 0)
     return;
   if ((id = GetTraversalID()) == -1)
     return;
   StoreSubclasses(result->value,1,(DEFCLASS *) clsptr,inhp,id);
   ReleaseTraversalID();
  }

/**************************************************************************
  NAME         : Slot...  Slot information access functions
  DESCRIPTION  : Groups the sources/facets/types/allowed-values/range or
                   cardinality  of a slot for a class into a multifield
                   value for dynamic perusal
  INPUTS       : 1) Generic pointer to class
                 2) Name of the slot
                 3) Data object buffer to hold the attributes of the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Creates a multifield storing the attributes for the slot
                   of the class
  NOTES        : None
 **************************************************************************/

globle VOID SlotFacets(clsptr,sname,result)
  VOID *clsptr;
  char *sname;
  DATA_OBJECT *result;
  {
   register int i;
   register SLOT_DESC *sp;

   if ((sp = SlotInfoSlot(result,(DEFCLASS *) clsptr,sname,"slot-facets")) == NULL)
     return;
#if INSTANCE_PATTERN_MATCHING
   result->end = 9;
   result->value = (VOID *) CreateMultifield(10);
   for (i = 1 ; i <= 10 ; i++)
     SetMFType(result->value,i,SYMBOL);
#else
   result->end = 8;
   result->value = (VOID *) CreateMultifield(9);
   for (i = 1 ; i <= 9 ; i++)
     SetMFType(result->value,i,SYMBOL);
#endif
   SetMFValue(result->value,1,AddSymbol(sp->multiple ? "MLT" : "SGL"));
   if (sp->noDefault)
     SetMFValue(result->value,2,AddSymbol("NIL"));
   else
     SetMFValue(result->value,2,AddSymbol(sp->dynamicDefault ? "DYN" : "STC"));
   SetMFValue(result->value,3,AddSymbol(sp->noInherit ? "NIL" : "INH"));
   if (sp->initializeOnly)
     SetMFValue(result->value,4,AddSymbol("INT"));
   else if (sp->noWrite)
     SetMFValue(result->value,4,AddSymbol("R"));
   else
     SetMFValue(result->value,4,AddSymbol("RW"));
   SetMFValue(result->value,5,AddSymbol(sp->shared ? "SHR" : "LCL"));
#if INSTANCE_PATTERN_MATCHING
   SetMFValue(result->value,6,AddSymbol(sp->reactive ? "RCT" : "NIL"));
   SetMFValue(result->value,7,AddSymbol(sp->composite ? "CMP" : "EXC"));
   SetMFValue(result->value,8,AddSymbol(sp->publicVisibility ? "PUB" : "PRV"));
   SetMFValue(result->value,9,AddSymbol(GetCreateAccessorString((VOID *) sp)));
   SetMFValue(result->value,10,sp->noWrite ? AddSymbol("NIL") : (VOID *) sp->overrideMessage);
#else
   SetMFValue(result->value,6,AddSymbol(sp->composite ? "CMP" : "EXC"));
   SetMFValue(result->value,7,AddSymbol(sp->publicVisibility ? "PUB" : "PRV"));
   SetMFValue(result->value,8,AddSymbol(GetCreateAccessorString((VOID *) sp)));
   SetMFValue(result->value,9,sp->noWrite ? AddSymbol("NIL") : (VOID *) sp->overrideMessage);
#endif
  }

globle VOID SlotSources(clsptr,sname,result)
  VOID *clsptr;
  char *sname;
  DATA_OBJECT *result;
  {
   register int i,classi;
   register SLOT_DESC *sp,*csp;
   CLASS_LINK *ctop,*ctmp;
   DEFCLASS *cls;
   
   if ((sp = SlotInfoSlot(result,(DEFCLASS *) clsptr,sname,"slot-sources")) == NULL)
     return;
   i = 1;
   ctop = get_struct(classLink);
   ctop->cls = sp->cls;
   ctop->nxt = NULL;
   if (sp->composite)
     {
      for (classi = 1 ; classi < sp->cls->allSuperclasses.classCount ; classi++)
        {
         cls = sp->cls->allSuperclasses.classArray[classi];
         csp = FindClassSlot(cls,sp->slotName->name);
         if ((csp != NULL) ? (csp->noInherit == 0) : CLIPS_FALSE)
           {
            ctmp = get_struct(classLink);
            ctmp->cls = cls;
            ctmp->nxt = ctop;
            ctop = ctmp;
            i++;
            if (csp->composite == 0)
              break;
           }
        }
     }
   result->end = i - 1;
   result->value = (VOID *) CreateMultifield(i);
   for (ctmp = ctop , i = 1 ; ctmp != NULL ; ctmp = ctmp->nxt , i++)
     {
      SetMFType(result->value,i,SYMBOL);
      SetMFValue(result->value,i,GetDefclassNamePointer((VOID *) ctmp->cls));
     }
   DeleteClassLinks(ctop);
  }

globle VOID SlotTypes(clsptr,sname,result)
  VOID *clsptr;
  char *sname;
  DATA_OBJECT *result;
  {
   register int i,j;
   register SLOT_DESC *sp;
   unsigned typemap;
   int msize;

   if ((sp = SlotInfoSlot(result,(DEFCLASS *) clsptr,sname,"slot-types")) == NULL)
     return;
   if ((sp->constraint != NULL) ? sp->constraint->anyAllowed : CLIPS_TRUE)
     {
      typemap = (unsigned) ~0;
      BitwiseClear(typemap,MULTIFIELD);
      msize = 8;
     }
   else
     {
      typemap = 0;
      msize = 0;
      if (sp->constraint->symbolsAllowed)
        {
         msize++;
         BitwiseSet(typemap,SYMBOL);
        }
      if (sp->constraint->stringsAllowed)
        {
         msize++;
         BitwiseSet(typemap,STRING);
        }
      if (sp->constraint->floatsAllowed)
        {
         msize++;
         BitwiseSet(typemap,FLOAT);
        }
      if (sp->constraint->integersAllowed)
        {
         msize++;
         BitwiseSet(typemap,INTEGER);
        }
      if (sp->constraint->instanceNamesAllowed)
        {
         msize++;
         BitwiseSet(typemap,INSTANCE_NAME);
        }
      if (sp->constraint->instanceAddressesAllowed)
        {
         msize++;
         BitwiseSet(typemap,INSTANCE_ADDRESS);
        }
      if (sp->constraint->externalAddressesAllowed)
        {
         msize++;
         BitwiseSet(typemap,EXTERNAL_ADDRESS);
        }
      if (sp->constraint->factAddressesAllowed)
        {
         msize++;
         BitwiseSet(typemap,FACT_ADDRESS);
        }
     }
   result->end = msize - 1;
   result->value = CreateMultifield(msize);
   i = 1;
   j = 0;
   while (i <= msize)
     {
      if (BitwiseTest(typemap,j))
       {
        SetMFType(result->value,i,SYMBOL);
        SetMFValue(result->value,i,
                   (VOID *) GetDefclassNamePointer((VOID *) PrimitiveClassMap[j]));
        i++;
       }
      j++;
     }
  }
  
globle VOID SlotAllowedValues(clsptr,sname,result)
  VOID *clsptr;
  char *sname;
  DATA_OBJECT *result;
  {
   register int i;
   register SLOT_DESC *sp;
   register EXPRESSION *exp;
   
   if ((sp = SlotInfoSlot(result,(DEFCLASS *) clsptr,sname,"slot-allowed-values")) == NULL)
     return;
   if ((sp->constraint != NULL) ? (sp->constraint->restrictionList == NULL) : CLIPS_TRUE)
     {
      result->type = SYMBOL;
      result->value = CLIPSFalseSymbol;
      return;
     }
   result->end = ExpressionSize(sp->constraint->restrictionList) - 1;
   result->value = CreateMultifield(result->end + 1);
   i = 1;
   exp = sp->constraint->restrictionList; 
   while (exp != NULL)
     {
      SetMFType(result->value,i,exp->type);
      SetMFValue(result->value,i,exp->value);
      exp = exp->nextArg;
      i++;
     }
  }
  
globle VOID SlotRange(clsptr,sname,result)
  VOID *clsptr;
  char *sname;
  DATA_OBJECT *result;
  {
   register SLOT_DESC *sp;

   if ((sp = SlotInfoSlot(result,(DEFCLASS *) clsptr,sname,"slot-range")) == NULL)
     return;
   if ((sp->constraint == NULL) ? CLIPS_FALSE :
       (sp->constraint->anyAllowed || sp->constraint->floatsAllowed || 
        sp->constraint->integersAllowed))
     {
      result->end = 1;
      result->value = CreateMultifield(2);
      SetMFType(result->value,1,sp->constraint->minValue->type);
      SetMFValue(result->value,1,sp->constraint->minValue->value);
      SetMFType(result->value,2,sp->constraint->maxValue->type);
      SetMFValue(result->value,2,sp->constraint->maxValue->value);
     }
   else
     {
      result->type = SYMBOL;
      result->value = CLIPSFalseSymbol;
      return;
     }
  }
  
globle VOID SlotCardinality(clsptr,sname,result)
  VOID *clsptr;
  char *sname;
  DATA_OBJECT *result;
  {
   register SLOT_DESC *sp;

   if ((sp = SlotInfoSlot(result,(DEFCLASS *) clsptr,sname,"slot-cardinality")) == NULL)
     return;
   if (sp->multiple == 0)
     return;
   result->end = 1;
   result->value = CreateMultifield(2);
   if (sp->constraint != NULL)
     {
      SetMFType(result->value,1,sp->constraint->minFields->type);
      SetMFValue(result->value,1,sp->constraint->minFields->value);
      SetMFType(result->value,2,sp->constraint->maxFields->type);
      SetMFValue(result->value,2,sp->constraint->maxFields->value);
     }
   else
     {
      SetMFType(result->value,1,INTEGER);
      SetMFValue(result->value,1,Zero);
      SetMFType(result->value,2,SYMBOL);
      SetMFValue(result->value,2,PositiveInfinity);
     }
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/*****************************************************
  NAME         : SlotInfoSupportFunction
  DESCRIPTION  : Support routine for slot-sources,
                   slot-facets, et. al.
  INPUTS       : 1) Data object buffer
                 2) Name of the CLIPS caller
                 3) Pointer to support function to call
  RETURNS      : Nothing useful
  SIDE EFFECTS : Support function called and data
                  object buffer set
  NOTES        : None
 *****************************************************/
static VOID SlotInfoSupportFunction(result,fnxname,fnx)
  DATA_OBJECT *result;
  char *fnxname;
#if ANSI_COMPILER
  VOID (*fnx)(VOID *,char *,DATA_OBJECT *);
#else
  VOID (*fnx)();
#endif
  {
   SYMBOL_HN *ssym;
   DEFCLASS *cls;
   
   ssym = CheckClassAndSlot(fnxname,&cls);
   if (ssym == NULL)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   (*fnx)((VOID *) cls,ValueToString(ssym),result);
  }

/*****************************************************************
  NAME         : CountSubclasses
  DESCRIPTION  : Counts the number of direct or indirect
                   subclasses for a class
  INPUTS       : 1) Address of class
                 2) Include (1) or exclude (0) indirect subclasses
                 3) Traversal id
  RETURNS      : The number of subclasses
  SIDE EFFECTS : None
  NOTES        : None
 *****************************************************************/
static int CountSubclasses(cls,inhp,tvid)
  DEFCLASS *cls;
  int inhp,tvid;
  {
   register unsigned i,cnt;
   register DEFCLASS *subcls;
   
   for (cnt = 0 , i = 0 ; i < cls->directSubclasses.classCount ; i++)
     {
      subcls = cls->directSubclasses.classArray[i];
      if (TestTraversalID(subcls->traversalRecord,tvid) == 0)
        {
         cnt++;
         SetTraversalID(subcls->traversalRecord,tvid);
         if (inhp && (subcls->directSubclasses.classCount != 0))
           cnt += CountSubclasses(subcls,inhp,tvid);
        }
     }
   return(cnt);
  }
  
/*********************************************************************
  NAME         : StoreSubclasses
  DESCRIPTION  : Stores the names of direct or indirect
                   subclasses for a class in a mutlifield
  INPUTS       : 1) Caller's multifield buffer
                 2) Starting index
                 3) Address of the class
                 4) Include (1) or exclude (0) indirect subclasses
                 5) Traversal id
  RETURNS      : The number of subclass names stored in the multifield
  SIDE EFFECTS : Multifield set with subclass names
  NOTES        : Assumes multifield is big enough to hold subclasses
 *********************************************************************/
static int StoreSubclasses(mfval,si,cls,inhp,tvid)
  VOID *mfval;
  int si;
  DEFCLASS *cls;
  int inhp,tvid;
  {
   register unsigned i,classi;
   register DEFCLASS *subcls;
   
   for (i = si , classi = 0 ; classi < cls->directSubclasses.classCount ; classi++)
     {
      subcls = cls->directSubclasses.classArray[classi];
      if (TestTraversalID(subcls->traversalRecord,tvid) == 0)
        {
         SetTraversalID(subcls->traversalRecord,tvid);
         SetMFType(mfval,i,SYMBOL);
         SetMFValue(mfval,i++,(VOID *) GetDefclassNamePointer((VOID *) subcls));
         if (inhp && (subcls->directSubclasses.classCount != 0))
           i += StoreSubclasses(mfval,(int) i,subcls,inhp,tvid);
        }
     }
   return(i - si);
  }
  
/*********************************************************
  NAME         : SlotInfoSlot
  DESCRIPTION  : Runtime support routine for slot-sources,
                   slot-facets, et. al. which looks up
                   a slot
  INPUTS       : 1) Data object buffer
                 2) Class pointer
                 3) Name-string of slot to find
                 4) The name of the calling function
  RETURNS      : Nothing useful
  SIDE EFFECTS : Support function called and data object
                  buffer initialized
  NOTES        : None
 *********************************************************/
static SLOT_DESC *SlotInfoSlot(result,cls,sname,fnxname)
  DATA_OBJECT *result;
  DEFCLASS *cls;
  char *sname,*fnxname;
  {
   SYMBOL_HN *ssym;
   int i;

   if ((ssym = FindSymbol(sname)) == NULL)
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(result);
      return(NULL);
     }
   i = FindInstanceTemplateSlot(cls,ssym);
   if (i == -1)
     {
      SlotExistError(sname,fnxname);
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(result);
      return(NULL);
     }
   result->type = MULTIFIELD;
   result->begin = 0;
   return(cls->instanceTemplate[i]);
  }

#endif
   
/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
