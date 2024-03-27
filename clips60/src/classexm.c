   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                 CLASS EXAMINATION MODULE            */
   /*******************************************************/

/**************************************************************/
/* Purpose: Class browsing and examination commands           */
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

#if ANSI_COMPILER
#include <string.h>
#endif

#include "argacces.h"
#include "classcom.h"
#include "classfun.h"
#include "classini.h"
#include "insfun.h"
#include "msgcom.h"
#include "msgfun.h"
#include "router.h"
#include "strngrtr.h"

#define _CLASSEXM_SOURCE_
#include "classexm.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
   
/* =========================================
   *****************************************
                MACROS AND TYPES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static int CheckTwoClasses(char *,DEFCLASS **,DEFCLASS **);
static SLOT_DESC *CheckSlotExists(char *,DEFCLASS **,BOOLEAN,BOOLEAN);
static SLOT_DESC *LookupSlot(DEFCLASS *,char *,BOOLEAN);

#if DEBUGGING_FUNCTIONS
static DEFCLASS *CheckClass(char *,char *);
static char *GetClassNameArgument(char *);
static VOID PrintClassBrowse(char *,DEFCLASS *,int);
static VOID DisplaySeparator(char *,char *,int,int);
static VOID DisplaySlotBasicInfo(char *,char *,char *,char *,DEFCLASS *);
static BOOLEAN PrintSlotSources(char *,SYMBOL_HN *,PACKED_CLASS_LINKS *,unsigned,int);
static VOID DisplaySlotConstraintInfo(char *,char *,char *,int,DEFCLASS *);
static char *ConstraintCode(CONSTRAINT_RECORD *,unsigned,unsigned);
#endif

#else

static int CheckTwoClasses();
static SLOT_DESC *CheckSlotExists();
static SLOT_DESC *LookupSlot();

#if DEBUGGING_FUNCTIONS
static DEFCLASS *CheckClass();
static char *GetClassNameArgument();
static VOID PrintClassBrowse();
static VOID DisplaySeparator();
static VOID DisplaySlotBasicInfo();
static BOOLEAN PrintSlotSources();
static VOID DisplaySlotConstraintInfo();
static char *ConstraintCode();
#endif

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
   
#if DEBUGGING_FUNCTIONS

/****************************************************************
  NAME         : BrowseClassesCommand
  DESCRIPTION  : Displays a "graph" of the class hierarchy
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : Syntax : CLIPS> (browse-classes [<class>])
 ****************************************************************/
globle VOID BrowseClassesCommand()
  {
   register DEFCLASS *cls;
   
   if (RtnArgCount() == 0)
      /* ================================================
         Find the OBJECT root class (has no superclasses)
         ================================================ */
      cls = LookupDefclassByMdlOrScope(OBJECT_TYPE_NAME);
   else
     {
      DATA_OBJECT tmp;
      
      if (ArgTypeCheck("browse-classes",1,SYMBOL,&tmp) == CLIPS_FALSE)
        return;
      cls = LookupDefclassByMdlOrScope(DOToString(tmp));
      if (cls == NULL)
        {
         ClassExistError("browse-classes",DOToString(tmp));
         return;
        }
     }
   BrowseClasses(WDISPLAY,(VOID *) cls);
  }
  
/****************************************************************
  NAME         : BrowseClasses
  DESCRIPTION  : Displays a "graph" of the class hierarchy
  INPUTS       : 1) The logical name of the output
                 2) Class pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ****************************************************************/
globle VOID BrowseClasses(logicalName,clsptr)
  char *logicalName;
  VOID *clsptr;
  {
   PrintClassBrowse(logicalName,(DEFCLASS *) clsptr,0);
  }

/****************************************************************
  NAME         : DescribeClassCommand
  DESCRIPTION  : Displays direct superclasses and
                   subclasses and the entire precedence
                   list for a class
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : Syntax : CLIPS> (describe-class <class-name>)
 ****************************************************************/
globle VOID DescribeClassCommand()
  {
   char *cname;
   DEFCLASS *cls;

   cname = GetClassNameArgument("describe-class");   
   if (cname == NULL)
     return;
   cls = CheckClass("describe-class",cname);
   if (cls == NULL)
     return;
   DescribeClass(WDISPLAY,(VOID *) cls);
  }
  
/******************************************************
  NAME         : DescribeClass
  DESCRIPTION  : Displays direct superclasses and
                   subclasses and the entire precedence
                   list for a class
  INPUTS       : 1) The logical name of the output
                 2) Class pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ******************************************************/
globle VOID DescribeClass(logicalName,clsptr)
  char *logicalName;
  VOID *clsptr;
  {
   DEFCLASS *cls;
   char buf[83],
        slotNamePrintFormat[12],
        overrideMessagePrintFormat[12];
   int i,messageBanner,
       slotNameLength,overrideMessageLength,
       maxSlotNameLength,maxOverrideMessageLength;
   
   cls = (DEFCLASS *) clsptr;   
   DisplaySeparator(logicalName,buf,82,'=');
   DisplaySeparator(logicalName,buf,82,'*');
   if (cls->abstract)
     PrintCLIPS(logicalName,"Abstract: direct instances of this class cannot be created.\n\n");
   else
     {
      PrintCLIPS(logicalName,"Concrete: direct instances of this class can be created.\n");
#if INSTANCE_PATTERN_MATCHING
      if (cls->reactive)
        PrintCLIPS(logicalName,"Reactive: direct instances of this class can match defrule patterns.\n\n");
      else
        PrintCLIPS(logicalName,"Non-reactive: direct instances of this class cannot match defrule patterns.\n\n");
#else
      PrintCLIPS(logicalName,"\n");
#endif
     }
   PrintPackedClassLinks(logicalName,"Direct Superclasses:",&cls->directSuperclasses);
   PrintPackedClassLinks(logicalName,"Inheritance Precedence:",&cls->allSuperclasses);
   PrintPackedClassLinks(logicalName,"Direct Subclasses:",&cls->directSubclasses);
   if (cls->instanceTemplate != NULL)
     {
      DisplaySeparator(logicalName,buf,82,'-');
      maxSlotNameLength = 5;
      maxOverrideMessageLength = 8;
      for (i = 0 ; i < cls->instanceSlotCount ; i++)
        {
         slotNameLength = strlen(ValueToString(cls->instanceTemplate[i]->slotName->name));
         if (slotNameLength > maxSlotNameLength)
           maxSlotNameLength = slotNameLength;
         if (cls->instanceTemplate[i]->noWrite == 0)
           {
            overrideMessageLength = 
              strlen(ValueToString(cls->instanceTemplate[i]->overrideMessage));
            if (overrideMessageLength > maxOverrideMessageLength)
              maxOverrideMessageLength = overrideMessageLength;
           }
        }
      if (maxSlotNameLength > 16)
        maxSlotNameLength = 16;
      if (maxOverrideMessageLength > 12)
        maxOverrideMessageLength = 12;
      sprintf(slotNamePrintFormat,"%%-%d.%ds : ",maxSlotNameLength,maxSlotNameLength);
      sprintf(overrideMessagePrintFormat,"%%-%d.%ds ",maxOverrideMessageLength,
                                              maxOverrideMessageLength);
      DisplaySlotBasicInfo(logicalName,slotNamePrintFormat,overrideMessagePrintFormat,buf,cls);
      PrintCLIPS(logicalName,"\nConstraint information for slots:\n\n");
      DisplaySlotConstraintInfo(logicalName,slotNamePrintFormat,buf,82,cls);
     }
   if (cls->handlerCount > 0)
     messageBanner = CLIPS_TRUE;
   else
     {
      messageBanner = CLIPS_FALSE;
      for (i = 1 ; i < cls->allSuperclasses.classCount ; i++)
        if (cls->allSuperclasses.classArray[i]->handlerCount > 0)
          {
           messageBanner = CLIPS_TRUE;
           break;
          }
     }
   if (messageBanner)
     {
      DisplaySeparator(logicalName,buf,82,'-');
      PrintCLIPS(logicalName,"Recognized message-handlers:\n");
      DisplayHandlersInLinks(logicalName,&cls->allSuperclasses,0);
     }
   DisplaySeparator(logicalName,buf,82,'*');
   DisplaySeparator(logicalName,buf,82,'=');
  }

#endif

/**********************************************************
  NAME         : GetCreateAccessorString
  DESCRIPTION  : Gets a string describing which
                 accessors are implicitly created
                 for a slot: R, W, RW or NIL
  INPUTS       : The slot descriptor
  RETURNS      : The string description
  SIDE EFFECTS : None
  NOTES        : Used by (describe-class) and (slot-facets)
 **********************************************************/
globle char *GetCreateAccessorString(vsd)
  VOID *vsd;
  {
   SLOT_DESC *sd = (SLOT_DESC *) vsd;
  
   if (sd->createReadAccessor && sd->createWriteAccessor)
     return("RW");
   if ((sd->createReadAccessor == 0) && (sd->createWriteAccessor == 0))
     return("NIL");
   else
     return(sd->createReadAccessor ? "R" : "W");
  }
  
/************************************************************
  NAME         : GetDefclassModuleCommand
  DESCRIPTION  : Determines to which module a class belongs
  INPUTS       : None
  RETURNS      : The symbolic name of the module
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (defclass-module <class-name>)
 ************************************************************/
globle SYMBOL_HN *GetDefclassModuleCommand()
  {
   return(GetConstructModuleCommand("defclass-module",DefclassConstruct));
  }
  
/*********************************************************************
  NAME         : SuperclassPCommand
  DESCRIPTION  : Determines if a class is a superclass of another
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if class-1 is a superclass of class-2
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (superclassp <class-1> <class-2>)
 *********************************************************************/
globle BOOLEAN SuperclassPCommand()
  {
   DEFCLASS *c1,*c2;
   
   if (CheckTwoClasses("superclassp",&c1,&c2) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   return(SuperclassP((VOID *) c1,(VOID *) c2));
  }
  
/***************************************************
  NAME         : SuperclassP
  DESCRIPTION  : Determines if the first class is
                 a superclass of the other
  INPUTS       : 1) First class
                 2) Second class
  RETURNS      : CLIPS_TRUE if first class is a
                 superclass of the first,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SuperclassP(firstClass,secondClass)
  VOID *firstClass,*secondClass;
  {
   return(HasSuperclass((DEFCLASS *) secondClass,(DEFCLASS *) firstClass));
  }

/*********************************************************************
  NAME         : SubclassPCommand
  DESCRIPTION  : Determines if a class is a subclass of another
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if class-1 is a subclass of class-2
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (subclassp <class-1> <class-2>)
 *********************************************************************/
globle BOOLEAN SubclassPCommand()
  {
   DEFCLASS *c1,*c2;
   
   if (CheckTwoClasses("subclassp",&c1,&c2) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   return(SubclassP((VOID *) c1,(VOID *) c2));
  }

/***************************************************
  NAME         : SubclassP
  DESCRIPTION  : Determines if the first class is
                 a subclass of the other
  INPUTS       : 1) First class
                 2) Second class
  RETURNS      : CLIPS_TRUE if first class is a
                 subclass of the first,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SubclassP(firstClass,secondClass)
  VOID *firstClass,*secondClass;
  {
   return(HasSuperclass((DEFCLASS *) firstClass,(DEFCLASS *) secondClass));
  }

/*********************************************************************
  NAME         : SlotExistPCommand
  DESCRIPTION  : Determines if a slot is present in a class
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the slot exists, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (slot-existp <class> <slot> [inherit])
 *********************************************************************/
globle int SlotExistPCommand()
  {
   DEFCLASS *cls;
   SLOT_DESC *sd;
   int inheritFlag = CLIPS_FALSE;
   DATA_OBJECT dobj;
   
   sd = CheckSlotExists("slot-existp",&cls,CLIPS_FALSE,CLIPS_TRUE);
   if (sd == NULL)
     return(CLIPS_FALSE);
   if (RtnArgCount() == 3)
     {
      if (ArgTypeCheck("slot-existp",3,SYMBOL,&dobj) == CLIPS_FALSE)
        return(CLIPS_FALSE);
      if (strcmp(DOToString(dobj),"inherit") != 0)
        {
         ExpectedTypeError1("slot-existp",3,"keyword \"inherit\"");
         SetEvaluationError(CLIPS_TRUE);
         return(CLIPS_FALSE);
        }
      inheritFlag = CLIPS_TRUE;
     }
   return((sd->cls == cls) ? CLIPS_TRUE : inheritFlag);
  }

/***************************************************
  NAME         : SlotExistP
  DESCRIPTION  : Determines if a slot exists
  INPUTS       : 1) The class
                 2) The slot name
                 3) A flag indicating if the slot
                    can be inherited or not
  RETURNS      : CLIPS_TRUE if slot exists,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SlotExistP(theDefclass,slotName,inheritFlag)
  VOID *theDefclass;
  char *slotName;
  BOOLEAN inheritFlag;
  {
   return((LookupSlot((DEFCLASS *) theDefclass,slotName,inheritFlag) != NULL)
           ? CLIPS_TRUE : CLIPS_FALSE);
  }
  
/************************************************************************************
  NAME         : MessageHandlerExistPCommand
  DESCRIPTION  : Determines if a message-handler is present in a class
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the message header is present, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (message-handler-existp <class> <hnd> [<type>])
 ************************************************************************************/
globle int MessageHandlerExistPCommand()
  {
   DEFCLASS *cls;
   SYMBOL_HN *mname;
   DATA_OBJECT temp;
   unsigned mtype = MPRIMARY;
   
   if (ArgTypeCheck("message-handler-existp",1,SYMBOL,&temp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   cls = LookupDefclassByMdlOrScope(DOToString(temp));
   if (cls == NULL)
     {
      ClassExistError("message-handler-existp",DOToString(temp));
      return(CLIPS_FALSE);
     }
   if (ArgTypeCheck("message-handler-existp",2,SYMBOL,&temp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   mname = (SYMBOL_HN *) GetValue(temp);
   if (RtnArgCount() == 3)
     {
      if (ArgTypeCheck("message-handler-existp",3,SYMBOL,&temp) == CLIPS_FALSE)
        return(CLIPS_FALSE);
      mtype = HandlerType("message-handler-existp",DOToString(temp));
      if (mtype == MERROR)
        {
         SetEvaluationError(CLIPS_TRUE);
         return(CLIPS_FALSE);
        }
     }
   if (FindHandlerByAddress(cls,mname,mtype) != NULL)
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }
  
/**********************************************************************
  NAME         : SlotWritablePCommand
  DESCRIPTION  : Determines if an existing slot can be written to
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the slot is writable, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (slot-writablep <class> <slot>)
 **********************************************************************/
globle BOOLEAN SlotWritablePCommand()
  {
   DEFCLASS *theDefclass;
   SLOT_DESC *sd;
   
   sd = CheckSlotExists("slot-writablep",&theDefclass,CLIPS_TRUE,CLIPS_TRUE);
   if (sd == NULL)
     return(CLIPS_FALSE);
   return(sd->noWrite ? CLIPS_FALSE : CLIPS_TRUE);
  }
  
/***************************************************
  NAME         : SlotWritableP
  DESCRIPTION  : Determines if a slot is writable
  INPUTS       : 1) The class
                 2) The slot name
  RETURNS      : CLIPS_TRUE if slot is writable,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SlotWritableP(theDefclass,slotName)
  VOID *theDefclass;
  char *slotName;
  {
   SLOT_DESC *sd;
   
   if ((sd = LookupSlot((DEFCLASS *) theDefclass,slotName,CLIPS_TRUE)) == NULL)
     return(CLIPS_FALSE);
   return(sd->noWrite ? CLIPS_FALSE : CLIPS_TRUE);
  }
  
/**********************************************************************
  NAME         : SlotInitablePCommand
  DESCRIPTION  : Determines if an existing slot can be initialized
                   via an init message-handler or slot-override
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the slot is writable, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (slot-initablep <class> <slot>)
 **********************************************************************/
globle BOOLEAN SlotInitablePCommand()
  {
   DEFCLASS *theDefclass;
   SLOT_DESC *sd;
   
   sd = CheckSlotExists("slot-initablep",&theDefclass,CLIPS_TRUE,CLIPS_TRUE);
   if (sd == NULL)
     return(CLIPS_FALSE);
   return((sd->noWrite && (sd->initializeOnly == 0)) ? CLIPS_FALSE : CLIPS_TRUE);
  }
  
/***************************************************
  NAME         : SlotInitableP
  DESCRIPTION  : Determines if a slot is initable
  INPUTS       : 1) The class
                 2) The slot name
  RETURNS      : CLIPS_TRUE if slot is initable,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SlotInitableP(theDefclass,slotName)
  VOID *theDefclass;
  char *slotName;
  {
   SLOT_DESC *sd;
   
   if ((sd = LookupSlot((DEFCLASS *) theDefclass,slotName,CLIPS_TRUE)) == NULL)
     return(CLIPS_FALSE);
   return((sd->noWrite && (sd->initializeOnly == 0)) ? CLIPS_FALSE : CLIPS_TRUE);
  }
  
/**********************************************************************
  NAME         : SlotPublicPCommand
  DESCRIPTION  : Determines if an existing slot is publicly visible
                   for direct reference by subclasses
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the slot is public, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (slot-publicp <class> <slot>)
 **********************************************************************/
globle BOOLEAN SlotPublicPCommand()
  {
   DEFCLASS *theDefclass;
   SLOT_DESC *sd;
   
   sd = CheckSlotExists("slot-publicp",&theDefclass,CLIPS_TRUE,CLIPS_FALSE);
   if (sd == NULL)
     return(CLIPS_FALSE);
   return(sd->publicVisibility ? CLIPS_TRUE : CLIPS_FALSE);
  }
    
/***************************************************
  NAME         : SlotPublicP
  DESCRIPTION  : Determines if a slot is public
  INPUTS       : 1) The class
                 2) The slot name
  RETURNS      : CLIPS_TRUE if slot is public,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SlotPublicP(theDefclass,slotName)
  VOID *theDefclass;
  char *slotName;
  {
   SLOT_DESC *sd;
   
   if ((sd = LookupSlot((DEFCLASS *) theDefclass,slotName,CLIPS_FALSE)) == NULL)
     return(CLIPS_FALSE);
   return(sd->publicVisibility ? CLIPS_TRUE : CLIPS_FALSE);
  }
  
/**********************************************************************
  NAME         : SlotDirectAccessPCommand
  DESCRIPTION  : Determines if an existing slot can be directly
                   referenced by the class - i.e., if the slot is
                   private, is the slot defined in the class
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the slot is private,
                    CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (slot-direct-accessp <class> <slot>)
 **********************************************************************/
globle BOOLEAN SlotDirectAccessPCommand()
  {
   DEFCLASS *theDefclass;
   SLOT_DESC *sd;
   
   sd = CheckSlotExists("slot-direct-accessp",&theDefclass,CLIPS_TRUE,CLIPS_TRUE);
   if (sd == NULL)
     return(CLIPS_FALSE);
   return((sd->publicVisibility || (sd->cls == theDefclass)) ? CLIPS_TRUE : CLIPS_FALSE);
  }
  
/***************************************************
  NAME         : SlotDirectAccessP
  DESCRIPTION  : Determines if a slot is directly
                 accessible from message-handlers
                 on class
  INPUTS       : 1) The class
                 2) The slot name
  RETURNS      : CLIPS_TRUE if slot is directly
                 accessible, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN SlotDirectAccessP(theDefclass,slotName)
  VOID *theDefclass;
  char *slotName;
  {
   SLOT_DESC *sd;
   
   if ((sd = LookupSlot((DEFCLASS *) theDefclass,slotName,CLIPS_TRUE)) == NULL)
     return(CLIPS_FALSE);
   return((sd->publicVisibility || (sd->cls == (DEFCLASS *) theDefclass)) ? 
           CLIPS_TRUE : CLIPS_FALSE);
  }

/********************************************************
  NAME         : ClassExistPCommand
  DESCRIPTION  : Determines if a class exists
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if class exists, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (class-existp <arg>)
 ********************************************************/
globle BOOLEAN ClassExistPCommand()
  {
   DATA_OBJECT temp;

   if (ArgTypeCheck("class-existp",1,SYMBOL,&temp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   return((LookupDefclassByMdlOrScope(DOToString(temp)) != NULL) ? CLIPS_TRUE : CLIPS_FALSE);
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/******************************************************
  NAME         : CheckTwoClasses
  DESCRIPTION  : Checks for exactly two class arguments
                    for a CLIPS function
  INPUTS       : 1) The function name
                 2) Caller's buffer for first class
                 3) Caller's buffer for second class
  RETURNS      : CLIPS_TRUE if both found, CLIPS_FALSE otherwise
  SIDE EFFECTS : Caller's buffers set
  NOTES        : Assumes exactly 2 arguments
 ******************************************************/
static int CheckTwoClasses(func,c1,c2)
  char *func;
  DEFCLASS **c1,**c2;
  {
   DATA_OBJECT temp;

   if (ArgTypeCheck(func,1,SYMBOL,&temp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   *c1 = LookupDefclassByMdlOrScope(DOToString(temp));
   if (*c1 == NULL)
     {
      ClassExistError(func,ValueToString(temp.value));
      return(CLIPS_FALSE);
     }
   if (ArgTypeCheck(func,2,SYMBOL,&temp) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   *c2 = LookupDefclassByMdlOrScope(DOToString(temp));
   if (*c2 == NULL)
     {
      ClassExistError(func,ValueToString(temp.value));
      return(CLIPS_FALSE);
     }
   return(CLIPS_TRUE);
  }
    
/***************************************************
  NAME         : CheckSlotExists
  DESCRIPTION  : Checks first two arguments of
                 a function for a valid class
                 and (inherited) slot
  INPUTS       : 1) The name of the function
                 2) A buffer to hold the found class
                 3) A flag indicating whether the
                    non-existence of the slot should
                    be an error
                 4) A flag indicating if the slot
                    can be inherited or not
  RETURNS      : NULL if slot not found, slot
                 descriptor otherwise
  SIDE EFFECTS : Class buffer set if no errors,
                 NULL on errors
  NOTES        : None
 ***************************************************/
static SLOT_DESC *CheckSlotExists(func,classBuffer,existsErrorFlag,inheritFlag)
  char *func;
  DEFCLASS **classBuffer;
  BOOLEAN existsErrorFlag,inheritFlag;
  {
   SYMBOL_HN *ssym;
   int slotIndex;
   SLOT_DESC *sd;
   
   ssym = CheckClassAndSlot(func,classBuffer);
   if (ssym == NULL)
     return(NULL);
   slotIndex = FindInstanceTemplateSlot(*classBuffer,ssym);
   if (slotIndex == -1)
     {
      if (existsErrorFlag)
        {
         SlotExistError(ValueToString(ssym),func);
         SetEvaluationError(CLIPS_TRUE);
        }
      return(NULL);
     }
   sd = (*classBuffer)->instanceTemplate[slotIndex];
   if ((sd->cls == *classBuffer) || inheritFlag)
     return(sd);
   PrintErrorID("CLASSEXM",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Inherited slot ");
   PrintCLIPS(WERROR,ValueToString(ssym));
   PrintCLIPS(WERROR," from class ");
   PrintClassName(WERROR,sd->cls,CLIPS_FALSE);
   PrintCLIPS(WERROR," is not valid for function ");
   PrintCLIPS(WERROR,func);
   PrintCLIPS(WERROR,"\n");
   SetEvaluationError(CLIPS_TRUE);
   return(NULL);
  }
  
/***************************************************
  NAME         : LookupSlot
  DESCRIPTION  : Finds a slot in a class
  INPUTS       : 1) The class
                 2) The slot name
                 3) A flag indicating if inherited
                    slots are OK or not
  RETURNS      : The slot descriptor address, or
                 NULL if not found
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static SLOT_DESC *LookupSlot(theDefclass,slotName,inheritFlag)
  DEFCLASS *theDefclass;
  char *slotName;
  BOOLEAN inheritFlag;
  {
   SYMBOL_HN *slotSymbol;
   int slotIndex;
   SLOT_DESC *sd;
   
   slotSymbol = FindSymbol(slotName);
   if (slotSymbol == NULL)
     return(NULL);
   slotIndex = FindInstanceTemplateSlot(theDefclass,slotSymbol);
   if (slotIndex == -1)
     return(NULL);
   sd = theDefclass->instanceTemplate[slotIndex];
   if ((sd->cls != theDefclass) && (inheritFlag == CLIPS_FALSE))
     return(NULL);
   return(sd);
  }
    
#if DEBUGGING_FUNCTIONS

/*****************************************************
  NAME         : CheckClass
  DESCRIPTION  : Used for to check class name for
                 CLIPS class accessor functions such
                 as ppdefclass and undefclass
  INPUTS       : 1) The name of the CLIPS function
                 2) Name of the class
  RETURNS      : The class address,
                   or NULL if ther was an error
  SIDE EFFECTS : None
  NOTES        : None
 ******************************************************/
static DEFCLASS *CheckClass(func,cname)
  char *func,*cname;
  {
   DEFCLASS *cls;

   cls = LookupDefclassByMdlOrScope(cname);
   if (cls == NULL)
     ClassExistError(func,cname);
   return(cls);
  }

/*********************************************************
  NAME         : GetClassNameArgument
  DESCRIPTION  : Gets a class name-string
  INPUTS       : Calling function name
  RETURNS      : Class name (NULL on errors)
  SIDE EFFECTS : None
  NOTES        : Assumes only 1 argument
 *********************************************************/
static char *GetClassNameArgument(fname)
  char *fname;
  {
   DATA_OBJECT temp;

   if (ArgTypeCheck(fname,1,SYMBOL,&temp) == CLIPS_FALSE)
     return(NULL);
   return(DOToString(temp));
  }

/****************************************************************
  NAME         : PrintClassBrowse
  DESCRIPTION  : Displays a "graph" of class and subclasses
  INPUTS       : 1) The logical name of the output
                 2) The class address
                 3) The depth of the graph
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ****************************************************************/
static VOID PrintClassBrowse(logicalName,cls,depth)
  char *logicalName;
  DEFCLASS *cls;
  int depth;
  {
   register unsigned i;
   
   for (i = 0 ; i < depth ; i++)
     PrintCLIPS(logicalName,"  ");
   PrintCLIPS(logicalName,GetDefclassName((VOID *) cls));
   if (cls->directSuperclasses.classCount > 1)
     PrintCLIPS(logicalName," *");
   PrintCLIPS(logicalName,"\n");
   for (i = 0 ;i < cls->directSubclasses.classCount ; i++)
     PrintClassBrowse(logicalName,cls->directSubclasses.classArray[i],depth+1);
  }

/*********************************************************
  NAME         : DisplaySeparator
  DESCRIPTION  : Prints a separator line for DescribeClass
  INPUTS       : 1) The logical name of the output
                 2) The buffer to use for the line
                 3) The buffer size
                 4) The character to use
  RETURNS      : Nothing useful
  SIDE EFFECTS : Buffer overwritten and displayed
  NOTES        : None
 *********************************************************/
static VOID DisplaySeparator(logicalName,buf,maxlen,sepchar)
  char *logicalName,*buf;
  int maxlen;
  int sepchar;
  {
   register int i;
   
   for (i = 0 ; i < maxlen-2 ; i++)
     buf[i] = (char) sepchar;
   buf[i++] = '\n';
   buf[i] = '\0';
   PrintCLIPS(logicalName,buf);
  }
  
/*************************************************************
  NAME         : DisplaySlotBasicInfo
  DESCRIPTION  : Displays a table summary of basic
                  facets for the slots of a class
                  including: 
                  single/multiple
                  default/no-default/default-dynamic
                  inherit/no-inherit
                  read-write/initialize-only/read-only
                  local/shared
                  composite/exclusive
                  reactive/non-reactive
                  public/private
                  create-accessor read/write
                  override-message
                  
                  The function also displays the source
                  class(es) for the facets
  INPUTS       : 1) The logical name of the output
                 2) A format string for use in sprintf
                    (for printing slot names)
                 3) A format string for use in sprintf
                    (for printing slot override message names)
                 4) A buffer to store the display in
                 5) A pointer to the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Buffer written to and displayed
  NOTES        : None
 *************************************************************/
static VOID DisplaySlotBasicInfo(logicalName,slotNamePrintFormat,
                                 overrideMessagePrintFormat,buf,cls)
  char *logicalName,*slotNamePrintFormat,*overrideMessagePrintFormat,*buf;
  DEFCLASS *cls;
  {
   register int i;
   SLOT_DESC *sp;
   char *createString;
   
   sprintf(buf,slotNamePrintFormat,"SLOTS");
#if INSTANCE_PATTERN_MATCHING
   strcat(buf,"FLD DEF PRP ACC STO MCH SRC VIS CRT ");
#else
   strcat(buf,"FLD DEF PRP ACC STO SRC VIS CRT ");
#endif
   PrintCLIPS(logicalName,buf);
   sprintf(buf,overrideMessagePrintFormat,"OVRD-MSG");
   PrintCLIPS(logicalName,buf);
   PrintCLIPS(logicalName,"SOURCE(S)\n");
   for (i = 0 ; i < cls->instanceSlotCount ; i++)
     {
      sp = cls->instanceTemplate[i];
      sprintf(buf,slotNamePrintFormat,ValueToString(sp->slotName->name));
      strcat(buf,sp->multiple ? "MLT " : "SGL ");
      if (sp->noDefault)
        strcat(buf,"NIL ");
      else
        strcat(buf,sp->dynamicDefault ? "DYN " : "STC ");
      strcat(buf,sp->noInherit ? "NIL " : "INH ");
      if (sp->initializeOnly)
        strcat(buf,"INT ");
      else if (sp->noWrite)
        strcat(buf," R  ");
      else
        strcat(buf,"RW  ");
      strcat(buf,sp->shared ? "SHR " : "LCL ");
#if INSTANCE_PATTERN_MATCHING
      strcat(buf,sp->reactive ? "RCT " : "NIL ");
#endif
      strcat(buf,sp->composite ? "CMP " : "EXC ");
      strcat(buf,sp->publicVisibility ? "PUB " : "PRV ");
      createString = GetCreateAccessorString(sp);
      if (createString[1] == '\0')
        strcat(buf," ");
      strcat(buf,createString);
      if ((createString[1] == '\0') ? CLIPS_TRUE : (createString[2] == '\0'))
        strcat(buf," ");
      strcat(buf," ");
      PrintCLIPS(logicalName,buf);
      sprintf(buf,overrideMessagePrintFormat,
              sp->noWrite ? "NIL" : ValueToString(sp->overrideMessage));
      PrintCLIPS(logicalName,buf);
      PrintSlotSources(logicalName,sp->slotName->name,&sp->cls->allSuperclasses,0,CLIPS_TRUE);
      PrintCLIPS(logicalName,"\n");
     }
  }

/***************************************************
  NAME         : PrintSlotSources
  DESCRIPTION  : Displays a list of source classes
                   for a composite class (in order
                   of most general to specific)
  INPUTS       : 1) The logical name of the output
                 2) The name of the slot
                 3) The precedence list of the class
                    of the slot (the source class
                    shold be first in the list)
                 4) The index into the packed
                    links array
                 5) Flag indicating whether to
                    disregard noniherit facet
  RETURNS      : CLIPS_TRUE if a class is printed, CLIPS_FALSE
                 otherwise
  SIDE EFFECTS : Recursively prints out appropriate
                 memebers from list in reverse order
  NOTES        : None
 ***************************************************/
static BOOLEAN PrintSlotSources(logicalName,sname,sprec,index,inhp)
  char *logicalName;
  SYMBOL_HN *sname;
  PACKED_CLASS_LINKS *sprec;
  unsigned index;
  int inhp;
  {
   SLOT_DESC *csp;
   
   if (index == sprec->classCount)
     return(CLIPS_FALSE);
   csp = FindClassSlot(sprec->classArray[index],sname);
   if ((csp != NULL) ? ((csp->noInherit == 0) || inhp) : CLIPS_FALSE)
     {
      if (csp->composite)
        {
         if (PrintSlotSources(logicalName,sname,sprec,index+1,CLIPS_FALSE))
           PrintCLIPS(logicalName," ");
        }
      PrintClassName(logicalName,sprec->classArray[index],CLIPS_FALSE);
      return(CLIPS_TRUE);
     }
   else
     return(PrintSlotSources(logicalName,sname,sprec,index+1,CLIPS_FALSE));
  }
        
/*********************************************************
  NAME         : DisplaySlotConstraintInfo
  DESCRIPTION  : Displays a table summary of type-checking
                  facets for the slots of a class
                  including: 
                  type
                  allowed-symbols
                  allowed-integers
                  allowed-floats
                  allowed-values
                  allowed-instance-names
                  range
                  min-number-of-elements
                  max-number-of-elements
                  
                  The function also displays the source
                  class(es) for the facets
  INPUTS       : 1) A format string for use in sprintf
                 2) A buffer to store the display in
                 3) Maximum buffer size
                 4) A pointer to the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Buffer written to and displayed
  NOTES        : None
 *********************************************************/
static VOID DisplaySlotConstraintInfo(logicalName,slotNamePrintFormat,buf,maxlen,cls)
  char *logicalName,*slotNamePrintFormat,*buf;
  int maxlen;
  DEFCLASS *cls;
  {
   register int i;
   CONSTRAINT_RECORD *cr;
   char *strdest = "***describe-class***";

   sprintf(buf,slotNamePrintFormat,"SLOTS");
   strcat(buf,"SYM STR INN INA EXA FTA INT FLT\n");
   PrintCLIPS(logicalName,buf);
   for (i = 0 ; i < cls->instanceSlotCount ; i++)
     {
      cr = cls->instanceTemplate[i]->constraint;
      sprintf(buf,slotNamePrintFormat,ValueToString(cls->instanceTemplate[i]->slotName->name));
      if (cr != NULL)
        {
         strcat(buf,ConstraintCode(cr,(unsigned) cr->symbolsAllowed,
                                      (unsigned) cr->symbolRestriction));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->stringsAllowed,
                                      (unsigned) cr->stringRestriction));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->instanceNamesAllowed,
                                      (unsigned) cr->instanceNameRestriction));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->instanceAddressesAllowed,0));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->externalAddressesAllowed,0));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->factAddressesAllowed,0));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->integersAllowed,
                                      (unsigned) cr->integerRestriction));
         strcat(buf,ConstraintCode(cr,(unsigned) cr->floatsAllowed,
                                      (unsigned) cr->floatRestriction));
         OpenStringDestination(strdest,buf + strlen(buf),maxlen - strlen(buf) - 1);
         if (cr->integersAllowed || cr->floatsAllowed || cr->anyAllowed)
           {
            PrintCLIPS(strdest,"RNG:[");
            PrintExpression(strdest,cr->minValue);
            PrintCLIPS(strdest,"..");
            PrintExpression(strdest,cr->maxValue);
            PrintCLIPS(strdest,"] ");
           }
         if (cls->instanceTemplate[i]->multiple)
           {
            PrintCLIPS(strdest,"CRD:[");
            PrintExpression(strdest,cr->minFields);
            PrintCLIPS(strdest,"..");
            PrintExpression(strdest,cr->maxFields);
            PrintCLIPS(strdest,"]");
           }
        }
      else
        {
         OpenStringDestination(strdest,buf,maxlen);
         PrintCLIPS(strdest," +   +   +   +   +   +   +   +  RNG:[-oo..+oo]");
         if (cls->instanceTemplate[i]->multiple)
           PrintCLIPS(strdest," CRD:[0..+oo]");
        }
      PrintCLIPS(strdest,"\n");
      CloseStringDestination(strdest);
      PrintCLIPS(logicalName,buf);
     }
  }
  
/******************************************************
  NAME         : ConstraintCode
  DESCRIPTION  : Gives a string code representing the
                 type of constraint
  INPUTS       : 1) The constraint record
                 2) Allowed Flag
                 3) Restricted Values flag
  RETURNS      : "    " for type not allowed
                 " +  " for any value of type allowed
                 " #  " for some values of type allowed
  SIDE EFFECTS : None
  NOTES        : Used by DisplaySlotConstraintInfo
 ******************************************************/
//Modify Jesus Savage 21-3-2023
static char *ConstraintCode(CONSTRAINT_RECORD *cr, unsigned allow, unsigned restri)
//static char *ConstraintCode(cr,allow,restrict)
  {
 /* CONSTRAINT_RECORD *cr;
  unsigned allow,restrict;
  {
   if (allow || cr->anyAllowed)
     return((restrict || cr->anyRestriction) ? " #  " : " +  ");*/
   return("    ");
  }
  

#endif
  
#endif
   
/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
