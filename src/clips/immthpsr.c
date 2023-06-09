   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*         IMPLICIT SYSTEM METHODS PARSING MODULE      */
   /*******************************************************/

/*************************************************************/
/* Purpose: Parsing routines for Implicit System Methods     */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/
   
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if DEFGENERIC_CONSTRUCT && (! BLOAD_ONLY) && (! RUN_TIME)

#if ANSI_COMPILER
#include <stdlib.h>
#endif

#if OBJECT_SYSTEM
#include "classcom.h"
#include "classfun.h"
#endif

#include "clipsmem.h"
#include "cstrnutl.h"
#include "extnfunc.h"
#include "genrcpsr.h"
#include "prccode.h"

#define _IMMTHPSR_SOURCE_
#include "immthpsr.h"

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

static VOID FormMethodsFromRestrictions(DEFGENERIC *,char *,EXPRESSION *);
static RESTRICTION *ParseRestrictionType(int);
static EXPRESSION *GenTypeExpression(EXPRESSION *,int,int,char *);

#else

static VOID FormMethodsFromRestrictions();
static RESTRICTION *ParseRestrictionType();
static EXPRESSION *GenTypeExpression();

#endif
      
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
   
/********************************************************
  NAME         : AddImplicitMethods
  DESCRIPTION  : Adds a method(s) for a generic function
                   for an overloaded system function
  INPUTS       : A pointer to a gneeric function
  RETURNS      : Nothing useful
  SIDE EFFECTS : Method added
  NOTES        : Method marked as system
                 Assumes no other methods already present
 ********************************************************/
globle VOID AddImplicitMethods(gfunc)
  DEFGENERIC *gfunc;
  {
   struct FunctionDefinition *sysfunc;
   EXPRESSION action;
   
   sysfunc = FindFunction(ValueToString(gfunc->header.name));
   if (sysfunc == NULL)
     return;
   action.type = FCALL;
   action.value = (VOID *) sysfunc;
   action.nextArg = NULL;
   action.argList = NULL;
   FormMethodsFromRestrictions(gfunc,sysfunc->restrictions,&action);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
  
/**********************************************************************
  NAME         : FormMethodsFromRestrictions
  DESCRIPTION  : Uses restriction string given in DefineFunction2()
                   for system function to create an equivalent method
  INPUTS       : 1) The generic function for the new methods
                 2) System function restriction string
                    (see DefineFunction2() last argument)
                 3) The actions to attach to a new method(s)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Implicit method(s) created
  NOTES        : None
 **********************************************************************/
static VOID FormMethodsFromRestrictions(gfunc,rstring,actions)
  DEFGENERIC *gfunc;
  char *rstring;
  EXPRESSION *actions;
  {
   DEFMETHOD *meth;
   EXPRESSION *plist,*tmp,*bot,*svBot;
   RESTRICTION *rptr;
   char theChar[2],defaultc;
   int min,max,mposn,needMinimumMethod;
   register int i,j;

   /* ===================================
      The system function will accept any
      number of any type of arguments
      =================================== */
   if (rstring == NULL)
     {
      tmp = get_struct(expr);
      rptr = get_struct(restriction);
      PackRestrictionTypes(rptr,NULL);
      rptr->query = NULL;
      tmp->argList = (EXPRESSION *) rptr;
      tmp->nextArg = NULL;
      meth = AddMethod(gfunc,NULL,0,0,tmp,1,0,CLIPSTrueSymbol,
                       PackExpression(actions),NULL,CLIPS_FALSE);
      meth->system = 1;
      DeleteTempRestricts(tmp);
      return;
     }
     
   /* ==============================
      Extract the range of arguments
      from the restriction string
      ============================== */
   theChar[1] = '\0';
   if (rstring[0] == '*')
     min = 0;
   else
     {
      theChar[0] = rstring[0];
      min = atoi(theChar);
     }
   if (rstring[1] == '*')
     max = -1;
   else
     {
      theChar[0] = rstring[1];
      max = atoi(theChar);
     }
   if (rstring[2] != '\0')
     {
      defaultc = rstring[2];
      j = 3;
     }
   else
     {
      defaultc = 'u';
      j= 2;
     }
     
   /* ================================================
      Form a list of method restrictions corresponding
      to the minimum number of arguments
      ================================================ */
   plist = bot = NULL;
   for (i = 0 ; i < min ; i++)
     {
      theChar[0] = (rstring[j] != '\0') ? rstring[j++] : defaultc;
      rptr = ParseRestrictionType((int) theChar[0]);
      tmp = get_struct(expr);
      tmp->argList = (EXPRESSION *) rptr;
      tmp->nextArg = NULL;
      if (plist == NULL)
        plist = tmp;
      else
        bot->nextArg = tmp;
      bot = tmp;
     }
     
   /* ===============================
      Remember where restrictions end
      for minimum number of arguments
      =============================== */
   svBot = bot;
   needMinimumMethod = CLIPS_TRUE;
   
   /* =======================================================
      Attach one or more new methods to correspond
      to the possible variations of the extra arguments
      
      Add a separate method for each specified extra argument
      ======================================================= */
   i = 0;
   while (rstring[j] != '\0')
     {
      if ((rstring[j+1] == '\0') && ((min + i + 1) == max))
        {
         defaultc = rstring[j];
         break;
        }
      rptr = ParseRestrictionType((int) rstring[j]);
      tmp = get_struct(expr);
      tmp->argList = (EXPRESSION *) rptr;
      tmp->nextArg = NULL;
      if (plist == NULL)
        plist = tmp;
      else
        bot->nextArg = tmp;
      bot = tmp;
      i++;
      j++;
      if ((rstring[j] != '\0') || ((min + i) == max))
        {
         FindMethodByRestrictions(gfunc,plist,min + i,NULL,&mposn);
         meth = AddMethod(gfunc,NULL,mposn,0,plist,min + i,0,NULL,
                          PackExpression(actions),NULL,CLIPS_TRUE);
         meth->system = 1;
        }
     }
     
   /* ==============================================
      Add a method to account for wildcard arguments
      and attach a query in case there is a limit
      ============================================== */
   if ((min + i) != max)
     {
      /* ================================================
         If a wildcard is present immediately after the
         minimum number of args - then the minimum case
         will already be handled by this method. We don't
         need to add an extra method for that case
         ================================================ */
      if (i == 0)
        needMinimumMethod = CLIPS_FALSE;
        
      rptr = ParseRestrictionType((int) defaultc);
      if (max != -1)
        {
         rptr->query = GenConstant(FCALL,(VOID *) FindFunction("<="));
         rptr->query->argList = GenConstant(FCALL,(VOID *) FindFunction("length$"));
         rptr->query->argList->argList = GenProcWildcardReference(min + i + 1);
         rptr->query->argList->nextArg = 
               GenConstant(INTEGER,(VOID *) AddLong((long) (max - min - i)));
        }
      tmp = get_struct(expr);
      tmp->argList = (EXPRESSION *) rptr;
      tmp->nextArg = NULL;
      if (plist == NULL)
        plist = tmp;
      else
        bot->nextArg = tmp;
      bot = tmp;
      FindMethodByRestrictions(gfunc,plist,min + i + 1,CLIPSTrueSymbol,&mposn);
      meth = AddMethod(gfunc,NULL,mposn,0,plist,min + i + 1,0,CLIPSTrueSymbol,
                       PackExpression(actions),NULL,CLIPS_FALSE);
      meth->system = 1;
     }

   /* ===================================================
      When extra methods had to be added because of
      different restrictions on the optional arguments OR
      the system function accepts a fixed number of args,
      we must add a specific method for the minimum case.
      Otherwise, the method with the wildcard covers it.
      =================================================== */
   if (needMinimumMethod)
     {
      if (svBot != NULL)
        {
         bot = svBot->nextArg;
         svBot->nextArg = NULL;
         DeleteTempRestricts(bot);
        }
      FindMethodByRestrictions(gfunc,plist,min,NULL,&mposn);
      meth = AddMethod(gfunc,NULL,mposn,0,plist,min,0,NULL,
                       PackExpression(actions),NULL,CLIPS_TRUE);
      meth->system = 1;
     }
   DeleteTempRestricts(plist);
  }

/*******************************************************************
  NAME         : ParseRestrictionType
  DESCRIPTION  : Takes a string of type character codes (as given in
                 DefineFunction2()) and converts it into a method
                 restriction structure
  INPUTS       : The type character code
  RETURNS      : The restriction
  SIDE EFFECTS : Restriction allocated
  NOTES        : None
 *******************************************************************/
static RESTRICTION *ParseRestrictionType(code)
  int code;
  {
   RESTRICTION *rptr;
   CONSTRAINT_RECORD *rv;
   EXPRESSION *types = NULL;
   
   rptr = get_struct(restriction);
   rptr->query = NULL;
   rv = ArgumentTypeToConstraintRecord(code);
   if (rv->anyAllowed == CLIPS_FALSE)
     {
      if (rv->symbolsAllowed && rv->stringsAllowed)
        types = GenTypeExpression(types,LEXEME_TYPE_CODE,-1,LEXEME_TYPE_NAME);
      else if (rv->symbolsAllowed)
        types = GenTypeExpression(types,SYMBOL,SYMBOL,NULL);
      else if (rv->stringsAllowed)
        types = GenTypeExpression(types,STRING,STRING,NULL);

      if (rv->floatsAllowed && rv->integersAllowed)
        types = GenTypeExpression(types,NUMBER_TYPE_CODE,-1,NUMBER_TYPE_NAME);
      else if (rv->integersAllowed)
        types = GenTypeExpression(types,INTEGER,INTEGER,NULL);
      else if (rv->floatsAllowed)
        types = GenTypeExpression(types,FLOAT,FLOAT,NULL);

      if (rv->instanceNamesAllowed && rv->instanceAddressesAllowed)
        types = GenTypeExpression(types,INSTANCE_TYPE_CODE,-1,INSTANCE_TYPE_NAME);
      else if (rv->instanceNamesAllowed)
        types = GenTypeExpression(types,INSTANCE_NAME,INSTANCE_NAME,NULL);
      else if (rv->instanceAddressesAllowed)
        types = GenTypeExpression(types,INSTANCE_ADDRESS,INSTANCE_ADDRESS,NULL);

      if (rv->externalAddressesAllowed && rv->instanceAddressesAllowed &&
          rv->factAddressesAllowed)
        types = GenTypeExpression(types,ADDRESS_TYPE_CODE,-1,ADDRESS_TYPE_NAME);
      else
        {
         if (rv->externalAddressesAllowed)
           types = GenTypeExpression(types,EXTERNAL_ADDRESS,EXTERNAL_ADDRESS,NULL);
         if (rv->instanceAddressesAllowed && (rv->instanceNamesAllowed == 0))
           types = GenTypeExpression(types,INSTANCE_ADDRESS,INSTANCE_ADDRESS,NULL);
         if (rv->factAddressesAllowed)
           types = GenTypeExpression(types,FACT_ADDRESS,FACT_ADDRESS,NULL);
        }

      if (rv->multifieldsAllowed)
        types = GenTypeExpression(types,MULTIFIELD,MULTIFIELD,NULL);
     }
   RemoveConstraint(rv);
   PackRestrictionTypes(rptr,types);
   return(rptr);
  }

/***************************************************
  NAME         : GenTypeExpression
  DESCRIPTION  : Creates an expression corresponding
                 to the type specified and adds it
                 to the front of a temporary type
                 list for a method restriction
  INPUTS       : 1) The top of the current type list
                 2) The type code when COOL is
                    not installed
                 3) The primitive type (-1 if not
                    a primitive type)
                 4) The name of the COOL class if
                    it is not a primitive type
  RETURNS      : The new top of the types list
  SIDE EFFECTS : Type node allocated and attached
  NOTES        : Restriction types in a non-COOL
                 environment are the type codes
                 given in CONSTANT.H.  In a COOL
                 environment, they are pointers
                 to classes
 ***************************************************/
#if IBM_TBC
#pragma argsused
#endif
static EXPRESSION *GenTypeExpression(top,nonCOOLCode,primitiveCode,COOLName)
  EXPRESSION *top;
  int nonCOOLCode,primitiveCode;
  char *COOLName;
  {
#if OBJECT_SYSTEM
#if MAC_MPW
#pragma unused(nonCOOLCode)
#endif
#endif
   EXPRESSION *tmp;
   
#if OBJECT_SYSTEM
   if (primitiveCode != -1)
     tmp = GenConstant(0,(VOID *) PrimitiveClassMap[primitiveCode]);
   else
     tmp = GenConstant(0,(VOID *) LookupDefclassByMdlOrScope(COOLName));
#else
   tmp = GenConstant(0,AddLong((long) nonCOOLCode));
#endif
   tmp->nextArg = top;
   return(tmp);
  }
  
#endif /* DEFGENERIC_CONSTRUCT && (! BLOAD_ONLY) && (! RUN_TIME) */

/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
