   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*              MESSAGE-HANDLER PARSER FUNCTIONS       */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
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

#if OBJECT_SYSTEM && (! BLOAD_ONLY) && (! RUN_TIME)

#if ANSI_COMPILER
#include <string.h>
#endif

#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "constrct.h"
#include "cstrcpsr.h"
#include "cstrnchk.h"
#include "exprnpsr.h"
#include "insfun.h"
#include "msgfun.h"
#include "pprint.h"
#include "prccode.h"
#include "router.h"
#include "scanner.h"
#include "strngrtr.h"

#define _MSGPSR_SOURCE_
#include "msgpsr.h"

extern struct token ObjectParseToken;

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define SELF_LEN         4
#define SELF_SLOT_REF   ':'

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

static BOOLEAN IsParameterSlotReference(char *);
static int SlotReferenceVar(EXPRESSION *,VOID *);
static int BindSlotReference(EXPRESSION *,VOID *);
static SLOT_DESC *CheckSlotReference(DEFCLASS *,int,VOID *,BOOLEAN,EXPRESSION *);
static VOID GenHandlerSlotReference(EXPRESSION *,int,SLOT_DESC *);

#else

static BOOLEAN IsParameterSlotReference();
static int SlotReferenceVar();
static int BindSlotReference();
static SLOT_DESC *CheckSlotReference();
static VOID GenHandlerSlotReference();

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle SYMBOL_HN *SELF_SYMBOL = NULL;

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
   
/***********************************************************************
  NAME         : ParseDefmessageHandler
  DESCRIPTION  : Parses a message-handler for a class of objects
  INPUTS       : The logical name of the input source
  RETURNS      : CLIPS_FALSE if successful parse, CLIPS_TRUE otherwise
  SIDE EFFECTS : Handler allocated and inserted into class
  NOTES        : CLIPS Syntax:
  
                 (defmessage-handler <class> <name> [<type>] [<comment>]
                    (<params>)
                    <action>*)
                    
                 <params> ::= <var>* | <var>* $?<name>
 ***********************************************************************/
globle int ParseDefmessageHandler(readSource)
  char *readSource;
  {
   DEFCLASS *cls;
   SYMBOL_HN *cname,*mname,*wildcard;
   unsigned mtype = MPRIMARY;
   int min,max,error,lvars;
   EXPRESSION *hndParams,*actions;
   HANDLER *hnd;
   
   SetPPBufferStatus(ON);
   FlushPPBuffer();          
   SetIndentDepth(3);    
   SavePPBuffer("(defmessage-handler ");
   
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     {      
      CannotLoadWithBloadMessage("defmessage-handler");
      return(CLIPS_TRUE);
     }
#endif
   cname = GetConstructNameAndComment(readSource,&ObjectParseToken,"defmessage-handler",
                                      NULL,NULL,"~",CLIPS_TRUE,CLIPS_FALSE,CLIPS_TRUE);
   if (cname == NULL)
     return(CLIPS_TRUE);
   cls = LookupDefclassByMdlOrScope(ValueToString(cname));
   if (cls == NULL)
     {
      PrintErrorID("MSGPSR",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"A class must be defined before its message-handlers.\n");
      return(CLIPS_TRUE);
     }
   if ((cls == PrimitiveClassMap[INSTANCE_NAME]) ||
       (cls == PrimitiveClassMap[INSTANCE_ADDRESS]) ||
       (cls == PrimitiveClassMap[INSTANCE_NAME]->directSuperclasses.classArray[0]))
     {
      PrintErrorID("MSGPSR",8,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Message-handlers cannot be attached to the class ");
      PrintCLIPS(WERROR,GetDefclassName((VOID *) cls));
      PrintCLIPS(WERROR,".\n");
      return(CLIPS_TRUE);
     }
   if (HandlersExecuting(cls))
     {
      PrintErrorID("MSGPSR",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot (re)define message-handlers during execution of \n");
      PrintCLIPS(WERROR,"  other message-handlers for the same class.\n");
      return(CLIPS_TRUE);
     }
   if (GetType(ObjectParseToken) != SYMBOL)
     {
      SyntaxErrorMessage("defmessage-handler");
      return(CLIPS_TRUE);
     }
   PPBackup();
   PPBackup();
   SavePPBuffer(" ");
   SavePPBuffer(ObjectParseToken.print_rep);
   SavePPBuffer(" ");
   mname = (SYMBOL_HN *) GetValue(ObjectParseToken);
   GetToken(readSource,&ObjectParseToken);
   if (GetType(ObjectParseToken) != LPAREN)
     {
      SavePPBuffer(" ");
      if (GetType(ObjectParseToken) != STRING)
        {
         if (GetType(ObjectParseToken) != SYMBOL)
           {
            SyntaxErrorMessage("defmessage-handler");
            return(CLIPS_TRUE);
           }
         mtype = HandlerType("defmessage-handler",DOToString(ObjectParseToken));
         if (mtype == MERROR)
           return(CLIPS_TRUE);
#if ! IMPERATIVE_MESSAGE_HANDLERS
         if (mtype == MAROUND)
           return(CLIPS_TRUE);
#endif
         GetToken(readSource,&ObjectParseToken);
         if (GetType(ObjectParseToken) == STRING)
           {
            SavePPBuffer(" ");
            GetToken(readSource,&ObjectParseToken);
           }
        }
      else
        {
         SavePPBuffer(" ");
         GetToken(readSource,&ObjectParseToken);
        }
     }
   PPBackup();
   PPBackup();
   PPCRAndIndent();
   SavePPBuffer(ObjectParseToken.print_rep);

   hnd = FindHandlerByAddress(cls,mname,mtype);
   if (GetPrintWhileLoading() && GetCompilationsWatch())
     {
      PrintCLIPS(WDIALOG,"   Handler ");
      PrintCLIPS(WDIALOG,ValueToString(mname));
      PrintCLIPS(WDIALOG," ");
      PrintCLIPS(WDIALOG,hndquals[mtype]);
      PrintCLIPS(WDIALOG,(hnd == NULL) ? " defined.\n" : " redefined.\n");
     }
   
   if ((hnd != NULL) ? hnd->system : CLIPS_FALSE)
     {
      PrintErrorID("MSGPSR",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,"System message-handlers may not be modified.\n");
      return(CLIPS_TRUE);
     }
      
   hndParams = GenConstant(SYMBOL,(VOID *) SELF_SYMBOL);
   hndParams = ParseProcParameters(readSource,&ObjectParseToken,hndParams,
                                    &wildcard,&min,&max,&error,IsParameterSlotReference);
   if (error)
     return(CLIPS_TRUE);
   PPCRAndIndent();
   ReturnContext = CLIPS_TRUE;
   actions = ParseProcActions("message-handler",readSource,
                              &ObjectParseToken,hndParams,wildcard,
                              SlotReferenceVar,BindSlotReference,&lvars,
                              (VOID *) cls);
   if (actions == NULL)
     {
      ReturnExpression(hndParams);
      return(CLIPS_TRUE);
     }
   if (GetType(ObjectParseToken) != RPAREN)
     {
      SyntaxErrorMessage("defmessage-handler");
      ReturnExpression(hndParams);
      ReturnPackedExpression(actions);
      return(CLIPS_TRUE);
     }
   PPBackup();
   PPBackup();
   SavePPBuffer(ObjectParseToken.print_rep);
   SavePPBuffer("\n");

   if (hnd != NULL)
     {
      ExpressionDeinstall(hnd->actions);
      ReturnPackedExpression(hnd->actions);
      if (hnd->ppForm != NULL)
        rm((VOID *) hnd->ppForm,
           (int) (sizeof(char) * (strlen(hnd->ppForm)+1)));
     }
   else
     {
      hnd = InsertHandlerHeader(cls,mname,(int) mtype);
      IncrementSymbolCount(hnd->name);
     }
   ReturnExpression(hndParams);
   
   hnd->minParams = min;
   hnd->maxParams = max;
   hnd->localVarCount = lvars;
   hnd->actions = actions;
   ExpressionInstall(hnd->actions);
#if DEBUGGING_FUNCTIONS

   /* ===================================================
      Old handler trace status is automatically preserved
      =================================================== */
   if (GetConserveMemory() == CLIPS_FALSE)
     hnd->ppForm = CopyPPBuffer();
   else
#endif
     hnd->ppForm = NULL;
   return(CLIPS_FALSE);
  }

/*******************************************************************************
  NAME         : CreateGetAndPutHandlers
  DESCRIPTION  : Creates two message-handlers with
                  the following syntax for the slot:
                  
                 (defmessage-handler <class> get-<slot-name> primary ()
                    ?self:<slot-name>)
                 
                 For single-field slots:
                 
                 (defmessage-handler <class> put-<slot-name> primary (?value)
                    (bind ?self:<slot-name> ?value))
                 
                 For multifield slots:
                 
                 (defmessage-handler <class> put-<slot-name> primary ($?value)
                    (bind ?self:<slot-name> ?value))
                    
  INPUTS       : The class slot descriptor
  RETURNS      : Nothing useful
  SIDE EFFECTS : Message-handlers created
  NOTES        : A put handler is not created for read-only slots
 *******************************************************************************/
globle VOID CreateGetAndPutHandlers(sd)
  SLOT_DESC *sd;
  {
   char *className,*slotName;
   int bufsz;
   char *buf,*handlerRouter = "*** Default Public Handlers ***";
   int oldPWL,oldCM;
   
   if ((sd->createReadAccessor == 0) && (sd->createWriteAccessor == 0))
     return;
   className = ValueToString(sd->cls->header.name);
   slotName = ValueToString(sd->slotName->name);
   
   bufsz = (int) (sizeof(char) * (strlen(className) + (strlen(slotName) * 2) + 80));
   buf = (char *) gm2(bufsz);
   
   oldPWL = GetPrintWhileLoading();
   SetPrintWhileLoading(CLIPS_FALSE);
   oldCM = SetConserveMemory(CLIPS_TRUE);
   
   if (sd->createReadAccessor)
     {
      sprintf(buf,"%s get-%s () ?self:%s)",className,slotName,slotName);
      if (OpenStringSource(handlerRouter,buf,0))
        {
         ParseDefmessageHandler(handlerRouter);
         DestroyPPBuffer();
         CloseStringSource(handlerRouter);
        }
     }
     
   if (sd->createWriteAccessor)
     {
      sprintf(buf,"%s put-%s (%svalue) (bind ?self:%s ?value))",
                  className,slotName,sd->multiple ? "$?" : "?",slotName);
      if (OpenStringSource(handlerRouter,buf,0))
        {
         ParseDefmessageHandler(handlerRouter);
         DestroyPPBuffer();
         CloseStringSource(handlerRouter);
        }
     }
     
   SetPrintWhileLoading(oldPWL);
   SetConserveMemory(oldCM);
   
   rm((VOID *) buf,bufsz);
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/*****************************************************************
  NAME         : IsParameterSlotReference
  DESCRIPTION  : Determines if a message-handler parameter is of
                 the form ?self:<name>, which is not allowed since
                 this is slot reference syntax
  INPUTS       : The paramter name
  RETURNS      : CLIPS_TRUE if the parameter is a slot reference,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 *****************************************************************/
static BOOLEAN IsParameterSlotReference(pname)
  char *pname;
  {
   if ((strncmp(pname,SELF_STRING,SELF_LEN) == 0) ?
                  (pname[SELF_LEN] == SELF_SLOT_REF) : CLIPS_FALSE)
     {
      PrintErrorID("MSGPSR",4,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Illegal slot reference in parameter list.\n");
      return(CLIPS_TRUE);
     }
   return(CLIPS_FALSE);
  }
  
/****************************************************************************
  NAME         : SlotReferenceVar
  DESCRIPTION  : Replaces direct slot references in handler body
                   with special function calls to reference active instance
                   at run-time
                 The slot in in the class bound at parse-time is always
                   referenced (early binding).
                 Slot references of the form ?self:<name> directly reference
                   ProcParamArray[0] (the message object - ?self) to
                   find the specified slot at run-time
  INPUTS       : 1) Variable expression
                 2) The class of the handler being parsed
  RETURNS      : 0 if not recognized, 1 if so, -1 on errors
  SIDE EFFECTS : Handler body SF_VARIABLE and MF_VARIABLE replaced with 
                   direct slot access function
  NOTES        : Objects are allowed to directly access their own slots
                 without sending a message to themselves.  Since the object
                 is "within the boundary of its internals", this does not
                 violate the encapsulation principle of OOP.
 ****************************************************************************/
static int SlotReferenceVar(varexp,userBuffer)
  EXPRESSION *varexp;
  VOID *userBuffer;
  {
   struct token itkn;
   int oldpp;
   SLOT_DESC *sd;
  
   if ((varexp->type != SF_VARIABLE) && (varexp->type != MF_VARIABLE))
     return(0);
   if ((strncmp(ValueToString(varexp->value),SELF_STRING,SELF_LEN) == 0) ?
               (ValueToString(varexp->value)[SELF_LEN] == SELF_SLOT_REF) : CLIPS_FALSE)
     {
      OpenStringSource("hnd-var",ValueToString(varexp->value) + SELF_LEN + 1,0);
      oldpp = GetPPBufferStatus();
      SetPPBufferStatus(OFF);
      GetToken("hnd-var",&itkn);
      SetPPBufferStatus(oldpp);
      CloseStringSource("hnd-var");
      if (itkn.type != STOP)
        {
         sd = CheckSlotReference((DEFCLASS *) userBuffer,itkn.type,itkn.value,
                                 CLIPS_FALSE,NULL);
         if (sd == NULL)
           return(-1);
         GenHandlerSlotReference(varexp,HANDLER_GET,sd);
         return(1);
        }
     }
   return(0);
  }
  
/****************************************************************************
  NAME         : BindSlotReference
  DESCRIPTION  : Replaces direct slot binds in handler body with special
                 function calls to reference active instance at run-time
                 The slot in in the class bound at parse-time is always
                 referenced (early binding).
                 Slot references of the form ?self:<name> directly reference
                   ProcParamArray[0] (the message object - ?self) to
                   find the specified slot at run-time
  INPUTS       : 1) Variable expression
                 2) The class for the message-handler being parsed
  RETURNS      : 0 if not recognized, 1 if so, -1 on errors
  SIDE EFFECTS : Handler body "bind" call replaced with  direct slot access
                   function
  NOTES        : Objects are allowed to directly access their own slots
                 without sending a message to themselves.  Since the object
                 is "within the boundary of its internals", this does not
                 violate the encapsulation principle of OOP.
 ****************************************************************************/
static int BindSlotReference(bindExp,userBuffer)
  EXPRESSION *bindExp;
  VOID *userBuffer;
  {
   char *bindName;
   struct token itkn;
   int oldpp;
   SLOT_DESC *sd;
   EXPRESSION *saveExp;
   
   bindName = ValueToString(bindExp->argList->value);
   if (strcmp(bindName,SELF_STRING) == 0)
     {
      PrintErrorID("MSGPSR",5,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Active instance parameter cannot be changed.\n");
      return(-1);
     }
   if ((strncmp(bindName,SELF_STRING,SELF_LEN) == 0) ?
               (bindName[SELF_LEN] == SELF_SLOT_REF) : CLIPS_FALSE)
     {
      OpenStringSource("hnd-var",bindName + SELF_LEN + 1,0);
      oldpp = GetPPBufferStatus();
      SetPPBufferStatus(OFF);
      GetToken("hnd-var",&itkn);
      SetPPBufferStatus(oldpp);
      CloseStringSource("hnd-var");
      if (itkn.type != STOP)
        {
         saveExp = bindExp->argList->nextArg;
         sd = CheckSlotReference((DEFCLASS *) userBuffer,itkn.type,itkn.value,
                                 CLIPS_TRUE,saveExp);
         if (sd == NULL)
           return(-1);
         GenHandlerSlotReference(bindExp,HANDLER_PUT,sd);
         bindExp->argList->nextArg = NULL;
         ReturnExpression(bindExp->argList);
         bindExp->argList = saveExp;
         return(1);
        }
     }
   return(0);
  }

/*********************************************************
  NAME         : CheckSlotReference
  DESCRIPTION  : Examines a ?self:<slot-name> reference
                 If the reference is a single-field or
                 global variable, checking and evaluation
                 is delayed until run-time.  If the
                 reference is a symbol, this routine
                 verifies that the slot is a legal
                 slot for the reference (i.e., it exists
                 in the class to which the message-handler
                 is being attached, it is visible and it
                 is writable for write reference)
  INPUTS       : 1) A buffer holding the class
                    of the handler being parsed
                 2) The type of the slot reference
                 3) The value of the slot reference
                 4) A flag indicating if this is a read
                    or write access
                 5) Value expression for write
  RETURNS      : Class slot on success, NULL on errors
  SIDE EFFECTS : Messages printed on errors.
  NOTES        : For static references, this function
                 insures that the slot is either
                 publicly visible or that the handler
                 is being attached to the same class in
                 which the private slot is defined.
 *********************************************************/
static SLOT_DESC *CheckSlotReference(theDefclass,theType,theValue,
                                     writeFlag,writeExpression)
  DEFCLASS *theDefclass;
  int theType;
  VOID *theValue;
  BOOLEAN writeFlag;
  EXPRESSION *writeExpression;
  {
   int slotIndex;
   SLOT_DESC *sd;
   int vCode;
   
   if (theType != SYMBOL)
     {
      PrintErrorID("MSGPSR",7,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Illegal value for ?self reference.\n");
      return(NULL);
     }
   slotIndex = FindInstanceTemplateSlot(theDefclass,(SYMBOL_HN *) theValue);
   if (slotIndex == -1)
     {
      PrintErrorID("MSGPSR",6,CLIPS_FALSE);
      PrintCLIPS(WERROR,"No such slot ");
      PrintCLIPS(WERROR,ValueToString(theValue));
      PrintCLIPS(WERROR," in class ");
      PrintCLIPS(WERROR,GetDefclassName((VOID *) theDefclass));
      PrintCLIPS(WERROR," for ?self reference.\n");
      return(NULL);
     }
   sd = theDefclass->instanceTemplate[slotIndex];
   if ((sd->publicVisibility == 0) && (sd->cls != theDefclass))
     {
      SlotVisibilityViolationError(sd,theDefclass);
      return(NULL);
     }
   if (! writeFlag)
     return(sd);
     
   /* =================================================
      If a slot is initialize-only, the WithinInit flag
      still needs to be checked at run-time, for the
      handler could be called out of the context of
      an init.
      ================================================= */
   if (sd->noWrite && (sd->initializeOnly == 0))
     {
      SlotAccessViolationError(ValueToString(theValue),
                               CLIPS_FALSE,(VOID *) theDefclass);
      return(NULL);
     }

   if (GetStaticConstraintChecking())
     {
      vCode = ConstraintCheckExpressionChain(writeExpression,sd->constraint);
      if (vCode != NO_VIOLATION)
        {
         PrintErrorID("CSTRNCHK",1,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Expression for ");
         PrintSlot(WERROR,sd,NULL,"direct slot write");
         ConstraintViolationErrorMessage(NULL,NULL,0,0,NULL,0,
                                         vCode,sd->constraint,CLIPS_FALSE);
         return(NULL);
        }
     }
   return(sd);
  }
    
/***************************************************
  NAME         : GenHandlerSlotReference
  DESCRIPTION  : Creates a bitmap of the class id
                 and slot index for the get or put
                 operation. The bitmap and operation
                 type are stored in the given
                 expression.
  INPUTS       : 1) The expression
                 2) The operation type
                 3) The class slot
  RETURNS      : Nothing useful
  SIDE EFFECTS : Bitmap created and expression
                 initialized
  NOTES        : None
 ***************************************************/
static VOID GenHandlerSlotReference(exp,theType,sd)
  EXPRESSION *exp;
  int theType;
  SLOT_DESC *sd;
  {
   HANDLER_SLOT_REFERENCE handlerReference;
   
   handlerReference.classID = (unsigned short) sd->cls->id;
   handlerReference.slotID = (unsigned) sd->slotName->id;
   exp->type = theType;
   exp->value =  AddBitMap((VOID *) &handlerReference,
                           (int) sizeof(HANDLER_SLOT_REFERENCE));
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
