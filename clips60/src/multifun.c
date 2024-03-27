   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             MULTIFIELD FUNCTIONS MODULE             */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian Donnell                                        */
/*      Barry Cameron                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _MULTIFUN_SOURCE_

#include "setup.h"

#if MULTIFIELD_FUNCTIONS || OBJECT_SYSTEM

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "clipsmem.h"
#include "argacces.h"
#include "multifld.h"
#include "router.h"
#include "multifun.h"
#include "exprnpsr.h"
#include "prcdrpsr.h"
#include "prcdrfun.h"
#if (! BLOAD_ONLY) && (! RUN_TIME)
#include "scanner.h"
#endif

#if OBJECT_SYSTEM
#include "object.h"
#endif

typedef struct fieldVarStack
  {
   int type;
   VOID *value;
   long index;
   struct fieldVarStack *nxt;
  } FIELD_VAR_STACK;

#if ANSI_COMPILER
#if ! RUN_TIME
   VOID                           MultifieldFunctionDefinitions(void);
#endif
#if MULTIFIELD_FUNCTIONS
   VOID                           DeleteFunction(DATA_OBJECT_PTR);
   VOID                           MVDeleteFunction(DATA_OBJECT_PTR);
   VOID                           ReplaceFunction(DATA_OBJECT_PTR);
   VOID                           MVReplaceFunction(DATA_OBJECT_PTR);
   VOID                           InsertFunction(DATA_OBJECT_PTR);
   VOID                           ExplodeFunction(DATA_OBJECT_PTR);
   VOID                          *ImplodeFunction(void);
   VOID                           SubseqFunction(DATA_OBJECT_PTR);
   VOID                           MVSubseqFunction(DATA_OBJECT_PTR);
   VOID                           FirstFunction(DATA_OBJECT_PTR);
   VOID                           RestFunction(DATA_OBJECT_PTR);
   VOID                           NthFunction(DATA_OBJECT_PTR);
   BOOLEAN                        SubsetpFunction(void);
   VOID                           MemberFunction(DATA_OBJECT_PTR);
   static int                     FindItemInSegment(int,VOID *,DATA_OBJECT_PTR);
#if (! BLOAD_ONLY) && (! RUN_TIME)
   static struct expr            *MultifieldPrognParser(struct expr *,char *);
   static VOID                    ReplaceMvPrognFieldVars(SYMBOL_HN *,struct expr *,int);
#endif
   VOID                           MultifieldPrognFunction(DATA_OBJECT_PTR);
   VOID                           GetMvPrognField(DATA_OBJECT_PTR);
   long                           GetMvPrognIndex(void);
#endif
   static VOID                    MVRangeError(int,int,int,char *);
#else
#if MULTIFIELD_FUNCTIONS
#if ! RUN_TIME
   VOID                           MultifieldFunctionDefinitions();
#endif
   VOID                           DeleteFunction();
   VOID                           MVDeleteFunction();
   VOID                           ReplaceFunction();
   VOID                           MVReplaceFunction();
   VOID                           InsertFunction();
   VOID                           ExplodeFunction();
   VOID                          *ImplodeFunction();
   VOID                           SubseqFunction();
   VOID                           MVSubseqFunction();
   VOID                           FirstFunction();
   VOID                           RestFunction();
   VOID                           NthFunction();
   BOOLEAN                        SubsetpFunction();
   VOID                           MemberFunction();
   static int                     FindItemInSegment();
#if (! BLOAD_ONLY) && (! RUN_TIME)
   static struct expr            *MultifieldPrognParser();
   static VOID                    ReplaceMvPrognFieldVars();
#endif
   VOID                           MultifieldPrognFunction();
   VOID                           GetMvPrognField();
   long                           GetMvPrognIndex();
#endif
   static VOID                    MVRangeError();
#endif

#endif

#if MULTIFIELD_FUNCTIONS

static FIELD_VAR_STACK *FieldVarStack = NULL;

#if ! RUN_TIME
/*********************************************/
/* MultifieldFunctionDefinitions:            */
/*********************************************/
globle VOID MultifieldFunctionDefinitions()
  {   
   DefineFunction2("first$",      'm', PTIF FirstFunction,   "FirstFunction", "11m");
   DefineFunction2("rest$",       'm', PTIF RestFunction,    "RestFunction", "11m");
   DefineFunction2("subseq$",     'm', PTIF SubseqFunction,  "SubseqFunction", "33im");
   DefineFunction2("delete$",     'm', PTIF DeleteFunction,  "DeleteFunction", "33im");
   DefineFunction2("replace$",    'm', PTIF ReplaceFunction, "ReplaceFunction","4**mii");
   DefineFunction2("insert$",     'm', PTIF InsertFunction,  "InsertFunction", "3**mi");
   DefineFunction2("explode$",    'm', PTIF ExplodeFunction, "ExplodeFunction", "11s");
   DefineFunction2("implode$",    's', PTIF ImplodeFunction, "ImplodeFunction", "11m");
   DefineFunction2("nth$",        'u', PTIF NthFunction,     "NthFunction", "22*im");
   DefineFunction2("member$",     'u', PTIF MemberFunction,  "MemberFunction", "22*um");
   DefineFunction2("subsetp",     'b', PTIF SubsetpFunction, "SubsetpFunction", "22*mm");
   DefineFunction2("progn$",    'u', PTIF MultifieldPrognFunction,
                   "MultifieldPrognFunction", NULL);
   
   DefineFunction2("str-implode", 's', PTIF ImplodeFunction, "ImplodeFunction", "11m");
   DefineFunction2("str-explode", 'm', PTIF ExplodeFunction, "ExplodeFunction", "11s");
   DefineFunction2("subset",      'b', PTIF SubsetpFunction, "SubsetpFunction", "22*mm");
   DefineFunction2("nth",         'u', PTIF NthFunction,     "NthFunction", "22*im");
   DefineFunction2("mv-replace",  'm', PTIF MVReplaceFunction, "MVReplaceFunction","33*im");
   DefineFunction2("member",      'u', PTIF MemberFunction,  "MemberFunction", "22*um");
   DefineFunction2("mv-subseq",   'm', PTIF MVSubseqFunction,  "MVSubseqFunction", "33*iim");
   DefineFunction2("mv-delete",   'm', PTIF MVDeleteFunction,"MVDeleteFunction", "22*im");
#if ! BLOAD_ONLY
   AddFunctionParser("progn$",MultifieldPrognParser);
#endif
   FuncSeqOvlFlags("progn$",CLIPS_FALSE,CLIPS_FALSE);
   DefineFunction2("(get-progn$-field)", 'u', PTIF GetMvPrognField, "GetMvPrognField", "00");
   DefineFunction2("(get-progn$-index)", 'l', PTIF GetMvPrognIndex, "GetMvPrognIndex", "00");
  }
#endif

/*****************************************************/
/* DeleteFunction:  Delete function for segments.  */
/*   Note: delete does have to create a new segment. */
/*****************************************************/
globle VOID DeleteFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT val_ptr1, val_ptr2, val_ptr3;

   if ((ArgTypeCheck("delete$",1,MULTIFIELD,&val_ptr1) == CLIPS_FALSE) ||
       (ArgTypeCheck("delete$",2,INTEGER,&val_ptr2) == CLIPS_FALSE) ||
       (ArgTypeCheck("delete$",3,INTEGER,&val_ptr3) == CLIPS_FALSE))
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
      return;
     }

   if (DeleteMultiValueField(returnValue,&val_ptr1,
            DOToInteger(val_ptr2),DOToInteger(val_ptr3),"delete$") == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
     }
  }
  
/*****************************************************/
/* MVDeleteFunction:  Delete function for segments.  */
/*   This function is now obsolete.                  */
/*****************************************************/
globle VOID MVDeleteFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT val_ptr1, val_ptr2;

   if ((ArgTypeCheck("mv-delete",1,INTEGER,&val_ptr1) == CLIPS_FALSE) ||
       (ArgTypeCheck("mv-delete",2,MULTIFIELD,&val_ptr2) == CLIPS_FALSE))
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
      return;
     }

   if (DeleteMultiValueField(returnValue,&val_ptr2,
            DOToInteger(val_ptr1),DOToInteger(val_ptr1),"mv-delete") == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
     }
  }

/******************************************************/
/* ReplaceFunction:  Replace function for segments. */
/******************************************************/
globle VOID ReplaceFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT val_ptr1, val_ptr2, val_ptr3, val_ptr4;
   EXPRESSION *fieldarg;

   if ((ArgTypeCheck("replace$",1,MULTIFIELD,&val_ptr1) == CLIPS_FALSE) ||
       (ArgTypeCheck("replace$",2,INTEGER,&val_ptr2) == CLIPS_FALSE) ||
       (ArgTypeCheck("replace$",3,INTEGER,&val_ptr3) == CLIPS_FALSE))
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
      return;
     }
   fieldarg = GetFirstArgument()->nextArg->nextArg->nextArg;
   if (fieldarg->nextArg != NULL)
     StoreInMultifield(&val_ptr4,fieldarg,CLIPS_TRUE);
   else
     EvaluateExpression(fieldarg,&val_ptr4);

   if (ReplaceMultiValueField(returnValue,&val_ptr1,DOToInteger(val_ptr2),
                   DOToInteger(val_ptr3),&val_ptr4,"replace$") == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
     }
  }
  
/******************************************************/
/* MVReplaceFunction:  Replace function for segments (obsolete). */
/******************************************************/
globle VOID MVReplaceFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT val_ptr1, val_ptr2, val_ptr3;

   if ((ArgTypeCheck("mv-replace",1,INTEGER,&val_ptr1) == CLIPS_FALSE) ||
       (ArgTypeCheck("mv-replace",2,MULTIFIELD,&val_ptr2) == CLIPS_FALSE))
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
      return;
     }
   
   EvaluateExpression(GetFirstArgument()->nextArg->nextArg,&val_ptr3);

   if (ReplaceMultiValueField(returnValue,&val_ptr2,DOToInteger(val_ptr1),
                   DOToInteger(val_ptr1),&val_ptr3,"mv-replace") == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
     }
  }
  
/******************************************************/
/* InsertFunction:  Insert function for segments.   */
/******************************************************/
globle VOID InsertFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT val_ptr1, val_ptr2, val_ptr3;
   EXPRESSION *fieldarg;

   if ((ArgTypeCheck("insert$",1,MULTIFIELD,&val_ptr1) == CLIPS_FALSE) ||
       (ArgTypeCheck("insert$",2,INTEGER,&val_ptr2) == CLIPS_FALSE))
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
      return;
     }
   fieldarg = GetFirstArgument()->nextArg->nextArg;
   if (fieldarg->nextArg != NULL)
     StoreInMultifield(&val_ptr3,fieldarg,CLIPS_TRUE);
   else
     EvaluateExpression(fieldarg,&val_ptr3);

   if (InsertMultiValueField(returnValue,&val_ptr1,DOToInteger(val_ptr2),
                             &val_ptr3,"insert$") == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(returnValue);
     }
  }
  
/*********************************************/
/* ExplodeFunction:  Explodes a string to a      */
/*   multifield value and returns the new value.      */
/*********************************************/
globle VOID ExplodeFunction(str_value)
  DATA_OBJECT_PTR str_value;
  {
   DATA_OBJECT val_ptr;
   struct multifield *seg_ptr;
   int end;

  if (ArgCountCheck("explode$",EXACTLY,1) == -1)
    {
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(str_value);
      return;
     }

  if (ArgTypeCheck("explode$",1,STRING,&val_ptr) == CLIPS_FALSE)
    {
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      SetMultifieldErrorValue(str_value);
      return;
     }

  seg_ptr = StringToMultifield(DOToString(val_ptr));
  if (seg_ptr == NULL)
    {
     seg_ptr = (struct multifield *) CreateMultifield(0);
     end = 0;
    }
  else
    { end = GetMFLength(seg_ptr); }

  /*=========================*/
  /* Return the new segment. */
  /*=========================*/

  SetpType(str_value,MULTIFIELD);
  SetpDOBegin(str_value,1);
  SetpDOEnd(str_value,end);
  SetpValue(str_value,(VOID *) seg_ptr);
  return;
  }

/****************************************************/
/* ImplodeFunction:  Implodes a segment variable */
/*   to a string and returns the string.            */
/****************************************************/
globle VOID *ImplodeFunction()
  {
   DATA_OBJECT val_ptr;
   int strsize = 0;
   int i, j;
   char *tmp_str, *ret_str;
   struct multifield *segPtr;
   VOID *rv;

   if (ArgCountCheck("implode$",EXACTLY,1) == -1)
     { return(AddSymbol("")); }

   if (ArgTypeCheck("implode$",1,MULTIFIELD,&val_ptr) == CLIPS_FALSE)
     { return(AddSymbol("")); }

   /*===================================================*/
   /* Determine the size of the string to be allocated. */
   /*===================================================*/

   segPtr = (struct multifield *) GetValue(val_ptr);
   for (i = GetDOBegin(val_ptr) ; i <= GetDOEnd(val_ptr) ; i++)
     {
      if (GetMFType(segPtr,i) == FLOAT)
        {
         tmp_str = FloatToString(ValueToDouble(GetMFValue(segPtr,i)));
         strsize += strlen(tmp_str) + 1;
        }
      else if (GetMFType(segPtr,i) == INTEGER)
        {
         tmp_str = LongIntegerToString(ValueToLong(GetMFValue(segPtr,i)));
         strsize += strlen(tmp_str) + 1;
        }
      else if (GetMFType(segPtr,i) == STRING)
        {
         strsize += strlen(ValueToString(GetMFValue(segPtr,i))) + 3;
         tmp_str = ValueToString(GetMFValue(segPtr,i));
         while(*tmp_str)
           {
            if(*tmp_str == '"')
              { strsize++; }
            tmp_str++;
           }
        }
#if OBJECT_SYSTEM
      else if (GetMFType(segPtr,i) == INSTANCE_NAME)
        { strsize += strlen(ValueToString(GetMFValue(segPtr,i))) + 3; }
      else if (GetMFType(segPtr,i) == INSTANCE_ADDRESS)
        { strsize += strlen(ValueToString(((INSTANCE_TYPE *)
                            GetMFValue(segPtr,i))->name)) + 3; }
#endif
      else
        { strsize += strlen(ValueToString(GetMFValue(segPtr,i))) + 1; }
     }

   /*=============================================*/
   /* Allocate the string and copy all components */
   /* of the MULTIFIELD variable to it.             */
   /*=============================================*/

   if (strsize == 0) return(AddSymbol(""));
   ret_str = (char *) gm2(strsize);
   for(j=0, i=GetDOBegin(val_ptr); i <= GetDOEnd(val_ptr) ; i++)
     {

      /*============================*/
      /* Convert numbers to strings */
      /*============================*/

      if (GetMFType(segPtr,i) == FLOAT)
        {
         tmp_str = FloatToString(ValueToDouble(GetMFValue(segPtr,i)));
         while(*tmp_str)
           {
            *(ret_str+j) = *tmp_str;
            j++, tmp_str++;
           }
        }
      else if (GetMFType(segPtr,i) == INTEGER)
        {
         tmp_str = LongIntegerToString(ValueToLong(GetMFValue(segPtr,i)));
         while(*tmp_str)
           {
            *(ret_str+j) = *tmp_str;
            j++, tmp_str++;
           }
        }

      /*=======================================*/
      /* Enclose strings in quotes and preceed */
      /* imbedded quotes with a backslash      */
      /*=======================================*/

      else if (GetMFType(segPtr,i) == STRING)
        {
         tmp_str = ValueToString(GetMFValue(segPtr,i));
         *(ret_str+j) = '"';
         j++;
         while(*tmp_str)
           {
            if(*tmp_str == '"')
              {
               *(ret_str+j) = '\\';
               j++;
              }
            *(ret_str+j) = *tmp_str;
            j++, tmp_str++;
           }
         *(ret_str+j) = '"';
         j++;
        }
#if OBJECT_SYSTEM
      else if (GetMFType(segPtr,i) == INSTANCE_NAME)
        {
         tmp_str = ValueToString(GetMFValue(segPtr,i));
         *(ret_str + j++) = '[';
         while(*tmp_str)
           {
            *(ret_str+j) = *tmp_str;
            j++, tmp_str++;
           }
         *(ret_str + j++) = ']';
        }
      else if (GetMFType(segPtr,i) == INSTANCE_ADDRESS)
        {
         tmp_str = ValueToString(((INSTANCE_TYPE *) GetMFValue(segPtr,i))->name);
         *(ret_str + j++) = '[';
         while(*tmp_str)
           {
            *(ret_str+j) = *tmp_str;
            j++, tmp_str++;
           }
         *(ret_str + j++) = ']';
        }
#endif
      else
        {
         tmp_str = ValueToString(GetMFValue(segPtr,i));
         while(*tmp_str)
           {
            *(ret_str+j) = *tmp_str;
            j++, tmp_str++;
           }
         }
      *(ret_str+j) = ' ';
      j++;
     }
   *(ret_str+j-1) = '\0';

   /*====================*/
   /* Return the string. */
   /*====================*/

   rv = AddSymbol(ret_str);
   rm(ret_str,strsize);
   return(rv);
  }

/****************************************************/
/* SubseqFunction: Subsequence function for segments. */
/****************************************************/
globle VOID SubseqFunction(sub_value)
  DATA_OBJECT_PTR sub_value;
  {
   DATA_OBJECT val_ptr;
   struct multifield *theList;
   int offset, start, end, length;

   /*===================================*/
   /* Get the segment to be subdivided. */
   /*===================================*/
  
   if (ArgTypeCheck("subseq$",1,MULTIFIELD,&val_ptr) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   theList = (struct multifield *) DOToPointer(val_ptr);
   offset = GetDOBegin(val_ptr);
   
   /*=============================================*/
   /* Get range arguments. If they are not within */
   /* appropriate ranges, return a null segment.  */
   /*=============================================*/
   
   if (ArgTypeCheck("subseq$",2,INTEGER,&val_ptr) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   start = DOToInteger(val_ptr);

   if (ArgTypeCheck("subseq$",3,INTEGER,&val_ptr) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   end = DOToInteger(val_ptr);

   if ((end < 1) || (end < start))
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }

   /*===================================================*/
   /* Adjust lengths  to conform to segment boundaries. */
   /*===================================================*/

   length = GetMFLength(theList);
   if (start > length)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   if (end > length) end = length;
   if (start < 1) start = 1;

   /*=========================*/
   /* Return the new segment. */
   /*=========================*/

   SetpType(sub_value,MULTIFIELD);
   SetpValue(sub_value,theList);
   SetpDOEnd(sub_value,offset + end - 1);
   SetpDOBegin(sub_value,offset + start - 1);
  }

/****************************************************/
/* MVSubseqFunction: Subsequence function for segments. */
/****************************************************/
globle VOID MVSubseqFunction(sub_value)
  DATA_OBJECT_PTR sub_value;
  {
   DATA_OBJECT val_ptr;
   struct multifield *theList;
   int offset, start, end, length;
   
   /*=============================================*/
   /* Get range arguments. If they are not within */
   /* appropriate ranges, return a null segment.  */
   /*=============================================*/
   
   if (ArgTypeCheck("mv-subseq",1,INTEGER,&val_ptr) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   start = DOToInteger(val_ptr);

   if (ArgTypeCheck("mv-subseq",2,INTEGER,&val_ptr) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   end = DOToInteger(val_ptr);

   if ((end < 1) || (end < start))
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }

   /*===================================*/
   /* Get the segment to be subdivided. */
   /*===================================*/
  
   if (ArgTypeCheck("mv-subseq",3,MULTIFIELD,&val_ptr) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   theList = (struct multifield *) DOToPointer(val_ptr);
   offset = GetDOBegin(val_ptr);
   
   /*===================================================*/
   /* Adjust lengths  to conform to segment boundaries. */
   /*===================================================*/

   length = GetMFLength(theList);
   if (start > length)
     {
      SetMultifieldErrorValue(sub_value);
      return;
     }
   if (end > length) end = length;
   if (start < 1) start = 1;

   /*=========================*/
   /* Return the new segment. */
   /*=========================*/

   SetpType(sub_value,MULTIFIELD);
   SetpValue(sub_value,theList);
   SetpDOEnd(sub_value,offset + end - 1);
   SetpDOBegin(sub_value,offset + start - 1);
  }
  
/**************************************************/
/* FirstFunction: Implements the first$ function. */
/**************************************************/
globle VOID FirstFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT theValue;
   struct multifield *theList;

   /*===================================*/
   /* Get the segment to be subdivided. */
   /*===================================*/
   
   if (ArgTypeCheck("first$",1,MULTIFIELD,&theValue) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(returnValue);
      return;
     }
     
   theList = (struct multifield *) DOToPointer(theValue);

   /*=========================*/
   /* Return the new segment. */
   /*=========================*/

   SetpType(returnValue,MULTIFIELD);
   SetpValue(returnValue,theList);
   if (GetDOEnd(theValue) >= GetDOBegin(theValue))
     { SetpDOEnd(returnValue,GetDOBegin(theValue)); }
   else
     { SetpDOEnd(returnValue,GetDOEnd(theValue)); }
   SetpDOBegin(returnValue,GetDOBegin(theValue));
  }
  
/************************************************/
/* RestFunction: Implements the rest$ function. */
/************************************************/
globle VOID RestFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT theValue;
   struct multifield *theList;

   /*===================================*/
   /* Get the segment to be subdivided. */
   /*===================================*/
   
   if (ArgTypeCheck("rest$",1,MULTIFIELD,&theValue) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(returnValue);
      return;
     }
     
   theList = (struct multifield *) DOToPointer(theValue);

   /*=========================*/
   /* Return the new segment. */
   /*=========================*/

   SetpType(returnValue,MULTIFIELD);
   SetpValue(returnValue,theList);
   if (GetDOBegin(theValue) > GetDOEnd(theValue))
     { SetpDOBegin(returnValue,GetDOBegin(theValue)); }
   else
     { SetpDOBegin(returnValue,GetDOBegin(theValue) + 1); }
   SetpDOEnd(returnValue,GetDOEnd(theValue));
  }
  
/****************************************/
/* NthFunction:                              */
/****************************************/
globle VOID NthFunction(nth_value)
  DATA_OBJECT_PTR nth_value;
  {
   DATA_OBJECT val_ptr1, val_ptr2;
   struct multifield *elm_ptr;
   int n;

   if (ArgCountCheck("nth$",EXACTLY,2) == -1)
     {
      SetpType(nth_value,SYMBOL);
      SetpValue(nth_value,(VOID *) AddSymbol("nil"));
      return;
     }

   if ((ArgTypeCheck("nth$",1,INTEGER,&val_ptr1) == CLIPS_FALSE) ||
       (ArgTypeCheck("nth$",2,MULTIFIELD,&val_ptr2) == CLIPS_FALSE))
     {
      SetpType(nth_value,SYMBOL);
      SetpValue(nth_value,(VOID *) AddSymbol("nil"));
      return;
     }

   n = (int) DOToLong(val_ptr1);
   if ((n > GetDOLength(val_ptr2)) || (n < 1))
     {
      SetpType(nth_value,SYMBOL);
      SetpValue(nth_value,(VOID *) AddSymbol("nil"));
      return;
     }

   elm_ptr = (struct multifield *) GetValue(val_ptr2);
   SetpType(nth_value,GetMFType(elm_ptr,n + GetDOBegin(val_ptr2) - 1));
   SetpValue(nth_value,GetMFValue(elm_ptr,n + GetDOBegin(val_ptr2) - 1));
  }

/* ------------------------------------------------------------------
 *    SubsetFunction:
 *               This function compares two multi-field variables
 *               to see if the first is a subset of the second. It
 *               does not consider order.
 *
 *    INPUTS:    Two arguments via CLIPS stack. First is the sublist
 *               multi-field variable, the second is the list to be
 *               compared to. Both should be of type MULTIFIELD.
 *
 *    OUTPUTS:   One floating point number, 1.0 if the first list
 *               is a subset of the second, else 0.0 if it is not.
 *
 *    NOTES:     This function is called from CLIPS with the subset
 *               command. Repeated values in the sublist must also
 *               be repeated in the main list.
 * ------------------------------------------------------------------
 */

globle BOOLEAN SubsetpFunction()
  {
   DATA_OBJECT item1, item2;
   int i;

   if (ArgCountCheck("subsetp",EXACTLY,2) == -1)
     return(CLIPS_FALSE);

   if (ArgTypeCheck("subsetp",1,MULTIFIELD,&item1) == CLIPS_FALSE)
     return(CLIPS_FALSE);

   if (ArgTypeCheck("subsetp",2,MULTIFIELD,&item2) == CLIPS_FALSE)
     return(CLIPS_FALSE);

   for (i = GetDOBegin(item1) ; i <= GetDOEnd(item1) ; i++)
     {
      if (FindItemInSegment(GetMFType((struct multifield *) GetValue(item1),i),
                            GetMFValue((struct multifield *) GetValue(item1),i),&item2) == 0)
        { return(CLIPS_FALSE); }
     }

   return(CLIPS_TRUE);
  }

/***************************************/
/* MemberFunction:                          */
/***************************************/
globle VOID MemberFunction(result)
  DATA_OBJECT_PTR result;
  {
   DATA_OBJECT item1, item2;
   int pos;

   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;

   if (ArgCountCheck("member$",EXACTLY,2) == -1) return;

   RtnUnknown(1,&item1);
   if ((GetType(item1) != SYMBOL) &&
       (GetType(item1) != STRING) &&
       (GetType(item1) != INTEGER) &&
       (GetType(item1) != EXTERNAL_ADDRESS) &&
#if OBJECT_SYSTEM
       (GetType(item1) != INSTANCE_NAME) &&
       (GetType(item1) != INSTANCE_ADDRESS) &&
#endif
       (GetType(item1) != FLOAT))
     {
      ExpectedTypeError1("member$",1,"primitive data type");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }

   if (ArgTypeCheck("member$",2,MULTIFIELD,&item2) == CLIPS_FALSE) return;

   pos = FindItemInSegment(item1.type,item1.value,&item2);

   if (pos != 0L)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong((long) pos);
     }
  }

/***************************************/
/* FindItemInSegment:                  */
/***************************************/
static int FindItemInSegment(searchType,searchValue,val_ptr)
  int searchType;
  VOID *searchValue;
  DATA_OBJECT_PTR val_ptr;
  {
   int mul_length, i;

   mul_length = GetpDOLength(val_ptr);
   for (i = 0 ; i < mul_length ; i++)
     {
      if ((searchValue == GetMFValue((struct multifield *) GetpValue(val_ptr),i + GetpDOBegin(val_ptr))) &&
          (searchType == GetMFType((struct multifield *) GetpValue(val_ptr),i + GetpDOBegin(val_ptr))))
        return(i+1);
     }

   return(CLIPS_FALSE);
  }

#if (! BLOAD_ONLY) && (! RUN_TIME)

/***************************************************/
/* MultifieldPrognParser                           */
/***************************************************/
static struct expr *MultifieldPrognParser(top,infile)
  struct expr *top;
  char *infile;
  {
   struct BindInfo *oldBindList,*newBindList,*prev;
   struct token tkn;
   struct expr *tmp;
   SYMBOL_HN *fieldVar = NULL;
   
   SavePPBuffer(" ");
   GetToken(infile,&tkn);
   
   /* ================================
      Simple form: progn$ <mf-exp> ...
      ================================ */
   if (tkn.type != LPAREN)
     {
      top->argList = ParseAtomOrExpression(infile,&tkn);
      if (top->argList == NULL)
        {
         ReturnExpression(top);
         return(NULL);
        }
     }
   else
     {
      GetToken(infile,&tkn);
      if (tkn.type != SF_VARIABLE)
        {
         if (tkn.type != SYMBOL)
           goto MvPrognParseError;
         top->argList = Function2Parse(infile,ValueToString(tkn.value));
         if (top->argList == NULL)
           {
            ReturnExpression(top);
            return(NULL);
           }
        }

      /* =========================================
         Complex form: progn$ (<var> <mf-exp>) ...
         ========================================= */
      else
        {
         fieldVar = (SYMBOL_HN *) tkn.value;
         SavePPBuffer(" ");
         top->argList = ParseAtomOrExpression(infile,NULL);
         if (top->argList == NULL)
           {
            ReturnExpression(top);
            return(NULL);
           }
         GetToken(infile,&tkn);
         if (tkn.type != RPAREN)
           goto MvPrognParseError;
         PPBackup();
         PPBackup();
         SavePPBuffer(tkn.printForm);
         SavePPBuffer(" ");
        }
     }

   if (CheckArgumentAgainstRestriction(top->argList,(int) 'm'))
     goto MvPrognParseError;
   oldBindList = GetParsedBindNames();
   SetParsedBindNames(NULL);
   IncrementIndentDepth(3);
   BreakContext = CLIPS_TRUE;
   ReturnContext = svContexts->rtn;
   PPCRAndIndent();
   top->argList->nextArg = GroupActions(infile,&tkn,CLIPS_TRUE,NULL);
   DecrementIndentDepth(3);
   PPBackup();
   PPBackup();
   SavePPBuffer(tkn.printForm);
   if (top->argList->nextArg == NULL)
     {
      SetParsedBindNames(oldBindList);
      ReturnExpression(top);
      return(NULL);
     }
   tmp = top->argList->nextArg;
   top->argList->nextArg = tmp->argList;
   tmp->argList = NULL;
   ReturnExpression(tmp);
   newBindList = GetParsedBindNames();
   prev = NULL;
   while (newBindList != NULL)
     {
      if ((fieldVar == NULL) ? CLIPS_FALSE :
          (strcmp(ValueToString(newBindList->name),ValueToString(fieldVar)) == 0))
        {
         ClearParsedBindNames();
         SetParsedBindNames(oldBindList);
         PrintErrorID("MULTIFUN",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Cannot rebind field variable in function progn$.\n");
         ReturnExpression(top);
         return(NULL);
        }
      prev = newBindList;
      newBindList = newBindList->next;
     }
   if (prev == NULL)
     SetParsedBindNames(oldBindList);
   else
     prev->next = oldBindList;
   if (fieldVar != NULL)
     ReplaceMvPrognFieldVars(fieldVar,top->argList->nextArg,0);
   return(top);
   
MvPrognParseError:
   SyntaxErrorMessage("progn$");
   ReturnExpression(top);
   return(NULL);
  }

/***************************************************/
/* ReplaceMvPrognFieldVars                         */
/***************************************************/
static VOID ReplaceMvPrognFieldVars(fieldVar,exp,depth)
  SYMBOL_HN *fieldVar;
  struct expr *exp;
  int depth;
  {
   int flen;
   
   flen = strlen(ValueToString(fieldVar));
   while (exp != NULL)
     {
      if ((exp->type != SF_VARIABLE) ? CLIPS_FALSE :
          (strncmp(ValueToString(exp->value),ValueToString(fieldVar),flen) == 0))
        {
         if (ValueToString(exp->value)[flen] == '\0')
           {
            exp->type = FCALL;
            exp->value = (VOID *) FindFunction((VOID *) "(get-progn$-field)");
            exp->argList = GenConstant(INTEGER,AddLong((long) depth));
           }
         else if (strcmp(ValueToString(exp->value) + flen,"-index") == 0)
           {
            exp->type = FCALL;
            exp->value = (VOID *) FindFunction((VOID *) "(get-progn$-index)");
            exp->argList = GenConstant(INTEGER,AddLong((long) depth));
           }
        }
      else if (exp->argList != NULL)
        {
         if ((exp->type == FCALL) && (exp->value == (VOID *) FindFunction("progn$")))
           ReplaceMvPrognFieldVars(fieldVar,exp->argList,depth+1);
         else
           ReplaceMvPrognFieldVars(fieldVar,exp->argList,depth);
        }
      exp = exp->nextArg;
     }
  }

#endif

/***************************************************/
/* MultifieldPrognFunction                         */
/***************************************************/
globle VOID MultifieldPrognFunction(result)
  DATA_OBJECT_PTR result;
  {
   EXPRESSION *exp;
   DATA_OBJECT argval;
   long i,end;
   FIELD_VAR_STACK *tmpField;
   
   tmpField = get_struct(fieldVarStack);
   tmpField->type = SYMBOL;
   tmpField->value = CLIPSFalseSymbol;
   tmpField->nxt = FieldVarStack;
   FieldVarStack = tmpField;
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   if (ArgTypeCheck("progn$",1,MULTIFIELD,&argval) == CLIPS_FALSE)
     {
      FieldVarStack = tmpField->nxt;
      rtn_struct(fieldVarStack,tmpField);
      return;
     }
   end = GetDOEnd(argval);
   for (i = GetDOBegin(argval) ; i <= end ; i++)
     {
      tmpField->type = GetMFType(argval.value,i);
      tmpField->value = GetMFValue(argval.value,i);
      tmpField->index = i;
      for (exp = GetFirstArgument()->nextArg ; exp != NULL ; exp = exp->nextArg)
        {
         EvaluateExpression(exp,result);
         if (HaltExecution || BreakFlag || ReturnFlag)
           {
            BreakFlag = CLIPS_FALSE;
            result->type = SYMBOL;
            result->value = CLIPSFalseSymbol;
            FieldVarStack = tmpField->nxt;
            rtn_struct(fieldVarStack,tmpField);
            return;
           }
        }
     }
   BreakFlag = CLIPS_FALSE;
   FieldVarStack = tmpField->nxt;
   rtn_struct(fieldVarStack,tmpField);
  }

/***************************************************/
/* GetMvPrognField                                 */
/***************************************************/
globle VOID GetMvPrognField(result)
  DATA_OBJECT_PTR result;
  {
   int depth;
   FIELD_VAR_STACK *tmpField;
   
   depth = ValueToInteger(GetFirstArgument()->value);
   tmpField = FieldVarStack;
   while (depth > 0)
     {
      tmpField = tmpField->nxt;
      depth--;
     }
   result->type = tmpField->type;
   result->value = tmpField->value;
  }

/***************************************************/
/* GetMvPrognField                                 */
/***************************************************/
globle long GetMvPrognIndex()
  {
   int depth;
   FIELD_VAR_STACK *tmpField;
   
   depth = ValueToInteger(GetFirstArgument()->value);
   tmpField = FieldVarStack;
   while (depth > 0)
     {
      tmpField = tmpField->nxt;
      depth--;
     }
   return(tmpField->index);
  }

#endif

#if OBJECT_SYSTEM || MULTIFIELD_FUNCTIONS

/**************************************************************************
  NAME         : ReplaceMultiValueField
  DESCRIPTION  : Performs a replace on the src multi-field value
                   storing the results in the dst multi-field value
  INPUTS       : 1) The destination value buffer
                 2) The source value (can be NULL)
                 3) Beginning of index range
                 4) End of range
                 5) The new field value
  RETURNS      : CLIPS_TRUE if successful, CLIPS_FALSE otherwise
  SIDE EFFECTS : Allocates and sets a ephemeral segment (even if new
                   number of fields is 0)
                 Src value segment is not changed
  NOTES        : index is NOT guaranteed to be valid
                 src is guaranteed to be a multi-field variable or NULL
 **************************************************************************/
globle int ReplaceMultiValueField(dst,src,rb,re,field,funcName)
  DATA_OBJECT *dst,*src,*field;
  int rb,re;
  char *funcName;
  {
   register int i,j,k;
   register FIELD_PTR deptr,septr;
   int srclen,dstlen;

   srclen = (src != NULL) ? (src->end - src->begin + 1) : 0;
   if ((re < rb) ||
       (rb < 1) || (re < 1) ||
       (rb > srclen) || (re > srclen))
     {
      MVRangeError(rb,re,srclen,funcName);
      return(CLIPS_FALSE);
     }
   rb = src->begin + rb - 1;
   re = src->begin + re - 1;
   if (field->type == MULTIFIELD)
     dstlen = srclen + GetpDOLength(field) - (re-rb+1);
   else
     dstlen = srclen + 1 - (re-rb+1);
   dst->type = MULTIFIELD;
   dst->begin = 0;
   dst->value = CreateMultifield(dstlen);
   dst->end = dstlen-1;
   for (i = 0 , j = src->begin ; j < rb ; i++ , j++)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i];
      septr = &((struct multifield *) src->value)->theFields[j];
      deptr->type = septr->type;
      deptr->value = septr->value;
     }
   if (field->type != MULTIFIELD)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i++];
      deptr->type = field->type;
      deptr->value = field->value;
     }
   else
     {
      for (k = field->begin ; k <= field->end ; k++ , i++)
        {
         deptr = &((struct multifield *) dst->value)->theFields[i];
         septr = &((struct multifield *) field->value)->theFields[k];
         deptr->type = septr->type;
         deptr->value = septr->value;
        }
     }
   while (j < re)
     j++;
   for (j++ ; i < dstlen ; i++ , j++)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i];
      septr = &((struct multifield *) src->value)->theFields[j];
      deptr->type = septr->type;
      deptr->value = septr->value;
     }
   return(CLIPS_TRUE);
  }

/**************************************************************************
  NAME         : InsertMultiValueField
  DESCRIPTION  : Performs an insert on the src multi-field value
                   storing the results in the dst multi-field value
  INPUTS       : 1) The destination value buffer
                 2) The source value (can be NULL)
                 3) The index for the change
                 4) The new field value
  RETURNS      : CLIPS_TRUE if successful, CLIPS_FALSE otherwise
  SIDE EFFECTS : Allocates and sets a ephemeral segment (even if new
                   number of fields is 0)
                 Src value segment is not changed
  NOTES        : index is NOT guaranteed to be valid
                 src is guaranteed to be a multi-field variable or NULL
 **************************************************************************/
globle int InsertMultiValueField(dst,src,index,field,funcName)
  DATA_OBJECT *dst,*src,*field;
  int index;
  char *funcName;
  {
   register int i,j,k;
   register FIELD *deptr, *septr;
   int srclen,dstlen;

   srclen = (src != NULL) ? (src->end - src->begin + 1) : 0;
   if (index < 1)
     {
      MVRangeError(index,index,srclen+1,funcName);
      return(CLIPS_FALSE);
     }
   if (index > (srclen + 1))
     index = srclen + 1;
   dst->type = MULTIFIELD;
   dst->begin = 0;
   if (src == NULL)
     {
      if (field->type == MULTIFIELD)
        {
         DuplicateMultifield(dst,field);
         AddToMultifieldList((struct multifield *) dst->value);
        }
      else
        {
         dst->value = CreateMultifield(0);
         dst->end = 0;
         deptr = &((struct multifield *) dst->value)->theFields[0];
         deptr->type = field->type;
         deptr->value = field->value;
        }
      return(CLIPS_TRUE);
     }
   dstlen = (field->type == MULTIFIELD) ? GetpDOLength(field) + srclen : srclen + 1;
   dst->value = CreateMultifield(dstlen);
   dst->end = dstlen-1;
   index--;
   for (i = 0 , j = src->begin ; j < index ; i++ , j++)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i];
      septr = &((struct multifield *) src->value)->theFields[j];
      deptr->type = septr->type;
      deptr->value = septr->value;
     }
   if (field->type != MULTIFIELD)
     {
      deptr = &((struct multifield *) dst->value)->theFields[index];
      deptr->type = field->type;
      deptr->value = field->value;
      i++;
     }
   else
     {
      for (k = field->begin ; k <= field->end ; k++ , i++)
        {
         deptr = &((struct multifield *) dst->value)->theFields[i];
         septr = &((struct multifield *) field->value)->theFields[k];
         deptr->type = septr->type;
         deptr->value = septr->value;
        }
     }
   for ( ; j <= src->end ; i++ , j++)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i];
      septr = &((struct multifield *) src->value)->theFields[j];
      deptr->type = septr->type;
      deptr->value = septr->value;
     }
   return(CLIPS_TRUE);
  }

/*******************************************************
  NAME         : MVRangeError
  DESCRIPTION  : Prints out an error messages for index
                   out-of-range errors in multi-field
                   access functions
  INPUTS       : 1) The bad range start
                 2) The bad range end
                 3) The max end of the range (min is
                     assumed to be 1)
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ******************************************************/
static VOID MVRangeError(brb,bre,max,funcName)
  int brb,bre,max;
  char *funcName;
  {
   PrintErrorID("MULTIFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Multifield index ");
   if (brb == bre)
     PrintLongInteger(WERROR,(long) brb);
   else
     {
      PrintCLIPS(WERROR,"range ");
      PrintLongInteger(WERROR,(long) brb);
      PrintCLIPS(WERROR,"..");
      PrintLongInteger(WERROR,(long) bre);
     }
   PrintCLIPS(WERROR," out of range 1..");
   PrintLongInteger(WERROR,(long) max);
   if (funcName != NULL)
     {
      PrintCLIPS(WERROR," in function ");
      PrintCLIPS(WERROR,funcName);
     }
   PrintCLIPS(WERROR,".\n");
  }

/**************************************************************************
  NAME         : DeleteMultiValueField
  DESCRIPTION  : Performs a modify on the src multi-field value
                   storing the results in the dst multi-field value
  INPUTS       : 1) The destination value buffer
                 2) The source value (can be NULL)
                 3) The beginning index for deletion
                 4) The ending index for deletion
  RETURNS      : CLIPS_TRUE if successful, CLIPS_FALSE otherwise
  SIDE EFFECTS : Allocates and sets a ephemeral segment (even if new
                   number of fields is 0)
                 Src value segment is not changed
  NOTES        : index is NOT guaranteed to be valid
                 src is guaranteed to be a multi-field variable or NULL
 **************************************************************************/
globle int DeleteMultiValueField(dst,src,rb,re,funcName)
  DATA_OBJECT *dst,*src;
  int rb,re;
  char *funcName;
  {
   register int i,j;
   register FIELD_PTR deptr,septr;
   int srclen,dstlen;

   srclen = (src != NULL) ? (src->end - src->begin + 1) : 0;
   if ((re < rb) ||
       (rb < 1) || (re < 1) ||
       (rb > srclen) || (re > srclen))
     {
      MVRangeError(rb,re,srclen,funcName);
      return(CLIPS_FALSE);
     }
   dst->type = MULTIFIELD;
   dst->begin = 0;
   if (srclen == 0)
    {
     dst->value = CreateMultifield(0);
     dst->end = -1;
     return(CLIPS_TRUE);
    }
   rb = src->begin + rb -1;
   re = src->begin + re -1;
   dstlen = srclen-(re-rb+1);
   dst->end = dstlen-1;
   dst->value = CreateMultifield(dstlen);
   for (i = 0 , j = src->begin ; j < rb ; i++ , j++)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i];
      septr = &((struct multifield *) src->value)->theFields[j];
      deptr->type = septr->type;
      deptr->value = septr->value;
     }
   while (j < re)
     j++;
   for (j++ ; i <= dst->end ; j++ , i++)
     {
      deptr = &((struct multifield *) dst->value)->theFields[i];
      septr = &((struct multifield *) src->value)->theFields[j];
      deptr->type = septr->type;
      deptr->value = septr->value;
     }
   return(CLIPS_TRUE);
  }

#endif


