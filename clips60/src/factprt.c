   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           FACT RETE PRINT FUNCTIONS MODULE          */
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

#define _FACTPRT_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT

#include "symbol.h"
#include "router.h"
#include "factgen.h"

#include "factprt.h"
  
/*************************************************************************/
/* PrintFactJNCompVars1: Print routine for the FactJNCompVars1 function. */
/*************************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactJNCompVars1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct factCompVarsJN1Call *hack;

   hack = (struct factCompVarsJN1Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-jn-cmp-vars1 ");
   if (hack->pass) PrintCLIPS(logicalName,"p ");
   else PrintCLIPS(logicalName,"n ");
   PrintLongInteger(logicalName,(long) hack->slot1);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->pattern2);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->slot2);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
/*************************************************************************/
/* PrintFactJNCompVars2: Print routine for the FactJNCompVars2 function. */
/*************************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactJNCompVars2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct factCompVarsJN2Call *hack;

   hack = (struct factCompVarsJN2Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-jn-cmp-vars2 ");
   if (hack->pass) PrintCLIPS(logicalName,"p ");
   else PrintCLIPS(logicalName,"n ");
   PrintLongInteger(logicalName,(long) hack->slot1);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->pattern2);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->slot2);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }  
  
/*************************************************************************/
/* PrintFactJNCompVars3: Print routine for the FactJNCompVars3 function. */
/*************************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactJNCompVars3(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct factCompVarsJN3Call *hack;

   hack = (struct factCompVarsJN3Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-jn-cmp-vars3 ");
   if (hack->pass) PrintCLIPS(logicalName,"p ");
   else PrintCLIPS(logicalName,"n ");
   
   PrintCLIPS(logicalName,"s");
   PrintLongInteger(logicalName,(long) hack->slot1);
   PrintCLIPS(logicalName," ");
   
   if (hack->fromBeginning1) PrintCLIPS(logicalName,"b ");
   else PrintCLIPS(logicalName,"e ");
   
   PrintCLIPS(logicalName,"f");
   PrintLongInteger(logicalName,(long) hack->offset1);
   PrintCLIPS(logicalName," ");
   
   PrintCLIPS(logicalName,"p");
   PrintLongInteger(logicalName,(long) hack->pattern2);
   PrintCLIPS(logicalName," ");
   
   PrintCLIPS(logicalName,"s");
   PrintLongInteger(logicalName,(long) hack->slot2);
   PrintCLIPS(logicalName," ");
   
   if (hack->fromBeginning2) PrintCLIPS(logicalName,"b ");
   else PrintCLIPS(logicalName,"e ");
   
   PrintCLIPS(logicalName,"f");
   PrintLongInteger(logicalName,(long) hack->offset2);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }  
  
/*************************************************************************/
/* PrintFactPNCompVars2: Print routine for the FactPNCompVars2 function. */
/*************************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactPNCompVars2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct factCompVarsPN2Call *hack;

   hack = (struct factCompVarsPN2Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-pn-cmp-vars ");
   if (hack->pass) PrintCLIPS(logicalName,"p ");
   else PrintCLIPS(logicalName,"n ");
   PrintLongInteger(logicalName,(long) hack->field1);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->field2);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }

/*************************************************************************/
/* PrintSlotLengthTest: Print routine for the FactPNCompVars2 function. */
/*************************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintSlotLengthTest(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER   
   struct factCheckLengthPNCall *hack;

   hack = (struct factCheckLengthPNCall *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(slot-length ");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName," ");
   if (hack->exactly) PrintCLIPS(logicalName,"= ");
   else PrintCLIPS(logicalName,">= ");
   PrintLongInteger(logicalName,(long) hack->minLength);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
/*********************************************************************/
/* PrintFactGetVarJN1: Print routine for the FactGetvarJN1 function. */
/*********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactGetVarJN1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factGetVarJN1Call *hack;

   hack = (struct factGetVarJN1Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-jn-getvar-1 ");
   if (hack->factAddress) PrintCLIPS(logicalName,"t ");
   else PrintCLIPS(logicalName,"f ");
   if (hack->allFields) PrintCLIPS(logicalName,"t ");
   else PrintCLIPS(logicalName,"f ");
   
   PrintLongInteger(logicalName,(long) hack->whichPattern);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->whichField);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
/*********************************************************************/
/* PrintFactGetVarJN2: Print routine for the FactGetvarJN2 function. */
/*********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactGetVarJN2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factGetVarJN2Call *hack;

   hack = (struct factGetVarJN2Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-jn-getvar-2 ");
   
   PrintLongInteger(logicalName,(long) hack->whichPattern);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
/*********************************************************************/
/* PrintFactGetVarJN3: Print routine for the FactGetvarJN3 function. */
/*********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactGetVarJN3(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factGetVarJN3Call *hack;

   hack = (struct factGetVarJN3Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-jn-getvar-3 ");
   if (hack->fromBeginning) PrintCLIPS(logicalName,"t ");
   else PrintCLIPS(logicalName,"f ");
   if (hack->fromEnd) PrintCLIPS(logicalName,"t ");
   else PrintCLIPS(logicalName,"f ");
         
   PrintLongInteger(logicalName,(long) hack->beginOffset);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->endOffset);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
/*********************************************************************/
/* PrintFactGetVarPN1: Print routine for the FactGetvarPN1 function. */
/*********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactGetVarPN1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factGetVarPN1Call *hack;

   hack = (struct factGetVarPN1Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-pn-getvar-1 ");
   if (hack->factAddress) PrintCLIPS(logicalName,"t ");
   else PrintCLIPS(logicalName,"f ");
   if (hack->allFields) PrintCLIPS(logicalName,"t F");
   else PrintCLIPS(logicalName,"f F");
   
   PrintLongInteger(logicalName,(long) hack->whichField);
   PrintCLIPS(logicalName," S");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }

/*********************************************************************/
/* PrintFactGetVarPN2: Print routine for the FactGetvarPN1 function. */
/*********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactGetVarPN2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factGetVarPN2Call *hack;

   hack = (struct factGetVarPN2Call *) ValueToBitMap(theValue);;
   PrintCLIPS(logicalName,"(fact-pn-getvar-2 S");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
/*********************************************************************/
/* PrintFactGetVarPN3: Print routine for the FactGetvarPN1 function. */
/*********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintFactGetVarPN3(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factGetVarPN3Call *hack;

   hack = (struct factGetVarPN3Call *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(fact-pn-getvar-3 ");
   
   if (hack->fromBeginning) PrintCLIPS(logicalName,"t ");
   else PrintCLIPS(logicalName,"f ");
   if (hack->fromEnd) PrintCLIPS(logicalName,"t B");
   else PrintCLIPS(logicalName,"f B");
   
   PrintLongInteger(logicalName,(long) hack->beginOffset);
   PrintCLIPS(logicalName," E");
   PrintLongInteger(logicalName,(long) hack->endOffset);
   PrintCLIPS(logicalName," S");
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  } 
  
/***********************************************************************/
/* PrintPNConstant2: Print routine for the PNConstant2 function. */
/***********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintPNConstant2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factConstantPN2Call *hack;

   hack = (struct factConstantPN2Call *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(fact-pn-constant2 ");
   
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   
   if (hack->testForEquality) PrintCLIPS(logicalName," = ");
   else PrintCLIPS(logicalName," != ");
   
   PrintAtom(logicalName,GetFirstArgument()->type,GetFirstArgument()->value);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  } 
  
/***********************************************************************/
/* PrintPNConstant4: Print routine for the PNConstant4 function. */
/***********************************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintPNConstant4(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {  
#if DEVELOPER
   struct factConstantPN4Call *hack;

   hack = (struct factConstantPN4Call *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(fact-pn-constant4 ");
   
   PrintLongInteger(logicalName,(long) hack->whichSlot);
   
   PrintCLIPS(logicalName," ");
   
   PrintLongInteger(logicalName,(long) hack->offset);
   
   if (hack->testForEquality) PrintCLIPS(logicalName," = ");
   else PrintCLIPS(logicalName," != ");
   
   PrintAtom(logicalName,GetFirstArgument()->type,GetFirstArgument()->value);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  } 
  
#endif


