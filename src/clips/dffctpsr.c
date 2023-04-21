   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                DEFFACTS PARSER MODULE               */
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

#define _DFFCTPSR_SOURCE_

#include "setup.h"

#if DEFFACTS_CONSTRUCT 

#include "clipsmem.h"
#include "router.h"
#include "cstrcpsr.h"
#include "factrhs.h"
#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif
#include "dffctdef.h"
#include "dffctbsc.h"

#include "dffctpsr.h"

/************************************************************/
/* ParseDeffacts: Coordinates all actions necessary for the */
/*   addition of a deffacts construct into the current      */
/*   environment. Called when parsing a construct after the */
/*   deffacts keyword has been found.                       */
/************************************************************/
globle int ParseDeffacts(readSource)
  char *readSource;
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   SYMBOL_HN *deffactsName;
   struct expr *temp;
   struct deffacts *newDeffacts;
   int deffactsError;
   struct token inputToken;

   /*=========================*/
   /* Parsing initialization. */
   /*=========================*/
   
   deffactsError = CLIPS_FALSE;
   SetPPBufferStatus(ON);

   FlushPPBuffer();
   SetIndentDepth(3);
   SavePPBuffer("(deffacts ");

   /*==========================================================*/
   /* Deffacts can not be added when a binary image is loaded. */
   /*==========================================================*/
   
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE)
     {
      CannotLoadWithBloadMessage("deffacts");
      return(CLIPS_TRUE);
     }
#endif

   /*============================*/
   /* Parse the deffacts header. */
   /*============================*/

   deffactsName = GetConstructNameAndComment(readSource,&inputToken,"deffacts",
                                             FindDeffacts,Undeffacts,"$",CLIPS_TRUE,
                                             CLIPS_TRUE,CLIPS_TRUE);
   if (deffactsName == NULL) { return(CLIPS_TRUE); }
   
   /*===============================================*/
   /* Parse the list of facts in the deffacts body. */
   /*===============================================*/

   temp = BuildRHSAssert(readSource,&inputToken,&deffactsError,CLIPS_FALSE,CLIPS_FALSE,"deffacts");

   if (deffactsError == CLIPS_TRUE) { return(CLIPS_TRUE); }

   if (ExpressionContainsVariables(temp,CLIPS_FALSE))
     {
      LocalVariableErrorMessage("a deffacts construct");
      ReturnExpression(temp);
      return(CLIPS_TRUE);
     }

   SavePPBuffer("\n");

   /*==========================*/
   /* Create the new deffacts. */
   /*==========================*/
   
   ExpressionInstall(temp);
   newDeffacts = get_struct(deffacts);
   newDeffacts->header.name = deffactsName;
   IncrementSymbolCount(deffactsName);
   newDeffacts->assertList = PackExpression(temp);
   newDeffacts->header.whichModule = (struct defmoduleItemHeader *)
                              GetModuleItem(NULL,FindModuleItem("deffacts")->moduleIndex);

   newDeffacts->header.next = NULL;
   ReturnExpression(temp);

   /*=======================================================*/
   /* Save the pretty print representation of the deffacts. */
   /*=======================================================*/
   
   if (GetConserveMemory() == CLIPS_TRUE)
     { newDeffacts->header.ppForm = NULL; }
   else
     { newDeffacts->header.ppForm = CopyPPBuffer(); }

   /*=============================================*/
   /* Add the deffacts to the appropriate module. */
   /*=============================================*/
   
   AddConstructToModule(&newDeffacts->header);

#endif /* (! RUN_TIME) && (! BLOAD_ONLY) */

   /*================================================================*/
   /* Return FALSE to indicate the deffacts was successfully parsed. */
   /*================================================================*/
   
   return(CLIPS_FALSE);
  }

#endif /* DEFFACTS_CONSTRUCT */


