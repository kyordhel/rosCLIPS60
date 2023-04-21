   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            FACT LHS PATTERN PARSING MODULE          */
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

#define _FACTLHS_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT && (! RUN_TIME) && (! BLOAD_ONLY)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "router.h"
#include "reorder.h"
#include "pattern.h"
#include "tmpltpsr.h"
#include "tmpltdef.h"
#include "tmpltlhs.h"
#include "modulutl.h"
#include "modulpsr.h"
#include "cstrcpsr.h"

#include "factlhs.h"

/******************************************************/
/* SequenceRestrictionParse: This routine coordinates */
/*   the parsing of an ordered fact pattern.          */
/*                                                    */
/* <ordered-pattern-CE> ::= (<symbol> <constraint>+)  */
/******************************************************/
globle struct lhsParseNode *SequenceRestrictionParse(readSource,theToken)
  char *readSource;
  struct token *theToken;
  {
   struct lhsParseNode *topNode = NULL;
   struct lhsParseNode *nextField;
   
   /*============================================*/
   /* Create pattern node for the relation name. */
   /*============================================*/

   topNode = GetLHSParseNode();
   topNode->type = SF_WILDCARD;
   topNode->negated = CLIPS_FALSE;
   topNode->index = -1;
   topNode->slotNumber = 1;
   topNode->bottom = GetLHSParseNode();
   topNode->bottom->type = SYMBOL;
   topNode->bottom->negated = CLIPS_FALSE;
   topNode->bottom->value = (VOID *) theToken->value;
   
   /*======================================================*/
   /* Connective constraints cannot be used in conjunction */
   /* with the first field of a pattern.                   */
   /*======================================================*/
   
   SavePPBuffer(" "); 
   GetToken(readSource,theToken);
   if ((theToken->type == OR_CONSTRAINT) || (theToken->type == AND_CONSTRAINT))
     {
      ReturnLHSParseNodes(topNode);
      SyntaxErrorMessage("the first field of a pattern");
      return(NULL);
     }   

   /*=================================================*/
   /* Treat the remaining fields of a flat pattern as */
   /* if they were contained in a multifield slot.    */
   /*=================================================*/
   
   nextField = RestrictionParse(readSource,theToken,CLIPS_TRUE,NULL,1,NULL,1);
   if (nextField == NULL)
     {
      ReturnLHSParseNodes(topNode);
      return(NULL);
     }

   topNode->right = nextField;
   
   /*================================================*/
   /* The pattern must end with a right parenthesis. */
   /*================================================*/
   
   if (theToken->type != RPAREN)
     {
      PPBackup();                        
      SavePPBuffer(" ");                  
      SavePPBuffer(theToken->printForm); 
      SyntaxErrorMessage("fact patterns");
      ReturnLHSParseNodes(topNode);
      return(NULL);
     }
     
   /*====================================*/
   /* Fix the pretty print output if the */
   /* slot contained no restrictions.    */
   /*====================================*/
   
   if (nextField->bottom == NULL)
     {
      PPBackup();  
      PPBackup();                        
      SavePPBuffer(")");  
     } 
     
   /*===================================*/
   /* If no errors, return the pattern. */
   /*===================================*/
   
   return(topNode);
  }

/****************************************************************/
/* CreateInitialFactPattern: Creates the pattern (initial-fact) */
/*   for use in rules which have no LHS patterns.               */
/****************************************************************/
globle struct lhsParseNode *CreateInitialFactPattern()
  {
   struct lhsParseNode *topNode;
   struct deftemplate *theDeftemplate;
   int count;

   theDeftemplate = (struct deftemplate *) 
                    FindImportedConstruct("deftemplate",NULL,"initial-fact",
                                          &count,CLIPS_TRUE,NULL);
   if (theDeftemplate == NULL)
     { CreateImpliedDeftemplate(AddSymbol("initial-fact"),CLIPS_FALSE); }

   topNode = GetLHSParseNode();
   topNode->type = SF_WILDCARD;
   topNode->index = 0;
   topNode->slotNumber = 1;
   
   topNode->bottom = GetLHSParseNode();
   topNode->bottom->type = SYMBOL;
   topNode->bottom->value = (VOID *) AddSymbol("initial-fact");
   
   return(topNode);
  }
  
/**********************************************************************/
/* FactPatternParserFind: This function is the pattern find function  */
/*   for facts. It tells the pattern parsing code that the specified  */
/*   pattern can be parsed as a fact pattern. By default, any pattern */
/*   beginning with a symbol can be parsed as a fact pattern. Since   */
/*   all patterns begin with a symbol, it follows that all patterns   */
/*   can be parsed as a fact pattern.                                 */
/**********************************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle int FactPatternParserFind(theRelation)
  SYMBOL_HN *theRelation;
  {
#if MAC_MPW
#pragma unused(theRelation)
#endif
   return(CLIPS_TRUE);
  }
  
/******************************************************/
/* FactPatternParse: This function is called to parse */
/*  both deftemplate and ordered fact patterns.       */
/******************************************************/
globle struct lhsParseNode *FactPatternParse(readSource,theToken)
  char *readSource;
  struct token *theToken;
  {
   struct deftemplate *theDeftemplate;
   int count;
   
   if (FindModuleSeparator(ValueToString(theToken->value)))
     {
      IllegalModuleSpecifierMessage();
      return(NULL);
     }
     
   theDeftemplate = (struct deftemplate *) 
                    FindImportedConstruct("deftemplate",NULL,ValueToString(theToken->value),
                                          &count,CLIPS_TRUE,NULL);
  
   if (count > 1)
     {
      AmbiguousReferenceErrorMessage("deftemplate",ValueToString(theToken->value));
      return(NULL);
     }
     
   if (theDeftemplate == NULL)
     { 
      if (FindImportExportConflict("deftemplate",((struct defmodule *) GetCurrentModule()),ValueToString(theToken->value)))
        {
         ImportExportConflictMessage("implied deftemplate",ValueToString(theToken->value),NULL,NULL);
         return(NULL);
        }

      theDeftemplate = CreateImpliedDeftemplate((SYMBOL_HN *) theToken->value,CLIPS_TRUE);
     }
     
   if (theDeftemplate->implied == CLIPS_FALSE) 
     { return(DeftemplateLHSParse(readSource,theDeftemplate)); }
   
   return(SequenceRestrictionParse(readSource,theToken));
  }
  
#endif



