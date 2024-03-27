   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               CONSTRAINT PARSER MODULE              */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Provides functions for constraint checking of        */
/*      data types.                                          */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian Donnell                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CSTRNPSR_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#if ANSI_COMPILER
#include <stdlib.h>
#endif

#include "setup.h"

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "scanner.h"
#include "cstrnutl.h"

#include "cstrnpsr.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if (! RUN_TIME) && (! BLOAD_ONLY)
#if ANSI_COMPILER
   static BOOLEAN                 ParseRangeCardinalityAttribute(char *,CONSTRAINT_RECORD *,
                                                      CONSTRAINT_PARSE_RECORD *,char *,int);
   static BOOLEAN                 ParseTypeAttribute(char *,CONSTRAINT_RECORD *);
   static VOID                    AddToRestrictionList(int,CONSTRAINT_RECORD *,
                                                       CONSTRAINT_RECORD *);
   static BOOLEAN                 ParseAllowedValuesAttribute(char *,char *,
                                                              CONSTRAINT_RECORD *,
                                                              CONSTRAINT_PARSE_RECORD *);
   static int                     GetConstraintTypeFromAllowedName(char *);
   static int                     GetConstraintTypeFromTypeName(char *);
   static int                     GetAttributeParseValue(char *,CONSTRAINT_PARSE_RECORD *);
   static VOID                    SetRestrictionFlag(int,CONSTRAINT_RECORD *,int);
   static VOID                    SetParseFlag(CONSTRAINT_PARSE_RECORD *,char *);
   static VOID                    NoConjunctiveUseError(char *,char *);
#else
   static BOOLEAN                 ParseRangeCardinalityAttribute();
   static BOOLEAN                 ParseTypeAttribute();
   static VOID                    AddToRestrictionList();
   static BOOLEAN                 ParseAllowedValuesAttribute();
   static int                     GetConstraintTypeFromAllowedName();
   static int                     GetConstraintTypeFromTypeName();
   static int                     GetAttributeParseValue();
   static VOID                    SetRestrictionFlag();
   static VOID                    SetParseFlag();
   static VOID                    NoConjunctiveUseError();
#endif
#endif
  
/********************************************************************/
/* CheckConstraintParseConflicts: Determines a constraint record */
/*   has any conflicts in the attribute specifications.             */
/********************************************************************/
globle BOOLEAN CheckConstraintParseConflicts(constraints)
  CONSTRAINT_RECORD *constraints;
  {
   if (constraints->anyAllowed == CLIPS_TRUE)
     { /* Do Nothing */ }
   else if (constraints->symbolRestriction && (constraints->symbolsAllowed == CLIPS_FALSE))
     {
      AttributeConflictErrorMessage("type","allowed-symbols");
      return(CLIPS_FALSE);
     }
   else if (constraints->stringRestriction && (constraints->stringsAllowed == CLIPS_FALSE))
     {
      AttributeConflictErrorMessage("type","allowed-strings");
      return(CLIPS_FALSE);
     }
   else if (constraints->integerRestriction && (constraints->integersAllowed == CLIPS_FALSE))
     {
      AttributeConflictErrorMessage("type","allowed-integers/numbers");
      return(CLIPS_FALSE);
     }
   else if (constraints->floatRestriction && (constraints->floatsAllowed == CLIPS_FALSE))
     {         
      AttributeConflictErrorMessage("type","allowed-floats/numbers");
      return(CLIPS_FALSE);
     }

   /*================================================================*/
   /* Check to see if range attribute conflicts with type attribute. */
   /*================================================================*/

   if ((constraints->maxValue != NULL) && (constraints->anyAllowed == CLIPS_FALSE))
     {
      if (((constraints->maxValue->type == INTEGER) &&
          (constraints->integersAllowed == CLIPS_FALSE)) ||
          ((constraints->maxValue->type == FLOAT) &&
           (constraints->floatsAllowed == CLIPS_FALSE)))
        {
         AttributeConflictErrorMessage("type","range");
         return(CLIPS_FALSE);
        }
     }

   if ((constraints->minValue != NULL) && (constraints->anyAllowed == CLIPS_FALSE))
     {
      if (((constraints->minValue->type == INTEGER) &&
          (constraints->integersAllowed == CLIPS_FALSE)) ||
          ((constraints->minValue->type == FLOAT) &&
           (constraints->floatsAllowed == CLIPS_FALSE)))
        {
         AttributeConflictErrorMessage("type","range");
         return(CLIPS_FALSE);
        }
     }
   return(CLIPS_TRUE);
  }  

/********************************************************************/
/* AttributeConflictErrorMessage:            */
/********************************************************************/
globle VOID AttributeConflictErrorMessage(attribute1,attribute2)
  char *attribute1, *attribute2;
  {
   PrintErrorID("CSTRNPSR",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"The ");
   PrintCLIPS(WERROR,attribute1);
   PrintCLIPS(WERROR," attribute conflicts with the ");
   PrintCLIPS(WERROR,attribute2);
   PrintCLIPS(WERROR," attribute.\n");
  }

#if (! RUN_TIME) && (! BLOAD_ONLY)

/***************************************************************************/
/* InitializeConstraintParseRecord: Initializes the values of a constraint */
/*   parse record which is used to determine whether one of the standard   */
/*   constraint specifications has already been parsed.                    */
/***************************************************************************/
globle VOID InitializeConstraintParseRecord(parsedConstraints)
  CONSTRAINT_PARSE_RECORD *parsedConstraints;
  {   
   parsedConstraints->type = CLIPS_FALSE;
   parsedConstraints->range = CLIPS_FALSE;
   parsedConstraints->allowedSymbols = CLIPS_FALSE;
   parsedConstraints->allowedStrings = CLIPS_FALSE;
   parsedConstraints->allowedLexemes = CLIPS_FALSE;
   parsedConstraints->allowedIntegers = CLIPS_FALSE;
   parsedConstraints->allowedFloats = CLIPS_FALSE;
   parsedConstraints->allowedNumbers = CLIPS_FALSE;
   parsedConstraints->allowedValues = CLIPS_FALSE;
   parsedConstraints->allowedInstanceNames = CLIPS_FALSE;
   parsedConstraints->cardinality = CLIPS_FALSE;
  }
  
/************************************************************************/
/* StandardConstraint: Returns TRUE if the specified name is one of the */
/*   standard constraints parseable by the routines in this module.     */
/************************************************************************/
globle BOOLEAN StandardConstraint(constraintName)
  char *constraintName;
  {   
   if ((strcmp(constraintName,"type") == 0) ||
       (strcmp(constraintName,"range") == 0) ||
       (strcmp(constraintName,"cardinality") == 0) ||
       (strcmp(constraintName,"allowed-symbols") == 0) ||
       (strcmp(constraintName,"allowed-strings") == 0) ||
       (strcmp(constraintName,"allowed-lexemes") == 0) ||
       (strcmp(constraintName,"allowed-integers") == 0) ||
       (strcmp(constraintName,"allowed-floats") == 0) ||
       (strcmp(constraintName,"allowed-numbers") == 0) ||
       (strcmp(constraintName,"allowed-instance-names") == 0) ||
       (strcmp(constraintName,"allowed-values") == 0))
       
     { return(CLIPS_TRUE); }
   
   return(CLIPS_FALSE);
  }
  
/***********************************************************************/
/* ParseStandardConstraint: Parses a standard constraint. Returns TRUE */
/*   if the constraint was successfully parsed, otherwise FALSE.       */
/***********************************************************************/
globle BOOLEAN ParseStandardConstraint(readSource,constraintName,
                                       constraints,parsedConstraints,
                                       multipleValuesAllowed)
  char *readSource;
  char *constraintName;
  CONSTRAINT_RECORD *constraints;
  CONSTRAINT_PARSE_RECORD *parsedConstraints;
  int multipleValuesAllowed;
  {   
   int rv = CLIPS_FALSE;
   
   /*=====================================================*/
   /* Determine if the attribute has already been parsed. */
   /*=====================================================*/
   
   if (GetAttributeParseValue(constraintName,parsedConstraints))
     {
      AlreadyParsedErrorMessage(constraintName," attribute");
      return(CLIPS_FALSE);
     }

   if (strcmp(constraintName,"range") == 0) 
     {
      rv = ParseRangeCardinalityAttribute(readSource,constraints,parsedConstraints,
                                          constraintName,multipleValuesAllowed);
     }
   else if (strcmp(constraintName,"cardinality") == 0) 
     {
      rv = ParseRangeCardinalityAttribute(readSource,constraints,parsedConstraints,
                                          constraintName,multipleValuesAllowed);
     }
   else if (strcmp(constraintName,"type") == 0) 
     {
      rv = ParseTypeAttribute(readSource,constraints);
     }
   else if ((strcmp(constraintName,"allowed-symbols") == 0) ||
            (strcmp(constraintName,"allowed-strings") == 0) ||
            (strcmp(constraintName,"allowed-lexemes") == 0) ||
            (strcmp(constraintName,"allowed-integers") == 0) ||
            (strcmp(constraintName,"allowed-floats") == 0) ||
            (strcmp(constraintName,"allowed-numbers") == 0) ||
            (strcmp(constraintName,"allowed-instance-names") == 0) ||
            (strcmp(constraintName,"allowed-values") == 0))
     {
      rv = ParseAllowedValuesAttribute(readSource,constraintName,
                                         constraints,parsedConstraints);
     }
     
   SetParseFlag(parsedConstraints,constraintName);
   return(rv);
  }

/***********************************************************/
/* OverlayConstraint: Overlays fields of source constraint */
/* record on destination based on which fields are set in  */
/* the parsed constraint record.                           */
/* Assumes AddConstraint has not yet been called for the   */
/* destination constraint record                           */
/***********************************************************/
globle VOID OverlayConstraint(pc,cdst,csrc)
  CONSTRAINT_PARSE_RECORD *pc;
  CONSTRAINT_RECORD *cdst,*csrc;
  {
   if (pc->type == 0)
     {
      cdst->anyAllowed = csrc->anyAllowed;
      cdst->symbolsAllowed = csrc->symbolsAllowed;
      cdst->stringsAllowed = csrc->stringsAllowed;
      cdst->floatsAllowed = csrc->floatsAllowed;
      cdst->integersAllowed = csrc->integersAllowed;
      cdst->instanceNamesAllowed = csrc->instanceNamesAllowed;
      cdst->instanceAddressesAllowed = csrc->instanceAddressesAllowed;
      cdst->externalAddressesAllowed = csrc->externalAddressesAllowed;
      cdst->factAddressesAllowed = csrc->factAddressesAllowed;
     }
   if (pc->range == 0)
     {
      ReturnExpression(cdst->minValue);
      ReturnExpression(cdst->maxValue);
      cdst->minValue = CopyExpression(csrc->minValue);
      cdst->maxValue = CopyExpression(csrc->maxValue);
     }
   if (pc->allowedValues == 0)
     {
      if ((pc->allowedSymbols == 0) &&
          (pc->allowedStrings == 0) &&
          (pc->allowedLexemes == 0) &&
          (pc->allowedIntegers == 0) &&
          (pc->allowedFloats == 0) &&
          (pc->allowedNumbers == 0) &&
          (pc->allowedInstanceNames == 0))
        {
         cdst->anyRestriction = csrc->anyRestriction;
         cdst->symbolRestriction = csrc->symbolRestriction;
         cdst->stringRestriction = csrc->stringRestriction;
         cdst->floatRestriction = csrc->floatRestriction;
         cdst->integerRestriction = csrc->integerRestriction;
         cdst->instanceNameRestriction = csrc->instanceNameRestriction;
         cdst->restrictionList = CopyExpression(csrc->restrictionList);
        }
      else
        {
         if ((pc->allowedSymbols == 0) && csrc->symbolRestriction)
           {
            cdst->symbolRestriction = 1;
            AddToRestrictionList(SYMBOL,cdst,csrc);
           }
         if ((pc->allowedStrings == 0) && csrc->stringRestriction)
           {
            cdst->stringRestriction = 1;
            AddToRestrictionList(STRING,cdst,csrc);
           }
         if ((pc->allowedLexemes == 0) && csrc->symbolRestriction && csrc->stringRestriction)
           {
            cdst->symbolRestriction = 1;
            cdst->stringRestriction = 1;
            AddToRestrictionList(SYMBOL,cdst,csrc);
            AddToRestrictionList(STRING,cdst,csrc);
           }
         if ((pc->allowedIntegers == 0) && csrc->integerRestriction)
           {
            cdst->integerRestriction = 1;
            AddToRestrictionList(INTEGER,cdst,csrc);
           }
         if ((pc->allowedFloats == 0) && csrc->floatRestriction)
           {
            cdst->floatRestriction = 1;
            AddToRestrictionList(FLOAT,cdst,csrc);
           }
         if ((pc->allowedNumbers == 0) && csrc->integerRestriction && csrc->floatRestriction)
           {
            cdst->integerRestriction = 1;
            cdst->floatRestriction = 1;
            AddToRestrictionList(INTEGER,cdst,csrc);
            AddToRestrictionList(FLOAT,cdst,csrc);
           }
         if ((pc->allowedInstanceNames == 0) && csrc->instanceNameRestriction)
           {
            cdst->instanceNameRestriction = 1;
            AddToRestrictionList(INSTANCE_NAME,cdst,csrc);
           }
        }
     }
     
   if (pc->cardinality == 0)
     {
      ReturnExpression(cdst->minFields);
      ReturnExpression(cdst->maxFields);
      cdst->minFields = CopyExpression(csrc->minFields);
      cdst->maxFields = CopyExpression(csrc->maxFields);
     }
  }

/************************************************************/
/* AddToRestrictionList: Prepends atoms of the specified    */
/* type from the source restriction list to the destination */
/************************************************************/
static VOID AddToRestrictionList(type,cdst,csrc)
  int type;
  CONSTRAINT_RECORD *cdst,*csrc;
  {
   struct expr *exp,*tmp;
   
   for (exp = csrc->restrictionList ; exp != NULL ; exp = exp->nextArg)
     {
      if (exp->type == type)
        {
         tmp = GenConstant(exp->type,exp->value);
         tmp->nextArg = cdst->restrictionList;
         cdst->restrictionList = tmp;
        }
     }
  } 
  
/***********************************************************/
/* ParseAllowedValuesAttribute: Parses the type attribute. */
/***********************************************************/
static BOOLEAN ParseAllowedValuesAttribute(readSource,constraintName,
                                           constraints,parsedConstraints)
  char *readSource;
  char *constraintName;
  CONSTRAINT_RECORD *constraints;
  CONSTRAINT_PARSE_RECORD *parsedConstraints;
  {
   struct token inputToken;
   int expectedType = UNKNOWN, error = CLIPS_FALSE;
   struct expr *newValue, *lastValue;
   int typeParsed = CLIPS_FALSE, variableParsed = CLIPS_FALSE;
   char *tempPtr;
        
   /*======================================================*/
   /* The allowed-values attribute is not allowed if other */
   /* allowed-... attributes have already been parsed.     */
   /*======================================================*/

   if ((strcmp(constraintName,"allowed-values") == 0) &&
       ((parsedConstraints->allowedSymbols) ||
        (parsedConstraints->allowedStrings) ||
        (parsedConstraints->allowedLexemes) ||
        (parsedConstraints->allowedIntegers) ||
        (parsedConstraints->allowedFloats) ||
        (parsedConstraints->allowedNumbers) ||
        (parsedConstraints->allowedInstanceNames)))
     {
      if (parsedConstraints->allowedSymbols) tempPtr = "allowed-symbols";
      else if (parsedConstraints->allowedStrings) tempPtr = "allowed-strings";
      else if (parsedConstraints->allowedLexemes) tempPtr = "allowed-lexemes";
      else if (parsedConstraints->allowedIntegers) tempPtr = "allowed-integers";
      else if (parsedConstraints->allowedFloats) tempPtr = "allowed-floats";
      else if (parsedConstraints->allowedNumbers) tempPtr = "allowed-numbers";
      else if (parsedConstraints->allowedInstanceNames) tempPtr = "allowed-instance-names";
      NoConjunctiveUseError("allowed-values",tempPtr);
      return(CLIPS_FALSE);
     }
     
   /*=======================================================*/
   /* The allowed-values/numbers/integers/floats attributes */
   /* are not allowed with the range attribute.             */
   /*=======================================================*/

   if (((strcmp(constraintName,"allowed-values") == 0) ||
        (strcmp(constraintName,"allowed-numbers") == 0) ||
        (strcmp(constraintName,"allowed-integers") == 0) ||
        (strcmp(constraintName,"allowed-floats") == 0)) &&
       (parsedConstraints->range))
     {      
      NoConjunctiveUseError(constraintName,"range");
      return(CLIPS_FALSE);
     }
        
   /*===================================================*/
   /* The allowed-... attributes are not allowed if the */
   /* allowed-values attribute has already been parsed. */
   /*===================================================*/

   if ((strcmp(constraintName,"allowed-values") != 0) && 
            (parsedConstraints->allowedValues))
     {
      NoConjunctiveUseError(constraintName,"allowed-values");
      return(CLIPS_FALSE);
     }

   /*==================================================*/
   /* The allowed-numbers attribute is not allowed if  */
   /* the allowed-integers or allowed-floats attribute */
   /* has already been parsed.                         */
   /*==================================================*/

   if ((strcmp(constraintName,"allowed-numbers") == 0) && 
       ((parsedConstraints->allowedFloats) || (parsedConstraints->allowedIntegers)))
     {
      if (parsedConstraints->allowedFloats) tempPtr = "allowed-floats";
      else tempPtr = "allowed-integers";
      NoConjunctiveUseError("allowed-numbers",tempPtr);
      return(CLIPS_FALSE);
     }
   
   /*============================================================*/
   /* The allowed-integers/floats attributes are not allowed if  */
   /* the allowed-numbers attribute has already been parsed.     */
   /*============================================================*/

   if (((strcmp(constraintName,"allowed-integers") == 0) ||
        (strcmp(constraintName,"allowed-floats") == 0)) &&
       (parsedConstraints->allowedNumbers))
     {
      NoConjunctiveUseError(constraintName,"allowed-number");
      return(CLIPS_FALSE);
     }
     
   /*==================================================*/
   /* The allowed-lexemes attribute is not allowed if  */
   /* the allowed-symbols or allowed-strings attribute */
   /* has already been parsed.                         */
   /*==================================================*/

   if ((strcmp(constraintName,"allowed-lexemes") == 0) && 
       ((parsedConstraints->allowedSymbols) || (parsedConstraints->allowedStrings)))
     {
      if (parsedConstraints->allowedSymbols) tempPtr = "allowed-symbols";
      else tempPtr = "allowed-strings";
      NoConjunctiveUseError("allowed-lexemes",tempPtr);
      return(CLIPS_FALSE);
     }
   
   /*============================================================*/
   /* The allowed-symbols/strings attributes are not allowed if  */
   /* the allowed-lexemes attribute has already been parsed.     */
   /*============================================================*/

   if (((strcmp(constraintName,"allowed-symbols") == 0) ||
        (strcmp(constraintName,"allowed-strings") == 0)) &&
       (parsedConstraints->allowedLexemes))
     {
      NoConjunctiveUseError(constraintName,"allowed-lexemes");
      return(CLIPS_FALSE);
     }

   /*========================*/
   /* Get the expected type. */
   /*========================*/
 
   expectedType = GetConstraintTypeFromAllowedName(constraintName);
   SetRestrictionFlag(expectedType,constraints,CLIPS_TRUE);

   /*==================================================*/
   /* Get the allowed values and add them to the list. */
   /*==================================================*/

   SavePPBuffer(" ");
   GetToken(readSource,&inputToken);
 
   lastValue = constraints->restrictionList;
   if (lastValue != NULL)
     { while (lastValue->nextArg != NULL) lastValue = lastValue->nextArg; }
     
   while (inputToken.type != RPAREN)
     {
      SavePPBuffer(" ");

      /*=================*/
      /* Check the type. */
      /*=================*/

      switch(inputToken.type)
        {
         case INTEGER:
           if ((expectedType != UNKNOWN) && 
               (expectedType != INTEGER) &&
               (expectedType != INTEGER_OR_FLOAT)) error = CLIPS_TRUE;
           typeParsed = CLIPS_TRUE;
           break;
         
         case FLOAT:
           if ((expectedType != UNKNOWN) && 
               (expectedType != FLOAT) &&
               (expectedType != INTEGER_OR_FLOAT)) error = CLIPS_TRUE;
           typeParsed = CLIPS_TRUE;
           break;
         
         case STRING:
           if ((expectedType != UNKNOWN) && 
               (expectedType != STRING) &&
               (expectedType != SYMBOL_OR_STRING)) error = CLIPS_TRUE;
           typeParsed = CLIPS_TRUE;
           break;
         
         case SYMBOL:
           if ((expectedType != UNKNOWN) && 
               (expectedType != SYMBOL) &&
               (expectedType != SYMBOL_OR_STRING)) error = CLIPS_TRUE;
           typeParsed = CLIPS_TRUE;
           break;
         
#if OBJECT_SYSTEM
         case INSTANCE_NAME:
           if ((expectedType != UNKNOWN) && 
               (expectedType != INSTANCE_NAME)) error = CLIPS_TRUE;
           typeParsed = CLIPS_TRUE;
           break;
#endif

         case SF_VARIABLE:
           if (strcmp(inputToken.printForm,"?VARIABLE") == 0) 
             { variableParsed = CLIPS_TRUE; }
           else 
             {
              char tempBuffer[120];
              sprintf(tempBuffer,"%s attribute",constraintName);
              SyntaxErrorMessage(tempBuffer);
              return(CLIPS_FALSE);
             }
             
           break;

         default:
           {
            char tempBuffer[120];
            sprintf(tempBuffer,"%s attribute",constraintName);
            SyntaxErrorMessage(tempBuffer);
           }
           return(CLIPS_FALSE);
        }

      if (typeParsed && variableParsed)
        {
         char tempBuffer[120];
         sprintf(tempBuffer,"%s attribute",constraintName);
         SyntaxErrorMessage(tempBuffer);
         return(CLIPS_FALSE);
        }      
        
      if (error)
        {
         PrintErrorID("CSTRNPSR",4,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Value does not match the expected type for the ");
         PrintCLIPS(WERROR,constraintName);
         PrintCLIPS(WERROR," attribute\n");
         return(CLIPS_FALSE);
        }

      /*===========================*/
      /* Add the type restriction. */
      /*===========================*/

      newValue = GenConstant(inputToken.type,inputToken.value);
      if (lastValue == NULL)
        { constraints->restrictionList = newValue; }
      else 
        { lastValue->nextArg = newValue; }
      lastValue = newValue;

      /*====================================*/
      /* Begin parsing the next type value. */
      /*====================================*/

      GetToken(readSource,&inputToken);
     }

   /*======================================================*/
   /* There must be at least one value for this attribute. */
   /*======================================================*/

   if ((! typeParsed) && (! variableParsed))
     {
      char tempBuffer[120];
      sprintf(tempBuffer,"%s attribute",constraintName);
      SyntaxErrorMessage(tempBuffer);
      return(CLIPS_FALSE);
     }

   /*======================================*/
   /* If ?VARIABLE was parsed, then remove */
   /* the restrictions for that type.      */
   /*======================================*/

   if (variableParsed)
     {
      switch(expectedType)
        {
         case UNKNOWN:
           constraints->anyRestriction = CLIPS_FALSE;
           break;

         case SYMBOL:
           constraints->symbolRestriction = CLIPS_FALSE;
           break;

         case STRING:
           constraints->stringRestriction = CLIPS_FALSE;
           break;

         case INTEGER:
           constraints->integerRestriction = CLIPS_FALSE;
           break;

         case FLOAT:
           constraints->floatRestriction = CLIPS_FALSE;
           break;

         case INTEGER_OR_FLOAT:
           constraints->floatRestriction = CLIPS_FALSE;
           constraints->integerRestriction = CLIPS_FALSE;
           break;
           
         case SYMBOL_OR_STRING:
           constraints->symbolRestriction = CLIPS_FALSE;
           constraints->stringRestriction = CLIPS_FALSE;
           break;
           
         case INSTANCE_NAME:
           constraints->instanceNameRestriction = CLIPS_FALSE;
           break;
        }
     }

   /*=====================================*/
   /* Fix up pretty print representation. */
   /*=====================================*/

   PPBackup();
   PPBackup();
   SavePPBuffer(")");

   return(1);
  }     

/**************************************************/
/* ParseTypeAttribute: Parses the type attribute. */
/**************************************************/
static VOID NoConjunctiveUseError(attribute1,attribute2)
  char *attribute1;
  char *attribute2;
  {
   PrintErrorID("CSTRNPSR",3,CLIPS_TRUE);
   PrintCLIPS(WERROR,"The ");
   PrintCLIPS(WERROR,attribute1);
   PrintCLIPS(WERROR," attribute cannot be used\n");
   PrintCLIPS(WERROR,"in conjunction with the ");
   PrintCLIPS(WERROR,attribute2);
   PrintCLIPS(WERROR," attribute.\n");
  }
      
/**************************************************/
/* ParseTypeAttribute: Parses the type attribute. */
/**************************************************/
static BOOLEAN ParseTypeAttribute(readSource,constraints)
  char *readSource;
  CONSTRAINT_RECORD *constraints;
  {
   int typeParsed = CLIPS_FALSE;
   int variableParsed = CLIPS_FALSE;
   int theType;
   struct token inputToken;

   SavePPBuffer(" ");
   GetToken(readSource,&inputToken);

   while (inputToken.type != RPAREN)
     {
      SavePPBuffer(" ");

      if (inputToken.type == SYMBOL)
        {
         if (variableParsed == CLIPS_TRUE)
           {
            SyntaxErrorMessage("type attribute");
            return(CLIPS_FALSE);
           }
         
         theType = GetConstraintTypeFromTypeName(ValueToString(inputToken.value));
         if (theType < 0)
           {
            SyntaxErrorMessage("type attribute");
            return(CLIPS_FALSE);
           }
              
         if (SetConstraintType(theType,constraints))
           {
            SyntaxErrorMessage("type attribute");
            return(CLIPS_FALSE);
           }

         constraints->anyAllowed = CLIPS_FALSE;
         typeParsed = CLIPS_TRUE;
        }
      else if (inputToken.type == SF_VARIABLE)
        {
         if (strcmp(inputToken.printForm,"?VARIABLE") == 0)
           {
            if (typeParsed == CLIPS_TRUE)
              {
               SyntaxErrorMessage("type attribute");
               return(CLIPS_FALSE);
              }

            typeParsed = CLIPS_TRUE;
            variableParsed = CLIPS_TRUE;
           }
         else
           {
            SyntaxErrorMessage("type attribute");
            return(CLIPS_FALSE);
           }
         }
      else
        {
         SyntaxErrorMessage("type attribute");
         return(CLIPS_FALSE);
        }

      /*====================================*/
      /* Begin parsing the next type value. */
      /*====================================*/

      GetToken(readSource,&inputToken);
     }

   /*=====================================*/
   /* Fix up pretty print representation. */
   /*=====================================*/

   PPBackup();
   PPBackup();
   SavePPBuffer(")");

   /*=======================================*/
   /* The type attribute must have a value. */
   /*=======================================*/

   if ((! typeParsed) && (! variableParsed))
     {
      SyntaxErrorMessage("type attribute");
      return(CLIPS_FALSE);
     }

   return(CLIPS_TRUE);
  }
  
/***************************************************************************/
/* ParseRangeCardinalityAttribute: Parses the range/cardinality attribute. */
/***************************************************************************/
static BOOLEAN ParseRangeCardinalityAttribute(readSource,constraints,parsedConstraints,
                                              constraintName,multipleValuesAllowed)
  char *readSource;
  CONSTRAINT_RECORD *constraints;
  CONSTRAINT_PARSE_RECORD *parsedConstraints;
  char *constraintName;
  int multipleValuesAllowed;
  {
   struct token inputToken;
   int range;
   char *tempPtr;

   if (strcmp(constraintName,"range") == 0) 
     {
      parsedConstraints->range = CLIPS_TRUE;
      range = CLIPS_TRUE;
     }
   else
     {
      parsedConstraints->cardinality = CLIPS_TRUE;
      range = CLIPS_FALSE;
     }
   
   /*===================================================================*/
   /* The cardinality attribute can only be used with multifield slots. */
   /*===================================================================*/
   
   if ((range == CLIPS_FALSE) &&
       (multipleValuesAllowed == CLIPS_FALSE))
     {
      PrintErrorID("CSTRNPSR",5,CLIPS_TRUE);
      PrintCLIPS(WERROR,"The cardinality attribute ");
      PrintCLIPS(WERROR,"can only be used with multifield slots.\n");
      return(CLIPS_FALSE);
     }
     
   /*====================================================*/
   /* The range attribute is not allowed with the        */
   /* allowed-values/numbers/integers/floats attributes. */
   /*====================================================*/

   if ((range == CLIPS_TRUE) && 
       (parsedConstraints->allowedValues ||
        parsedConstraints->allowedNumbers ||
        parsedConstraints->allowedIntegers ||
        parsedConstraints->allowedFloats))
     {
      if (parsedConstraints->allowedValues) tempPtr = "allowed-values";
      else if (parsedConstraints->allowedIntegers) tempPtr = "allowed-integers";
      else if (parsedConstraints->allowedFloats) tempPtr = "allowed-floats";
      else if (parsedConstraints->allowedNumbers) tempPtr = "allowed-numbers";
      NoConjunctiveUseError("range",tempPtr);
      return(CLIPS_FALSE);
     }

   /*==========================*/
   /* Parse the minimum value. */
   /*==========================*/

   SavePPBuffer(" ");
   GetToken(readSource,&inputToken);
   if ((inputToken.type == INTEGER) || ((inputToken.type == FLOAT) && range))
     { 
      if (range)
        {
         ReturnExpression(constraints->minValue);
         constraints->minValue = GenConstant(inputToken.type,inputToken.value); 
        }
      else
        {
         ReturnExpression(constraints->minFields);
         constraints->minFields = GenConstant(inputToken.type,inputToken.value); 
        }
     }
   else if ((inputToken.type == SF_VARIABLE) && (strcmp(inputToken.printForm,"?VARIABLE") == 0))
     { /* Do nothing. */ }
   else
     {
      char tempBuffer[120];
      sprintf(tempBuffer,"%s attribute",constraintName);
      SyntaxErrorMessage(tempBuffer);
      return(CLIPS_FALSE);
     }

   /*==========================*/
   /* Parse the maximum value. */
   /*==========================*/

   SavePPBuffer(" ");
   GetToken(readSource,&inputToken);
   if ((inputToken.type == INTEGER) || ((inputToken.type == FLOAT) && range))
     {
      if (range)
        {
         ReturnExpression(constraints->maxValue);
         constraints->maxValue = GenConstant(inputToken.type,inputToken.value); 
        }
      else
        {
         ReturnExpression(constraints->maxFields);
         constraints->maxFields = GenConstant(inputToken.type,inputToken.value); 
        }
     }
   else if ((inputToken.type == SF_VARIABLE) && (strcmp(inputToken.printForm,"?VARIABLE") == 0))
     { /* Do nothing. */ }
   else
     {
      char tempBuffer[120];
      sprintf(tempBuffer,"%s attribute",constraintName);
      SyntaxErrorMessage(tempBuffer);
      return(CLIPS_FALSE);
     }

   /*================================*/
   /* Parse the closing parenthesis. */
   /*================================*/

   GetToken(readSource,&inputToken);
   if (inputToken.type != RPAREN)
     {
      SyntaxErrorMessage("range attribute");
      return(CLIPS_FALSE);
     }

   /*====================================================*/
   /* Minimum value must be less than the maximum value. */
   /*====================================================*/

   if (range)
     {
      if (CompareNumbers(constraints->minValue->type,
                         constraints->minValue->value,
                         constraints->maxValue->type,
                         constraints->maxValue->value) == GREATER_THAN)
        {
         PrintErrorID("CSTRNPSR",2,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Minimum range value must be less than\n");
         PrintCLIPS(WERROR,"or equal to the maximum range value\n");
         return(CLIPS_FALSE);
        }
     }
   else
     {
      if (CompareNumbers(constraints->minFields->type,
                         constraints->minFields->value,
                         constraints->maxFields->type,
                         constraints->maxFields->value) == GREATER_THAN)
        {
         PrintErrorID("CSTRNPSR",2,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Minimum cardinality value must be less than\n");
         PrintCLIPS(WERROR,"or equal to the maximum cardinality value\n");
         return(CLIPS_FALSE);
        }
     }

   return(CLIPS_TRUE);
  }

/********************************************************************/
/* GetConstraintTypeFromAllowedName:                                */
/********************************************************************/
static int GetConstraintTypeFromAllowedName(constraintName)
  char *constraintName;
  {
   if (strcmp(constraintName,"allowed-values") == 0) return(UNKNOWN);
   else if (strcmp(constraintName,"allowed-symbols") == 0) return(SYMBOL);
   else if (strcmp(constraintName,"allowed-strings") == 0) return(STRING);
   else if (strcmp(constraintName,"allowed-lexemes") == 0) return(SYMBOL_OR_STRING);
   else if (strcmp(constraintName,"allowed-integers") == 0) return(INTEGER);
   else if (strcmp(constraintName,"allowed-numbers") == 0) return(INTEGER_OR_FLOAT);
   else if (strcmp(constraintName,"allowed-instance-names") == 0) return(INSTANCE_NAME);
   else if (strcmp(constraintName,"allowed-floats") == 0) return(FLOAT);

   return(-1);
  }
  
/********************************************************************/
/* GetConstraintTypeFromTypeName:                                */
/********************************************************************/
static int GetConstraintTypeFromTypeName(constraintName)
  char *constraintName;
  {
   if (strcmp(constraintName,"SYMBOL") == 0) return(SYMBOL);
   else if (strcmp(constraintName,"STRING") == 0) return(STRING);
   else if (strcmp(constraintName,"LEXEME") == 0) return(SYMBOL_OR_STRING);
   else if (strcmp(constraintName,"INTEGER") == 0) return(INTEGER);
   else if (strcmp(constraintName,"FLOAT") == 0) return(FLOAT);
   else if (strcmp(constraintName,"NUMBER") == 0) return(INTEGER_OR_FLOAT);
   else if (strcmp(constraintName,"INSTANCE-NAME") == 0) return(INSTANCE_NAME);
   else if (strcmp(constraintName,"INSTANCE-ADDRESS") == 0) return(INSTANCE_ADDRESS);
   else if (strcmp(constraintName,"INSTANCE") == 0) return(INSTANCE_OR_INSTANCE_NAME);
   else if (strcmp(constraintName,"EXTERNAL-ADDRESS") == 0) return(EXTERNAL_ADDRESS);
   else if (strcmp(constraintName,"FACT-ADDRESS") == 0) return(FACT_ADDRESS);

   return(-1);
  }

/********************************************************************/
/* GetAttributeParseValue:                          */
/********************************************************************/
static int GetAttributeParseValue(constraintName,parsedConstraints)
  char *constraintName;
  CONSTRAINT_PARSE_RECORD *parsedConstraints;
  {
   if (strcmp(constraintName,"type") == 0) 
     { return(parsedConstraints->type); }
   else if (strcmp(constraintName,"range") == 0) 
     { return(parsedConstraints->range); }
   else if (strcmp(constraintName,"cardinality") == 0) 
     { return(parsedConstraints->cardinality); }
   else if (strcmp(constraintName,"allowed-values") == 0) 
     { return(parsedConstraints->allowedValues); }
   else if (strcmp(constraintName,"allowed-symbols") == 0)
     { return(parsedConstraints->allowedSymbols); }
   else if (strcmp(constraintName,"allowed-strings") == 0) 
     { return(parsedConstraints->allowedStrings); }
   else if (strcmp(constraintName,"allowed-lexemes") == 0) 
     { return(parsedConstraints->allowedLexemes); }
   else if (strcmp(constraintName,"allowed-instance-names") == 0) 
     { return(parsedConstraints->allowedInstanceNames); }
   else if (strcmp(constraintName,"allowed-integers") == 0)
     { return(parsedConstraints->allowedIntegers); }
   else if (strcmp(constraintName,"allowed-floats") == 0) 
     { return(parsedConstraints->allowedFloats); }
   else if (strcmp(constraintName,"allowed-numbers") == 0) 
     { return(parsedConstraints->allowedNumbers); }
   
   return(CLIPS_TRUE);
  }
  
/********************************************************************/
/* SetRestrictionFlag                        */
/********************************************************************/
static VOID SetRestrictionFlag(restriction,constraints,value)
  int restriction;
  CONSTRAINT_RECORD *constraints;
  int value;
  {
   switch (restriction)
     {
      case UNKNOWN:
         constraints->anyRestriction = value;
         break;
         
      case SYMBOL:
         constraints->symbolRestriction = value;
         break;
         
      case STRING:
         constraints->stringRestriction = value;
         break;
         
      case INTEGER:
         constraints->integerRestriction = value;
         break;
         
      case FLOAT:
         constraints->floatRestriction = value;
         break;
         
      case INTEGER_OR_FLOAT:
         constraints->integerRestriction = value;
         constraints->floatRestriction = value; 
         break; 
         
      case SYMBOL_OR_STRING:
         constraints->symbolRestriction = value;
         constraints->stringRestriction = value; 
         break;   
         
      case INSTANCE_NAME:
         constraints->instanceNameRestriction = value;
         break;      
     }
  }
  
/********************************************************************/
/* SetParseFlag:                        */
/********************************************************************/
static VOID SetParseFlag(parsedConstraints,constraintName)
  CONSTRAINT_PARSE_RECORD *parsedConstraints;
  char *constraintName;
  {
   if (strcmp(constraintName,"range") == 0) 
     { parsedConstraints->range = CLIPS_TRUE; }
   else if (strcmp(constraintName,"type") == 0) 
     { parsedConstraints->type = CLIPS_TRUE; }
   else if (strcmp(constraintName,"cardinality") == 0) 
     { parsedConstraints->cardinality = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-symbols") == 0)
     { parsedConstraints->allowedSymbols = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-strings") == 0) 
     { parsedConstraints->allowedStrings = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-lexemes") == 0)
     { parsedConstraints->allowedLexemes = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-integers") == 0) 
     { parsedConstraints->allowedIntegers = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-floats") == 0) 
     { parsedConstraints->allowedFloats = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-numbers") == 0)
     { parsedConstraints->allowedNumbers = CLIPS_TRUE; }
   else if (strcmp(constraintName,"allowed-values") == 0) 
     { parsedConstraints->allowedValues = CLIPS_TRUE; }
  }
  

#endif

