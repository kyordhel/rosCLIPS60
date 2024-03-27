   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              CONSTRUCT COMPILER MODULE              */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*      Barry Cameron                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CONSCOMP_SOURCE_

#include "setup.h"

#if CONSTRUCT_COMPILER && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "symbol.h"
#include "clipsmem.h"
#include "constant.h"
#include "exprnpsr.h"
#include "cstrccom.h"
#include "constrct.h"
#include "argacces.h"
#include "cstrncmp.h"
#include "router.h"
#include "utility.h"
#include "modulcmp.h"

#if DEFRULE_CONSTRUCT
#include "network.h"
#endif

#if DEFFUNCTION_CONSTRUCT
#include "dffnxcmp.h"
#endif

#if DEFTEMPLATE_CONSTRUCT
#include "tmpltcmp.h"
#endif

#if DEFGLOBAL_CONSTRUCT
#include "globlcmp.h"
#endif

#if DEFGENERIC_CONSTRUCT
#include "genrccmp.h"
#endif

#if OBJECT_SYSTEM
#include "objcmp.h"
#endif

#include "conscomp.h"

   /* ==============================================
      The codes F,I,B,S,E,P,L and C are not included
      because those are already taken:
      
      B: BitMap hash nodes
      C: Constraint hash nodes
      E: Expression hash nodes
      F: Float hash nodes
      I: Integer hash nodes
      L: Bitmaps
      P: Functions
      S: Symbol hash nodes
      ============================================== */
      
#define PRIMARY_CODES   "ADGHJKMNOQRTUVWXYZ"
#define PRIMARY_LEN     18
#define SECONDARY_CODES "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
#define SECONDARY_LEN   26

#define FSIZE 80

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   VOID                               ConstructsToCCommand(void);
   static int                         GenerateCode(char *,int,int);
   static VOID                        ListUserFunctions(FILE *);
   static int                         FunctionsToCode(char *);
   static int                         SetUpInitFile(char *);
   static VOID                        DumpExpression(struct expr *);
   static VOID                        MarkConstruct(struct constructHeader *,VOID *);
   static VOID                        HashedExpressionsToCode(VOID);
#else
   VOID                               ConstructsToCCommand();
   static int                         GenerateCode();
   static VOID                        ListUserFunctions();
   static int                         FunctionsToCode();
   static int                         SetUpInitFile();
   static VOID                        DumpExpression();
   static VOID                        MarkConstruct();
   static VOID                        HashedExpressionsToCode();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle int                       ImageID;
   globle FILE                     *HeaderFP;
   globle int                       MaxIndices = 2000;
   
/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static FILE                     *ExpressionFP;
   static char                     *FilePrefix;
   static BOOLEAN                   ExpressionHeader;
   static int                       ExpressionCount;
   static int                       ExpressionVersion;
   static struct CodeGeneratorItem *ListOfCodeGeneratorItems = NULL;

/*********************************************************/
/* ConstructsToCCommand:  Converts the rule network into the */
/*   corresponding C data structures.                    */
/*********************************************************/
globle VOID ConstructsToCCommand()
  {
   char *fileName;
   DATA_OBJECT theArg;
   int argCount;
   int id, max;
#if VAX_VMS || IBM_MSC || IBM_TBC || IBM_ICB || IBM_ZTC 
   int i;
#endif

   /*============================================*/
   /* Check for appropriate number of arguments. */
   /*============================================*/

   if ((argCount = ArgRangeCheck("constructs-to-c",2,3)) == -1) return;

   /*====================================*/
   /* Get the file name to place C code. */
   /*====================================*/

   if (ArgTypeCheck("constructs-to-c",1,SYMBOL_OR_STRING,&theArg) == CLIPS_FALSE)
     { return; }

   fileName = DOToString(theArg);

#if VAX_VMS || IBM_MSC || IBM_TBC || IBM_ICB || IBM_ZTC
   /*Check for '.' VAX or IBM PC fileName */
   for (i = 0 ; *(fileName+i) ; i++)
     {
      if (*(fileName+i) == '.')
        {
         PrintErrorID("CONSCOMP",1,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Invalid file name ");
         PrintCLIPS(WERROR,fileName);
         PrintCLIPS(WERROR," contains \'.\'\n");
         return;
        }
      }
#endif

   if (strlen(fileName) > 3)
     {
      PrintWarningID("CONSCOMP",1,CLIPS_FALSE);
      PrintCLIPS(WWARNING,"Base file name exceeds 3 characters.\n");
      PrintCLIPS(WWARNING,"  This may cause files to be overwritten if file name length\n");
      PrintCLIPS(WWARNING,"  is limited on your platform.\n");
     }
     
   /*==============================*/
   /* Get the remaining arguments. */
   /*==============================*/

   if (ArgTypeCheck("constructs-to-c",2,INTEGER,&theArg) == CLIPS_FALSE)
     { return; }

   id = DOToInteger(theArg);
   if (id < 0)
     {
      ExpectedTypeError1("constructs-to-c",2,"positive integer");
      return;
     }

   if (argCount == 3)
     {
      if (ArgTypeCheck("constructs-to-c",3,INTEGER,&theArg) == CLIPS_FALSE)
        { return; }

      max = DOToInteger(theArg);

      if (max < 0)
        {
         ExpectedTypeError1("constructs-to-c",3,"positive integer");
         return;
        }
     }
   else
     { max = 10000; }

   GenerateCode(fileName,id,max); 
  }

/********************************************************/
/* GenerateCode:                                        */
/********************************************************/
static int GenerateCode(fileName,theImageID,max)
  char *fileName;
  int theImageID, max;
  {
   char fname[FSIZE];
   int fileVersion;
   struct CodeGeneratorItem *cgPtr;

   MaxIndices = max;

   /*===================================================*/
   /* Call the before functions before generating code. */
   /*===================================================*/
   
   cgPtr = ListOfCodeGeneratorItems;
   while (cgPtr != NULL)
     {
      if (cgPtr->beforeFunction != NULL) (*cgPtr->beforeFunction)();
      cgPtr = cgPtr->next;
     }

   /*=================================================*/
   /* Do a periodic Cleanup without using heuristics. */
   /*=================================================*/

   PeriodicCleanup(CLIPS_FALSE,CLIPS_FALSE);

   FilePrefix = fileName;
   ImageID = theImageID;
   ExpressionFP = NULL;
   ExpressionVersion = 1;
   ExpressionHeader = CLIPS_TRUE;
   ExpressionCount = 0;

   /*=====================================================*/
   /* Open a header file for dumping general information. */
   /*=====================================================*/

   sprintf(fname,"%s.h",fileName);
   if ((HeaderFP = fopen(fname,"w")) == NULL)
     {
      OpenErrorMessage("constructs-to-c",fname);
      return(0);
     }
   
   fprintf(HeaderFP,"#ifndef _CONSTRUCT_COMPILER_HEADER_\n");
   fprintf(HeaderFP,"#define _CONSTRUCT_COMPILER_HEADER_\n\n");

   fprintf(HeaderFP,"#include <stdio.h>\n");
   fprintf(HeaderFP,"#include \"setup.h\"\n");
   fprintf(HeaderFP,"#include \"expressn.h\"\n");
   fprintf(HeaderFP,"#include \"extnfunc.h\"\n");
   fprintf(HeaderFP,"#include \"clips.h\"\n");
   fprintf(HeaderFP,"\n#define VS (VOID *)\n");
   fprintf(HeaderFP,"\n");

   /*=========================================================*/
   /* Give extern declarations for user and system functions. */
   /*=========================================================*/

   ListUserFunctions(HeaderFP);

   fprintf(HeaderFP,"\n#endif\n\n");
   fprintf(HeaderFP,"/****************************/\n");
   fprintf(HeaderFP,"/* EXTERN ARRAY DEFINITIONS */\n");
   fprintf(HeaderFP,"/****************************/\n\n");

   /*===============================================*/
   /* Give definitions of the external and internal */
   /* functions to be accessed by test structures   */
   /* and function definition structures.           */
   /*===============================================*/

   AtomicValuesToCode(fileName);

   FunctionsToCode(fileName);
   
   HashedExpressionsToCode();
   
   ConstraintsToCode(fileName,4,HeaderFP,ImageID,MaxIndices);
   
   /*===================================================================*/
   /* Call each code generator item (e.g. for deffacts, defrules, etc). */
   /*===================================================================*/
   
   fileVersion = 5;
   cgPtr = ListOfCodeGeneratorItems;
   while (cgPtr != NULL)
     {
      if (cgPtr->generateFunction != NULL)
        {
         (*cgPtr->generateFunction)(fileName,fileVersion,HeaderFP,ImageID,MaxIndices);
         fileVersion++;
        }
      cgPtr = cgPtr->next;
     }

   RestoreAtomicValueBuckets();

   if (ExpressionFP != NULL)
     {
      fprintf(ExpressionFP,"};\n");
      fclose(ExpressionFP);
     }


   /*====================================*/
   /* Write the initialization function. */
   /*====================================*/

   SetUpInitFile(fileName);

   fclose(HeaderFP); 
   
   return(1);
  }

/**********************************************************/
/* ListUserFunctions:  Loop through the list of function  */
/*    definitions, and define them as external functions. */
/**********************************************************/
static VOID ListUserFunctions(fp)
  FILE *fp;
  {
   struct FunctionDefinition *fctn_ptr;

   fprintf(fp,"\n");
   fprintf(fp,"/************************************/\n");
   fprintf(fp,"/* EXTERNAL FUNCTION DEFINITIONS    */\n");
   fprintf(fp,"/************************************/\n\n");

   fctn_ptr = GetFunctionList();
   while (fctn_ptr != NULL)
     {
      fprintf(fp,"extern ");
      switch(fctn_ptr->returnValueType)
        {
         case 'i':
         case 'b':
           fprintf(fp,"int ");
           break;

         case 'l':
           fprintf(fp,"long ");
           break;

         case 'f':
           fprintf(fp,"float ");
           break;

         case 'd':
           fprintf(fp,"double ");
           break;

     
         case 'w':
         case 's':
         case 'o':
           fprintf(fp,"SYMBOL_HN *");
           break;

         case 'c':
           fprintf(fp,"char ");
           break;

         case 'a':
         case 'x':
           fprintf(fp,"VOID * ");
           break;

         case 'v':
         case 'm':
         case 'u':
         case 'n':
         case 'j':
         case 'k':
           fprintf(fp,"VOID ");
           break;
           
         default:
           CLIPSSystemError("CONSCOMP",1);
           break;
        }

      fprintf(fp,"%s(",fctn_ptr->actualFunctionName);
      switch(fctn_ptr->returnValueType)
        {
         case 'i':
         case 'b':
         case 'l':
         case 'f':
         case 'd':
         case 'w':
         case 's':
         case 'o':
         case 'c':
         case 'a':
         case 'x':
         case 'v':
           fprintf(fp,"VOID_ARG");
           break;

         case 'm':
         case 'u':
         case 'n':
         case 'j': 
         case 'k':
           fprintf(fp,"DATA_OBJECT_PTR_ARG");
           break;
        }
        
      fprintf(fp,");\n");
      
      fctn_ptr = fctn_ptr->next;
     }
  }

/*****************************************************/
/* FunctionsToCode: Produce the structure definitions */
/*   for the function list.                          */
/*****************************************************/
static int FunctionsToCode(fileName)
  char *fileName;
  {
   int i = 0;
   FILE *fp;
   int version = 1;
   int newHeader = CLIPS_TRUE;
   struct FunctionDefinition *fctnPtr;

   fctnPtr = GetFunctionList();
   while (fctnPtr != NULL)
     {
      fctnPtr->bsaveIndex = i++;
      fctnPtr = fctnPtr->next;
     }

   /*==================*/
   /* Create the file. */
   /*==================*/

   if ((fp = NewCFile(fileName,2,version,CLIPS_FALSE)) == NULL) return(0);

   /*===============================================*/
   /* Construct the definition of the function list */
   /* from the definitions of the functions.        */
   /*===============================================*/

   fprintf(fp,"\n\n");
   fprintf(fp,"/************************************/\n");
   fprintf(fp,"/* FUNCTION LIST DEFINITION         */\n");
   fprintf(fp,"/************************************/\n\n");

   i = 1;
   fctnPtr = GetFunctionList();
   while (fctnPtr != NULL)
     {
      if (newHeader)
        {
         fprintf(fp,"struct FunctionDefinition P%d_%d[] = {\n",ImageID,version);
         fprintf(HeaderFP,"extern struct FunctionDefinition P%d_%d[];\n",ImageID,version);
         newHeader = CLIPS_FALSE;
        }

      fprintf(fp,"{");
      PrintSymbolReference(fp,fctnPtr->callFunctionName);
      fprintf(fp,",\"%s\",",fctnPtr->actualFunctionName);
      fprintf(fp,"'%c',",fctnPtr->returnValueType);
      fprintf(fp,"PTIF %s,",fctnPtr->actualFunctionName);
      fprintf(fp,"NULL,");
      if (fctnPtr->restrictions != NULL) fprintf(fp,"\"%s\",",fctnPtr->restrictions);
      else fprintf(fp,"NULL,");
      fprintf(fp,"0,0,0,");

      PrintFunctionReference(fp,fctnPtr->next);

      i++;
      fctnPtr = fctnPtr->next;
      if ((i > MaxIndices) || (fctnPtr == NULL))
        {
         fprintf(fp,"}};\n");
         fclose(fp);
         i = 1;
         version++;
         if (fctnPtr != NULL)
           {
            if ((fp = NewCFile(fileName,2,version,CLIPS_FALSE)) == NULL) return(0);
            newHeader = CLIPS_TRUE;
           }
        }
      else
        { fprintf(fp,"},\n"); }
     }

   return(1);
  }


/************************************************************/
/* PrintFunctionReference:                          */
/************************************************************/
globle VOID PrintFunctionReference(fp,funcPtr)
  FILE *fp;
  struct FunctionDefinition *funcPtr;
  {
   if (funcPtr == NULL) fprintf(fp,"NULL");
   else
      fprintf(fp,"&P%d_%d[%d]",ImageID,
                                  (funcPtr->bsaveIndex / MaxIndices) + 1,
                                   funcPtr->bsaveIndex % MaxIndices);
  }

/************************************************************/
/* SetUpInitFile: Produces the initialization function */
/*   for installing this rule set.                          */
/************************************************************/
static int SetUpInitFile(fileName)
  char *fileName;
  {
   char fname[FSIZE];
   FILE *fp;
   struct CodeGeneratorItem *cgPtr;

   sprintf(fname,"%s.c",fileName);
   if ((fp = fopen(fname,"w")) == NULL)
     {
      OpenErrorMessage("constructs-to-c",fname);
      return(0);
     }

   fprintf(fp,"#include \"%s.h\"\n",fileName);
   fprintf(fp,"\n");
   fprintf(fp,"#include \"utility.h\"\n");
   fprintf(fp,"#include \"generate.h\"\n");
   fprintf(fp,"#include \"expressn.h\"\n"); 
   fprintf(fp,"#include \"extnfunc.h\"\n");
   fprintf(fp,"#include \"objrtmch.h\"\n");
   fprintf(fp,"#include \"rulebld.h\"\n\n");

   fprintf(HeaderFP,"#if ANSI_COMPILER\n");
   fprintf(HeaderFP,"   VOID InitCImage_%d(void);\n",ImageID);
   fprintf(HeaderFP,"#else\n");
   fprintf(HeaderFP,"   VOID InitCImage_%d();\n",ImageID);
   fprintf(HeaderFP,"#endif\n");

   fprintf(fp,"\n");
   fprintf(fp,"/*******************************************/\n");
   fprintf(fp,"/* CONSTRUCT IMAGE INITIALIZATION FUNCTION */\n");
   fprintf(fp,"/*******************************************/\n");

   fprintf(fp,"\nVOID InitCImage_%d()\n",ImageID);
   fprintf(fp,"  {\n");

   fprintf(fp,"   Clear();\n");
   fprintf(fp,"   PeriodicCleanup(CLIPS_TRUE,CLIPS_FALSE);\n");
   fprintf(fp,"   SetSymbolTable(sht%d);\n",ImageID);
   fprintf(fp,"   SetFloatTable(fht%d);\n",ImageID);
   fprintf(fp,"   SetIntegerTable(iht%d);\n",ImageID);
   fprintf(fp,"   SetBitMapTable(bmht%d);\n",ImageID);
   fprintf(fp,"   RefreshBooleanSymbols();\n");
   fprintf(fp,"   InstallFunctionList(P%d_1);\n\n",ImageID);
   fprintf(fp,"   InitExpressionPointers();\n\n");

   /*===============================*/
   /* User specific initialization. */
   /*===============================*/

   cgPtr = ListOfCodeGeneratorItems;
   while (cgPtr != NULL)
     {
      if (cgPtr->initFunction != NULL)
        {
         (*cgPtr->initFunction)(fp,ImageID,MaxIndices);
         fprintf(fp,"\n");
        }
      cgPtr = cgPtr->next;
     }

   /*================================*/
   /* Close the initialization file. */
   /*================================*/

   fprintf(fp,"  }\n");

   fclose(fp);

   return(1);
  }


/**************************************************/
/* NewCFile: Opens a new file for writing C code. */
/**************************************************/
globle FILE *NewCFile(fileName,id,version,reopenOldFile)
  char *fileName;
  int id, version, reopenOldFile;
  {
   char fname[FSIZE];
   FILE *newFP;

   sprintf(fname,"%s%d_%d.c",fileName,id,version);

   newFP = fopen(fname,reopenOldFile ? "a" : "w");
     
   if (newFP == NULL)
     {
      OpenErrorMessage("constructs-to-c",fname);
      return(NULL);
     }
   
   if (reopenOldFile == CLIPS_FALSE)
     {
      fprintf(newFP,"#include \"%s.h\"\n",fileName);
      fprintf(newFP,"\n");
     }
   return(newFP);
  }
  
/**********************************************************/
/* HashedExpressionsToCode:   */
/**********************************************************/
static VOID HashedExpressionsToCode()
  {
   unsigned i;
   EXPRESSION_HN *exphash;
   
   for (i = 0 ; i < EXPRESSION_HASH_SIZE ; i++)
     for (exphash = ExpressionHashTable[i] ; exphash != NULL ; exphash = exphash->nxt)
        { 
         exphash->bsaveID = ExpressionCount + (MaxIndices * ExpressionVersion);
         ExpressionToCode(NULL,exphash->exp);
        }
  }

/*********************************************************/
/* PrintHashedExpressionReference:    */
/*********************************************************/
globle VOID PrintHashedExpressionReference(theFile,theExpression,imageID,maxIndices)
  FILE *theFile;
  struct expr *theExpression;
  int imageID;
  int maxIndices;
  {  
   long theIDValue;
   
   if (theExpression == NULL)
     { fprintf(theFile,"NULL"); }
   else
     {
      theIDValue = HashedExpressionIndex(theExpression);
      
      fprintf(theFile,"&E%d_%ld[%ld]",
                      imageID,
                      theIDValue / maxIndices,
                      theIDValue % maxIndices);
     }
  } 
  
/**********************************************************/
/* ExpressionToCode:   */
/**********************************************************/
globle int ExpressionToCode(fp,exprPtr)
  FILE *fp;
  struct expr *exprPtr;
  {
   /*========================================*/
   /* Print the reference to the expression. */
   /*========================================*/

   if (exprPtr == NULL)
     {
      if (fp != NULL) fprintf(fp,"NULL");
      return(0);
     }
   else if (fp != NULL)
     { fprintf(fp,"&E%d_%d[%d]",ImageID,ExpressionVersion,ExpressionCount); }

   /*==================================================*/
   /* Create a new expression code file, if necessary. */
   /*==================================================*/

   if (ExpressionHeader == CLIPS_TRUE)
     {
      if ((ExpressionFP = NewCFile(FilePrefix,3,ExpressionVersion,CLIPS_FALSE)) == NULL)
        { return(-1); }

      fprintf(ExpressionFP,"struct expr E%d_%d[] = {\n",ImageID,ExpressionVersion);
      fprintf(HeaderFP,"extern struct expr E%d_%d[];\n",ImageID,ExpressionVersion);
      ExpressionHeader = CLIPS_FALSE;
     }
   else
     { fprintf(ExpressionFP,",\n"); }

   /*===========================*/
   /* Dump the expression code. */
   /*===========================*/

   DumpExpression(exprPtr);

   /*=========================================*/
   /* Close the expression file if necessary. */
   /*=========================================*/

   if (ExpressionCount >= MaxIndices)
     {
      ExpressionCount = 0;
      ExpressionVersion++;
      fprintf(ExpressionFP,"};\n");
      fclose(ExpressionFP);
      ExpressionFP = NULL;
      ExpressionHeader = CLIPS_TRUE;
     }

   return(1);
  }

/**********************************************************/
/* DumpExpression:   */
/**********************************************************/
static VOID DumpExpression(exprPtr)
  struct expr *exprPtr;
  {

   while (exprPtr != NULL)
     {
      fprintf(ExpressionFP,"{");
      fprintf(ExpressionFP,"%d,",exprPtr->type);
      fprintf(ExpressionFP,"VS ");
      switch (exprPtr->type)
        {
         case FCALL:
           PrintFunctionReference(ExpressionFP,exprPtr->value);
           break;

         case INTEGER:
           PrintIntegerReference(ExpressionFP,exprPtr->value);
           break;

         case FLOAT:
           PrintFloatReference(ExpressionFP,exprPtr->value);
           break;

         case PCALL:
#if DEFFUNCTION_CONSTRUCT
           PrintDeffunctionReference(ExpressionFP,(DEFFUNCTION *) exprPtr->value,
                                     ImageID,MaxIndices);
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;

         case GCALL:
#if DEFGENERIC_CONSTRUCT
           PrintGenericFunctionReference(ExpressionFP,(DEFGENERIC *) exprPtr->value,
                                         ImageID,MaxIndices);
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;
           
         case DEFTEMPLATE_PTR:
#if DEFTEMPLATE_CONSTRUCT
           DeftemplateCConstructReference(ExpressionFP,exprPtr->value,ImageID,MaxIndices);
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;
           
         case DEFGLOBAL_PTR:
#if DEFGLOBAL_CONSTRUCT
           DefglobalCConstructReference(ExpressionFP,exprPtr->value,ImageID,MaxIndices);
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;

         case DEFCLASS_PTR:
#if OBJECT_SYSTEM
           PrintClassReference(ExpressionFP,(DEFCLASS *) exprPtr->value,ImageID,MaxIndices);
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;
           
          case FACT_ADDRESS:
#if DEFTEMPLATE_CONSTRUCT
           fprintf(ExpressionFP,"&DummyFact");
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;
          
         case INSTANCE_ADDRESS:
#if OBJECT_SYSTEM
           fprintf(ExpressionFP,"&DummyInstance");
#else
           fprintf(ExpressionFP,"NULL");
#endif
           break;

         case STRING:
         case SYMBOL:
         case INSTANCE_NAME:
         case GBL_VARIABLE:
           PrintSymbolReference(ExpressionFP,exprPtr->value);
           break;
         
         default:
           if (PrimitivesArray[exprPtr->type] == NULL) 
             { fprintf(ExpressionFP,"NULL"); }
           else if (PrimitivesArray[exprPtr->type]->bitMap) 
             { PrintBitMapReference(ExpressionFP,exprPtr->value); }
           else 
             { fprintf(ExpressionFP,"NULL"); }
           break;
        }
        
      fprintf(ExpressionFP,",");

      ExpressionCount++;
      if (exprPtr->argList == NULL)
        { fprintf(ExpressionFP,"NULL,"); }
      else
        {
         fprintf(ExpressionFP,"&E%d_%d[%d],",ImageID,ExpressionVersion,
                                                       ExpressionCount);
        }

      if (exprPtr->nextArg == NULL)
        { fprintf(ExpressionFP,"NULL}"); }
      else
        {
         fprintf(ExpressionFP,"&E%d_%d[%d]}",ImageID,ExpressionVersion,
                              ExpressionCount + ExpressionSize(exprPtr->argList));
        }

      if (exprPtr->argList != NULL)
        {
         fprintf(ExpressionFP,",\n");
         DumpExpression(exprPtr->argList);
        }

      exprPtr = exprPtr->nextArg;
      if (exprPtr != NULL) fprintf(ExpressionFP,",\n");
     }
  }

/*******************************************/
/* ConstructsToCCommandDefinition:              */
/*******************************************/
globle VOID ConstructsToCCommandDefinition()
  {
   DefineFunction2("constructs-to-c", 'v', PTIF ConstructsToCCommand, "ConstructsToCCommand", "23*kii");
  }

/*******************************************************************************/
/* AddCodeGeneratorItem: Adds another code generator item to the list of items */
/*   for which code is generated by the constructs-to-c function. Typically    */
/*   each construct has its own code generator item.                           */
/*******************************************************************************/
globle struct CodeGeneratorItem *AddCodeGeneratorItem(name,priority,beforeFunction,
                                                      initFunction,generateFunction,
                                                      arrayCount)
  char *name;
  int priority;
  VOID (*beforeFunction)(VOID_ARG);
#if ANSI_COMPILER
  VOID (*initFunction)(FILE *,int,int);
  int (*generateFunction)(char *,int,FILE *,int,int);
#else
  VOID (*initFunction)();
  int (*generateFunction)();
#endif
  int arrayCount;
  {
   struct CodeGeneratorItem *newPtr, *currentPtr, *lastPtr = NULL;
   static int theCount = 0;
   register int i;
   char theBuffer[3];

   newPtr = get_struct(CodeGeneratorItem);

   newPtr->name = name;
   newPtr->beforeFunction = beforeFunction;
   newPtr->initFunction = initFunction;
   newPtr->generateFunction = generateFunction;
   newPtr->priority = priority;

   if (arrayCount != 0)
     {
      /* ================================================
         Maximum number of arrays currently limited to 47
         ================================================ */
      if ((arrayCount + theCount) > (PRIMARY_LEN + SECONDARY_LEN))
        {
         CLIPSSystemError("CONSCOMP",2);
         ExitCLIPS(2);
        }
      newPtr->arrayNames = (char **) gm2((int) (sizeof(char *) * arrayCount));
      for (i = 0 ; i < arrayCount ; i++)
        {
         if (theCount < PRIMARY_LEN)
           sprintf(theBuffer,"%c",PRIMARY_CODES[theCount]);
         else
           sprintf(theBuffer,"%c_",SECONDARY_CODES[theCount - PRIMARY_LEN]);
         theCount++;
         newPtr->arrayNames[i] = (char *) gm2((int) (strlen(theBuffer) + 1));
         strcpy(newPtr->arrayNames[i],theBuffer);
        }
     }
   else
     newPtr->arrayNames = NULL;
     
   if (ListOfCodeGeneratorItems == NULL)
     {
      newPtr->next = NULL;
      ListOfCodeGeneratorItems = newPtr;
      return(newPtr);
     }
     
   currentPtr = ListOfCodeGeneratorItems;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = ListOfCodeGeneratorItems;
      ListOfCodeGeneratorItems = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }

   return(newPtr);
  }
  
/******************************************************************/
/* CloseFileIfNeeded:   */
/******************************************************************/
globle FILE *CloseFileIfNeeded(theFile,theCount,arrayVersion,maxIndices,canBeReopened,codeFile)
  FILE *theFile;
  int *theCount;
  int *arrayVersion;
  int maxIndices;
  int *canBeReopened;
  struct CodeGeneratorFile *codeFile;
  {
   if (*theCount >= maxIndices)
    {
     if (canBeReopened != NULL)
       *canBeReopened = CLIPS_FALSE;
     if (theFile == NULL)
       {
        if ((canBeReopened == NULL) || (codeFile == NULL))
          {
           CLIPSSystemError("CONSCOMP",3);
           ExitCLIPS(2);
          }
        if (codeFile->filePrefix == NULL)
          return(NULL);
        theFile = NewCFile(codeFile->filePrefix,codeFile->id,codeFile->version,CLIPS_TRUE);
        if (theFile == NULL)
          {
           CLIPSSystemError("CONSCOMP",4);
           ExitCLIPS(2);
          }
       }
     fprintf(theFile,"};\n");
     fclose(theFile);
     *theCount = 0;
     (*arrayVersion)++;
     return(NULL);
    }
    
   if (canBeReopened != NULL)
     {
      *canBeReopened = CLIPS_TRUE;
      fclose(theFile);
      return(NULL);
     }
   return(theFile);
  }
  
/******************************************************************/
/* OpenFileIfNeeded:   */
/******************************************************************/
globle FILE *OpenFileIfNeeded(theFile,fileName,fileID,imageID,fileCount,arrayVersion,
                              headerFP,structureName,structPrefix,reopenOldFile,codeFile)
  FILE *theFile;
  char *fileName;
  int fileID;
  int imageID;
  int *fileCount;
  int arrayVersion;
  FILE *headerFP;
  char *structureName;
  char *structPrefix;
  int reopenOldFile;
  struct CodeGeneratorFile *codeFile;
  {
   char arrayName[80];
   char *newName;
   int newID,newVersion;
   
   /* =============================================
      If a file is being reopened, make
      sure to use the same version number as before
      ============================================= */
   if (reopenOldFile)
     {
      if (codeFile == NULL)
        {
         CLIPSSystemError("CONSCOMP",5);
         ExitCLIPS(2);  
        }
      newName = codeFile->filePrefix;
      newID = codeFile->id;
      newVersion = codeFile->version;
     }
   else
     {
      newVersion = *fileCount;
      newID = fileID;
      newName = fileName;
      if (codeFile != NULL)
        {
         codeFile->version = newVersion;
         codeFile->filePrefix = newName;
         codeFile->id = newID;
        }
     }
     
   if (theFile == NULL)
     {
      if ((theFile = NewCFile(newName,newID,newVersion,reopenOldFile)) == NULL)
        return(NULL);
      if (reopenOldFile == CLIPS_FALSE)
        {
         (*fileCount)++;
         sprintf(arrayName,"%s%d_%d",structPrefix,imageID,arrayVersion);
#if SHORT_LINK_NAMES
         if (strlen(arrayName) > 6)
           {
            PrintWarningID("CONSCOMP",2,CLIPS_FALSE);
            PrintCLIPS(WWARNING,"Array name ");
            PrintCLIPS(WWARNING,arrayName);
            PrintCLIPS(WWARNING,"exceeds 6 characters in length.\n");
            PrintCLIPS(WWARNING,"   This variable may be indistinguishable from another by the linker.\n");
           }
#endif      
         fprintf(theFile,"%s %s[] = {\n",structureName,arrayName);
         fprintf(headerFP,"extern %s %s[];\n",structureName,arrayName);
        }
      else
       { fprintf(theFile,",\n"); }
     }
   else
     { fprintf(theFile,",\n"); }
    
   return(theFile);
  }
  
/*************************************************/
/* MarkConstructBsaveIDs: Mark all occurences of */
/*  a specific construct with a unique ID.       */
/*************************************************/
globle VOID MarkConstructBsaveIDs(constructModuleIndex)
  int constructModuleIndex;
  {
   long theCount = 0;      
   DoForAllConstructs(MarkConstruct,constructModuleIndex,CLIPS_FALSE,&theCount);
  }
  
/*************************************************************/
/* MarkConstruct: Sets the bsaveID for a specific construct. */
/*  Used with the MarkConstructBsaveIDs function to mark all */
/*  occurences of a specific construct with a unique ID.     */
/*************************************************************/
static VOID MarkConstruct(theConstruct,vTheBuffer)
  struct constructHeader *theConstruct;
  VOID *vTheBuffer;
  {
   long *count = (long *) vTheBuffer;
   
   theConstruct->bsaveID = (*count)++;
  }

/***********************************************************/
/* ConstructHeaderToCode: Writes the C code representation */
/*   of a single construct header to the specified file.   */
/***********************************************************/
globle VOID ConstructHeaderToCode(theFile,theConstruct,imageID,maxIndices,moduleCount,
                                  constructModulePrefix,constructPrefix)
  FILE *theFile;
  struct constructHeader *theConstruct;
  int imageID;
  int maxIndices;
  int moduleCount;
  char *constructModulePrefix;
  char *constructPrefix;
  {
   /*================*/
   /* Construct Name */
   /*================*/
   
   fprintf(theFile,"{");
   
   PrintSymbolReference(theFile,theConstruct->name); 
   
   /*===================*/
   /* Pretty Print Form */
   /*===================*/
   
   fprintf(theFile,",NULL,");                        
         
   /*====================*/
   /* Construct Module */
   /*====================*/

   fprintf(theFile,"MIHS &%s%d_%d[%d],",
                   constructModulePrefix,
                   imageID,          
                   (moduleCount / maxIndices) + 1,
                   moduleCount % maxIndices);
                
   /*==========*/
   /* Bsave ID */
   /*==========*/
                       
   fprintf(theFile,"0,"); 
                
   /*================*/
   /* Next Construct */
   /*================*/

   if (theConstruct->next == NULL)
     { fprintf(theFile,"NULL}"); }
   else
     {
      fprintf(theFile,"CHS &%s%d_%ld[%ld]}",
                      constructPrefix,
                      imageID,
                      (theConstruct->next->bsaveID / maxIndices) + 1,
                      theConstruct->next->bsaveID % maxIndices);
     }
  }
  
/***********************************************************/
/* ConstructModuleToCode: Writes the C code representation */
/*   of a single construct module to the specified file.   */
/***********************************************************/
globle VOID ConstructModuleToCode(theFile,theModule,imageID,maxIndices,
                                  constructIndex,constructPrefix)
  FILE *theFile;
  struct defmodule *theModule;
  int imageID;
  int maxIndices;
  int constructIndex;
  char *constructPrefix;
  {
   struct defmoduleItemHeader *theModuleItem;
   
   /*======================*/
   /* Associated Defmodule */
   /*======================*/
   
   fprintf(theFile,"{"); 
   
   theModuleItem = (struct defmoduleItemHeader *) 
                   GetModuleItem(theModule,constructIndex);
                   
   PrintDefmoduleReference(theFile,theModule);
   
   fprintf(theFile,","); 
      
   /*=============================*/
   /* First Construct Module Item */
   /*=============================*/
   
   if (theModuleItem->firstItem == NULL) fprintf(theFile,"NULL,");
   else fprintf(theFile,"CHS &%s%d_%ld[%ld],",
                        constructPrefix,
                        imageID,
                        (long) (theModuleItem->firstItem->bsaveID / maxIndices) + 1,
                        (long) theModuleItem->firstItem->bsaveID % maxIndices);
                           
   /*============================*/
   /* Last Construct Module Item */
   /*============================*/
            
   if (theModuleItem->lastItem == NULL) fprintf(theFile,"NULL");
   else fprintf(theFile,"CHS &%s%d_%ld[%ld]",
                        constructPrefix,
                        imageID,
                        (long) (theModuleItem->lastItem->bsaveID / maxIndices) + 1,
                        (long) theModuleItem->lastItem->bsaveID % maxIndices);
                                    
   fprintf(theFile,"}"); 
  }
  
#else

#if ANSI_COMPILER
   VOID                               ConstructsToCCommand(void);
#else
   VOID                               ConstructsToCCommand();
#endif

      /*====================================*/
      /* Definition for rule compiler stub. */
      /*====================================*/

   VOID ConstructsToCCommand() {}

#endif
