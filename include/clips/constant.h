   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                CONSTANTS HEADER FILE                */
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

#ifndef _H_constant

#define _H_constant

#define FALSE 0
#define TRUE 1

#define CLIPS_FALSE   0
#define CLIPS_TRUE    1
#define EXACTLY       0
#define AT_LEAST      1
#define NO_MORE_THAN  2
#define RANGE         3

#define OFF           0
#define ON            1
#define LHS           0
#define RHS           1
#define NEGATIVE      0
#define POSITIVE      1
#define EOS        '\0'

#define INSIDE        0
#define OUTSIDE       1

#define LESS_THAN     0
#define GREATER_THAN  1
#define EQUAL         2

#define GLOBAL_SAVE   0
#define LOCAL_SAVE    1
#define VISIBLE_SAVE  2

/*************************/
/* TOKEN AND TYPE VALUES */
/*************************/

#define OBJECT_TYPE_NAME               "OBJECT"
#define USER_TYPE_NAME                 "USER"
#define PRIMITIVE_TYPE_NAME            "PRIMITIVE"
#define NUMBER_TYPE_NAME               "NUMBER"
#define INTEGER_TYPE_NAME              "INTEGER"
#define FLOAT_TYPE_NAME                "FLOAT"
#define SYMBOL_TYPE_NAME               "SYMBOL"
#define STRING_TYPE_NAME               "STRING"
#define MULTIFIELD_TYPE_NAME           "MULTIFIELD"
#define LEXEME_TYPE_NAME               "LEXEME"
#define ADDRESS_TYPE_NAME              "ADDRESS"
#define EXTERNAL_ADDRESS_TYPE_NAME     "EXTERNAL-ADDRESS"
#define FACT_ADDRESS_TYPE_NAME         "FACT-ADDRESS"
#define INSTANCE_TYPE_NAME             "INSTANCE"
#define INSTANCE_NAME_TYPE_NAME        "INSTANCE-NAME"
#define INSTANCE_ADDRESS_TYPE_NAME     "INSTANCE-ADDRESS"

/*************************************************************************/
/* The values of these constants should not be changed.  They are set to */
/* start after the primitive type codes in CONSTANT.H.  These codes are  */
/* used to let the generic function bsave image be used whether COOL is  */
/* present or not.                                                       */
/*************************************************************************/
   
#define OBJECT_TYPE_CODE                9
#define PRIMITIVE_TYPE_CODE            10
#define NUMBER_TYPE_CODE               11
#define LEXEME_TYPE_CODE               12
#define ADDRESS_TYPE_CODE              13
#define INSTANCE_TYPE_CODE             14

/****************************************************/
/* The first 9 primitive types need to retain their */
/* values!! Sorted arrays depend on their values!!  */
/****************************************************/

#define FLOAT                           0
#define INTEGER                         1
#define SYMBOL                          2
#define STRING                          3
#define MULTIFIELD                      4
#define EXTERNAL_ADDRESS                5
#define FACT_ADDRESS                    6
#define INSTANCE_ADDRESS                7
#define INSTANCE_NAME                   8

#define FCALL                          10  
#define GCALL                          11
#define PCALL                          12
#define GBL_VARIABLE                   13
#define MF_GBL_VARIABLE                14

#define SF_VARIABLE                    15
#define MF_VARIABLE                    16
#define SF_WILDCARD                    17
#define MF_WILDCARD                    18
#define BITMAPARRAY                    19

#define SCALL_CMP_JN_VARS1             20
#define SCALL_PN_CONSTANT4             21
#define SCALL_PN_CONSTANT2             22
#define SCALL_EQ_FIELD                 23
#define SCALL_NEQ_FIELD                24
#define SCALL_GET_VAR1                 25
#define SCALL_STORE_MULTIFIELD         26
#define FACT_GETVAR_PN1                27
#define SCALL_CMP_JN_VARS2             28
#define SCALL_GET_VAR2                 29
#define SCALL_GET_FIELD2               30
#define SCALL_CMP_JN_VARS3             31
#define SCALL_GET_VAR3                 32
#define SCALL_GET_FIELD3               33
#define SCALL_CMP_JN_VARS4             34
#define SCALL_GET_VAR4                 35
#define SCALL_GET_FIELD4               36
#define SCALL_LENGTH_TEST              37
#define FACT_GETVAR_PN2                38
#define FACT_GETVAR_PN3                39
#define SCALL_CMP_PN_VARS1             40
#define SCALL_CMP_PN_VARS2             41
#define SCALL_CMP_PN_VARS3             42
#define SCALL_CMP_PN_VARS4             43
#define DEFTEMPLATE_PTR                44

#define OBJ_GET_SLOT_PNVAR1            45
#define OBJ_GET_SLOT_PNVAR2            46
#define OBJ_GET_SLOT_JNVAR1            47
#define OBJ_GET_SLOT_JNVAR2            48
#define OBJ_SLOT_LENGTH                49
#define OBJ_PN_CONSTANT                50
#define OBJ_PN_CMP1                    51
#define OBJ_JN_CMP1                    52
#define OBJ_PN_CMP2                    53
#define OBJ_JN_CMP2                    54
#define OBJ_PN_CMP3                    55
#define OBJ_JN_CMP3                    56
#define DEFCLASS_PTR                   57
#define HANDLER_GET                    58
#define HANDLER_PUT                    59

#define DEFGLOBAL_PTR                  60

#define PROC_PARAM                     65
#define PROC_WILD_PARAM                66
#define PROC_GET_BIND                  67
#define PROC_BIND                      68

#define PATTERN_CE                     80
#define AND_CE                         81
#define OR_CE                          82
#define NOT_CE                         83
#define TEST_CE                        84
#define NAND_CE                        85
#define EXISTS_CE                      86
#define FORALL_CE                      87

#define NOT_CONSTRAINT                 90
#define AND_CONSTRAINT                 91
#define OR_CONSTRAINT                  92
#define PREDICATE_CONSTRAINT           93
#define RETURN_VALUE_CONSTRAINT        94

#define LPAREN                         100
#define RPAREN                         101
#define STOP                           102
#define UNKNOWN                        103

#define RVOID                          105

#define INTEGER_OR_FLOAT               110
#define SYMBOL_OR_STRING               111
#define INSTANCE_OR_INSTANCE_NAME      112

typedef long int FACT_ID;

#endif






