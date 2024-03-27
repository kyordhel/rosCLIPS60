   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 PRETTY PRINT MODULE                 */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Chris Culbert                                        */
/*      Brian Donnell                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _PPRINT_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>
#include <ctype.h>

#include "setup.h"
#include "constant.h"
#include "clipsmem.h"
#include "utility.h"

#include "pprint.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static int              PPBufferStatus = OFF;
   static int              IndentationDepth = 0;
   static int              PPBufferPos = 0;
   static int              PPBufferMax = 0;
   static int              PPBackupOnce = 0;
   static int              PPBackupTwice = 0;
   static char            *PrettyPrintBuffer = NULL;

/*******************************************************/
/* FlushPPBuffer: Resets the pretty print save buffer. */
/*******************************************************/
globle int FlushPPBuffer()
  {
   if (PrettyPrintBuffer == NULL) return(0);
   PPBackupOnce = 0;
   PPBackupTwice = 0;
   PPBufferPos = 0;
   PrettyPrintBuffer[0] = EOS;
   return(1);
  }

/*********************************************************************/
/* DestroyPPBuffer: Resets and removes the pretty print save buffer. */
/*********************************************************************/
globle VOID DestroyPPBuffer()
  {
   PPBackupOnce = 0;
   PPBackupTwice = 0;
   PPBufferPos = 0;
   if (PrettyPrintBuffer != NULL) rm(PrettyPrintBuffer,PPBufferMax);
   PrettyPrintBuffer = NULL;
   PPBufferMax = 0;
  }

/*********************************************/
/* SavePPBuffer: Appends a string to the end */
/*   of the pretty print save buffer.        */
/*********************************************/
globle VOID SavePPBuffer(str)
  char *str;
  {
   long int longSize;
   int normalSize;
   
   /*==========================================*/
   /* If the pretty print buffer isn't needed, */
   /* then don't bother writing to it.         */
   /*==========================================*/
   
   if (PPBufferStatus == OFF) return;

   /*==================================================*/
   /* The pretty print buffer is limited in size to    */
   /* the maximum size of a signed int. Any characters */
   /* beyond that number are discarded.                */
   /*==================================================*/
   
   normalSize = strlen(str);
   longSize = (long) normalSize;
   longSize += (long) PPBufferPos + 513L;
   normalSize += PPBufferPos + 513;
   if (normalSize != longSize) return;
   
   /*================================================*/
   /* If the pretty print buffer isn't big enough to */
   /* contain the string, then increase its size.    */
   /*================================================*/
   
   if (strlen(str) + PPBufferPos + 1 >= PPBufferMax)
     {
      PrettyPrintBuffer = genrealloc(PrettyPrintBuffer,(unsigned) PPBufferMax,
                                     (unsigned) PPBufferMax + 512);
      PPBufferMax += 512;
     }

   /*==================================================*/
   /* Remember the previous tokens saved to the pretty */
   /* print buffer in case it is necessary to back up. */
   /*==================================================*/
   
   PPBackupTwice = PPBackupOnce;
   PPBackupOnce = PPBufferPos;

   /*=============================================*/
   /* Save the string to the pretty print buffer. */
   /*=============================================*/
   
   PrettyPrintBuffer = AppendToString(str,PrettyPrintBuffer,&PPBufferPos,&PPBufferMax);
  }

/***************************************************/
/* PPBackup:  Removes the last string added to the */
/*   pretty print save buffer.  Only capable of    */
/*   backing up for the two most recent additions. */
/***************************************************/
globle VOID PPBackup()
  {
   if ((PPBufferStatus == OFF) || (PrettyPrintBuffer == NULL)) return;

   PPBufferPos = PPBackupOnce;
   PPBackupOnce = PPBackupTwice;
   PrettyPrintBuffer[PPBufferPos] = EOS;
  }

/**************************************************/
/* CopyPPBuffer: Makes a copy of the pretty print */
/*   save buffer.                                 */
/**************************************************/
globle char *CopyPPBuffer()
  {
   int length;
   char *newString;

   length = (1 + strlen(PrettyPrintBuffer)) * (int) sizeof (char);
   newString = (char *) gm2(length);

   strcpy(newString,PrettyPrintBuffer);
   return(newString);
  }

/************************************************************/
/* GetPPBuffer: Returns a pointer to the PrettyPrintBuffer. */
/************************************************************/
globle char *GetPPBuffer()
  {
   return(PrettyPrintBuffer);
  }

/*******************************************/
/* PPCRAndIndent: Prints white spaces into */
/*   the pretty print buffer.              */
/*******************************************/
globle VOID PPCRAndIndent()
  {
   int i;
   char buffer[120];

   buffer[0] = '\n';

   for (i = 1 ; i <= IndentationDepth ; i++)
     { buffer[i] = ' '; }
   buffer[i] = EOS;

   SavePPBuffer(buffer);
  }

/**************************************************************************/
/* IncrementIndentDepth: Increments IndentationDepth for pretty printing. */
/**************************************************************************/
globle VOID IncrementIndentDepth(value)
  int value;
  {
   IndentationDepth += value;
  }

/**************************************************************************/
/* DecrementIndentDepth: Decrements IndentationDepth for pretty printing. */
/**************************************************************************/
globle int DecrementIndentDepth(value)
  int value;
  {
   IndentationDepth -= value;
   return(IndentationDepth);
  }

/**************************************************************/
/* SetIndentDepth: Sets IndentationDepth for pretty printing. */
/**************************************************************/
globle int SetIndentDepth(value)
  int value;
  {
   IndentationDepth = value;
   return(IndentationDepth);
  }

/**********************************************************/
/* SetPPBufferStatus: Sets PPBufferStatus flag to boolean */
/*   value of ON or OFF.                                  */
/**********************************************************/
globle VOID SetPPBufferStatus(value)
  int value;
  {
   PPBufferStatus = value;
  }

/************************************************************/
/* GetPPBufferStatus: Returns value of PPBufferStatus flag. */
/************************************************************/
globle int GetPPBufferStatus()
  {
   return(PPBufferStatus);
  }