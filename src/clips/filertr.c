   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               FILE I/O ROUTER MODULE                */
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

#define _FILERTR_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "constant.h"
#include "clipsmem.h"
#include "router.h"

#include "filertr.h"

/************************************************************************/
/* FILE DESCRIPTOR LIST                                                 */
/*   This data structure houses the nodes which link the file id tags   */
/*   used by CLIPS external I/O functions with the appropriate streams. */
/************************************************************************/
typedef struct filelist
  {
   char *fileid;
   FILE *stream;
   struct filelist *next;
  } filelist;

typedef filelist *fileptr;

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     ExitFile(int);
   static int                     PrintFile(char *,char *);
   static int                     GetcFile(char *);
   static int                     UngetcFile(int,char *);
#else
   static int                     ExitFile();
   static int                     PrintFile();
   static int                     GetcFile();
   static int                     UngetcFile();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static fileptr              ListOfFileRouters = NULL;

/*********************************************************/
/* InitializeFileRouter: Initializes output streams. */
/*********************************************************/
globle VOID InitializeFileRouter()
  {
   AddRouter("fileio",0,FindFile,
             PrintFile,GetcFile,
             UngetcFile,ExitFile);
  }

/*****************************************/
/* FindFptr: Returns a pointer to a file */
/*   stream for a given logical name.    */
/*****************************************/
globle FILE *FindFptr(logicalName)
  char *logicalName;
  {
   fileptr fptr;

   /*========================================================*/
   /* Check to see if standard input or output is requested. */
   /*========================================================*/

   if (strcmp(logicalName,"stdout") == 0)
     { return(stdout); }
   else if (strcmp(logicalName,"stdin") == 0)
     { return(stdin); }
   else if (strcmp(logicalName,WTRACE) == 0)
     { return(stdout); }
   else if (strcmp(logicalName,WDIALOG) == 0)
     { return(stdout); }
   else if (strcmp(logicalName,WCLIPS) == 0)
     { return(stdout); }
   else if (strcmp(logicalName,WDISPLAY) == 0)
     { return(stdout); }
   else if (strcmp(logicalName,WERROR) == 0)
     { return(stdout); }
   else if (strcmp(logicalName,WWARNING) == 0)
     { return(stdout); }

   /*==============================================================*/
   /* Otherwise, look up the logical name on the global file list. */
   /*==============================================================*/

   fptr = ListOfFileRouters;
   while ((fptr != NULL) ? (strcmp(logicalName,fptr->fileid) != 0) : CLIPS_FALSE)
     { fptr = fptr->next; }

   if (fptr != NULL) return(fptr->stream);

   return(NULL);
  }

/*****************************************/
/* FindFile: Returns a pointer to a file */
/*   stream for a given logical name.    */
/*****************************************/
globle int FindFile(logicalName)
  char *logicalName;
  {
   if (FindFptr(logicalName) != NULL) return(CLIPS_TRUE);

   return(CLIPS_FALSE);
  }

/********************************************/
/* ExitFile:  Exit routine for file router. */
/********************************************/
#if IBM_TBC
#pragma argsused
#endif
static int ExitFile(num)
  int num;
  {
#if MAC_MPW
#pragma unused(num)
#endif
#if BASIC_IO
   CloseAllFiles();
#endif
   return(1);
  }

/*********************************************/
/* PrintFile: Print routine for file router. */
/*********************************************/
static int PrintFile(logicalName,str)
  char *logicalName, *str;
  {
   FILE *fptr;

   fptr = FindFptr(logicalName);
   fprintf(fptr,"%s",str);
   fflush(fptr);
   return(1);
  }

/*******************************************/
/* GetcFile: Getc routine for file router. */
/*******************************************/
static int GetcFile(logicalName)
  char *logicalName;
  {
   FILE *fptr;
   int theChar;

   fptr = FindFptr(logicalName);

   theChar = getc(fptr);

   /* The following code prevents Control-D on UNIX   */
   /* machines from terminating all input from stdin. */
   if ((fptr == stdin) && (theChar == EOF)) clearerr(stdin);

   return(theChar);
  }

/***********************************************/
/* UngetcFile: Ungetc routine for file router. */
/***********************************************/
static int UngetcFile(ch,logicalName)
  int ch;
  char *logicalName;
  {
   FILE *fptr;

   fptr = FindFptr(logicalName);
   return(ungetc(ch,fptr));
  }


/************************************************************************/
/* OpenFile: Opens a file with the specified access mode and stores the */
/*   opened stream as well as the file id tag on the global file list.  */
/*   Returns a non-zero value if the file was succesfully opened.       */
/************************************************************************/
globle int OpenFile(fname,fmode,fid)
  char *fname,*fmode,*fid;
  {

   FILE *newstream;
   fileptr fptr, prev;

   newstream = fopen(fname,fmode);

   /*==================================================================*/
   /* Make sure the file can be opened with the specified access mode. */
   /*==================================================================*/

   if (newstream == NULL)  return(0);

   /*=====================================*/
   /* Add stream and file id tag to list. */
   /*=====================================*/

   if (ListOfFileRouters == NULL)
     {
      ListOfFileRouters = get_struct(filelist);
      ListOfFileRouters->fileid = (char *) gm2 ((int) strlen(fid) + 1);
      strcpy(ListOfFileRouters->fileid,fid);
      ListOfFileRouters->stream = newstream;
      ListOfFileRouters->next = NULL;
     }
   else
     {
      fptr = ListOfFileRouters;
      prev = fptr;
      while (fptr != NULL)
        {
         prev = fptr;
         fptr = fptr->next;
        }
      fptr = get_struct(filelist);
      fptr->fileid = (char *) gm2 ((int) strlen(fid) + 1);
      strcpy(fptr->fileid,fid);
      fptr->stream = newstream;
      fptr->next = NULL;
      prev->next = fptr;
     }

   return(1);
  }

/******************************************************************/
/* CloseFile:  Closes a file with the specified file id tag.      */
/*   Returns a non-zero value if the file was succesfully closed. */
/******************************************************************/
globle int CloseFile(fid)
  char *fid;
  {
   fileptr fptr, prev;

   /*=====================================================*/
   /* Locate the file with the given id in the file list. */
   /*=====================================================*/

   fptr = ListOfFileRouters;
   prev = NULL;

   while (fptr != NULL)
     {
      if (strcmp(fptr->fileid,fid) == 0)
        {
         fclose(fptr->stream);
         rm(fptr->fileid,(int) strlen(fptr->fileid) + 1);
         if (prev == NULL)
           { ListOfFileRouters = fptr->next; }
         else
           { prev->next = fptr->next; }
         rm(fptr,(int) sizeof(filelist));

         return(1);
        }

      prev = fptr;
      fptr = fptr->next;
     }

   return(0);
  }

/*****************************************************************/
/* CloseAllFiles: Closes all files opened the file io utilities. */
/*****************************************************************/
globle int CloseAllFiles()
  {
   fileptr fptr, prev;

   if (ListOfFileRouters == NULL) return(0);

   fptr = ListOfFileRouters;

   while (fptr != NULL)
     {
      fclose(fptr->stream);
      prev = fptr;
      rm(fptr->fileid,(int) strlen(fptr->fileid) + 1);
      fptr = fptr->next;
      rm(prev,(int) sizeof(filelist));
     }

   ListOfFileRouters = NULL;

   return(1);
  }



