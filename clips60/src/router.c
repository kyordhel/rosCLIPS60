   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  I/O ROUTER MODULE                  */
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

#define _ROUTER_SOURCE_

#include <stdio.h>
//#include <tcl.h>
//#include <tk.h>
#include <fcntl.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "constant.h"
#include "clipsmem.h"
#include "filertr.h"
#include "strngrtr.h"
#include "extnfunc.h"
#include "argacces.h"
#include "sysdep.h"

#include "router.h"

struct router
  {
   char *name;
   int active;
   int priority;
#if ANSI_COMPILER
   int (*query)(char *);
   int (*printer)(char *,char *);
   int (*exiter)(int);
   int (*charget)(char *);
   int (*charunget)(int,char *);
#else
   int (*query)();
   int (*printer)();
   int (*exiter)();
   int (*charget)();
   int (*charunget)();
#endif
   struct router *next;
  };

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     QueryRouter(char *,struct router *);
#else
   static int                     QueryRouter();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct router       *ListOfRouters = NULL;
   static FILE                *FastLoadFilePtr = NULL;
   static FILE                *FastSaveFilePtr = NULL;
   static int                  Abort;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle char                *WWARNING = "wwarning";
   globle char                *WERROR = "werror";
   globle char                *WTRACE = "wtrace";
   globle char                *WDIALOG = "wdialog";
   globle char                *WCLIPS  = "wclips";
   globle char                *WDISPLAY = "wdisplay";
   globle int                  CLIPSInputCount = -1;

/****************************************/
/* GLOBAL EXTERNAL VARIABLE DEFINITIONS */
/****************************************/	

/*extern int TkClipsFlag;
extern char TkClipsCmd[1000][256];*/


/*********************************************************/
/* InitializeDefaultRouters: Initializes output streams. */
/*********************************************************/
globle VOID InitializeDefaultRouters()
  {
#if (! RUN_TIME)
   DefineFunction2("exit",    'v', PTIF ExitCommand,    "ExitCommand", "00");
#endif
   InitializeFileRouter();
   InitializeStringRouter();
  }

/***************************************/
/* PrintCLIPS: Generic print function. */
/***************************************/
globle int PrintCLIPS(logicalName, str)
  char *logicalName;
  char *str;
  {
   struct router *currentPtr;

   if (((char *) FastSaveFilePtr) == logicalName)
     {
      fprintf(FastSaveFilePtr,"%s",str);
      return(2);
     }

   currentPtr = ListOfRouters;
   while (currentPtr != NULL)
     {
      if ((currentPtr->printer != NULL) ? QueryRouter(logicalName,currentPtr) : CLIPS_FALSE)
        {
         (*currentPtr->printer) (logicalName,str);
         return(1);
        }
      currentPtr = currentPtr->next;
     }

   if (strcmp(WERROR,logicalName) != 0) UnrecognizedRouterMessage(logicalName);

   return(0);
  }
  
/**********************************************/
/* GetcCLIPS: Generic get character function. */
/**********************************************/
globle int GetcCLIPS(logicalName)
  char *logicalName;
  {
   struct router *currentPtr;
   int inchar, arg;



   if (((char *) FastLoadFilePtr) == logicalName)
     {
      inchar = getc(FastLoadFilePtr);
      if (inchar == '\r') return('\n');

      if (inchar != '\b')
        { return(inchar); }

      return(inchar);
     }

   currentPtr = ListOfRouters;
   while (currentPtr != NULL)
     {
      if ((currentPtr->charget != NULL) ? QueryRouter(logicalName,currentPtr) : CLIPS_FALSE)
        {
         arg = fcntl(0, F_GETFL, arg);

         fcntl(0, F_SETFL, arg | O_NONBLOCK);
         while ((inchar = (*currentPtr->charget) (logicalName))<0) {

		fcntl(0, F_SETFL, arg);
/*      		if (TkClipsFlag)	
	        {
	             TkClipsFlag--;
        	     RouteCommand(TkClipsCmd[TkClipsFlag]);
      		}*/
      		//Tcl_DoOneEvent(TCL_DONT_WAIT);
		fcntl(0, F_SETFL, arg | O_NONBLOCK);
	 }
	 fcntl(0, F_SETFL, arg); 
	
/*       inchar = (*currentPtr->charget) (logicalName);*/

         if (inchar == '\r') return('\n');

         if (inchar != '\b')
           { return(inchar); }

         return(inchar);
        }
      currentPtr = currentPtr->next;
     }

   UnrecognizedRouterMessage(logicalName);
   return(-1);
  }

/***************************************************/
/* UngetcCLIPS:  Generic unget character function. */
/***************************************************/
globle int UngetcCLIPS(ch,logicalName)
  int ch;
  char *logicalName;
  {
   struct router *currentPtr;

   if (((char *) FastLoadFilePtr) == logicalName)
     { return(ungetc(ch,FastLoadFilePtr)); }

   currentPtr = ListOfRouters;
   while (currentPtr != NULL)
     {
      if ((currentPtr->charunget != NULL) ? QueryRouter(logicalName,currentPtr) : CLIPS_FALSE)
        { return((*currentPtr->charunget) (ch,logicalName)); }
      currentPtr = currentPtr->next;
     }

   UnrecognizedRouterMessage(logicalName);
   return(-1);
  }
  
/**********************************************/
/* ExitCommand: Exits the CLIPS environment. */
/*   Syntax:  (exit)                          */
/**********************************************/
globle VOID ExitCommand()
  {
   if (ArgCountCheck("exit",EXACTLY,0) == -1) return;
#if VAX_VMS
   ExitCLIPS(-2);  /* Fix for weird VMS com file problem. */
#else
   ExitCLIPS(-1);
#endif
   return;
  }
  
/**************************************/
/* ExitCLIPS:  Generic exit function. */
/**************************************/
globle VOID ExitCLIPS(num)
  int num;
  {
   struct router *currentPtr, *nextPtr;

   Abort = CLIPS_FALSE;
   currentPtr = ListOfRouters;
   while (currentPtr != NULL)
     {
      nextPtr = currentPtr->next;
      if (currentPtr->active == CLIPS_TRUE)
        { if (currentPtr->exiter != NULL) (*currentPtr->exiter) (num); }
      currentPtr = nextPtr;
     }

   if (Abort) return;
   genexit(num);
  }

/********************************************/
/* AbortExit: Forces ExitCLIPS to terminate */
/*   after calling all closing routers.     */
/********************************************/
globle VOID AbortExit()
  {
   Abort = CLIPS_TRUE;
  }

/***********************************************************/
/* AddRouter:  Adds a routing structure to the route list. */
/***********************************************************/
globle BOOLEAN AddRouter(routerName,priority,queryFunction,printFunction,
                         getcFunction,ungetcFunction,exitFunction)
  char *routerName;
  int priority;
#if ANSI_COMPILER
  int (*queryFunction)(char *);
  int (*printFunction)(char *,char *);
  int (*getcFunction)(char *);
  int (*ungetcFunction)(int,char *);
  int (*exitFunction)(int);
#else
  int (*queryFunction)();
  int (*printFunction)();
  int (*getcFunction)();
  int (*ungetcFunction)();
  int (*exitFunction)();
#endif
  {
   struct router *newPtr, *lastPtr, *currentPtr;

   newPtr = get_struct(router);

   newPtr->name = routerName;
   newPtr->active = CLIPS_TRUE;
   newPtr->priority = priority;
   newPtr->query = queryFunction;
   newPtr->printer = printFunction;
   newPtr->exiter = exitFunction;
   newPtr->charget = getcFunction;
   newPtr->charunget = ungetcFunction;
   newPtr->next = NULL;

   if (ListOfRouters == NULL)
     {
      ListOfRouters = newPtr;
      return(1);
     }

   lastPtr = NULL;
   currentPtr = ListOfRouters;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = ListOfRouters;
      ListOfRouters = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }

   return(1);
  }

/*************************************************************/
/* DeleteRouter:  Removes a router from the list of routers. */
/*************************************************************/
globle int DeleteRouter(routerName)
  char *routerName;
  {
   struct router *currentPtr, *lastPtr;

   currentPtr = ListOfRouters;
   lastPtr = NULL;

   while (currentPtr != NULL)
     {
      if (strcmp(currentPtr->name,routerName) == 0)
        {
         if (lastPtr == NULL)
           {
            ListOfRouters = currentPtr->next;
            rm(currentPtr,(int) sizeof(struct router));
            return(1);
           }
         lastPtr->next = currentPtr->next;
         rm(currentPtr,(int) sizeof(struct router));
         return(1);
        }
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   return(0);
  }

/**********************************************************************/
/* QueryRouters:  Determines if any router recognizes a logical name. */
/**********************************************************************/
globle int QueryRouters(logicalName)
  char *logicalName;
  {
   struct router *currentPtr;

   currentPtr = ListOfRouters;
   while (currentPtr != NULL)
     {
      if (QueryRouter(logicalName,currentPtr) == CLIPS_TRUE) return(CLIPS_TRUE);
      currentPtr = currentPtr->next;
     }

   return(CLIPS_FALSE);
  }

/************************************************/
/* QueryRouter: Determines if a specific router */
/*    recognizes a logical name.                */
/************************************************/
static int QueryRouter(logicalName,currentPtr)
  char *logicalName;
  struct router *currentPtr;
  {
   /*===================================================*/
   /* If the router is inactive, then it can't respond. */
   /*===================================================*/
   
   if (currentPtr->active == CLIPS_FALSE)
     { return(CLIPS_FALSE); }

   /*=============================================================*/
   /* If the router has no query function, then it can't respond. */
   /*=============================================================*/
   
   if (currentPtr->query == NULL) return(CLIPS_FALSE);

   /*=========================================*/
   /* Call the router's query function to see */
   /* if it recognizes the logical name.      */
   /*=========================================*/
   
   if ( (*currentPtr->query) (logicalName) == CLIPS_TRUE )
     { return(CLIPS_TRUE); }

   return(CLIPS_FALSE);
  }

/****************************************************/
/* DeactivateRouter: Deactivates a specific router. */
/****************************************************/
globle int DeactivateRouter(routerName)
  char *routerName;
  {
   struct router *currentPtr;

   currentPtr = ListOfRouters;

   while (currentPtr != NULL)
     {
      if (strcmp(currentPtr->name,routerName) == 0)
        {
         currentPtr->active = CLIPS_FALSE;
         return(CLIPS_TRUE);
        }
      currentPtr = currentPtr->next;
     }

   return(CLIPS_FALSE);
  }

/************************************************/
/* ActivateRouter: Activates a specific router. */
/************************************************/
globle int ActivateRouter(routerName)
  char *routerName;
  {
   struct router *currentPtr;

   currentPtr = ListOfRouters;

   while (currentPtr != NULL)
     {
      if (strcmp(currentPtr->name,routerName) == 0)
        {
         currentPtr->active = CLIPS_TRUE;
         return(CLIPS_TRUE);
        }
      currentPtr = currentPtr->next;
     }

   return(CLIPS_FALSE);
  }

/********************************************************/
/* SetFastLoad: Used to bypass router system for loads. */
/********************************************************/
globle VOID SetFastLoad(filePtr)
  FILE *filePtr;
  { FastLoadFilePtr = filePtr; }

/********************************************************/
/* SetFastSave: Used to bypass router system for saves. */
/********************************************************/
globle VOID SetFastSave(filePtr)
  FILE *filePtr;
  { FastSaveFilePtr = filePtr; }

/********************************************************/
/* GetFastLoad: Used to bypass router system for loads. */
/********************************************************/
globle FILE *GetFastLoad()
  { return(FastLoadFilePtr); }

/********************************************************/
/* GetFastSave: Used to bypass router system for saves. */
/********************************************************/
globle FILE *GetFastSave()
  { return(FastSaveFilePtr); }

/************************************************************/
/* UnrecognizedRouterMessage: Standard error message for an */
/*   unrecognized router name.                              */
/************************************************************/
globle VOID UnrecognizedRouterMessage(logicalName)
  char *logicalName;
  {
   PrintErrorID("ROUTER",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Logical name ");
   PrintCLIPS(WERROR,logicalName);
   PrintCLIPS(WERROR," was not recognized by any routers\n");
  }
  




