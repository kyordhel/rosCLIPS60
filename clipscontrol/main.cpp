#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ncurses.h>

#include "ncurseswin.h"


int main(int argc, char** argv){
	NCursesWin ncw;
	ncw.poll();
	// getch();
	return 0;
}


