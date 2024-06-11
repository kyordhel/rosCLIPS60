/* ** *****************************************************************
* ncursesdialogwin.cpp
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/

#include "ncursesdialogwin.h"

BEGIN_NAMESPACE

NCursesDialogWin::NCursesDialogWin():
	win(NULL), result(DialogResult::None),
	rows(-1), cols(-1), posY(-1), posX(-1){
}


NCursesDialogWin::~NCursesDialogWin(){
	hide();
}


DialogResult NCursesDialogWin::getResult(){
	if(win) return DialogResult::None;
	if(result == DialogResult::None) return DialogResult::Cancel;
	return result;
}


std::string NCursesDialogWin::getUserInput(){
	return "";
}


void NCursesDialogWin::hide(){
	if(!win) return;
	wborder(win, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	wclear(win);
	wrefresh(win);
	delwin(win);
	win = NULL;
}


void NCursesDialogWin::show(){
	if(win) return;

	if( (rows < 1) || (cols < 1) || (posX < 1) || (posY < 1) ){
		int scrrows, scrcols;
		getmaxyx(stdscr, scrrows, scrcols);
		if(rows < 1) rows = scrrows / 2;
		if(cols < 1) cols = scrcols  / 2;
		if(posY < 1) posY = (scrrows - rows) / 2;
		if(posX < 1) posX = (scrcols - cols) / 2;
	}

	win = newwin(rows, cols, posY, posX);
	draw();
}


bool NCursesDialogWin::isVisible(){
	return win != NULL;
}


END_NAMESPACE
