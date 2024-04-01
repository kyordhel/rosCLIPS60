#include "ncurseswin.h"
#include <signal.h>
#include <thread>

#define ctrl(x) ((x) & 0x1f)

static
void ctrlc_handler(int signum) {}


/* ** *****************************************************************
*
* Constructors
*
** ** ****************************************************************/
NCursesWin::NCursesWin() :
	exit(false), top(NULL), mid(NULL), bottom(NULL),
	watchFlags(-1),
	currMod(KPMode::Default), inputAction(InputAction::None)
{
	cmdstrbase.push_back((char)0);
	inputPrompt.reserve(25);
	inputBuffer.reserve(255);
	signal(SIGINT, ctrlc_handler);
	initscr();            // Start curses mode
	start_color();
	curs_set(0);
	// clear();
	noecho();
	raw();
	// cbreak();			  // Line buffering disabled. Pass on everything
	// printw("w: %d, h: %d\n", COLS, LINES);
	createWindows();
	keypad(bottom, TRUE); // Enable Fn keys

	use_default_colors();
	init_pair((int16_t)WatchColor::Enabled,  COLOR_BLACK, COLOR_GREEN);
	init_pair((int16_t)WatchColor::Disabled, COLOR_BLACK,   COLOR_RED);
	init_pair((int16_t)WatchColor::Unknown,  COLOR_CYAN,  COLOR_BLACK);

	updateTop("CLIPS Control");
	resetBottomDefault();
}



NCursesWin::~NCursesWin(){
	// End curses mode
	destroyWindows();
	endwin();
}


/* ** *****************************************************************
*
* GUI
*
** ** *****************************************************************/
void NCursesWin::createWindows(){
	if (top || mid || bottom) destroyWindows();

	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	top = newwin(     2, cols,      0, 0); // <- h, w, y, x
	mid = newwin(rows-6, cols,      2, 0);
	bottom = newwin(  4, cols, rows-4, 0);
	refresh(); // Print it on to the real screen
}



void NCursesWin::destroyWindows(){
	if(top) delwin(top);
	if(mid) delwin(mid);
	if(bottom) delwin(bottom);
	top = mid = bottom = NULL;
	refresh();
}


void NCursesWin::exitPoll(){
	exit = true;
}


void NCursesWin::poll(){
	uint32_t c;
	wtimeout(bottom, 250);
	while(!exit){
		if ((c = wgetch(bottom)) == ERR) continue;
		switch(currMod){
			case KPMode::LogLvl:
				handleKeyLogLvl(c);
				break;

			case KPMode::Input:
				handleKeyInput(c);
				break;

			default:
				handleKeyDefault(c);
				break;
		}
	}
}


void NCursesWin::handleKeyDefault(const uint32_t& c){
	// if (c == 20){
	// 	mvwprintw(mid, 24, 0, "Key pressed is (%3d) F12", c, c);
	// 	wrefresh(mid);
	// 	continue;
	// }
	// wrefresh(mid);
	switch(c){
		case ctrl('X'):
			exit = true;
			return;

		case 'A': case 'a': sendPrintAgenda(); break;
		case 'F': case 'f': sendPrintFacts();  break;
		case 'R': case 'r': sendPrintRules();  break;

		case KEY_F(1): sendWatchFunc();  break;
		case KEY_F(2): sendWatchGlob();  break;
		case KEY_F(3): sendWatchFacts(); break;
		case KEY_F(4): sendWatchRules(); break;

		case KEY_F(5): sendRun(1); break;
		case KEY_F(6): sendRun(0); break;
 		case KEY_F(7):
			inputAction = InputAction::Run;
			shiftToInputMode("Run: ", true);
			break;

		case ctrl('C'): case 'C': case 'c':
			inputAction = InputAction::RawCmd;
			shiftToInputMode("Command: ");
			break;

		case ctrl('R'):
			sendReset();
			break;

		case ctrl('O'):
			inputAction = InputAction::Load;
			shiftToInputMode("File to load: ");
			break;

		case ctrl('L'):
			shiftToLogLvlMode();

		default:
			// mvwprintw(mid, 24, 0, "Charcter pressed is = %3d Hopefully it can be printed as '%c'", c, c);
			// refresh();
			break;
	}

}


void NCursesWin::handleKeyInput(const uint32_t& c){
	switch(c){
		case ctrl('X'):
			exit = true;
			return;

		case ctrl('C'):
			currMod = KPMode::Default;
			curs_set(0);
			resetBottomDefault();
			break;

		case KEY_ENTER: case '\n': case '\r':
			handleInputNL();
			break;

		case KEY_BACKSPACE:
			handleInputBS();
			break;
	}

	if(
		( inputNumericOnly && (c >= '0') && (c <= '9') ) ||
		( !inputNumericOnly && (c >= 0x20) && (c <= 0x7f) )
	){
		inputBuffer.push_back(c);
		waddch(bottom, c);
	}

}


void NCursesWin::handleKeyLogLvl(const uint32_t& c){
	switch(c){
		case ctrl('C'):
			currMod = KPMode::Default;
			curs_set(0);
			resetBottomDefault();
			break;

		case '1': case 'E': case 'e':
			sendLogLvl(1);
			break;

		case '2': case 'W': case 'w':
			sendLogLvl(2);
			break;

		case '3': case 'I': case 'i':
			sendLogLvl(3);
			break;
	}
	resetBottomDefault();
}


void NCursesWin::handleInputBS(){
	if(inputBuffer.length() < 1) return;
	inputBuffer.pop_back();
	size_t curNewPos = inputPrompt.length() + inputBuffer.length();
	wmove(bottom, 0, curNewPos);
	wattron(bottom, A_REVERSE);
	waddch(bottom, ' ');
	wattroff(bottom, A_REVERSE);
	wmove(bottom, 0, curNewPos);
}


void NCursesWin::handleInputNL(){
	switch(inputAction){
		case InputAction::Load:
			sendLoad(inputBuffer);
			break;
		case InputAction::RawCmd:
			sendCommand(inputBuffer);
			break;
		case InputAction::Run:
			sendRun(std::stoi(inputBuffer));
			break;
	}
	inputAction = InputAction::None;
	shiftToDefaultMode();
}


void NCursesWin::print(const std::string& s){
	printmid(s);
}


void NCursesWin::printmid(const std::string& str){
	static std::list<std::string> history;
	int rows, cols;
	getmaxyx(mid, rows, cols);

	std::string s(str);
	if(*(s.end()) != '\n') s+='\n';
	if(history.size() >= rows){
		while(history.size() >= rows)
			history.pop_front();
		// mvwprintw(mid, 0, 0, s.c_str());
		wclear(mid);
		wmove(mid, 0, 0);
		for(auto& line: history)
			wprintw(mid, "%s", line.c_str());
	}

	// if(!history.empty() )
	history.push_back(s);
	wprintw(mid, "%s", s.c_str());
	wrefresh(mid);
	// mvwprintw(mid, 1, 5, "Screen size %dx%d", rows, cols);
}


void NCursesWin::printBottomOptions(const std::vector<hotkey>& options){
	int col = 0, row = 1, kpad, lpad, width;
	for(auto&& tuple: options){
		std::string key, label;
		std::tie(key, label) = tuple;
		wattron(bottom, A_REVERSE);
		kpad = col ? 2 - key.length() : 0;
		mvwprintw(bottom, row, 19*col+kpad, "%s", key.c_str() );
		wattroff(bottom, A_REVERSE);
		lpad = 19*col + 3;
		mvwprintw(bottom, row, lpad, "%s", label.c_str() );
		if(++row > 3){ row = 1; ++col; }
	}
}


void NCursesWin::publish(const std::string& s){
	if(s.length() < 1) return;
	printmid("Published: " + (s[0] == 0 ? s.substr(1) : s) + "\n");
	for(const auto& f: publishers) f(s);
}


void NCursesWin::resetBottomInput(const std::string& prompt){
	int rows, cols;

	getmaxyx(stdscr, rows, cols);
	int rpad = cols - prompt.length();

	wclear(bottom);
	wattron(bottom, A_REVERSE);
	mvwprintw(bottom, 0, 0, "%s%*s", prompt.c_str(), rpad, "");
	wattroff(bottom, A_REVERSE);
	// mvwprintw(bottom, 2, 0, "^C");
	// mvwprintw(bottom, 2, 3, "Cancel" );

	std::vector<hotkey> options;
	options.push_back(hotkey( "", ""));
	options.push_back(hotkey( "^C", "Cancel"));
	options.push_back(hotkey( "^X", "Exit"));
	printBottomOptions(options);

	inputPrompt = prompt;
	wmove(bottom, 0, prompt.length());
	curs_set(2);
	wrefresh(mid);
	wrefresh(bottom);
}


void NCursesWin::addPublisher(const pubfunc& f){
	publishers.push_back(f);
}


// void NCursesWin::removePublisher(const pubfunc& f){
// 	for(size_t i = 0; i < publishers.size(); ++i){
// 		if(publishers[i].target() == f.target())
// 			publishers.erase(publishers.begin() + i--);
// 	}
// }


void NCursesWin::resetBottomDefault(){
	std::vector<hotkey> options;
	options.push_back(hotkey( "^O", "Load File"));
	options.push_back(hotkey( "^R", "Reset"));
	options.push_back(hotkey( "^X", "Exit"));

	// options.push_back(hotkey( "F1", "Watch Functions"));
	options.push_back(hotkey( "F1", "W. Functions"));
	options.push_back(hotkey( "^L", "Log Level"));
	options.push_back(hotkey( " C", "Enter Command"));

	// options.push_back(hotkey( "F2", "Watch Globals"));
	options.push_back(hotkey( "F2", "W. Globals"));
	options.push_back(hotkey( " A", "Print Agenda"));
	options.push_back(hotkey( "F5", "Run 1"));

	options.push_back(hotkey( "F3", "Watch Facts"));
	options.push_back(hotkey( " F", "Print Facts"));
	options.push_back(hotkey( "F6", "Run 0"));

	options.push_back(hotkey( "F4", "Watch Rules"));
	options.push_back(hotkey( " R", "Print Rules"));
	options.push_back(hotkey( "F7", "Run n"));

	updateBottom(" Quick Menu ", options);
}


void NCursesWin::resetBottomLogLevel(){
	std::vector<hotkey> options;
	options.push_back(hotkey( "^C", "Cancel"));
	options.push_back(hotkey( "", ""));
	options.push_back(hotkey( "", ""));

	options.push_back(hotkey( "1", "Error"));
	options.push_back(hotkey( "E", "Error"));
	options.push_back(hotkey( "", ""));

	options.push_back(hotkey( "2", "Warning"));
	options.push_back(hotkey( "W", "Warning"));
	options.push_back(hotkey( "", ""));

	options.push_back(hotkey( "3", "Info"));
	options.push_back(hotkey( "I", "Info"));
	options.push_back(hotkey( "", ""));

	updateBottom("Set log level", options);
}


void NCursesWin::setWatchFlags(int flags){
	if(flags == watchFlags) return;
	watchFlags = flags;
	updateWatches(true);
}


void NCursesWin::shiftToDefaultMode(){
	currMod = KPMode::Default;
	curs_set(0);
	resetBottomDefault();
}


void NCursesWin::shiftToInputMode(const std::string& prompt, bool numeric){
	inputNumericOnly = numeric;
	currMod = KPMode::Input;
	inputBuffer.clear();
	resetBottomInput(prompt);
	curs_set(1);
}


void NCursesWin::shiftToLogLvlMode(){
	currMod = KPMode::LogLvl;
	curs_set(0);
	resetBottomLogLevel();
}


void NCursesWin::updateBottom(const std::string& title, const std::vector<hotkey>& options){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	int xpad = (cols - title.length())/2;

	wclear(bottom);

	printBottomOptions(options);
	wrefresh(bottom);

	wattron(bottom, A_REVERSE);
	mvwprintw(bottom, 0, xpad, "%s", title.c_str());
	wattroff(bottom, A_REVERSE);
}


void NCursesWin::updateTop(const std::string& mid){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);

	int lpad = (cols - mid.length())/2;
	int rpad = cols - lpad - mid.length();

	wattron(top, A_REVERSE);
	mvwprintw(top, 0, 0, "%*s%s%*s",
		lpad, "",
		mid.c_str(),
		rpad, ""
	);
	wattroff(top, A_REVERSE);
	updateWatches();
	wrefresh(top);
}


void NCursesWin::updateWatch(size_t col, size_t colw, const std::string& wname, const WatchColor& color){
	std::string pwname;

	if( colw < wname.length() )
		pwname = wname.substr(0, colw);
	else if( colw < (wname.length()+6) )
		pwname = "W. " + wname;
	else
		pwname = "Watch " + wname;
	if( colw > (wname.length() + 6) ){
		if(color == WatchColor::Unknown)  pwname += " (?)";
		if(color == WatchColor::Enabled)  pwname += " (on)";
		if(color == WatchColor::Disabled) pwname += " (off)";
	}

	int lpad = (colw - pwname.length())/2;
	int rpad = colw - lpad - pwname.length();
	wattron(top, (int16_t)COLOR_PAIR(color));
	mvwprintw(top, 1, col*colw, "%*s%s%*s",
		lpad, "",
		pwname.c_str(),
		rpad, ""
	);
	wattroff(top, (int16_t)COLOR_PAIR(color));
}


void NCursesWin::updateWatches(bool refresh){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	int col = 0;
	int colw = cols / 4;

	if(watchFlags == -1){
		updateWatch(col++, colw, "Functions", WatchColor::Unknown);
		updateWatch(col++, colw, "Globals",   WatchColor::Unknown);
		updateWatch(col++, colw, "Facts",     WatchColor::Unknown);
		updateWatch(col++, colw, "Rules",     WatchColor::Unknown);
	}
	else{
		// Facts = 0x01, Rules = 0x02, Globals = 0x40, Deffunctions = 0x80
		updateWatch(col++, colw, "Functions", (watchFlags & 0x80) ? WatchColor::Enabled : WatchColor::Disabled);
		updateWatch(col++, colw, "Globals",   (watchFlags & 0x40) ? WatchColor::Enabled : WatchColor::Disabled);
		updateWatch(col++, colw, "Facts",     (watchFlags & 0x01) ? WatchColor::Enabled : WatchColor::Disabled);
		updateWatch(col++, colw, "Rules",     (watchFlags & 0x02) ? WatchColor::Enabled : WatchColor::Disabled);
	}
	if(refresh) wrefresh(top);
}



/* ** *****************************************************************
*
* ROS-related
*
** ** *****************************************************************/
void NCursesWin::sendCommand(const std::string& cmd){
	publish(cmdstrbase + "raw " + cmd);
}


void NCursesWin::sendLoad(const std::string& file){
	publish(cmdstrbase + "load " + file);
}


void NCursesWin::sendLogLvl(uint8_t lvl){
	// publish(cmdstrbase + "log " + std::to_string(lvl));
	sendCommand("(bind ?*logLevel* " + std::to_string(lvl) + ")");
}


void NCursesWin::sendPrintAgenda(){
	publish(cmdstrbase + "print agenda");
}


void NCursesWin::sendPrintFacts(){
	publish(cmdstrbase + "print facts");
}


void NCursesWin::sendPrintRules(){
	publish(cmdstrbase + "print rules");
}


void NCursesWin::sendRun(uint32_t n){
	publish(cmdstrbase + "run " + std::to_string(n));
}


void NCursesWin::sendReset(){
	publish(cmdstrbase + "reset");
}


void NCursesWin::sendWatchFunc(){
	publish(cmdstrbase + "watch functions");
}


void NCursesWin::sendWatchGlob(){
	publish(cmdstrbase + "watch globals");
}


void NCursesWin::sendWatchFacts(){
	publish(cmdstrbase + "watch facts");
}


void NCursesWin::sendWatchRules(){
	publish(cmdstrbase + "watch rules");
}

