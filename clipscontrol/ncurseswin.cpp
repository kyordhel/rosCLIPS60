#include "ncurseswin.h"
#include <signal.h>
#include <thread>

#define ctrl(x) ((x) & 0x1f)

BEGIN_NAMESPACE

static
void ctrlc_handler(int signum) {}


/* ** *****************************************************************
*
* Constructors
*
** ** ****************************************************************/
NCursesWin::NCursesWin() :
	exit(false), top(NULL), mid(NULL), bottom(NULL),
	headingC("CLIPS Control"), headingR("rosclips: OFF"),
	watchFlags(-1), runN(0),
	currMod(KPMode::Default), inputAction(InputAction::None),
	clipsStatus(CLIPSStatus::Offline)
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
	init_pair(0x00, -1, -1);
	init_pair(0x10+(int16_t)WatchStatus::Enabled,  COLOR_BLACK, COLOR_GREEN);
	init_pair(0x10+(int16_t)WatchStatus::Disabled, COLOR_WHITE,   COLOR_BLUE);
	init_pair(0x10+(int16_t)WatchStatus::Unknown,   COLOR_CYAN,  COLOR_BLACK);

	init_pair(0x20+(int16_t)CLIPSStatus::Online,   -1, COLOR_GREEN | 0x08);
	init_pair(0x20+(int16_t)CLIPSStatus::Offline,  -1, COLOR_RED);
	init_pair(0x20+(int16_t)CLIPSStatus::Unknown,  -1, -1);

	updateTop();
	resetBottomDefault();

	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	// printmid("Rows: " + std::to_string(rows) + "\n");
	// printmid("Cols: " + std::to_string(cols) + "\n");
	// printmid("COLORS: " + std::to_string(COLORS) + "\n");
	// printmid("COLOR_PAIRS: " + std::to_string(COLOR_PAIRS) + "\n");
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
	scrollok(mid, true);
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
		if (c == KEY_RESIZE){
			resize();
			continue;
		}
		switch(currMod){
			case KPMode::LogLvl:
				handleKeyLogLvl(c);
				break;

			case KPMode::Input:
				handleKeyInput(c);
				break;

			case KPMode::TglWatches:
				handleKeyTglWatches(c);
				break;

			default:
				handleKeyDefault(c);
				break;
		}
	}
}

void NCursesWin::resize(){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);

	wresize(top, rows, cols);
	updateTop();

	wresize(mid, rows-6, cols);
	wclear(mid); wmove(mid, 0, 0);
	for(auto& line: history)
		wprintw(mid, (trimLines ? line.substr(0, cols) : line).c_str() );
	wrefresh(mid);

	mvwin(bottom, rows-4, 0);
	wresize(bottom, 4, cols);
	// wrefresh(bottom);

	switch(currMod){
		case KPMode::Input:
			resetBottomInput(inputPrompt);
			break;
		case KPMode::LogLvl:
			resetBottomLogLevel();
			break;
		// case KPMode::Default:
		default:
			resetBottomDefault();
			break;
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

		case 'g': sendPrintAgenda(); break;
		case 'f': sendPrintFacts();  break;
		case 'i': sendPrintRules();  break;

		case KEY_F(1): case 'U': sendWatchFunc();  break;
		case KEY_F(2): case 'B': sendWatchGlob();  break;
		case KEY_F(3): case 'F': sendWatchFacts(); break;
		case KEY_F(4): case 'R': sendWatchRules(); break;

		case KEY_F(5): case 'r': sendRun(1); break;
		case KEY_F(6): case 'E': case 'e': sendRun(runN); break;
 		case KEY_F(7): case 'N': case 'n':
			inputAction = InputAction::Run;
			inputBuffer = std::to_string(runN);
			shiftToInputMode("Run: ", true);
			break;

		case 'C': case 'c':
			inputAction = InputAction::RawCmd;
			inputBuffer = std::string(prevCmd);
			shiftToInputMode("Command: ");
			break;

		case 'A': case 'a':
			inputAction = InputAction::Assert;
			inputBuffer = std::string(prevFact);
			shiftToInputMode("Fact: ");
			break;

		case ctrl('C'):
			sendClear();
			break;

		case ctrl('R'):
			sendReset();
			break;

		case 'L': case 'l':
			inputAction = InputAction::Load;
			inputBuffer = std::string(prevLdFile);
			shiftToInputMode("File to load: ");
			break;

		case ctrl('L'):
			shiftToLogLvlMode();
			break;

		case 'T': case 't':
			shiftToToggleWatchesMode();
			break;

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
			savePreviousInput();
			resetBottomDefault();
			break;

		case KEY_ENTER: case '\n': case '\r':
			savePreviousInput();
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
	currMod = KPMode::Default;
	resetBottomDefault();
}


void NCursesWin::handleKeyTglWatches(const uint32_t& c){
	switch(c){
		case ctrl('C'):
			currMod = KPMode::Default;
			curs_set(0);
			resetBottomDefault();
			break;

		case '1': sendWatchFunc();  break;
		case '2': sendWatchGlob();  break;
		case '3': sendWatchFacts(); break;
		case '4': sendWatchRules(); break;

		case 'A': case 'a':
			sendWatchFunc();
			sendWatchGlob();
			sendWatchFacts();
			sendWatchRules();
			break;
	}
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
		case InputAction::Assert:
			sendAssert(inputBuffer);
			break;
		case InputAction::Run:
			// sendRun(runN);
			break;
	}
	inputAction = InputAction::None;
	shiftToDefaultMode();
}


void NCursesWin::savePreviousInput(){
	switch(inputAction){
		case InputAction::Load:
			prevLdFile = inputBuffer;
			break;
		case InputAction::RawCmd:
			prevCmd = inputBuffer;
			break;
		case InputAction::Assert:
			prevFact = inputBuffer;
			break;
		case InputAction::Run:
			runN = std::stoi(inputBuffer);
			break;
	}
}


void NCursesWin::print(const std::string& s){
	printmid(s);
}


void NCursesWin::printmid(const std::string& str, const bool& log){
	history.push_back(str);
	if(!trimLines)
		wprintw(mid, str.c_str() );
	else{
		int rows, cols;
		getmaxyx(mid, rows, cols);
		std::string s = str.substr(0, cols);
		wprintw(mid, s.c_str() );
	}
	wrefresh(mid);
}


void NCursesWin::printBottomOptions(const std::vector<hotkey>& options){
	int col = 0, row = 1, kpad, lpad, lblOffset, cols, rows;
	getmaxyx(mid, rows, cols);
	if(cols <= 72) lblOffset = 2;
	else lblOffset = 3;
	// else lblWidth = 19;
	int colWidth = 3 * cols / options.size();
	for(auto&& hk: options){
		std::string key = hk[0];
		std::string label = hk[1];
		wattron(bottom, A_REVERSE);
		kpad = col ? 2 - key.length() : 0;
		mvwprintw(bottom, row, colWidth*col+kpad, "%s", key.c_str() );
		wattroff(bottom, A_REVERSE);
		lpad = colWidth*col + lblOffset;

		wcolor_set(bottom, hk.getColor(), NULL);
		mvwprintw(bottom, row, lpad, "%s", label.substr(0, colWidth - lblOffset).c_str() );
		wcolor_set(bottom, 0, NULL);
		if(++row > 3){ row = 1; ++col; }
	}
}


void NCursesWin::publish(const std::string& s){
	if(s.length() < 1) return;
	// printmid("Published: " + (s[0] == 0 ? s.substr(1) : s) + "\n");
	for(const auto& f: publishers) f(s);
}


void NCursesWin::resetBottomInput(const std::string& prompt){
	static std::vector<hotkey> options = {
		hotkey::None,
		hotkey( "^c", "Cancel"),
		hotkey( "^x", "Exit")
	};

	int rows, cols;

	getmaxyx(stdscr, rows, cols);
	int rpad = cols - prompt.length();

	wclear(bottom);
	wattron(bottom, A_REVERSE);
	mvwprintw(bottom, 0, 0, "%s%*s", prompt.c_str(), rpad, "");
	wattroff(bottom, A_REVERSE);
	// mvwprintw(bottom, 2, 0, "^C");
	// mvwprintw(bottom, 2, 3, "Cancel" );

	printBottomOptions(options);

	inputPrompt = prompt;
	wmove(bottom, 0, prompt.length());
	wprintw(bottom, inputBuffer.c_str());
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
	static std::vector<hotkey> options = {
		hotkey( " l", "Load File"),
		hotkey( "^r", "Reset"),
		hotkey( "^x", "Exit"),

	// Column 2
		hotkey( "^c", "Clear"),
		// hotkey( "^l", "Log Level"),
		hotkey( " a", "Assert fact"),
		hotkey( " c", "Enter Command"),

	// Column 3
		hotkey( " t", "Toggle watches"),
		hotkey( " g", "Print Agenda"),
		hotkey( " r", "Run 1", COLOR_BLUE | 0x08),

	// Column 4
		hotkey( " F", "Watch Facts"),
		hotkey( " f", "Print Facts"),
		hotkey( " e", "Run n (n=0 all)", COLOR_BLUE | 0x08),

	// Column 5
		hotkey( " R", "Watch Rules"),
		hotkey( " i", "Print Rules"),
		hotkey( " n", "Set Run n", COLOR_BLUE | 0x08)
	};

	updateBottom(" Quick Menu ", options);
}


void NCursesWin::resetBottomLogLevel(){
	static std::vector<hotkey> options = {
		hotkey( "^c", "Cancel"),
		hotkey::None,
		hotkey::None,

		hotkey( "1", "Error"),
		hotkey( "e", "Error"),
		hotkey::None,

		hotkey( "2", "Warning"),
		hotkey( "w", "Warning"),
		hotkey::None,

		hotkey( "3", "Info"),
		hotkey( "i", "Info"),
		hotkey::None
	};

	updateBottom("Set log level", options);
}


void NCursesWin::resetBottomTglWatches(){
	static std::vector<hotkey> options = {
		hotkey( " a", "Toggle all"),
		hotkey::None,
		hotkey( "^c", "Cancel"),

		hotkey( " 1", "Functions"),
		hotkey::None,
		hotkey::None,

		hotkey( " 2", "Globals"),
		hotkey::None,
		hotkey::None,

		hotkey( " 3", "Facts"),
		hotkey::None,
		hotkey::None,

		hotkey( " 4", "Rules"),
		hotkey::None,
		hotkey::None
	};

	updateBottom("Toggle watches", options);
}


void NCursesWin::setWatchFlags(int flags){
	if(flags == watchFlags) return;
	watchFlags = flags;
	updateWatches(true);
}


void NCursesWin::setCLIPSStatus(const CLIPSStatus& status){
	if(status == clipsStatus) return;
	clipsStatus = status;
	headingR = "rosclips: ";
	if(status == CLIPSStatus::Online){
		print(headingR + "Online\n");
		headingR+= "ON";
	}else{
		print(headingR + "Offline\n");
		headingR+= "OFF";
	}
	updateTopR();
	wrefresh(top);
}


void NCursesWin::shiftToDefaultMode(){
	currMod = KPMode::Default;
	curs_set(0);
	resetBottomDefault();
}


void NCursesWin::shiftToInputMode(const std::string& prompt, bool numeric){
	inputNumericOnly = numeric;
	currMod = KPMode::Input;
	//inputBuffer.clear();
	resetBottomInput(prompt);
	curs_set(1);
}


void NCursesWin::shiftToLogLvlMode(){
	currMod = KPMode::LogLvl;
	curs_set(0);
	resetBottomLogLevel();
}


void NCursesWin::shiftToToggleWatchesMode(){
	currMod = KPMode::TglWatches;
	curs_set(0);
	resetBottomTglWatches();
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


void NCursesWin::updateTop(){
	updateTopL();
	updateTopR();
	updateTopC();
	updateWatches();
	wrefresh(top);
}


void NCursesWin::updateTopL(){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);

	int width = cols / 3;
	int rpad = width - headingL.length();

	wattron(top, A_REVERSE);
	mvwprintw(top, 0, 0, "%s%*s",
		headingL.c_str(),
		rpad, ""
	);
	wattroff(top, A_REVERSE);
}


void NCursesWin::updateTopC(){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);

	int width = cols - (2 * cols / 3);
	int lpad = (width - headingC.length())/2;
	int rpad = width - lpad - headingC.length();

	wattron(top, A_REVERSE);
	mvwprintw(top, 0, cols/3, "%*s%s%*s",
		lpad, "",
		headingC.c_str(),
		rpad, ""
	);
	wattroff(top, A_REVERSE);
}


void NCursesWin::updateTopR(){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);

	int width = cols / 3;
	int lpad = width - headingR.length();

	short color = 0x20 + (short)clipsStatus;
	wattron(top, COLOR_PAIR(color) | A_REVERSE);
	// wcolor_set(top, color, NULL);
	mvwprintw(top, 0, cols - cols/3, "%*s%s",
		lpad, "",
		headingR.c_str()
	);
	// wcolor_set(top, 0, NULL);
	wattroff(top, COLOR_PAIR(color) | A_REVERSE);
}


void NCursesWin::updateWatch(size_t col, size_t colw, const std::string& wname, const WatchStatus& status){
	std::string pwname;

	if( colw < wname.length() )
		pwname = wname.substr(0, colw);
	else if( colw < (wname.length()+6) )
		pwname = "W. " + wname;
	else
		pwname = "Watch " + wname;
	if( colw > (wname.length() + 6) ){
		if(status == WatchStatus::Unknown)  pwname += " (?)";
		if(status == WatchStatus::Enabled)  pwname += " (on)";
		if(status == WatchStatus::Disabled) pwname += " (off)";
	}
	short color = 0x10 + (short)status;

	int lpad = (colw - pwname.length())/2;
	int rpad = colw - lpad - pwname.length();
	wattron(top, COLOR_PAIR(color));
	mvwprintw(top, 1, col, "%*s%s%*s",
		lpad, "",
		pwname.c_str(),
		rpad, ""
	);
	wattroff(top, COLOR_PAIR(color));
}


void NCursesWin::updateWatches(bool refresh){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	int col = 0;
	int colw = cols / 4;
	int clw1 = cols - 3*colw;

	if(watchFlags == -1){
		updateWatch(col,       clw1, "Functions", WatchStatus::Unknown);
		updateWatch(col+=clw1, colw, "Globals",   WatchStatus::Unknown);
		updateWatch(col+=colw, colw, "Facts",     WatchStatus::Unknown);
		updateWatch(col+=colw, colw, "Rules",     WatchStatus::Unknown);
	}
	else{
		// Facts = 0x01, Rules = 0x02, Globals = 0x40, Deffunctions = 0x80
		updateWatch(col,       clw1, "Functions", (watchFlags & 0x80) ? WatchStatus::Enabled : WatchStatus::Disabled);
		updateWatch(col+=clw1, colw, "Globals",   (watchFlags & 0x40) ? WatchStatus::Enabled : WatchStatus::Disabled);
		updateWatch(col+=colw, colw, "Facts",     (watchFlags & 0x01) ? WatchStatus::Enabled : WatchStatus::Disabled);
		updateWatch(col+=colw, colw, "Rules",     (watchFlags & 0x02) ? WatchStatus::Enabled : WatchStatus::Disabled);
	}
	if(refresh) wrefresh(top);
}



/* ** *****************************************************************
*
* ROS-related
*
** ** *****************************************************************/
void NCursesWin::sendAssert(const std::string& fact){
	if( (fact.front() == '(') && (fact.back() == ')') )
		sendCommand("(assert " + fact + ")");
	else
		sendCommand("(assert (" + fact + "))");
}


void NCursesWin::sendCommand(const std::string& cmd){
	publish(cmdstrbase + "raw " + cmd);
}


void NCursesWin::sendClear(){
	publish(cmdstrbase + "clear");
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


void NCursesWin::sendRun(int n){
	if(n <= 0) n = -1;
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


END_NAMESPACE
