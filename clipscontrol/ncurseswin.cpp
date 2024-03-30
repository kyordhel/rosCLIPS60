#include "ncurseswin.h"
#include <signal.h>

#define ctrl(x) ((x) & 0x1f)

static
void ctrlc_handler(int signum) {}


/* ** *****************************************************************
*
* Constructors
*
** ** ****************************************************************/
NCursesWin::NCursesWin() :
	top(NULL), mid(NULL), bottom(NULL),
	currMod(KPMode::Default)
{

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

	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	mvwprintw(mid, 0, 5, "%s", "Hello world!");
	mvwprintw(mid, 1, 5, "Screen size %dx%d", rows, cols);
	wrefresh(mid);

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



void NCursesWin::poll(){
	uint32_t c;
	bool exit;
	while(true){
		c = wgetch(bottom);
		switch(currMod){
			case KPMode::LogLvl:
				handleKeyLogLvl(c, exit);
				break;

			case KPMode::Input:
				handleKeyInput(c, exit);
				break;

			default:
				handleKeyDefault(c, exit);
				break;
		}
		if(exit) return;
	}
}


void NCursesWin::handleKeyDefault(const uint32_t& c, bool& exit){
	// if (c == 20){
	// 	mvwprintw(mid, 24, 0, "Key pressed is (%3d) F12", c, c);
	// 	wrefresh(mid);
	// 	continue;
	// }
	// wrefresh(mid);
	exit = false;
	switch(c){
		case ctrl('X'):
			exit = true;
			return;

		case 'A': case 'a':
			sendPrintAgenda();
			break;

		case 'F': case 'f':
			sendPrintFacts();
			break;

		case 'R': case 'r':
			sendPrintRules();
			break;

		case KEY_F(5):
			sendRun(1);
			break;

		case KEY_F(6):
			sendRun(0);
			break;

		case KEY_F(7):
			shiftToInputMode("Run: ", true);
			break;

		case ctrl('C'): case 'C': case 'c':
			shiftToInputMode("Command: ");
			break;

		case ctrl('R'):
			sendReset();
			break;

		case ctrl('O'):
			shiftToInputMode("File to load: ");
			break;

		case ctrl('L'):
			shiftToLogLvlMode();

		default:
			mvwprintw(mid, 24, 0, "Charcter pressed is = %3d Hopefully it can be printed as '%c'", c, c);
			// refresh();
			break;
	}

}


void NCursesWin::handleKeyInput(const uint32_t& c, bool& exit){
	exit = false;
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


void NCursesWin::handleKeyLogLvl(const uint32_t& c, bool& exit){
	exit = false;
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
	shiftToDefaultMode();
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


void NCursesWin::updateWatches(){
	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	int col = 0;
	int colw = cols / 4;

	updateWatch(col++, colw, "Functions", WatchColor::Unknown);
	updateWatch(col++, colw, "Globals", WatchColor::Disabled);
	updateWatch(col++, colw, "Facts", WatchColor::Enabled);
	updateWatch(col++, colw, "Rules", WatchColor::Enabled);
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


/* ** *****************************************************************
*
* ROS-related
*
** ** *****************************************************************/
void NCursesWin::sendCommand(const std::string& cmd){
}


void NCursesWin::sendLoad(const std::string& file){
}


void NCursesWin::sendLogLvl(uint8_t lvl){
}


void NCursesWin::sendPrintAgenda(){
}


void NCursesWin::sendPrintFacts(){
}


void NCursesWin::sendPrintRules(){
}


void NCursesWin::sendRun(uint32_t n){
}


void NCursesWin::sendReset(){
}


void NCursesWin::sendWatchFunc(){
}


void NCursesWin::sendWatchGlob(){
}


void NCursesWin::sendWatchFacts(){
}


void NCursesWin::sendWatchRules(){
}

