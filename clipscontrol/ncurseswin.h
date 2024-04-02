#ifndef __NCURSES_WIN_H__
#define __NCURSES_WIN_H__

#include <list>
#include <tuple>
#include <string>
#include <vector>
#include <functional>
#include <ncurses.h>


typedef std::tuple<std::string, std::string> hotkey;
typedef std::function<void(const std::string& s)> pubfunc;

class NCursesWin{
public:
	static const int MINCOLS = 60;

public:
	NCursesWin();
	~NCursesWin();

private:
	enum class KPMode{
		Default = 0,
		Input   = 1,
		LogLvl  = 2
	};

	enum class WatchColor : int16_t{
		Enabled  = 1,
		Disabled = 2,
		Unknown  = 3
	};

	enum class InputAction{
		None    = 0,
		Load    = 1,
		RawCmd  = 2,
		Run     = 3
	};

public:
	void poll();
	void exitPoll();
	void addPublisher(const pubfunc& f);
	void setWatchFlags(int flags);
	void print(const std::string& s);
	// void removePublisher(const pubfunc& f);

private:
	void createWindows();
	void destroyWindows();
	void resetBottomDefault();
	void resetBottomInput(const std::string& prompt);
	void resetBottomLogLevel();
	void resize();
	void updateTop();
	void updateBottom(const std::string& title, const std::vector<hotkey>& options);
	void updateWatch(size_t xpos, size_t colw, const std::string& wname, const WatchColor& color);
	void updateWatches(bool refresh=false);
	void printmid(const std::string& str, const bool& log = true);
	void printBottomOptions(const std::vector<hotkey>& options);
	void handleKeyDefault(const uint32_t& c);
	void handleKeyInput(const uint32_t& c);
	void handleKeyLogLvl(const uint32_t& c);
	void shiftToDefaultMode();
	void shiftToInputMode(const std::string& prompt, bool numeric=false);
	void shiftToLogLvlMode();
	void handleInputBS();
	void handleInputNL();

	void sendCommand(const std::string& cmd);
	void sendLoad(const std::string& file);
	void sendLogLvl(uint8_t lvl);
	void sendPrintAgenda();
	void sendPrintFacts();
	void sendPrintRules();
	void sendRun(uint32_t n);
	void sendReset();
	void sendWatchFunc();
	void sendWatchGlob();
	void sendWatchFacts();
	void sendWatchRules();
	void publish(const std::string& s);

private:
	bool exit;
	WINDOW *top;
	WINDOW *mid;
	WINDOW *bottom;
	KPMode currMod;
	std::string inputPrompt;
	std::string inputBuffer;
	bool inputNumericOnly;
	std::vector<pubfunc> publishers;
	std::string cmdstrbase;
	InputAction inputAction;
	int watchFlags;
	bool trimLines;
	std::list<std::string> history;
};

#endif // __NCURSES_WIN_H__