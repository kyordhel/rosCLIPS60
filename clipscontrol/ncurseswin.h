#ifndef __NCURSES_WIN_H__
#define __NCURSES_WIN_H__

#include <tuple>
#include <string>
#include <vector>
#include <ncurses.h>


typedef std::tuple<std::string, std::string> hotkey;

class NCursesWin{
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

public:
	void poll();

private:
	void createWindows();
	void destroyWindows();
	void resetBottomDefault();
	void resetBottomInput(const std::string& prompt);
	void resetBottomLogLevel();
	void updateTop(const std::string& mid);
	void updateBottom(const std::string& title, const std::vector<hotkey>& options);
	void updateWatch(size_t xpos, size_t colw, const std::string& wname, const WatchColor& color);
	void updateWatches();
	void printBottomOptions(const std::vector<hotkey>& options);
	void handleKeyDefault(const uint32_t& c, bool& exit);
	void handleKeyInput(const uint32_t& c, bool& exit);
	void handleKeyLogLvl(const uint32_t& c, bool& exit);
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

private:
	WINDOW *top;
	WINDOW *mid;
	WINDOW *bottom;
	KPMode currMod;
	std::string inputPrompt;
	std::string inputBuffer;
	bool inputNumericOnly;

};

#endif // __NCURSES_WIN_H__