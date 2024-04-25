#ifndef __HOTKEY_H__
#define __HOTKEY_H__

#include <string>
#include "namespace.h"

BEGIN_NAMESPACE

class hotkey{

public:
	hotkey(const std::string& key,
		   const std::string& label,
		   const short& fgColor = -1,
		   const short& bgColor = -1);

public:
	std::string getKey() const;
	std::string getLabel() const;
	void setLabel(const std::string& lbl);
	const short getColor() const;
	std::string operator[](const uint8_t& ix) const;

private:
	std::string key;
	std::string label;
	short ixColorPair;

public:
	static const hotkey None;

private:
	static const short ColorsetOffset;
	static short colorsetCount;
};

END_NAMESPACE
#endif //__HOTKEY_H__
