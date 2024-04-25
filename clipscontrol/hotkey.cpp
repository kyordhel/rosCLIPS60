#include "hotkey.h"
#include <ncurses.h>


BEGIN_NAMESPACE

const short hotkey::ColorsetOffset = 0xa0;
const hotkey hotkey::None("", "");

short hotkey::colorsetCount  = 0;

hotkey::hotkey(const std::string& key, const std::string& label,
			   const short& fgColor, const short& bgColor):
	key(key), label(label),
	ixColorPair(++colorsetCount + ColorsetOffset)
{
	if(can_change_color() && has_colors())
		init_pair(ixColorPair, fgColor, bgColor);
	else
		ixColorPair = 0;
}

std::string hotkey::operator[](const uint8_t& ix) const{
	return ix ? label : key;
}

std::string hotkey::getKey() const{
	return key;
}

std::string hotkey::getLabel() const{
	return label;
}

void hotkey::setLabel(const std::string& lbl){
	label = std::string(lbl);
}

const short hotkey::getColor() const{
	return ixColorPair;
}


END_NAMESPACE
