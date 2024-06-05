/* ** *****************************************************************
* hotkey.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file hotkey.h
 * Definition of the hotkey class: A class that encapsulates
 * information of key combination-label for the GUI
 * between ROS and CLIPS.
 */

#ifndef __HOTKEY_H__
#define __HOTKEY_H__

/** @cond */
#include <string>
/** @endcond */
#include "namespace.h"

BEGIN_NAMESPACE

/**
 * Encapsulates hotkey information for the NCurses window.
 * Hotkeys are a key combination and a label.
 */
class hotkey{

public:
	/**
	 * Initializes a new instance of hotkey
	 * @param key      The combination of keys
	 * @param label    The display label to print on the GUI
	 * @param fgColor  Optional. The foreground color (nCurses index)
	 *                 of the label. Default: -1
	 * @param bgColor  Optional. The background color (nCurses index)
	 *                 of the label. Default: -1
	 */
	hotkey(const std::string& key,
		   const std::string& label,
		   const short& fgColor = -1,
		   const short& bgColor = -1);

public:
	/**
	 * Gets the keycode (key combination) of the hotkey
	 * @return The keycode (key combination) of the hotkey
	 */
	std::string getKey() const;

	/**
	 * Gets the display label of the hotkey
	 * @return The display label of the hotkey
	 */
	std::string getLabel() const;

	/**
	 * Sets the display label of the hotkey
	 * @param lbl  The display label of the hotkey
	 */
	void setLabel(const std::string& lbl);

	/**
	 * Gets the index of the foreground and background colors
	 * @return the color code of the hotkey
	 */
	const short getColor() const;

	/**
	 * Allows access to the key and label of the hotkey as if it were a vector
	 * @param  ix Index of the element to fetch
	 * @return    The hotkey key element if ix is zero, the label otherwise.
	 */
	std::string operator[](const uint8_t& ix) const;

private:
	/**
	 * The key combination string
	 */
	std::string key;
	/**
	 * The label string
	 */
	std::string label;
	/**
	 * Index of the foreground and background colors
	 */
	short ixColorPair;

public:
	/**
	 * Represents an empty hotkey with no empty values
	 */
	static const hotkey None;

private:
	static const short ColorsetOffset;
	static short colorsetCount;
};

END_NAMESPACE
#endif //__HOTKEY_H__
