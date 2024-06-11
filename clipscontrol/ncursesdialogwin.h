/* ** *****************************************************************
* ncursesdialogwin.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file ncursesdialogwin.h
 * Definition of the NCursesDialogWin class: Base class for modal and
 * dialog windows.
 */

#ifndef __NCURSES_DIALOG_WIN_H__
#define __NCURSES_DIALOG_WIN_H__
#pragma once

/** @cond */
#include <string>
#include <vector>
#include <ncurses.h>
#include "namespace.h"
/** @endcond */

BEGIN_NAMESPACE

/**
 * Enumerates the user's choice when a dialog is closed
 */
enum class DialogResult{
	/**
	 * No choice, modal still open
	 */
	None,
	/**
	 * Operation cancelled
	 */
	Cancel,
	/**
	 * OK
	 */
	Ok,
	/**
	 * Retry
	 */
	Retry,
	/**
	 * Yes
	 */
	Yes,
	/**
	 * No
	 */
	No
};

/**
 * Base class for modal and dialog windows.
 */
class NCursesDialogWin{
public:
	/**
	 * Initializes a new instance of NCursesDialogWin
	 */
	NCursesDialogWin();
	~NCursesDialogWin();

private:
	NCursesDialogWin(const NCursesDialogWin&)            = delete;
	NCursesDialogWin& operator=(const NCursesDialogWin&) = delete;

public:
	/**
	 * When the modal closes, contains the end-of-interaction choice
	 * picked by the user; or None if the modal remains open.
	 * @return Modal closure choice.
	 */
	DialogResult getResult();

	/**
	 * When overriden in a derived class retrieves a string
	 * representation of the interaction with the user if any, like a
	 * file name or input text. Otherwise returns an empty string.
	 * @return A string representation of the interaction with the user
	 */
	virtual std::string getUserInput();

	/**
	 * Displays the modal window.
	 * It normally involves creating the window.
	 */
	virtual void show();

	/**
	 * Hides the modal window.
	 * It normally involves destroying the window.
	 */
	virtual void hide();

	/**
	 * Gets a value indicating whether the modal window is visible
	 * @return A value indicating whether the modal window is visible
	 */
	virtual bool isVisible();


	/**
	 * When overriden in a derived class, it handles a keypress passed
	 * by the parent window.
	 * @param c They NCurses keycode to handle
	 * @return  true if the key was handled, false otherwise
	 */
	virtual bool handleKey(const uint32_t& c) = 0;

	/**
	 * When overriden in a derived class, it draws the modal window
	 */
	virtual void draw()                       = 0;

// protected:

protected:
	/**
	 * The NCurses window used to display the modal
	 */
	WINDOW *win;

	/**
	 * The X position of the modal window (column number).
	 * If negative, the window is horizontally centered. Default: -1.
	 */
	int posX;

	/**
	 * The Y position of the modal window (row number).
	 * If negative, the window is vertically centered. Default: -1.
	 */
	int posY;

	/**
	 * The width (number of columns) of the modal window.
	 * If negative, the default window width is half the width of the
	 * screen. Default: -1.
	 */
	int cols;

	/**
	 * The width (number of rows) of the modal window.
	 * If negative, the default window width is half the height of the
	 * screen. Default: -1.
	 */
	int rows;

	/**
	 * Contains the end-of-interaction choice, or None if the modal
	 * remains open.
	 */
	DialogResult result;
};

END_NAMESPACE

#endif // __NCURSES_DIALOG_WIN_H__