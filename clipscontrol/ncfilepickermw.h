/* ** *****************************************************************
* ncfilepickermw.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file ncfilepickermw.h
 * Definition of the NCFilePickerMW class: Displays a list of files and
 * allow to select one.
 */

#ifndef __NC_FILE_PICKER_MW_H__
#define __NC_FILE_PICKER_MW_H__
#pragma once

/** @cond */
#include <set>
#include <string>
#include <vector>
#include <ncurses.h>
#include <boost/filesystem/path.hpp>

#include "namespace.h"
/** @endcond */

#include "ncursesdialogwin.h"

namespace fs = boost::filesystem;
BEGIN_NAMESPACE

/**
 * Modal window to explore the filesystem and pick a file
 */
class NCFilePickerMW : public NCursesDialogWin{
public:
	/**
	 * Initializes a new instance of NCursesDialogWin
	 * @param basepath  Optional. The path of the directory whose
	 *                  contents will be loaded/explored by the
	 *                  dialogue window. Default: cwd().
	 * @param filters   Optional. A set of filters such as file
	 *                  extensions to narrow the list of displayed
	 *                  files. An empty set will show all files.
	 *                  Does not support wildcards. Default: empty set.
	 */
	NCFilePickerMW(const std::string& basepath=".", const std::set<std::string>& filters={});
	~NCFilePickerMW();

private:
	NCFilePickerMW(const NCFilePickerMW&)            = delete;
	NCFilePickerMW& operator=(const NCFilePickerMW&) = delete;

public:

	/**
	 * Gets the base directory shown by the modal dialog
	 * @return The base directory
	 */
	std::string getPath();

	/**
	 * Sets the base directory, causing the modal to refresh its
	 * contents to whatever files may be in the new path.
	 * @param  path The directory to inspect
	 * @return      true if the new path was set and its contents
	 *              read, false otherwise
	 */
	bool setPath(const std::string& path);

	/**
	 * Retrieves the path of the selected file, if any.
	 * @return The path of the selected file.
	 */
	virtual std::string getUserInput();

	/**
	 * Handles a keypress passed by the parent window.
	 * @param c They NCurses keycode to handle
	 * @return  true if the key was handled, false otherwise
	 */
	virtual bool handleKey(const uint32_t& c);

	/**
	 * Displays the modal window.
	 */
	virtual void show();

protected:
	/**
	 * Draws the modal window
	 */
	virtual void draw();

	/**
	 * Draws the title of the modal window
	 */
	void drawTitle();

	/**
	 * Updates the contents of \p files, reading again the \p path directory.
	 */
	void refreshFilesList();

	/**
	 * Activates the highlighted item, either changing the path or
	 * closing the dialogue and returning a filepath.
	 */
	void selectItem();

	/**
	 * Sets the base directory, causing the modal to refresh its
	 * contents to whatever files may be in the new path.
	 * @param  path The directory to inspect
	 * @return      true if the new path was set and its contents
	 *              read, false otherwise
	 */
	bool setPath(const fs::path& path);

	/**
	 * Sorts the content of \p files, placing directories on top.
	 */
	void sortFiles();

protected:
	/**
	 * The path (directory) being explored by the modal.
	 */
	fs::path path;
	/**
	 * The list of files contained in the \p path directory.
	 */
	std::vector<fs::path> files;
	/**
	 * Set of filters (e.g. file extensions) to narrow the search.
	 */
	std::set<std::string> filters;
	/**
	 * Zero-based index of the first path to display.
	 */
	int ixFirst;
	/**
	 * Zero-based index of the highlighted (selected) path.
	 */
	int ixSelected;
};

END_NAMESPACE

#endif // __NC_FILE_PICKER_MW_H__
