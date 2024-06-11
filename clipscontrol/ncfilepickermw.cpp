/* ** *****************************************************************
* ncfilepickermw.cpp
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
#include "ncursesdialogwin.h"
#include "ncfilepickermw.h"
#include <algorithm>
#include <boost/filesystem/operations.hpp>


/* ** *****************************************************************
*
* Helper functions
*
* ** *****************************************************************/
static inline
void tolower(std::string& s){
	std::transform(s.begin(), s.end(), s.begin(),
    	[](unsigned char c){ return std::tolower(c); });
}


static inline
bool pathCompare(const fs::path& a, const fs::path& b){
	if(!(fs::is_directory(a) ^ fs::is_directory(b))) return a < b;
	return fs::is_directory(a);
}

/* ** *****************************************************************
*
* Class members
*
* ** *****************************************************************/

BEGIN_NAMESPACE


NCFilePickerMW::NCFilePickerMW(const std::string& basepath, const std::set<std::string>& filters):
	path(basepath), filters(filters), ixFirst(0), ixSelected(0){
		setPath(fs::path("."));
}


NCFilePickerMW::~NCFilePickerMW(){
}


std::string NCFilePickerMW::getPath(){
	return path.string();
}


bool NCFilePickerMW::setPath(const std::string& path){
	return setPath( fs::system_complete(fs::path(path)) );
}


bool NCFilePickerMW::setPath(const fs::path& path){
	if( !fs::exists(path) || !fs::is_directory(path) ) return false;
	this->path = fs::canonical(path);
	refreshFilesList();
	werase(win);
	draw();
	return true;
}


std::string NCFilePickerMW::getUserInput(){
	if(result == DialogResult::Ok) return files[ixSelected].string();
	return "";
}


bool NCFilePickerMW::handleKey(const uint32_t& c){
	int first;
	switch(c){
		// case ctrl('X'):
		// 	hide();
		// 	return true;

		case KEY_UP:
			ixSelected = std::max(0, ixSelected - 1);
			first = std::min(ixSelected, ixFirst);
			if(first != ixFirst){
				ixFirst = first;
				werase(win);
			}
			draw();
			return true;

		case KEY_DOWN:
			ixSelected = std::min( (int)files.size()-1, ixSelected + 1);
			first = std::max(0, 3 + ixSelected - rows);
			if(first != ixFirst){
				ixFirst = first;
				werase(win);
			}
			draw();
			return true;

		case KEY_ENTER: case '\n': case '\r':
			selectItem();
			return true;
	}
	return false;
}


void NCFilePickerMW::selectItem(){
	fs::path& path = files[ixSelected];
	if( fs::is_directory(path) ){
		setPath(path);
		return;
	}
	result = DialogResult::Ok;
	hide();
}


void NCFilePickerMW::show(){
	refreshFilesList();
	NCursesDialogWin::show();
}


void NCFilePickerMW::draw(){
	if(!win) return;
	box(win, 0, 0);
	drawTitle();

	for(size_t i = 0; i < (rows-2) && (ixFirst + i) < files.size() ; ++i){
		if( (ixFirst + i) == ixSelected )
			wattron(win, A_REVERSE);
		fs::path& p = files[ixFirst + i];
		if(path.has_parent_path() && ((ixFirst + i) == 0))
			mvwprintw(win, 1+i, 1, "..");
		else{
			std::string pname = p.filename().string();
			if(fs::is_directory(p)) pname+= "/";
			if( pname.length() > (cols -2) )
				pname = pname.substr(0, cols-2);
			mvwprintw(win, 1+i, 1, pname.c_str());
		}
		wattroff(win, A_REVERSE);
	}

	mvwprintw(win, rows-1, 2, " Up\u2191 Down\u2193 Select\u21b2 ");
	wrefresh(win);
}


void NCFilePickerMW::drawTitle(){
	std::string title(" Open CLIPS file ");
	int x = (cols - title.length())/2;
	mvwprintw(win, 0, x, title.c_str());
}


void NCFilePickerMW::refreshFilesList(){
	files.clear();

	fs::directory_iterator end;
	for ( fs::directory_iterator itr(path); itr != end; ++itr ){
		if(itr->path().filename().string()[0] == '.') continue;
		if(fs::is_directory( itr->path() )){
			files.push_back(itr->path());
			continue;
		}
		std::string ext = itr->path().extension().string();
		tolower(ext);
		if( !filters.empty() && (filters.find(ext) != filters.end()) )
			files.push_back(itr->path());
	}

	sortFiles();
	if(path.has_parent_path())
		files.insert(files.begin(), path.parent_path());
	ixSelected = ixFirst = 0;
}


void NCFilePickerMW::sortFiles(){
	std::sort(files.begin(), files.end(), pathCompare);
}

END_NAMESPACE
