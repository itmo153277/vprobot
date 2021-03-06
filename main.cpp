/*
 vprobot
 Copyright (C) 2015 Ivanov Viktor

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <json/json.h>
#include <cstdlib>
#include <cerrno>

#include "model/scene.h"
#include "model/parser.h"
#include "ui/ui.h"

using namespace ::std;
using namespace ::vprobot;
using namespace ::vprobot::scene;
using namespace ::vprobot::ui;

int ParseAndRun(const char *in_file) {
	std::ifstream inp(in_file);
	std::stringstream json;

	if (inp.fail()) {
		clog << strerror(errno) << endl;
		return EXIT_FAILURE;
	}
	json << inp.rdbuf();

	Json::Reader reader;
	Json::Value root;

	if (!reader.parse(json.str(), root))
		return EXIT_FAILURE;

	CScene *oScene = Scene(root["scene"]);

	if (oScene == NULL)
		return EXIT_FAILURE;

	CUI UI(*oScene, root["presentation"]);

	UI.Process([&] {oScene->Simulate();});
	delete oScene;
	return EXIT_SUCCESS;
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		cerr << "Usage: " << argv[0] << " model_file" << endl;
		return EXIT_FAILURE;
	}
	return ParseAndRun(argv[1]);
}
