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

#include "parser.h"

#include <cstddef>

using namespace ::std;
using namespace ::vprobot;
using namespace ::vprobot::map;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::scene;

CScene *vprobot::Scene(const Json::Value &SceneObject) {
	return NULL;
}

/* CNormalScene */

vprobot::scene::CNormalScene::CNormalScene(CMap *Map, RobotSet Robots,
		CControlSystem *ControlSystem) :
		m_Map(Map), m_Robots(Robots), m_ControlSystem(ControlSystem) {
	m_Measures = new const SMeasures *[m_Robots.size()];
}

vprobot::scene::CNormalScene::~CNormalScene() {
	delete m_Measures;
	delete m_ControlSystem;
	for (auto r: m_Robots)
		delete r;
	delete [] m_Map;
}

void vprobot::scene::CNormalScene::Simulate() {
	size_t i;

	for (i = 0; i < m_Robots.size(); i++) {
		m_Measures[i] = &(m_Robots[i]->Measure());
	}

	const ControlCommand * const &cmd = m_ControlSystem->GetCommands(m_Measures);

	for (i = 0; i < m_Robots.size(); i++) {
		m_Robots[i]->ExecuteCommand(cmd[i]);
	}
}
