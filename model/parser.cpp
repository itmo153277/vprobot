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
#include <string>
#include <functional>

using namespace ::std;
using namespace ::vprobot;
using namespace ::vprobot::map;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::scene;

CScene *vprobot::Scene(const Json::Value &SceneObject) {
	CMap *Map = NULL;
	CNormalScene::RobotSet Robots;
	CControlSystem *ControlSystem = NULL;

	string MapType(SceneObject["map_type"].asString());
	string RobotType(SceneObject["robot_type"].asString());
	string ControlSystemType(SceneObject["control_system_type"].asString());
	size_t RobotsCount = SceneObject["robots_count"].asInt();

	const int cMapTypes = 2;
	static const char *MapAliases[cMapTypes] = {"Point", "Line"};

	function<CMap *()> MapConstructers[cMapTypes] = {
			[&]() {return new CPointMap(SceneObject["map"]);},
			[&]() {return new CLineMap(SceneObject["map"]);}};
	const int cRobotTypes = 3;
	static const char *RobotAliases[cRobotTypes] = {"WithExactPosition",
			"WithPointsPosition", "WithScanner"};
	function<CRobot *()> RobotConstructors[cRobotTypes] =
			{
					[&]() {return new CRobotWithExactPosition(SceneObject["robot"]);},
					[&]() {return new CRobotWithPointsPosition(SceneObject["robot"], *Map);},
					[&]() {return new CRobotWithScanner(SceneObject["robot"], *Map);}};
	const int cControlSystemsTypes = 1;
	static const char *ControlSystemAliases[cControlSystemsTypes] = {
			"Sequential"};
	function<CControlSystem *()> ControlSystemConstructors[cControlSystemsTypes] =
			{
					[&]() {return new CSequentialControlSystem(SceneObject["control_system"]);}};

	size_t i;
	for (i = 0; i < cMapTypes; i++) {
		if (MapType == MapAliases[i]) {
			Map = MapConstructers[i]();
			break;
		}
	}
	if (Map == NULL)
		return NULL;
	for (i = 0; i < cRobotTypes; i++) {
		if (RobotType == RobotAliases[i]) {
			for (size_t j = RobotsCount; j > 0; j--) {
				CRobot *r = RobotConstructors[i]();

				r->InitPresentations(
						SceneObject["robot_presentations"][(Json::ArrayIndex) j
								- 1]);
				Robots.push_back(r);
			}
			break;
		}
	}
	if (Robots.size() == 0) {
		delete[] Map;
		return NULL;
	}
	for (i = 0; i < cControlSystemsTypes; i++) {
		if (ControlSystemType == ControlSystemAliases[i]) {
			ControlSystem = ControlSystemConstructors[i]();
			break;
		}
	}
	if (ControlSystem == NULL) {
		for (auto r : Robots)
			delete r;
		delete Map;
		return NULL;
	}
	return new CNormalScene(Map, Robots, ControlSystem);
}

/* CNormalScene */

vprobot::scene::CNormalScene::CNormalScene(CMap *Map, RobotSet Robots,
		CControlSystem *ControlSystem) :
		m_Map(Map), m_Robots(Robots), m_ControlSystem(ControlSystem), m_Commands(
		NULL) {
	m_Measures = new const SMeasures *[m_Robots.size()];
}

vprobot::scene::CNormalScene::~CNormalScene() {
	delete m_ControlSystem;
	for (auto r : m_Robots)
		delete r;
	delete m_Map;
	delete[] m_Measures;
}

/* Выполнить симуляцию */
bool vprobot::scene::CNormalScene::Simulate() {
	size_t i;

	if (m_Commands != NULL) {
		for (i = 0; i < m_Robots.size(); i++) {
			m_Robots[i]->ExecuteCommand(m_Commands[i]);
		}
	}
	for (i = 0; i < m_Robots.size(); i++) {
		m_Measures[i] = &(m_Robots[i]->Measure());
	}
	m_Commands = m_ControlSystem->GetCommands(m_Measures);
	return m_Commands != NULL;
}

/* Нарисовать презентацию */
void vprobot::scene::CNormalScene::DrawPresentation(
		vprobot::presentation::CPresentationDriver &Driver,
		const std::string &Name) {
	m_Map->UpdatePresentation(Driver, Name);
	for (auto r : m_Robots) {
		r->UpdatePresentation(Driver, Name);
	}
	m_ControlSystem->UpdatePresentation(Driver, Name);
}
