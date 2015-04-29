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
#include <sstream>
#include <functional>

#include "localization/ekf.h"

using namespace ::std;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::map;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::control::localization;
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
	const int cControlSystemsTypes = 2;
	static const char *ControlSystemAliases[cControlSystemsTypes] = {
			"Sequential", "EKF Localization"};
	function<CControlSystem *()> ControlSystemConstructors[cControlSystemsTypes] =
			{
					[&]() {return new CSequentialControlSystem(SceneObject["control_system"]);},
					[&]() {return new CEKFLocalization(SceneObject["control_system"]);}};

	size_t i;
	for (i = 0; i < cMapTypes; i++) {
		if (MapType == MapAliases[i]) {
			Map = MapConstructers[i]();
			Map->InitPresentations(SceneObject["map"]["presentations"]);
			break;
		}
	}
	if (Map == NULL)
		return NULL;
	for (i = 0; i < cRobotTypes; i++) {
		if (RobotType == RobotAliases[i]) {
			for (size_t j = RobotsCount; j > 0; j--) {
				CRobot *r = RobotConstructors[i]();

				r->SetState(
						SceneObject["robot_states"][(Json::ArrayIndex) j - 1]);
				r->InitPresentations(SceneObject["robot"]["presentations"]);
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
			ControlSystem->InitPresentations(
					SceneObject["control_system"]["presentations"]);
			break;
		}
	}
	if (ControlSystem == NULL) {
		for (auto r : Robots)
			delete r;
		delete Map;
		return NULL;
	}
	return new CNormalScene(Map, Robots, ControlSystem,
			SceneObject["presentations"]);
}

/* CNormalScene */

vprobot::scene::CNormalScene::CNormalScene(CMap *Map, RobotSet Robots,
		CControlSystem *ControlSystem, const Json::Value &PresentationObject) :
		m_Map(Map), m_Robots(Robots), m_ControlSystem(ControlSystem), m_Commands(
		NULL), m_Time(0), m_Info(*this) {
	m_Measures = new const SMeasures *[m_Robots.size()];
	m_Info.InitPresentations(PresentationObject);
}

vprobot::scene::CNormalScene::~CNormalScene() {
	delete m_ControlSystem;
	for (auto r : m_Robots)
		delete r;
	delete m_Map;
	delete[] m_Measures;
}

/* Выполнить симуляцию */
void vprobot::scene::CNormalScene::Simulate() {
	size_t i;

	if (m_Commands != NULL || m_Time == 0) {
		if (m_Commands != NULL) {
			for (i = 0; i < m_Robots.size(); i++) {
				m_Robots[i]->ExecuteCommand(m_Commands[i]);
			}
		}
		for (i = 0; i < m_Robots.size(); i++) {
			m_Measures[i] = &(m_Robots[i]->Measure());
		}
		m_Time++;
		m_Commands = NULL;
		m_sState = SimulationWorking;
	} else {
		m_Commands = m_ControlSystem->GetCommands(m_Measures);
		if (m_Commands == NULL)
			m_sState = SimulationEnd;
		else
			m_sState = SimulationWait;
	}
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
	m_Info.UpdatePresentation(Driver, Name);
}

/* CNormalScene::CInfo */

vprobot::scene::CNormalScene::CInfo::CInfo(const CNormalScene &Scene) :
		CPresentationProvider(), m_Scene(Scene) {
}

vprobot::scene::CNormalScene::CInfo::~CInfo() {
}

/* Парсинг параметров для экрана */
SPresentationParameters *vprobot::scene::CNormalScene::CInfo::ParsePresentation(
		const Json::Value &PresentationObject) {
	return new SInfoPresentationPrameters(PresentationObject);
}

/* Отображаем данные */
void vprobot::scene::CNormalScene::CInfo::DrawPresentation(
		const SPresentationParameters *Params, CPresentationDriver &Driver) {
	const SInfoPresentationPrameters *i_Params =
			dynamic_cast<const SInfoPresentationPrameters *>(Params);

	if (i_Params != NULL) {
		stringstream outstr;

		outstr << "Simulation step: " << m_Scene.m_Time << flush;
		Driver.PutText(i_Params->x, i_Params->y, outstr.str().c_str(), 0, 0, 0,
				255);
		outstr.str(string());
		outstr << "Status: ";
		switch (m_Scene.GetSimlationState()) {
			case SimulationWait:
				outstr << "Waiting";
				break;
			case SimulationWorking:
				outstr << "Working";
				break;
			case SimulationEnd:
				outstr << "Stopped";
				break;
			default:
				outstr << "Unknown";
				break;
		}
		outstr << flush;
		Driver.PutText(i_Params->x, i_Params->y + i_Params->dy,
				outstr.str().c_str(), 0, 0, 0, 255);
	}
}
