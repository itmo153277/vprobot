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

#ifndef __PARSER_H_
#define __PARSER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <json/json.h>
#include "scene.h"
#include "map.h"
#include "robot.h"
#include "control.h"

namespace vprobot {

namespace scene {

class CNormalScene: public CScene {
public:
	typedef std::vector<vprobot::robot::CRobot *> RobotSet;
private:
	/* Карта */
	vprobot::map::CMap *m_Map;
	/* Роботы */
	RobotSet m_Robots;
	/* Система управления */
	vprobot::control::CControlSystem *m_ControlSystem;
	/* Измерения */
	const vprobot::robot::SMeasures **m_Measures;

	CNormalScene(const CNormalScene &Scene) = default;
public:
	CNormalScene(vprobot::map::CMap *Map, RobotSet Robots,
			vprobot::control::CControlSystem *ControlSystem);
	~CNormalScene();

	/* Выполнить симуляцию */
	bool Simulate();
};

}

vprobot::scene::CScene *Scene(const Json::Value &SceneObject);

}

#endif
