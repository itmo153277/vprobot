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

#include "robot.h"
#include "line.h"
#include "../types.h"

#include <cmath>

using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::line;
using namespace ::vprobot::map;
using namespace ::vprobot::robot;

/* CRobot */

vprobot::robot::CRobot::CRobot(const Json::Value &RobotObject) :
		m_State() {
	m_State << RobotObject["x"].asDouble(), RobotObject["y"].asDouble(), RobotObject["angle"].asDouble();
	m_Radius = 1 / RobotObject["radius"].asDouble();
	m_Length = RobotObject["len"].asDouble();
}

vprobot::robot::CRobot::~CRobot() {
}

/* Выполнить команду */
void vprobot::robot::CRobot::ExecuteCommand(const Control &Command) {
	Point dx;
	double angle = Command[0] * Command[1];

	if (EqualsZero(Command[1])) { /* Бесконечный радиус поворота */
		dx << Command[0], 0;
	} else {
		dx << sin(angle) / Command[1], (1 - cos(angle)) / Command[1];
	}
	m_State += MatrixConvert((State() << Rotate(dx, m_State[2]), angle));
	m_State[2] = CorrectAngle(m_State[2]);
}

void vprobot::robot::CRobot::ExecuteCommand(const ControlCommand &Command) {
	Control Cmd;

	switch (Command) {
	case Nothing:
		Cmd << 0, 0;
		break;
	case Forward:
		Cmd << m_Length, 0;
		break;
	case ForwardLeft:
		Cmd << m_Length, m_Radius;
		break;
	case ForwardRight:
		Cmd << m_Length, -m_Radius;
		break;
	case Backward:
		Cmd << -m_Length, 0;
		break;
	case BackwardLeft:
		Cmd << -m_Length, -m_Radius;
		break;
	case BackwardRight:
		Cmd << -m_Length, m_Radius;
		break;
	}
	ExecuteCommand(Cmd);
}

/* CRobotWithExactPosition */

vprobot::robot::CRobotWithExactPosition::CRobotWithExactPosition(
		const Json::Value &RobotObject) :
		CRobot(RobotObject), m_Measure() {
}

vprobot::robot::CRobotWithExactPosition::~CRobotWithExactPosition() {
}

/* Произвести измерения */
const SMeasures &vprobot::robot::CRobotWithExactPosition::Measure() {
	m_Measure.Value = m_State;
	return m_Measure;
}

/* CRobotWithPointsPosition */

vprobot::robot::CRobotWithPointsPosition::CRobotWithPointsPosition(
		const Json::Value &RobotObject, CMap &Map) :
		CRobot(RobotObject), m_Measure(), m_Map(Map) {
	m_Count = RobotObject["points_count"].asInt();
	m_Measure.Value.resize(m_Count);
}

vprobot::robot::CRobotWithPointsPosition::~CRobotWithPointsPosition() {
}

/* Произвести измерения */
const SMeasures &vprobot::robot::CRobotWithPointsPosition::Measure() {
	size_t i;
	Point r(m_State[0], m_State[1]);

	for (i = 0; i < m_Count; i++) {
		m_Measure.Value[i] = m_Map.GetDistance(r, i);
	}
	return m_Measure;
}

/* CRobotWithPointsPosition */

vprobot::robot::CRobotWithScanner::CRobotWithScanner(
		const Json::Value &RobotObject, CMap &Map) :
		CRobot(RobotObject), m_Measure(), m_Map(Map) {
	m_Count = RobotObject["measures_count"].asInt();
	m_Measure.Value.resize(m_Count);
	m_MaxAngle = RobotObject["max_angle"].asDouble();
}

vprobot::robot::CRobotWithScanner::~CRobotWithScanner() {
}

/* Произвести измерения */
const SMeasures &vprobot::robot::CRobotWithScanner::Measure() {
	size_t i;
	double angle = m_State[2] - m_MaxAngle, da = m_MaxAngle * 2 / m_Count;
	Point r(m_State[0], m_State[1]);

	for (i = 0; i < m_Count; i++, angle += da) {
		m_Measure.Value[i] = m_Map.GetDistance(r, angle);
	}
	return m_Measure;
}
