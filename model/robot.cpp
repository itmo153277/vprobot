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

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::line;
using namespace ::vprobot::map;
using namespace ::vprobot::robot;

/* CRobot */

vprobot::robot::CRobot::CRobot(const Json::Value &RobotObject) :
		CPresentationProvider(), m_State() {
	m_State.s_State << RobotObject["x"].asDouble(), RobotObject["y"].asDouble(), RobotObject["angle"].asDouble();
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
	m_State.s_State +=
			(State() << Rotate(dx, m_State.s_State[2]), angle).finished();
	m_State.s_State[2] = CorrectAngle(m_State.s_State[2]);
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
			Cmd << -m_Length, m_Radius;
			break;
		case BackwardRight:
			Cmd << -m_Length, -m_Radius;
			break;
	}
	ExecuteCommand(Cmd);
}

SPresentationParameters *vprobot::robot::CRobot::ParsePresentation(
		const Json::Value &PresentationObject) {
	const string &DrawType = PresentationObject["draw"].asString();

	if (DrawType == "Position") {
		return new SRobotPresentationPrameters(DrawType);
	}
	return NULL;
}

void vprobot::robot::CRobot::DrawPresentation(
		const SPresentationParameters *Params, CPresentationDriver &Driver) {
	const SRobotPresentationPrameters *i_Params =
			dynamic_cast<const SRobotPresentationPrameters *>(Params);

	if (i_Params != NULL && i_Params->m_OutType == "Position") {
		Driver.DrawLine(m_State.s_State[0], m_State.s_State[1],
				m_State.s_State[0] + cos(m_State.s_State[2]) * 0.5,
				m_State.s_State[1] + sin(m_State.s_State[2]) * 0.5, 0, 0, 255,
				255);
		Driver.DrawCircle(m_State.s_State[0], m_State.s_State[1], 0.3, 255, 0,
				0, 255);
	}
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
	m_Measure.Value = m_State.s_State;
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
	Point r(m_State.s_State[0], m_State.s_State[1]);

	for (i = 0; i < m_Count; i++) {
		m_Measure.Value[i] = m_Map.GetDistance(r, i);
	}
	return m_Measure;
}

/* CRobotWithPointsScanner */

vprobot::robot::CRobotWithScanner::CRobotWithScanner(
		const Json::Value &RobotObject, CMap &Map) :
		CRobot(RobotObject), m_Measure(), m_Map(Map) {
	m_Count = RobotObject["measures_count"].asInt();
	m_Measure.Value.resize(m_Count);
	m_MaxAngle = RobotObject["max_angle"].asDouble();
	m_MaxLength = RobotObject["max_length"].asDouble();
}

vprobot::robot::CRobotWithScanner::~CRobotWithScanner() {
}

SPresentationParameters *vprobot::robot::CRobotWithScanner::ParsePresentation(
		const Json::Value &PresentationObject) {
	const string &DrawType = PresentationObject["draw"].asString();

	if (DrawType == "Measurements") {
		return new SRobotPresentationPrameters(DrawType);
	}
	return CRobot::ParsePresentation(PresentationObject);
}

void vprobot::robot::CRobotWithScanner::DrawPresentation(
		const SPresentationParameters *Params, CPresentationDriver &Driver) {
	const SRobotPresentationPrameters *i_Params =
			dynamic_cast<const SRobotPresentationPrameters *>(Params);

	if (i_Params != NULL && i_Params->m_OutType == "Measurements") {
		double *mx, *my, da = m_MaxAngle * 2 / m_Count, angle = PI / 2
				- m_MaxAngle + da / 2;
		size_t i;

		mx = new double[m_Count + 1];
		my = new double[m_Count + 1];
		*mx = 0;
		*my = 0;
		for (i = 0; i < m_Count; i++, angle += da) {
			double d = m_Measure.Value[i];

			if (EqualsZero(d))
				d = m_MaxLength;
			mx[i + 1] = d * cos(angle);
			my[i + 1] = d * sin(angle);
		}
		Driver.DrawShape(mx, my, m_Count + 1, 0, 0, 0, 0, 242, 242, 242, 255);
		for (i = 0; i < m_Count; i++) {
			if (EqualsZero(m_Measure.Value[i]))
				continue;
			Driver.DrawCircle(mx[i + 1], my[i + 1], 0.1, 128, 128, 128, 255);
		}
		Driver.DrawLine(0, 0, 0, 0.5, 0, 0, 255, 255);
		Driver.DrawCircle(0, 0, 0.3, 255, 0, 0, 255);
		delete[] mx;
		delete[] my;
	}
	CRobot::DrawPresentation(Params, Driver);
}

/* Произвести измерения */
const SMeasures &vprobot::robot::CRobotWithScanner::Measure() {
	size_t i;
	double angle = m_State.s_State[2] - m_MaxAngle, da = m_MaxAngle * 2
			/ m_Count;
	Point r(m_State.s_State[0], m_State.s_State[1]);

	for (i = 0; i < m_Count; i++, angle += da) {
		double d = m_Map.GetDistance(r, angle);

		if (GreaterThan(d, m_MaxLength))
			d = 0;
		m_Measure.Value[i] = d;
	}
	return m_Measure;
}
