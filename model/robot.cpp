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
using namespace ::vprobot::robot;

/* CRobot */

vprobot::robot::CRobot::CRobot(): m_State(0,0,0) {
}

vprobot::robot::CRobot::~CRobot() {
}

/* Выполнить команду */
void vprobot::robot::CRobot::ExecuteCommand(const Control &Command) {
	Point dx;
	double angle = Command[0] * Command[1];

	if (EqualsZero(Command[1])) { /* Бесконечный радиус поворота */
		dx[0] = Command[0];
		dx[1] = 0;
	} else {
		dx[0] = sin(angle) / Command[1];
		dx[1] = (1 - cos(angle)) / Command[1];
	}
	Point rdx = Rotate(dx, m_State[2]);
	State ds;

	ds << rdx, angle;
	m_State += ds;
}
