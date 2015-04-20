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

#ifndef __ROBOT_H_
#define __ROBOT_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <Eigen/Dense>

namespace vprobot {

namespace robot {

/* Базовый класс для измерений */
struct SMeasures {
};

/* Тип для управления */
typedef Eigen::Vector2d Control;

/* Базовый класс робота */
class CRobot {
private:
	CRobot(const CRobot &Robot) {}
protected:
	/* Состояние робота */
	typedef Eigen::Vector3d State;

	State m_State;
public:
	CRobot();
	virtual ~CRobot();

	/* Выполнить команду */
	void ExecuteCommand(const Control &Command);
	/* Произвести измерения */
	virtual const SMeasures &Measure() const = 0;
};

}

}

#endif
