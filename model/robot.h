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

#include <cstddef>
#include <Eigen/Dense>
#include <json/json.h>
#include "map.h"

namespace vprobot {

namespace robot {

/* Базовый класс для измерений */
struct SMeasures {
};

/* Измеряем точное положение робота */
struct SMeasuresExactPosition: public SMeasures {
	Eigen::Vector3d Value;
};

/* Измеряем точное положение маяков */
struct SMeasuresPointsPosition: public SMeasures {
	Eigen::VectorXd Value;
};

/* Измеряем расстояние по направлениям */
struct SMeasuresDistances: public SMeasures {
	Eigen::VectorXd Value;
};

/* Тип для управления */
typedef Eigen::Vector2d Control;

/* Базовый класс робота */
class CRobot {
private:
	CRobot(const CRobot &Robot) = default;
protected:
	/* Состояние робота */
	typedef Eigen::Vector3d State;

	State m_State;
public:
	CRobot(const Json::Value &RobotObject);
	virtual ~CRobot();

	/* Выполнить команду */
	void ExecuteCommand(const Control &Command);
	/* Произвести измерения */
	virtual const SMeasures &Measure() = 0;
};

/* Робот, точно возвращающий позицию */
class CRobotWithExactPosition: public CRobot {
private:
	CRobotWithExactPosition(const CRobotWithExactPosition &Robot) = default;

	/* Измерение*/
	SMeasuresExactPosition m_Measure;
public:
	CRobotWithExactPosition(const Json::Value &RobotObject);
	~CRobotWithExactPosition();

	/* Произвести измерения */
	const SMeasures &Measure();
};

/* Робот, возвращающий позицию точек */
class CRobotWithPointsPosition: public CRobot {
private:
	CRobotWithPointsPosition(const CRobotWithPointsPosition &Robot) = default;

	/* Измерение*/
	SMeasuresPointsPosition m_Measure;
	/* Храним ссылку на карту */
	::vprobot::map::CMap &m_Map;
	/* Колличество измерений */
	std::size_t m_Count;
public:
	CRobotWithPointsPosition(const Json::Value &RobotObject, ::vprobot::map::CMap &Map);
	~CRobotWithPointsPosition();

	/* Произвести измерения */
	const SMeasures &Measure();
};

/* Робот, возвращающий расстояния до препятствий */
class CRobotWithScanner: public CRobot {
private:
	CRobotWithScanner(const CRobotWithScanner &Robot) = default;

	/* Измерение*/
	SMeasuresDistances m_Measure;
	/* Храним ссылку на карту */
	::vprobot::map::CMap &m_Map;
	/* Колличество измерений */
	std::size_t m_Count;
	/* Угол отклонения */
	double m_MaxAngle;
public:
	CRobotWithScanner(const Json::Value &RobotObject, ::vprobot::map::CMap &Map);
	~CRobotWithScanner();

	/* Произвести измерения */
	const SMeasures &Measure();
};

}

}

#endif
