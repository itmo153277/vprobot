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
#include <iostream>
#include <Eigen/Dense>
#include <json/json.h>
#include "line.h"
#include "map.h"

#include "../types.h"

namespace vprobot {

namespace robot {

/* Базовый класс для измерений */
struct SMeasures {
	virtual ~SMeasures() = default;

	/* Вывод значений */
	virtual void PrintToStream(std::ostream &os) const {
	}
};

/* Измеряем точное положение робота */
struct SMeasuresExactPosition: public SMeasures {
	Eigen::Vector3d Value;

	/* Вывод значений */
	void PrintToStream(std::ostream &os) const {
		os << Value.transpose();
	}
};

/* Измеряем точное положение маяков */
struct SMeasuresPointsPosition: public SMeasures {
	Eigen::VectorXd Value;

	/* Вывод значений */
	void PrintToStream(std::ostream &os) const {
		os << Value;
	}
};

/* Измеряем расстояние по направлениям */
struct SMeasuresDistances: public SMeasures {
	Eigen::VectorXd Value;

	/* Вывод значений */
	void PrintToStream(std::ostream &os) const {
		int i;
		double angle;
		double ca = vprobot::PI / Value.rows();
		for (i = 0, angle = 0; i < Value.rows(); i++, angle += ca) {
			line::Point r = line::Rotate(MatrixConvert((line::Point() << Value[i], 0)), angle);
			os << r.transpose() << std::endl;
		}
	}
};

/* Тип для управления */
typedef Eigen::Vector2d Control;
/* Команды */
enum ControlCommand {
	Nothing = 0,
	Forward,
	ForwardRight,
	ForwardLeft,
	Backward,
	BackwardRight,
	BackwardLeft
};

/* Базовый класс робота */
class CRobot {
private:
	CRobot(const CRobot &Robot) = default;
protected:
	typedef Eigen::Vector3d State;

	/* Состояние робота */
	State m_State;
	/* Радиус поворота */
	double m_Radius;
	/* Длина перемещений */
	double m_Length;
public:
	CRobot(const Json::Value &RobotObject);
	virtual ~CRobot();

	/* Выполнить команду */
	void ExecuteCommand(const Control &Command);
	void ExecuteCommand(const ControlCommand &Command);
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
	CRobotWithPointsPosition(const Json::Value &RobotObject,
			::vprobot::map::CMap &Map);
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
	/* Дальность */
	double m_MaxLength;
public:
	CRobotWithScanner(const Json::Value &RobotObject,
			::vprobot::map::CMap &Map);
	~CRobotWithScanner();

	/* Произвести измерения */
	const SMeasures &Measure();
};

}

}

#endif
