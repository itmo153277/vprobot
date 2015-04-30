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
#include <string>
#include <random>
#include <Eigen/Dense>
#include <json/json.h>
#include "presentation.h"
#include "line.h"
#include "map.h"

namespace vprobot {

namespace robot {

/* Базовый класс для измерений */
struct SMeasures {

};

/* Измеряем точное положение робота */
struct SMeasuresExactPosition: public SMeasures {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
class CRobot: public vprobot::presentation::CPresentationProvider {
private:
	/* Вывод данных */
	struct SRobotPresentationPrameters: public vprobot::presentation::SPresentationParameters {
		std::string m_OutType;
		SRobotPresentationPrameters(const std::string &OutType) :
				m_OutType(OutType) {
		}
	};

	CRobot(const CRobot &Robot) = default;
protected:
	typedef Eigen::Vector3d State;
	struct SState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		State s_State;
	};

	/* Состояние робота */
	SState m_State;
	/* Радиус поворота */
	double m_Radius;
	/* Погрешность обратного радиуса */
	double m_DRadius;
	/* Длина перемещений */
	double m_Length;
	/* Погрешность перемещения */
	double m_DLength;
	/* Генератор случайных чисел */
	std::default_random_engine m_Generator;

	/* Парсинг параметров для экрана */
	vprobot::presentation::SPresentationParameters *ParsePresentation(
			const Json::Value &PresentationObject);
	/* Отображаем данные */
	void DrawPresentation(
			const vprobot::presentation::SPresentationParameters *Params,
			vprobot::presentation::CPresentationDriver &Driver);
public:
	CRobot(const Json::Value &RobotObject);
	virtual ~CRobot();

	/* Выполнить команду */
	void ExecuteCommand(const Control &Command);
	void ExecuteCommand(const ControlCommand &Command);
	/* Произвести измерения */
	virtual const SMeasures &Measure() = 0;
	/* Установить текущее состояние */
	void SetState(const Json::Value &StateObject);
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
	/* Погрешность измерения */
	double m_DDist;
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
	/* Вывод данных */
	struct SRobotPresentationPrameters: public vprobot::presentation::SPresentationParameters {
		std::string m_OutType;
		SRobotPresentationPrameters(const std::string &OutType) :
				m_OutType(OutType) {
		}
	};

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
	/* Погрешность измерения */
	double m_DDist;
	double m_DAngle;

	CRobotWithScanner(const CRobotWithScanner &Robot) = default;
protected:
	/* Парсинг параметров для экрана */
	vprobot::presentation::SPresentationParameters *ParsePresentation(
			const Json::Value &PresentationObject);
	/* Отображаем данные */
	void DrawPresentation(
			const vprobot::presentation::SPresentationParameters *Params,
			vprobot::presentation::CPresentationDriver &Driver);
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
