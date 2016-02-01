/*
 vprobot
 Copyright (C) 2016 Ivanov Viktor

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

#ifndef __AI_SIMPLE_AI_H_
#define __AI_SIMPLE_AI_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstddef>
#include <vector>
#include <Eigen/Dense>
#include <json/json.h>
#include "../presentation.h"
#include "../robot.h"
#include "../control.h"

namespace vprobot {

namespace control {

namespace simple_ai {

class CSimpleAI: public CControlSystem {
public:
	/* Графическая карта */
	typedef Eigen::MatrixXd GridMap;
private:
	/* Обратный радус поворота */
	double m_Radius;
	double m_DRadius;
	/* Длина перемещения */
	double m_Len;
	double m_DLen;
	/* Угол отклонения */
	double m_MaxAngle;
	double m_DAngle;
	/* Дальность */
	double m_MaxLength;
	double m_DDist;
	/* Значения для обновления карты */
	double m_Occ;
	double m_Free;
	/* Габариты робота */
	double m_RobotWidth;
	double m_RobotHeight;
	/* Графическая карта */
	GridMap m_Map;
	/* Размеры карты */
	double m_MapWidth;
	double m_MapHeight;
	std::size_t m_NumWidth;
	std::size_t m_NumHeight;
	/* Начальная позиция */
	double m_StartX;
	double m_StartY;
	/* Библиотека команд */
	std::size_t m_NumCommands;
	vprobot::robot::ControlCommand **m_CommandLibrary;
	/* Состояния роботов */
	struct SState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Eigen::Vector3d s_MeanState;
	};
	typedef std::vector<SState> StateSet;
	StateSet m_States;
	/* Критерий окончания */
	double m_EndC;

	/* Вывод данных */
	struct SGridPresentationPrameters: public vprobot::presentation::SPresentationParameters {
		std::size_t m_Num;

		SGridPresentationPrameters(const std::size_t Num) :
				m_Num(Num) {
		}
	};

	/* Обновить состояния */
	void UpdateStates(const vprobot::robot::ControlCommand *Commands,
			StateSet &States);
	/* Обновить карту */
	void UpdateMap(GridMap &Map, const StateSet &States);
	/* Проверить на фол */
	bool CheckForFoul(const GridMap &Map, const StateSet &States);
	/* Генерировать команды */
	bool GenerateCommands();
protected:
	/* Парсинг параметров для экрана */
	vprobot::presentation::SPresentationParameters *ParsePresentation(
			const Json::Value &PresentationObject);
	/* Отображаем данные */
	void DrawPresentation(
			const vprobot::presentation::SPresentationParameters *Params,
			double IndicatorZoom,
			vprobot::presentation::CPresentationDriver &Driver);
public:
	CSimpleAI(const Json::Value &ControlSystemObject);
	~CSimpleAI();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

}

#endif
