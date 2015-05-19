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

#ifndef __AI_AI_H_
#define __AI_AI_H_

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

namespace ai {

class CAIControlSystem: public CControlSystem {
public:
	/* Графическая карта */
	typedef Eigen::MatrixXd GridMap;
	/* Набор карт */
	typedef std::vector<GridMap> MapSet;
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
	/* Размеры карты */
	double m_MapWidth;
	double m_MapHeight;
	std::size_t m_NumWidth;
	std::size_t m_NumHeight;
	/* Начальная позиция */
	double m_StartX;
	double m_StartY;
	/* Набор карт */
	MapSet m_MapSet;
	/* Граница для поиска маяков */
	double m_DetectionThreshold;
	/* Границы учета маяков */
	double m_DeleteBeacon;
	double m_AddBeacon;
	/* Значения для обновления данных о маяках */
	double m_BeaconExists;
	double m_BeaconNotExists;
	/* Количество частиц на робота */
	std::size_t m_NumParticles;
	std::size_t m_NumAddParticles;
	/* Количество симуляций MCTS */
	std::size_t m_NumSimulations;
	/* Параметры функции оценки */
	double m_CT;
	double m_Tmin;
	double m_Cp;
	/* Команды */
	vprobot::robot::ControlCommand *m_Command;

	/* Состояния роботов */
	struct SState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Eigen::Vector3d s_MeanState;
	};
	typedef std::vector<SState> StateSet;
	StateSet m_States;

	/* Вывод данных */
	struct SGridPresentationPrameters: public vprobot::presentation::SPresentationParameters {
		std::size_t m_Num;

		SGridPresentationPrameters(const std::size_t Num) :
				m_Num(Num) {
		}
	};

	/* Генерировать команды */
	bool GenerateCommands();

	CAIControlSystem(const CAIControlSystem &AIControlSystem) = default;
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
	CAIControlSystem(const Json::Value &ControlSystemObject);
	~CAIControlSystem();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

}

#endif
