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

#ifndef __MAP_GRID_H_
#define __MAP_GRID_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstddef>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <json/json.h>
#include "../presentation.h"
#include "../robot.h"
#include "../control.h"

namespace vprobot {

namespace control {

namespace mapping {

/* СУ с графической картой */
class CGridMapper: public CSequentialControlSystem {
public:
	/* Графическая карта */
	typedef Eigen::MatrixXd GridMap;
	/* Набор карт */
	typedef std::vector<GridMap> MapSet;
private:
	/* Обратный радус поворота */
	double m_Radius;
	/* Длина перемещения */
	double m_Len;
	/* Угол отклонения */
	double m_MaxAngle;
	/* Дальность */
	double m_MaxLength;
	/* Значения для обновления карты */
	double m_Occ;
	double m_Free;
	/* Размеры карты */
	double m_MapWidth;
	double m_MapHeight;
	std::size_t m_NumWidth;
	std::size_t m_NumHeight;
	/* Набор карт */
	MapSet m_MapSet;

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

	CGridMapper(const CGridMapper &GridMapper) = default;
protected:
	/* Парсинг параметров для экрана */
	vprobot::presentation::SPresentationParameters *ParsePresentation(
			const Json::Value &PresentationObject);
	/* Отображаем данные */
	void DrawPresentation(
			const vprobot::presentation::SPresentationParameters *Params,
			vprobot::presentation::CPresentationDriver &Driver);
public:
	CGridMapper(const Json::Value &ControlSystemObject);
	~CGridMapper();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

}

#endif
