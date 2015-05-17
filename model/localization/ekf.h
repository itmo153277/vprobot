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

#ifndef __LOC_EKF_H_
#define __LOC_EKF_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <Eigen/Dense>
#include <json/json.h>
#include "../presentation.h"
#include "../line.h"
#include "../robot.h"
#include "../control.h"

namespace vprobot {

namespace control {

namespace localization {

/* СУ с наблюдателем EKF */
class CEKFLocalization: public CSequentialControlSystem {
private:
	typedef std::vector<line::Point, Eigen::aligned_allocator<line::Point>> MapList;

	/* Содержание карты */
	MapList m_List;

	/* Параметры робота */
	/* Обратный радус поворота */
	double m_Radius;
	/* Погрешность радиуса */
	double m_DRadius;
	/* Длина перемещения */
	double m_Len;
	/* Погрешность перемещения */
	double m_DLen;
	/* Погрешность по дистанции */
	double m_DDist;

	/* Состояния для каждого робота */
	struct SState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/* Мат ожидание состояния */
		Eigen::Vector3d s_MeanState;
		/* Матрица ковариаций */
		Eigen::Matrix3d s_CovState;
	};
	typedef std::vector<SState> StateSet;
	StateSet m_States;

	/* Функция обработки */
	void Process(const vprobot::robot::SMeasures * const *Measurements);

	CEKFLocalization(const CEKFLocalization &ControlSystem) = default;
protected:
	/* Отображаем данные */
	void DrawPresentation(
			const vprobot::presentation::SPresentationParameters *Params,
			double IndicatorZoom,
			vprobot::presentation::CPresentationDriver &Driver);
public:
	CEKFLocalization(const Json::Value &ControlSystemObject);
	~CEKFLocalization();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

}

#endif
