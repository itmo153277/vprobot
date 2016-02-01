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

#ifndef __AI_MCTS_AI_H_
#define __AI_MCTS_AI_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstddef>
#include <random>
#include <functional>
#include <vector>
#include <Eigen/Dense>
#include <json/json.h>
#include "../presentation.h"
#include "../robot.h"
#include "../control.h"

namespace vprobot {

namespace control {

namespace mcts_ai {

class CMCTSAI: public CControlSystem {
public:
	/* Графическая карта */
	typedef Eigen::MatrixXd GridMap;
	/* Сгенерированная карта */
	typedef Eigen::MatrixXi BinaryMap;
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
	/* Количество холостых ходов */
	std::size_t m_AddMoves;
	/* Генератор случайных чисел */
	std::default_random_engine m_Generator;
	/* Распределение */
	std::uniform_real_distribution<double> m_Distribution;
	/* Функция генератора случайных чисел */
	std::function<double()> RandomFunction;

	/* Семпл данных */
	struct SSample {
		int Time;
		double Y;

		SSample(double CurY): Time(0), Y(CurY) {
		}
	};

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
	/* Проверить на фол */
	bool CheckForFoul(const BinaryMap &Map, const StateSet &States);
	/* Генерировать команды */
	bool GenerateCommands();
	/* Пройти по кругу */
	double GoAround(GridMap &Map, const BinaryMap &GeneratedMap,
			const StateSet &States);
	/* Пройти по линии */
	double GoLinear(GridMap &Map, const BinaryMap &GeneratedMap,
			BinaryMap &VisitedMap, double x0, double y0, double xf, double yf,
			double angle);
	/* Обработать точку */
	bool GoExact(GridMap &Map, const BinaryMap &GeneratedMap,
			BinaryMap &VisitedMap, int x, int y, int px, int py, double &CurY);
	/* Преобразовать x в номер */
	int ConvertX(double x);
	/* Преобразовать y в номер */
	int ConvertY(double y);
	/* Сгенерировать семпл */
	void GenerateSample(SSample &Sample, const GridMap &Map, const StateSet &States);

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
	CMCTSAI(const Json::Value &ControlSystemObject);
	~CMCTSAI();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

}

#endif
