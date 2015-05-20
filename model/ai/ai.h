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
#include <functional>
#include <random>
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
	/* Габариты робота */
	double m_RobotWidth;
	double m_RobotHeight;
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
	/* Критерий окончания */
	double m_EndC;
	/* Библиотека команд */
	std::size_t m_NumCommands;
	vprobot::robot::ControlCommand **m_CommandLibrary;
	/* Генератор случайных чисел */
	std::default_random_engine m_Generator;
	/* Функция генератора случайных чисел */
	std::function<double()> RandomFunction;

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

	/* Ветви дерева */
	struct STreeNode {
		/* Родитель */
		STreeNode *Parent;
		/* Дети */
		STreeNode **Childs;
		/* Текущий вес */
		double Weight;
		/* Сумма весов детей */
		double WeightSum;
		/* Лучшее значение функционала */
		double BestY;
		/* Текущее значение функционала */
		double SelfY;
		/* Прибыль */
		double Q;
		/* Количество посещений */
		std::size_t n_vis;
		/* Карта */
		GridMap Map;
		/* Состояния роботов */
		StateSet States;
	};
	/* Дерево */
	STreeNode *m_Tree;

	/* Инициализация ветви */
	void InitializeNode(STreeNode *Node, STreeNode *Parent, const GridMap &Map,
			const StateSet &States);
	/* Посчитать значение функционала */
	void UpdateY(STreeNode *Node, int Level);
	/* Добавить ветвь */
	void AddChild(STreeNode *Node, STreeNode *FreeNode, int Level);
	/* Обновить состояния */
	void UpdateStates(const vprobot::robot::ControlCommand *Commands,
			StateSet &States);
	/* Обновить карту */
	void UpdateMap(STreeNode *Node, const GridMap &ParentMap);

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
