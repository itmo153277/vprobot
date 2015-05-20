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

#include "ai.h"

#include <cmath>
#include <cstring>

#include "../../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::control::ai;

/* CAIControlSystem */

vprobot::control::ai::CAIControlSystem::CAIControlSystem(
		const Json::Value &ControlSystemObject) :
		CControlSystem(ControlSystemObject), m_Generator(), m_States() {
	random_device rd;

	m_Generator.seed(rd());
	RandomFunction = [&] {return generate_canonical<double, 10>(m_Generator);};

	double i_Occ, i_Free;

	m_Radius = 1 / ControlSystemObject["radius"].asDouble();
	m_DRadius = ControlSystemObject["dradius"].asDouble();
	m_Len = ControlSystemObject["len"].asDouble();
	m_DLen = ControlSystemObject["dlen"].asDouble();
	m_MaxAngle = ControlSystemObject["max_angle"].asDouble();
	m_DAngle = ControlSystemObject["dangle"].asDouble();
	m_MaxLength = ControlSystemObject["max_length"].asDouble();
	m_DDist = ControlSystemObject["ddist"].asDouble();
	m_RobotWidth = ControlSystemObject["robot_width"].asDouble();
	m_RobotHeight = ControlSystemObject["robot_height"].asDouble();
	i_Occ = ControlSystemObject["prob_occ"].asDouble();
	i_Free = ControlSystemObject["prob_free"].asDouble();
	m_Occ = log(i_Occ / (1 - i_Occ));
	m_Free = log(i_Free / (1 - i_Free));
	m_MapWidth = ControlSystemObject["map_width"].asDouble();
	m_MapHeight = ControlSystemObject["map_height"].asDouble();
	m_NumWidth = ControlSystemObject["num_width"].asInt();
	m_NumHeight = ControlSystemObject["num_height"].asInt();
	m_StartX = ControlSystemObject["start_x"].asDouble();
	m_StartY = ControlSystemObject["start_y"].asDouble();

	size_t i;
	const Json::Value Params = ControlSystemObject["robot_params"];

	m_NumCommands = 1;
	for (i = 0; i < m_Count; i++) {
		const Json::Value RobotParams = Params[static_cast<Json::ArrayIndex>(i)];

		m_MapSet.push_back(GridMap::Zero(m_NumWidth, m_NumHeight));
		m_States.emplace_back();
		m_States[i].s_MeanState << RobotParams["x"].asDouble(), RobotParams["y"].asDouble(), RobotParams["angle"].asDouble();
		m_NumCommands *= MaxCommand;
	}

	double i_Exists, i_NExists;

	m_DetectionThreshold = ControlSystemObject["beacons_threshold"].asDouble();
	m_DeleteBeacon = ControlSystemObject["beacons_delete"].asDouble();
	m_AddBeacon = ControlSystemObject["beacons_add"].asDouble();
	i_Exists = ControlSystemObject["beacons_exists"].asDouble();
	i_NExists = ControlSystemObject["beacons_not_exists"].asDouble();
	m_BeaconExists = log(i_Exists / (1 - i_Exists));
	m_BeaconNotExists = log(i_NExists / (1 - i_NExists));
	m_NumParticles = ControlSystemObject["robot_particles"].asInt();
	m_NumAddParticles = ControlSystemObject["robot_move_particles"].asInt();
	m_NumSimulations = ControlSystemObject["num_simulations"].asInt();
	m_CT = ControlSystemObject["c_t"].asDouble();
	m_Tmin = ControlSystemObject["t_min"].asDouble();
	m_Cp = ControlSystemObject["c_p"].asDouble();
	m_EndC = ControlSystemObject["end_c"].asDouble();
	m_Tree = new STreeNode[m_NumSimulations];
	for (i = 0; i < m_NumSimulations; i++) {
		m_Tree[i].Childs = new STreeNode *[m_NumCommands];
		m_Tree[i].Map = GridMap::Zero(m_NumWidth, m_NumHeight);
	}

	size_t j;

	m_CommandLibrary = new ControlCommand *[m_NumCommands];
	m_CommandLibrary[0] = new ControlCommand[m_Count];
	memset(m_CommandLibrary[0], 0, sizeof(ControlCommand) * m_Count);
	for (i = 1; i < m_NumCommands; i++) {
		m_CommandLibrary[i] = new ControlCommand[m_Count];
		memcpy(m_CommandLibrary[i], m_CommandLibrary[i - 1],
				sizeof(ControlCommand) * m_Count);
		for (j = 0; j < m_Count; j++) {
			m_CommandLibrary[i][j] =
					static_cast<ControlCommand>(m_CommandLibrary[i][j] + 1);
			if (m_CommandLibrary[i][j] == MaxCommand) {
				m_CommandLibrary[i][j] = Nothing;
			} else
				break;
		}
	}
}

vprobot::control::ai::CAIControlSystem::~CAIControlSystem() {
	size_t i;

	for (i = 0; i < m_NumCommands; i++) {
		delete[] m_CommandLibrary[i];
	}
	delete[] m_CommandLibrary;
	for (i = 0; i < m_NumSimulations; i++) {
		delete[] m_Tree[i].Childs;
	}
}

/* Получить команду */
const ControlCommand * const vprobot::control::ai::CAIControlSystem::GetCommands(
		const SMeasures * const *Measurements) {
	size_t i;

	if (m_LastCommand != NULL) { /* TODO FastSLAM для локализации */
		UpdateStates(m_LastCommand, m_States);
	}
	if (Measurements != NULL) { /* TODO FastSLAM для локализации */
		for (i = 0; i < m_Count; i++) {
			const SMeasuresDistances *i_Measurement =
					dynamic_cast<const SMeasuresDistances *>(Measurements[i]);

			if (i_Measurement == NULL)
				continue;

			double da = m_MaxAngle * 2 / i_Measurement->Value.rows(), dx =
					m_MapWidth / m_NumWidth, dy = m_MapHeight / m_NumHeight,
					dd = sqrt(dx * dx + dy * dy);
			size_t x, y;

			for (x = 0; x < m_NumWidth; x++)
				for (y = 0; y < m_NumHeight; y++) {
					double cx = dx * (x + 0.5) + m_StartX
							- m_States[i].s_MeanState[0], cy = dy * (y + 0.5)
							+ m_StartY - m_States[i].s_MeanState[1], nangle;
					int nx;

					if (GreaterThan(abs(cx), m_MaxLength)
							|| GreaterThan(abs(cy), m_MaxLength))
						continue;
					nangle = atan2(cy, cx);
					nx = static_cast<int>((CorrectAngle(
							nangle - m_States[i].s_MeanState[2]) + m_MaxAngle)
							/ da);
					if (nx >= 0 && nx < i_Measurement->Value.rows()) {
						double md, d = i_Measurement->Value[nx], cd = sqrt(
								cx * cx + cy * cy);

						if (EqualsZero(d)) {
							md = 0;
							d = m_MaxLength;
						} else
							md = d + dd;
						if (LessOrEquals(cd, d)) {
							m_MapSet[i].row(x)[y] += m_Free;
						} else if (LessOrEquals(cd, md)) {
							m_MapSet[i].row(x)[y] += m_Occ;
						}
					}

				}
		}
	}
	if (GenerateCommands())
		m_LastCommand = NULL;
	return m_LastCommand;
}

SPresentationParameters *vprobot::control::ai::CAIControlSystem::ParsePresentation(
		const Json::Value &PresentationObject) {
	return new SGridPresentationPrameters(PresentationObject["robot"].asInt());
}

/* Отображаем данные */
void vprobot::control::ai::CAIControlSystem::DrawPresentation(
		const SPresentationParameters *Params, double IndicatorZoom,
		CPresentationDriver &Driver) {
	const SGridPresentationPrameters *i_Params =
			dynamic_cast<const SGridPresentationPrameters *>(Params);
	if (i_Params != NULL) {
		GridMap OutMap;

		if (i_Params->m_Num > 0)
			OutMap = m_MapSet[i_Params->m_Num - 1];
		else {
			OutMap = GridMap::Zero(m_NumWidth, m_NumHeight);
			for (auto m : m_MapSet)
				OutMap += m;
		}

		size_t i, j;
		double dx = m_MapWidth / m_NumWidth, dy = m_MapHeight / m_NumHeight, cx,
				cy;

		for (i = 0, cx = 0; i < m_NumWidth; i++, cx += dx)
			for (j = 0, cy = 0; j < m_NumHeight; j++, cy += dy) {
				double l = exp(OutMap.row(i)[j]);
				int val = 255 - static_cast<int>(255 * l / (1 + l));

				Driver.DrawRectangle(cx, cy, cx + dx, cy + dy, val, val, val,
						255);
			}
	}
}

/* Генерировать команды */
bool vprobot::control::ai::CAIControlSystem::GenerateCommands() {
	size_t n = 1;
	GridMap OutMap = GridMap::Zero(m_NumWidth, m_NumHeight);

	for (auto m : m_MapSet)
		OutMap += m;
	InitializeNode(m_Tree, NULL, OutMap, m_States);
	while (n < m_NumSimulations) {
		AddChild(m_Tree, m_Tree + n, 1);
		n++;
	}

	double diff = abs(m_Tree[0].SelfY - m_Tree[0].BestY);

	if (GreaterThan(m_EndC, diff))
		return true;
	for (n = 0; n < m_NumCommands; n++) {
		if (Equals(m_Tree[0].BestY, m_Tree[0].Childs[n]->BestY)) {
			m_LastCommand = m_CommandLibrary[n];
			return false;
		}
	}
	m_LastCommand = m_CommandLibrary[0];
	return false;
}

/* Инициализация ветви */
void vprobot::control::ai::CAIControlSystem::InitializeNode(STreeNode *Node,
		STreeNode *Parent, const GridMap &Map, const StateSet &States) {
	size_t i;

	Node->Map = Map;
	Node->States = States;
	Node->Parent = Parent;
	Node->n_vis = 0;
	for (i = 0; i < m_NumCommands; i++) {
		Node->Childs[i] = NULL;
	}
}

/* Посчитать значение функционала */
void vprobot::control::ai::CAIControlSystem::UpdateY(STreeNode *Node,
		int Level) {
	double Y = 0;
	size_t i, j;

	for (i = 0; i < m_NumWidth; i++)
		for (j = 0; j < m_NumHeight; j++) {
			Y += 1 / cosh(Node->Map.row(i)[j] * 0.64);
		}
	Node->SelfY = Y;
	Node->BestY = Node->SelfY;
	Node->Q = 1 - m_CT * (Level / m_Tmin + Y / m_NumWidth / m_NumHeight);

	STreeNode *Parent = Node->Parent, *Child = Node;

	while (Parent != NULL) {
		if (Parent->BestY > Child->SelfY)
			Parent->BestY = Child->SelfY;
		if (Parent->Q < Child->Q)
			Parent->Q = Child->Q;
		Parent->n_vis++;
		if (Parent->n_vis >= m_NumCommands) {
			Parent->WeightSum = 0;
			for (i = 0; i < m_NumCommands; i++) {
				Parent->Childs[i]->Weight =
						Parent->Childs[i]->Q
								+ 2 * m_Cp
										* sqrt(
												2 * log(Parent->n_vis + 1)
														/ (Parent->Childs[i]->n_vis
																+ 1));
				Parent->WeightSum += Parent->Childs[i]->Weight;
			}
		}
		Child = Child->Parent;
		Parent = Parent->Parent;
	}
}

/* Добавить ветвь */
void vprobot::control::ai::CAIControlSystem::AddChild(STreeNode *Node,
		STreeNode *FreeNode, int Level) {
	if (Node->n_vis < m_NumCommands) {
		size_t r, i = 0;

		if (Node->n_vis == (m_NumCommands - 1)) {
			r = 1;
		} else
			r = static_cast<size_t>(RandomFunction()
					* (m_NumCommands - Node->n_vis - 1)) + 1;
		for (;;) {
			if (Node->Childs[i] != NULL) {
				i++;
				continue;
			}
			r--;
			if (r == 0)
				break;
			i++;
		}
		Node->Childs[i] = FreeNode;
		InitializeNode(FreeNode, Node, Node->Map, Node->States);
		UpdateStates(m_CommandLibrary[i], FreeNode->States);
		UpdateMap(FreeNode, Node->Map);
		UpdateY(FreeNode, Level);
	} else {
		double w = RandomFunction() * Node->WeightSum;
		size_t i;

		for (i = 0; i < m_NumCommands; i++) {
			w -= Node->Childs[i]->Weight;
			if (LessOrEqualsZero(w)) {
				AddChild(Node->Childs[i], FreeNode, Level + 1);
				break;
			}
		}
	}
}

/* Обновить состояния */
void vprobot::control::ai::CAIControlSystem::UpdateStates(
		const ControlCommand *Commands, StateSet &States) {
	size_t i;
	for (i = 0; i < m_Count; i++) {
		if (Commands[i] == Nothing)
			continue;

		Vector2d u;

		switch (Commands[i]) {
			case Forward:
				u << m_Len, 0;
				break;
			case ForwardLeft:
				u << m_Len, m_Radius;
				break;
			case ForwardRight:
				u << m_Len, -m_Radius;
				break;
			case Backward:
				u << -m_Len, 0;
				break;
			case BackwardLeft:
				u << -m_Len, m_Radius;
				break;
			case BackwardRight:
				u << -m_Len, -m_Radius;
				break;
			default:
				break;
		}
		double nangle = States[i].s_MeanState[2];
		double dx, dy;

		if (EqualsZero(u[1])) {
			dx = u[0] * cos(nangle);
			dy = u[0] * sin(nangle);
		} else {
			nangle = CorrectAngle(nangle + u[0] * u[1]);
			dx = (sin(nangle) - sin(States[i].s_MeanState[2])) / u[1];
			dy = (cos(States[i].s_MeanState[2]) - cos(nangle)) / u[1];
		}
		Vector3d OldMean = States[i].s_MeanState;
		States[i].s_MeanState << OldMean[0] + dx, OldMean[1] + dy, nangle;
	}
}

/* Обновить карту */
void vprobot::control::ai::CAIControlSystem::UpdateMap(STreeNode *Node,
		const GridMap &ParentMap) {

}
