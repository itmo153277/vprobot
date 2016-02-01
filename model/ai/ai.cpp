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
	m_Time = 0;
	m_Tree = new STreeNode[m_NumSimulations];
	for (i = 0; i < m_NumSimulations; i++) {
		m_Tree[i].Childs = new STreeNode *[m_NumCommands];
		m_Tree[i].Fouls = new bool[m_NumCommands];
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
		delete[] m_Tree[i].Fouls;
	}
	delete[] m_Tree;
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
						} else {
							md = d + dd;
						}
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

		for (i = 0, cx = m_StartX; i < m_NumWidth; i++, cx += dx)
			for (j = 0, cy = m_StartY; j < m_NumHeight; j++, cy += dy) {
				double l = exp(OutMap.row(i)[j]);
				int val = 255 - static_cast<int>(255 * l / (1 + l));

				Driver.DrawRectangle(cx, cy, cx + dx, cy + dy, val, val, val,
						255);
			}
		/*if (m_LastCommand != NULL) {
		 for (i = 0; i < m_Count; i++) {
		 Driver.DrawLine(m_Tree->States[i].s_MeanState[0],
		 m_Tree->States[i].s_MeanState[1],
		 m_Tree->States[i].s_MeanState[0]
		 + cos(m_Tree->States[i].s_MeanState[2]) * 0.8
		 * IndicatorZoom,
		 m_Tree->States[i].s_MeanState[1]
		 + sin(m_Tree->States[i].s_MeanState[2]) * 0.8
		 * IndicatorZoom, 0, 0, 255, 255);
		 Driver.DrawCircle(m_Tree->States[i].s_MeanState[0],
		 m_Tree->States[i].s_MeanState[1], 0.3 * IndicatorZoom,
		 255, 0, 0, 255);
		 }
		 }*/
	}
}

/* Генерировать команды */
bool vprobot::control::ai::CAIControlSystem::GenerateCommands() {
	size_t n = 1;
	GridMap OutMap = GridMap::Zero(m_NumWidth, m_NumHeight);

	for (auto m : m_MapSet)
		OutMap += m;
	InitializeNode(m_Tree, NULL, OutMap, m_States);
	UpdateY(m_Tree, m_Time);
	while (n < m_NumSimulations) {
		AddChild(m_Tree, m_Tree + n, m_Time + 1);
		n++;
	}
	m_Time++;

	/* Debug output */
	cout << m_Tree[0].SelfY << endl << m_Tree[0].BestY << endl << m_Tree[0].Q
			<< endl << m_Tree[0].EndPoint << endl;

	double diff = abs(m_Tree[0].SelfY - m_Tree[0].BestY);

	if (GreaterThan(m_EndC, diff))
		return true;
	for (n = 0; n < m_NumCommands; n++) {
		if (m_Tree[0].Fouls[n] || m_Tree[0].Childs[n] == NULL)
			continue;
		if (Equals(m_Tree[0].BestY, m_Tree[0].Childs[n]->BestY)) {
			m_LastCommand = m_CommandLibrary[n];
			/* Debug output */
			cout << n << endl;
			return false;
		}
	}
	m_LastCommand = m_CommandLibrary[0];
	/* Debug output */
	cout << 0 << endl;
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
	Node->n_foul = 0;
	for (i = 0; i < m_NumCommands; i++) {
		Node->Childs[i] = NULL;
		Node->Fouls[i] = false;
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
	Node->EndPoint = Level;

	STreeNode *Parent = Node->Parent, *Child = Node;

	while (Parent != NULL) {
		if (GreaterOrEquals(Parent->BestY, Child->BestY)) {
			Parent->BestY = Child->BestY;
			Parent->EndPoint = Child->EndPoint;
		}
		Parent->n_vis++;
		Parent->Q = 0;
		Parent->WeightSum = 0;
		for (i = 0; i < m_NumCommands; i++) {
			if (Parent->Fouls[i] || Parent->Childs[i] == NULL)
				continue;
			if (LessThan(Parent->Q, Parent->Childs[i]->Q)) {
				Parent->Q = Parent->Childs[i]->Q;
			}
			Parent->Childs[i]->Weight = Parent->Childs[i]->Q
					+ 2 * m_Cp
							* sqrt(
									2 * log(Parent->n_vis + 1)
											/ (Parent->Childs[i]->n_vis + 1));
			Parent->WeightSum += Parent->Childs[i]->Weight;
		}
		Child = Child->Parent;
		Parent = Parent->Parent;
	}
}

/* Добавить ветвь */
void vprobot::control::ai::CAIControlSystem::AddChild(STreeNode *Node,
		STreeNode *FreeNode, int Level) {
	if ((Node->n_vis + Node->n_foul) < m_NumCommands) {
		size_t r, i = 0;

		if ((Node->n_vis + Node->n_foul) == (m_NumCommands - 1)) {
			r = 1;
		} else
			r = static_cast<size_t>(RandomFunction()
					* (m_NumCommands - Node->n_vis - Node->n_foul - 1)) + 1;
		for (;;) {
			if (Node->Childs[i] != NULL || Node->Fouls[i]) {
				i++;
				continue;
			}
			r--;
			if (r == 0)
				break;
			i++;
		}
		StateSet i_States = Node->States;

		UpdateStates(m_CommandLibrary[i], i_States);
		if (i != 0 && CheckForFoul(Node, i_States)) {
			Node->n_foul++;
			Node->Fouls[i] = true;
			AddChild(Node, FreeNode, Level);
		} else {
			Node->Childs[i] = FreeNode;
			InitializeNode(FreeNode, Node, Node->Map, i_States);
			UpdateMap(FreeNode);
			UpdateY(FreeNode, Level);
		}
	} else {
		double w = RandomFunction() * Node->WeightSum;
		size_t i;

		for (i = 0; i < m_NumCommands; i++) {
			if (Node->Fouls[i])
				continue;
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

/* Проверить на фол */
bool vprobot::control::ai::CAIControlSystem::CheckForFoul(STreeNode *Node,
		const StateSet &States) {
	size_t i;

	for (i = 0; i < m_Count; i++) {
		int rx, ry;

		rx = static_cast<int>((States[i].s_MeanState[0] - m_StartX) / m_MapWidth
				* m_NumWidth);
		ry = static_cast<int>((States[i].s_MeanState[1] - m_StartY)
				/ m_MapHeight * m_NumHeight);
		if (rx < 0 || ry < 0 || rx >= static_cast<int>(m_NumWidth)
				|| ry >= static_cast<int>(m_NumHeight)
				|| !LessThanZero(Node->Map.row(rx)[ry]))
			return true;
	}
	return false;
}

/* Обновить карту */
void vprobot::control::ai::CAIControlSystem::UpdateMap(STreeNode *Node) {
	size_t i, x, y;
	double dx = m_MapWidth / m_NumWidth, dy = m_MapHeight / m_NumHeight;
	GridMap odMap = GridMap::Zero(m_NumWidth, m_NumHeight);

	for (i = 0; i < m_Count; i++) {
		GridMap dMap = GridMap::Zero(m_NumWidth, m_NumHeight);
		int mx_a, mx_b, my_a, my_b;
		double rx, ry;

		mx_a = static_cast<int>((Node->States[i].s_MeanState[0] - m_MaxLength
				- m_StartX) / dx - 0.5);
		mx_b = static_cast<int>((Node->States[i].s_MeanState[0] + m_MaxLength
				- m_StartX) / dx - 0.5);
		my_a = static_cast<int>((Node->States[i].s_MeanState[1] - m_MaxLength
				- m_StartY) / dy - 0.5);
		my_b = static_cast<int>((Node->States[i].s_MeanState[1] + m_MaxLength
				- m_StartY) / dy - 0.5);
		if (mx_a < 0)
			mx_a = 0;
		if (my_a < 0)
			my_a = 0;
		if (mx_b >= static_cast<int>(m_NumWidth))
			mx_b = static_cast<int>(m_NumWidth - 1);
		if (my_b >= static_cast<int>(m_NumHeight))
			my_b = static_cast<int>(m_NumHeight - 1);
		for (x = static_cast<size_t>(mx_a); x <= static_cast<size_t>(mx_b); x++)
			for (y = static_cast<size_t>(my_a); y <= static_cast<size_t>(my_b);
					y++) {
				if (!EqualsZero(dMap.row(x)[y]))
					continue;
				rx = dx * (x + 0.5) + m_StartX;
				ry = dy * (y + 0.5) + m_StartY;

				double cx = rx - Node->States[i].s_MeanState[0], cy = ry
						- Node->States[i].s_MeanState[1], nangle;

				nangle = CorrectAngle(
						atan2(cy, cx) - Node->States[i].s_MeanState[2]);
				if (GreaterThan(abs(nangle), m_MaxAngle)
						|| GreaterThan(cx * cx + cy * cy,
								m_MaxLength * m_MaxLength))
					continue;

				int tx = LessThanZero(cx) ? -1 : 1, ty =
						LessThanZero(cy) ? -1 : 1, ox = static_cast<int>(abs(cx)
						/ dx), oy = static_cast<int>(abs(cy) / dy), k;
				size_t ft = 0;

				if ((static_cast<int>(x) - ox * tx)
						>= static_cast<int>(m_NumWidth))
					ox = (m_NumWidth - x - 1) * tx;
				if ((static_cast<int>(y) - oy * ty)
						>= static_cast<int>(m_NumHeight))
					oy = (m_NumHeight - y - 1) * ty;
				if (ox > oy) {
					double t = abs(cy / cx) * dx;
					for (k = ox; k >= 0; k--) {
						size_t fx = x - k * tx;
						double ny = ry - k * t * ty;
						size_t fy = static_cast<size_t>((ny - m_StartY) / dy);

						if (Node->Map.row(fx)[fy] > 0) {
							dMap.row(fx)[fy] = m_Occ;
							break;
						} else {
							if (k < ox && ft != fy) {
								if (Node->Map.row(fx)[ft] > 0)
									break;
								if (Node->Map.row(fx - tx)[fy] > 0)
									break;
							}
							ft = fy;
							dMap.row(fx)[fy] = m_Free;
						}
					}
				} else {
					double t = abs(cx / cy) * dy;
					for (k = oy; k >= 0; k--) {
						size_t fy = y - k * ty;
						double nx = rx - k * t * tx;
						size_t fx = static_cast<size_t>((nx - m_StartX) / dx);

						if (Node->Map.row(fx)[fy] > 0) {
							dMap.row(fx)[fy] = m_Occ;
							break;
						} else {
							if (k < oy && ft != fx) {
								if (Node->Map.row(ft)[fy] > 0)
									break;
								if (Node->Map.row(fx)[fy - ty] > 0)
									break;
							}
							ft = fx;
							dMap.row(fx)[fy] = m_Free;
						}
					}
				}
			}
		odMap += dMap;
	}
	Node->Map += odMap;
}
