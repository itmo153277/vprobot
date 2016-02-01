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

#include "mcts-ai.h"

#include <cmath>
#include <cstring>

#include "../../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::control::mcts_ai;

/* CMCTSAI */

vprobot::control::mcts_ai::CMCTSAI::CMCTSAI(
		const Json::Value &ControlSystemObject) :
		CControlSystem(ControlSystemObject), m_Generator(), m_States(), m_Distribution(
				0, 1) {
	random_device rd;

	m_Generator.seed(rd());
	RandomFunction = [&] {return m_Distribution(m_Generator);};

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
	m_Map = GridMap::Zero(m_NumWidth, m_NumHeight);
	m_StartX = ControlSystemObject["start_x"].asDouble();
	m_StartY = ControlSystemObject["start_y"].asDouble();

	size_t i;
	const Json::Value Params = ControlSystemObject["robot_params"];

	m_NumCommands = 1;
	for (i = 0; i < m_Count; i++) {
		const Json::Value RobotParams = Params[static_cast<Json::ArrayIndex>(i)];

		m_States.emplace_back();
		m_States[i].s_MeanState << RobotParams["x"].asDouble(), RobotParams["y"].asDouble(), RobotParams["angle"].asDouble();
		m_NumCommands *= MaxCommand;
	}

	m_EndC = ControlSystemObject["end_c"].asDouble();
	m_AddMoves = ControlSystemObject["add_moves"].asInt();

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

vprobot::control::mcts_ai::CMCTSAI::~CMCTSAI() {
	size_t i;

	for (i = 0; i < m_NumCommands; i++) {
		delete[] m_CommandLibrary[i];
	}
	delete[] m_CommandLibrary;
}

SPresentationParameters *vprobot::control::mcts_ai::CMCTSAI::ParsePresentation(
		const Json::Value &PresentationObject) {
	return new SGridPresentationPrameters(PresentationObject["robot"].asInt());
}

void vprobot::control::mcts_ai::CMCTSAI::DrawPresentation(
		const SPresentationParameters *Params, double IndicatorZoom,
		CPresentationDriver &Driver) {
	const SGridPresentationPrameters *i_Params =
			dynamic_cast<const SGridPresentationPrameters *>(Params);
	if (i_Params != NULL) {
		size_t i, j;
		double dx = m_MapWidth / m_NumWidth, dy = m_MapHeight / m_NumHeight, cx,
				cy;

		for (i = 0, cx = m_StartX; i < m_NumWidth; i++, cx += dx)
			for (j = 0, cy = m_StartY; j < m_NumHeight; j++, cy += dy) {
				double l = exp(m_Map.row(i)[j]);
				int val = 255 - static_cast<int>(255 * l / (1 + l));

				Driver.DrawRectangle(cx, cy, cx + dx, cy + dy, val, val, val,
						255);
			}
	}
}

const ControlCommand * const vprobot::control::mcts_ai::CMCTSAI::GetCommands(
		const SMeasures * const *Measurements) {
	size_t i;

	if (m_LastCommand != NULL) {
		UpdateStates(m_LastCommand, m_States);
	}
	if (Measurements != NULL) {
		for (i = 0; i < m_Count; i++) {
			const SMeasuresDistances *i_Measurement =
					dynamic_cast<const SMeasuresDistances *>(Measurements[i]);

			if (i_Measurement == NULL)
				continue;

			double da = m_MaxAngle * 2 / i_Measurement->Value.rows(), dx =
					m_MapWidth / m_NumWidth, dy = m_MapHeight / m_NumHeight,
					dd = sqrt(dx * dx + dy * dy);
			size_t x, y;

			for (x = 0; x < m_NumWidth; x++) {
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
							m_Map.row(x)[y] += m_Free;
						} else if (LessOrEquals(cd, md)) {
							m_Map.row(x)[y] += m_Occ;
						}
					}

				}
			}
		}
	}
	if (GenerateCommands())
		m_LastCommand = NULL;
	return m_LastCommand;
}

void vprobot::control::mcts_ai::CMCTSAI::UpdateStates(
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

bool vprobot::control::mcts_ai::CMCTSAI::CheckForFoul(const BinaryMap &Map,
		const StateSet &States) {
	size_t i;

	for (i = 0; i < m_Count; i++) {
		int rx, ry;

		rx = ConvertX(States[i].s_MeanState[0]);
		ry = ConvertX(States[i].s_MeanState[1]);
		if (rx < 0 || ry < 0 || rx >= static_cast<int>(m_NumWidth)
				|| ry >= static_cast<int>(m_NumHeight) || !Map.row(rx)[ry])
			return true;
	}
	return false;
}

bool vprobot::control::mcts_ai::CMCTSAI::GenerateCommands() {
	double deltaY = 0;
	double Y = 0;
	size_t i, j;

	for (i = 0; i < m_NumWidth; i++) {
		for (j = 0; j < m_NumHeight; j++) {
			Y += 1 / cosh(m_Map.row(i)[j] * 0.64);
		}
	}

	SSample Sample(0);

	for (i = 0; i < 10; i++) {
		GenerateSample(Sample, m_Map, m_States);
	}
	cout << "Mean time: " << Sample.Time / 10.0 << endl;
	cout << "Mean Y: " << Sample.Y / 10.0 + Y << endl;

	if (GreaterThan(m_EndC, deltaY))
		return true;
	return false;
}

int vprobot::control::mcts_ai::CMCTSAI::ConvertX(double x) {
	return static_cast<int>((x - m_StartX) / m_MapWidth * m_NumWidth);
}

int vprobot::control::mcts_ai::CMCTSAI::ConvertY(double y) {
	return static_cast<int>((y - m_StartY) / m_MapHeight * m_NumHeight);
}

double vprobot::control::mcts_ai::CMCTSAI::GoAround(GridMap &Map,
		const BinaryMap &GeneratedMap, const StateSet &States) {
	double deltaY = 0;
	size_t i;

	for (i = 0; i < m_Count; i++) {
		BinaryMap VisitedMap = BinaryMap::Zero(m_NumWidth, m_NumHeight);
		double drx = States[i].s_MeanState[0], dry = States[i].s_MeanState[1],
				angle = States[i].s_MeanState[2], cx = (drx - m_StartX)
						/ m_MapWidth * m_NumWidth, cy = (dry - m_StartY)
						/ m_MapHeight * m_NumHeight;
		int rx = ConvertX(drx);
		int ry = ConvertX(dry);
		int x = max(
				max(ConvertX(drx + m_MaxLength) - rx,
						rx - ConvertX(drx - m_MaxLength)),
				max(ConvertY(dry + m_MaxLength) - ry,
						ry - ConvertY(dry - m_MaxLength)));
		int y = 0;
		int decisionOver2 = 1 - x;

		while (y <= x) {
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					x + rx + 0.5, y + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					y + rx + 0.5, x + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					-x + rx + 0.5, y + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					-y + rx + 0.5, x + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					x + rx + 0.5, -y + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					y + rx + 0.5, -x + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					-x + rx + 0.5, -y + ry + 0.5, angle);
			deltaY += GoLinear(Map, GeneratedMap, VisitedMap, cx, cy,
					-y + rx + 0.5, -x + ry + 0.5, angle);
			y++;
			if (decisionOver2 <= 0) {
				decisionOver2 += 2 * y + 1;
			} else {
				x--;
				decisionOver2 += 2 * (y - x) + 1;
			}
		}
	}
	return deltaY;
}

double vprobot::control::mcts_ai::CMCTSAI::GoLinear(GridMap &Map,
		const BinaryMap &GeneratedMap, BinaryMap &VisitedMap, double x0,
		double y0, double xf, double yf, double angle) {
	double cy = yf - y0, cx = xf - x0;

	if (GreaterThan(abs(CorrectAngle(atan2(cy, cx) - angle)), m_MaxAngle))
		return 0;

	double deltaY = 0;
	double diff = 0;
	double dd;
	int tx = cx < 0 ? -1 : 1, ty = cy < 0 ? -1 : 1, rx = static_cast<int>(x0),
			ry = static_cast<int>(y0), fx = static_cast<int>(xf), fy =
					static_cast<int>(yf), x, y, px = rx, py = ry;

	if (GreaterThan(abs(cx), abs(cy))) {
		dd = abs(cy / cx);
		y = ry;
		for (x = rx; x != fx; px = x, x += tx) {
			if (!GoExact(Map, GeneratedMap, VisitedMap, x, y, px, py, deltaY))
				break;
			diff += dd;
			bool endFlag = false;
			while (diff > 0.5) {
				dd -= 1;
				py = y;
				y += ty;
				endFlag = GoExact(Map, GeneratedMap, VisitedMap, x, y, px, py,
						deltaY);
				if (endFlag)
					break;
			}
			if (endFlag)
				break;
		}
	} else {
		dd = abs(cx / cy);
		x = rx;
		for (y = ry; y != fy; py = y, y += ty) {
			if (!GoExact(Map, GeneratedMap, VisitedMap, x, y, px, py, deltaY))
				break;
			diff += dd;
			bool endFlag = false;
			while (diff > 0.5) {
				dd -= 1;
				px = x;
				x += tx;
				endFlag = GoExact(Map, GeneratedMap, VisitedMap, x, y, px, py,
						deltaY);
				if (endFlag)
					break;
			}
			if (endFlag)
				break;
		}
	}
	return deltaY;
}

bool vprobot::control::mcts_ai::CMCTSAI::GoExact(GridMap &Map,
		const BinaryMap &GeneratedMap, BinaryMap &VisitedMap, int x, int y,
		int px, int py, double &CurY) {
	if (x < 0 || x >= m_NumWidth || y < 0 || y >= m_NumHeight)
		return false;
	if (VisitedMap.row(x)[y])
		return true;
	VisitedMap.row(x)[y] = 1;

	double oldP = Map.row(x)[y];
	bool endFlag;

	if (GeneratedMap.row(x)[y]) {
		Map.row(x)[y] += m_Occ;
		endFlag = true;
	} else {
		Map.row(x)[y] += m_Free;
		endFlag = false;
		if (px != x) {
			if (GeneratedMap.row(px)[y]) {
				endFlag = true;
			} else if (py != y) {
				if (GeneratedMap.row(px)[py] || GeneratedMap.row(x)[py]) {
					endFlag = true;
				}
			}
		} else if (py != y) {
			if (GeneratedMap.row(x)[py]) {
				endFlag = true;
			}
		}
	}
	CurY += 1 / cosh(Map.row(x)[y] * 0.64) - 1 / cosh(oldP * 0.64);
	return endFlag;
}

/* Сгенерировать семпл */
void vprobot::control::mcts_ai::CMCTSAI::GenerateSample(SSample &Sample,
		const GridMap &Map, const StateSet &States) {
	StateSet TempStates = States;
	GridMap TempMap = Map;
	BinaryMap GeneratedMap(m_NumWidth, m_NumHeight);
	size_t i, j;
	double diff;

	for (i = 0; i < m_NumWidth; i++) {
		for (j = 0; j < m_NumHeight; j++) {
			double t = RandomFunction();

			t = log(t / (1 - t));
			GeneratedMap.row(i)[j] = GreaterOrEquals(t, Map.row(i)[j]) ? 1 : 0;
		}
	}
	i = m_AddMoves;
	for (;;) {
		const ControlCommand *cmd = m_CommandLibrary[0];
		int cmdNum = static_cast<int>(RandomFunction() * (m_NumCommands - 1)),
				cmdRand = cmdNum;
		for (;;) {
			cmdRand++;
			if (cmdRand == m_NumCommands) {
				cmdRand = 0;
			}
			if (cmdRand == cmdNum)
				break;
			StateSet CheckStates = TempStates;
			UpdateStates(m_CommandLibrary[cmdRand], CheckStates);
			if (!CheckForFoul(GeneratedMap, CheckStates)) {
				cmd = m_CommandLibrary[cmdRand];
				break;
			}
		}
		UpdateStates(cmd, TempStates);
		diff = GoAround(TempMap, GeneratedMap, States);
		Sample.Time++;
		Sample.Y += diff;
		if (GreaterThan(m_EndC, abs(diff))) {
			i--;
			if (i == 0)
				break;
		} else {
			i = m_AddMoves;
		}
	}
}
