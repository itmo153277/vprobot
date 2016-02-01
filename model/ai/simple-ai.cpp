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

#include "simple-ai.h"

#include <cmath>
#include <cstring>

#include "../../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::control::simple_ai;

/* CSimpleAI */

vprobot::control::simple_ai::CSimpleAI::CSimpleAI(
		const Json::Value &ControlSystemObject) :
		CControlSystem(ControlSystemObject), m_States() {
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

vprobot::control::simple_ai::CSimpleAI::~CSimpleAI() {
	size_t i;

	for (i = 0; i < m_NumCommands; i++) {
		delete[] m_CommandLibrary[i];
	}
	delete[] m_CommandLibrary;
}

SPresentationParameters *vprobot::control::simple_ai::CSimpleAI::ParsePresentation(
		const Json::Value &PresentationObject) {
	return new SGridPresentationPrameters(PresentationObject["robot"].asInt());
}

void vprobot::control::simple_ai::CSimpleAI::DrawPresentation(
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

const ControlCommand * const vprobot::control::simple_ai::CSimpleAI::GetCommands(
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
							m_Map.row(x)[y] += m_Free;
						} else if (LessOrEquals(cd, md)) {
							m_Map.row(x)[y] += m_Occ;
						}
					}

				}
		}
	}
	if (GenerateCommands())
		m_LastCommand = NULL;
	return m_LastCommand;
}

void vprobot::control::simple_ai::CSimpleAI::UpdateStates(
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

bool vprobot::control::simple_ai::CSimpleAI::GenerateCommands() {
	size_t i, x, y;
	GridMap TempMap;
	StateSet TempStates;
	double BestY;
	double CurY;
	m_LastCommand = NULL;
	for (i = 0; i < m_NumCommands; i++) {
		TempMap = m_Map;
		TempStates = m_States;
		UpdateStates(m_CommandLibrary[i], TempStates);
		if (CheckForFoul(TempMap, TempStates))
			continue;
		UpdateMap(TempMap, TempStates);
		CurY = 0;
		for (x = 0; x < m_NumWidth; x++) {
			for (y = 0; y < m_NumHeight; y++) {
				CurY += 1 / cosh(TempMap.row(x)[y] * 0.64);
			}
		}
		if (m_LastCommand == NULL || LessThan(CurY, BestY)) {
			BestY = CurY;
			m_LastCommand = m_CommandLibrary[i];
		}
	}
	CurY = 0;
	for (x = 0; x < m_NumWidth; x++) {
		for (y = 0; y < m_NumHeight; y++) {
			CurY += 1 / cosh(m_Map.row(x)[y] * 0.64);
		}
	}
	if (m_LastCommand == NULL) {
		m_LastCommand = m_CommandLibrary[0];
		BestY = CurY;
	}

	cout << BestY << ' ' << CurY << endl;

	double diff = abs(CurY - BestY);

	if (GreaterThan(m_EndC, diff))
		return true;
	return false;
}

bool vprobot::control::simple_ai::CSimpleAI::CheckForFoul(const GridMap &Map,
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
				|| !LessThanZero(Map.row(rx)[ry]))
			return true;
	}
	return false;
}

void vprobot::control::simple_ai::CSimpleAI::UpdateMap(GridMap &Map,
		const StateSet &States) {
	size_t i, x, y;
	double dx = m_MapWidth / m_NumWidth, dy = m_MapHeight / m_NumHeight;
	GridMap odMap = GridMap::Zero(m_NumWidth, m_NumHeight);

	for (i = 0; i < m_Count; i++) {
		GridMap dMap = GridMap::Zero(m_NumWidth, m_NumHeight);
		int mx_a, mx_b, my_a, my_b;
		double rx, ry;

		mx_a = static_cast<int>((States[i].s_MeanState[0] - m_MaxLength
				- m_StartX) / dx - 0.5);
		mx_b = static_cast<int>((States[i].s_MeanState[0] + m_MaxLength
				- m_StartX) / dx - 0.5);
		my_a = static_cast<int>((States[i].s_MeanState[1] - m_MaxLength
				- m_StartY) / dy - 0.5);
		my_b = static_cast<int>((States[i].s_MeanState[1] + m_MaxLength
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

				double cx = rx - States[i].s_MeanState[0], cy = ry
						- States[i].s_MeanState[1], nangle;

				nangle = CorrectAngle(atan2(cy, cx) - States[i].s_MeanState[2]);
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

						if (Map.row(fx)[fy] > 0) {
							dMap.row(fx)[fy] = m_Occ;
							break;
						} else {
							if (k < ox && ft != fy) {
								if (Map.row(fx)[ft] > 0)
									break;
								if (Map.row(fx - tx)[fy] > 0)
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

						if (Map.row(fx)[fy] > 0) {
							dMap.row(fx)[fy] = m_Occ;
							break;
						} else {
							if (k < oy && ft != fx) {
								if (Map.row(ft)[fy] > 0)
									break;
								if (Map.row(fx)[fy - ty] > 0)
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
	Map += odMap;
}
