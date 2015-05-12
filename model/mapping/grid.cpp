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

#include "grid.h"

#include <cmath>

#include "../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;
using namespace ::vprobot::control::mapping;

/* CGridMapper */

vprobot::control::mapping::CGridMapper::CGridMapper(
		const Json::Value &ControlSystemObject) :
		CSequentialControlSystem(ControlSystemObject), m_MapSet(), m_States() {
	double i_Occ, i_Free;

	m_Radius = 1 / ControlSystemObject["radius"].asDouble();
	m_Len = ControlSystemObject["len"].asDouble();
	m_MaxAngle = ControlSystemObject["max_angle"].asDouble();
	m_MaxLength = ControlSystemObject["max_length"].asDouble();
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

	for (i = 0; i < m_Count; i++) {
		const Json::Value RobotParams = Params[static_cast<Json::ArrayIndex>(i)];

		m_MapSet.push_back(GridMap::Zero(m_NumWidth, m_NumHeight));
		m_States.emplace_back();
		m_States[i].s_MeanState << RobotParams["x"].asDouble(), RobotParams["y"].asDouble(), RobotParams["angle"].asDouble();
	}
}

vprobot::control::mapping::CGridMapper::~CGridMapper() {
}

/* Получить команду */
const ControlCommand * const vprobot::control::mapping::CGridMapper::GetCommands(
		const SMeasures * const *Measurements) {
	size_t i;

	if (m_LastCommand != NULL) {
		for (i = 0; i < m_Count; i++) {
			if (m_LastCommand[i] == Nothing)
				continue;

			Vector2d u;

			switch (m_LastCommand[i]) {
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
			double nangle = m_States[i].s_MeanState[2];
			double dx, dy;

			if (EqualsZero(u[1])) {
				dx = u[0] * cos(nangle);
				dy = u[0] * sin(nangle);
			} else {
				nangle = CorrectAngle(nangle + u[0] * u[1]);
				dx = (sin(nangle) - sin(m_States[i].s_MeanState[2])) / u[1];
				dy = (cos(m_States[i].s_MeanState[2]) - cos(nangle)) / u[1];
			}
			Vector3d OldMean = m_States[i].s_MeanState;
			m_States[i].s_MeanState << OldMean[0] + dx, OldMean[1] + dy, nangle;
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
				size_t x, y, j;

				for (x = 0; x < m_NumWidth; x++)
					for (y = 0; y < m_NumHeight; y++) {
						double cx = dx * (x + 0.5) + m_StartX
								- m_States[i].s_MeanState[0], cy = dy
								* (y + 0.5) + m_StartY
								- m_States[i].s_MeanState[1], nangle;
						int nx;

						nangle = atan2(cy, cx);
						nx = static_cast<int>((CorrectAngle(
								nangle - m_States[i].s_MeanState[2])
								+ m_MaxAngle) / da);
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
	}
	return CSequentialControlSystem::GetCommands(Measurements);
}

SPresentationParameters *vprobot::control::mapping::CGridMapper::ParsePresentation(
		const Json::Value &PresentationObject) {
	return new SGridPresentationPrameters(PresentationObject["robot"].asInt());
}

/* Отображаем данные */
void vprobot::control::mapping::CGridMapper::DrawPresentation(
		const SPresentationParameters *Params, CPresentationDriver &Driver) {
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
