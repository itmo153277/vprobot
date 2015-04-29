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

#include "ekf.h"

#include <cstddef>
#include <cmath>

#include "../../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::robot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::control;
using namespace ::vprobot::control::localization;

/* CEKFLocalization */

vprobot::control::localization::CEKFLocalization::CEKFLocalization(
		const Json::Value &ControlSystemObject) :
		CSequentialControlSystem(ControlSystemObject), m_States() {
	m_Radius = 1 / ControlSystemObject["radius"].asDouble();
	m_DRadius = ControlSystemObject["dradius"].asDouble();
	m_Len = ControlSystemObject["len"].asDouble();
	m_DLen = ControlSystemObject["dlen"].asDouble();
	m_DAngle = ControlSystemObject["dangle"].asDouble();
	m_DDist = ControlSystemObject["ddist"].asDouble();

	Json::ArrayIndex i;
	const Json::Value Params = ControlSystemObject["robot_params"];

	for (i = 0; i < Params.size(); i++) {
		StateSet::reverse_iterator ri;
		const Json::Value i_Param = Params[i];

		m_States.emplace_back();
		ri = m_States.rbegin();
		ri->s_MeanState << i_Param["x"].asDouble(), i_Param["y"].asDouble(), i_Param["angle"].asDouble();
		ri->s_CovState << pow(i_Param["dx"].asDouble(), 2), 0, 0, 0, pow(
				i_Param["dy"].asDouble(), 2), 0, 0, 0, pow(
				i_Param["dangle"].asDouble(), 2);
	}
}

vprobot::control::localization::CEKFLocalization::~CEKFLocalization() {
}

/* Отображаем данные */
void vprobot::control::localization::CEKFLocalization::DrawPresentation(
		const SPresentationParameters *Params, CPresentationDriver &Driver) {
	Driver.PutText(2, 2, "Localization data here", 0, 0, 0, 255);
}

/* Обработка */
void vprobot::control::localization::CEKFLocalization::Process(
		const SMeasures * const *Measurements) {
	if (m_LastCommand != NULL) {
		size_t i;
		for (i = 0; i < m_Count; i++) {
			if (m_LastCommand[i] == Nothing)
				continue;

			Vector2d u;
			Matrix2d CovControl;
			Matrix3d G;
			Matrix<double, 3, 2> V;

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
				V << cos(nangle), -u[0] * dy / 2, sin(nangle), u[0] * dx / 2, 0, u[0];
			} else {
				nangle = CorrectAngle(nangle + u[0] * u[1]);
				dx = (sin(nangle) - sin(m_States[i].s_MeanState[2])) / u[1];
				dy = (cos(m_States[i].s_MeanState[2]) - cos(nangle)) / u[1];
				V << cos(nangle), (u[0] * cos(nangle) - dx) / u[1], sin(nangle), (u[0]
						* sin(nangle) - dy) / u[1], u[1], u[0];
			}
			G << 1, 0, -dy, 0, 1, dx, 0, 0, 1;
			CovControl << m_DLen * m_DLen, 0, 0, m_DRadius * m_DRadius;
			m_States[i].s_CovState = G * m_States[i].s_CovState * G.transpose()
					+ V * CovControl * V.transpose();
			m_States[i].s_MeanState << m_States[i].s_MeanState[0] + dx, m_States[i].s_MeanState[1]
					+ dy, nangle;
		}
	}
}

/* Получить команду */
const ControlCommand * const vprobot::control::localization::CEKFLocalization::GetCommands(
		const SMeasures * const *Measurements) {
	Process(Measurements);
	return CSequentialControlSystem::GetCommands(Measurements);
}
