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

using namespace ::std;
using namespace ::vprobot::robot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::control;
using namespace ::vprobot::control::localization;

/* CEKFLocalization */

vprobot::control::localization::CEKFLocalization::CEKFLocalization(
		const Json::Value &ControlSystemObject) :
		CSequentialControlSystem(ControlSystemObject), m_States() {
	m_Radius = ControlSystemObject["radius"].asDouble();
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

}

/* Получить команду */
const ControlCommand * const vprobot::control::localization::CEKFLocalization::GetCommands(
		const SMeasures * const *Measurements) {
	Process(Measurements);
	return CSequentialControlSystem::GetCommands(Measurements);
}
