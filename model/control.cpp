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

#include "control.h"

#include <string>

using namespace ::std;
using namespace ::vprobot::robot;
using namespace ::vprobot::control;

/* CControlSystem */

vprobot::control::CControlSystem::CControlSystem(
		const Json::Value &ControlSystemObject) :
		m_LastCommand(NULL) {
	m_Count = ControlSystemObject["count"].asInt();
}

vprobot::control::CControlSystem::~CControlSystem() {
}

/* CSequentialControlSystem */

vprobot::control::CSequentialControlSystem::CSequentialControlSystem(
		const Json::Value &ControlSystemObject) :
		CControlSystem(ControlSystemObject), m_Set() {
	Json::ArrayIndex i;
	const Json::Value Commands = ControlSystemObject["commands"];

	for (i = 0; i < Commands.size(); i++) {
		ControlCommand *cmd = new ControlCommand[m_Count];
		Json::ArrayIndex j;

		for (j = 0; j < m_Count; j++) {
			string CommandName(Commands[i][j].asString());
			static const char Aliases[][7] = {"N", "F", "FL", "FR", "B", "BL",
					"BR"};
			static const ControlCommand AliasValues[7] = {Nothing, Forward,
					ForwardLeft, ForwardRight, Backward, BackwardLeft,
					BackwardRight};
			int k;

			for (k = 0; k < 7; k++) {
				if (CommandName != Aliases[k])
					continue;
				cmd[j] = AliasValues[k];
				break;
			}
		}
		m_Set.push_back(cmd);
	}
	m_Pos = m_Set.begin();
}

vprobot::control::CSequentialControlSystem::~CSequentialControlSystem() {
	for (auto c : m_Set) {
		delete[] c;
	}
}

const ControlCommand * const vprobot::control::CSequentialControlSystem::GetCommands(
		const vprobot::robot::SMeasures * const *Measurements) {
	if (m_Pos == m_Set.end())
		m_LastCommand = NULL;
	else {
		m_LastCommand = *m_Pos;
		m_Pos++;
	}
	return m_LastCommand;
}
