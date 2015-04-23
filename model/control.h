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

#ifndef __CONTROL_H_
#define __CONTROL_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstddef>
#include <vector>
#include <json/json.h>
#include "robot.h"

namespace vprobot {

namespace control {

/* Базовый класс системы управления */
class CControlSystem {
private:
	CControlSystem(const CControlSystem &ControlSystem) = default;
public:
	CControlSystem() = default;
	virtual ~CControlSystem() = default;

	/* Получить команду */
	virtual const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements) = 0;
};

/* Система управления, выполняющая заданную последовательность */
class CSequentialControlSystem: public CControlSystem {
private:
	typedef std::vector<vprobot::robot::ControlCommand *> CommandSet;

	/* Количество роботов */
	std::size_t m_Count;
	/* Набор команд */
	CommandSet m_Set;
	/* Текущая позиция в наборе */
	CommandSet::iterator m_Pos;
	CSequentialControlSystem(const CSequentialControlSystem &ControlSystem) = default;
public:
	CSequentialControlSystem(const Json::Value &ControlSystemObject);
	~CSequentialControlSystem();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

#endif
