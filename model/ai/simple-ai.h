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

#ifndef __AI_SIMPLE_AI_H_
#define __AI_SIMPLE_AI_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <json/json.h>
#include "../presentation.h"
#include "../robot.h"
#include "../control.h"

namespace vprobot {

namespace control {

namespace simple_ai {

class CSimpleAI: public CControlSystem {
public:
	CSimpleAI(const Json::Value &ControlSystemObject);
	~CSimpleAI();

	/* Получить команду */
	const vprobot::robot::ControlCommand * const GetCommands(
			const vprobot::robot::SMeasures * const *Measurements);
};

}

}

}

#endif
