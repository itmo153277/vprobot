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

#ifndef __SCENE_H_
#define __SCENE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <json/json.h>
#include "presentation.h"

namespace vprobot {

namespace scene {

/* Класс сцены */
class CScene: public vprobot::presentation::CPresentationHandler {
private:
	CScene(const CScene &Scene) = default;
public:
	CScene() = default;
	virtual ~CScene() = default;

	/* Выполнить симуляцию */
	virtual void Simulate() = 0;
};

}

}

#endif
