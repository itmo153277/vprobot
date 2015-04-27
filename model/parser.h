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

#ifndef __PARSER_H_
#define __PARSER_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <json/json.h>
#include "presentation.h"
#include "scene.h"
#include "map.h"
#include "robot.h"
#include "control.h"

namespace vprobot {

namespace scene {

class CNormalScene: public CScene {
public:
	typedef std::vector<vprobot::robot::CRobot *> RobotSet;
private:
	/* Карта */
	vprobot::map::CMap *m_Map;
	/* Роботы */
	RobotSet m_Robots;
	/* Система управления */
	vprobot::control::CControlSystem *m_ControlSystem;
	/* Измерения */
	const vprobot::robot::SMeasures **m_Measures;
	/* Команды */
	const vprobot::robot::ControlCommand *m_Commands;
	/* Текущий шаг */
	int m_Time;
	/* Текущий статус */
	enum {
		m_Working,
		m_Stopped
	} m_Status;
	/* Класс для вывода информации */
	class CInfo: public vprobot::presentation::CPresentationProvider {
	private:
		/* Вывод данных */
		struct SInfoPresentationPrameters: public vprobot::presentation::SPresentationParameters {
			double x;
			double y;
			double dy;
			SInfoPresentationPrameters(const Json::Value &InfoData) :
					SPresentationParameters() {
				x = InfoData["x"].asDouble();
				y = InfoData["y"].asDouble();
				dy = InfoData["dy"].asDouble();
			}
		};
		/* Ссылка на сцену */
		const CNormalScene &m_Scene;

		CInfo(const CInfo &Info) = default;
	protected:
		/* Парсинг параметров для экрана */
		vprobot::presentation::SPresentationParameters *ParsePresentation(
				const Json::Value &PresentationObject);
		/* Отображаем данные */
		void DrawPresentation(
				const vprobot::presentation::SPresentationParameters *Params,
				vprobot::presentation::CPresentationDriver &Driver);
	public:
		CInfo(const CNormalScene &Scene);
		~CInfo();
	} m_Info;

	CNormalScene(const CNormalScene &Scene) = default;
public:
	CNormalScene(vprobot::map::CMap *Map, RobotSet Robots,
			vprobot::control::CControlSystem *ControlSystem,
			const Json::Value &PresentationObject);
	~CNormalScene();

	/* Выполнить симуляцию */
	bool Simulate();
	/* Нарисовать презентацию */
	void DrawPresentation(vprobot::presentation::CPresentationDriver &Driver,
			const std::string &Name);
};

}

vprobot::scene::CScene *Scene(const Json::Value &SceneObject);

}

#endif
