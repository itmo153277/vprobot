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

#ifndef __SDL_UI_H_
#define __SDL_UI_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <SDL2/SDL.h>
#include <json/json.h>
#include "../../model/presentation.h"

namespace vprobot {

namespace ui {

/* Класс интерфейса пользователя */
class CUI {
private:
	/* Ссылка на обработчик презентаций */
	const vprobot::presentation::CPresentationHandler &m_Handler;
	/* Окно */
	SDL_Window *m_Window;
	/* Поверхность окна */
	SDL_Surface *m_Surface;
	/* Поток обработки сообщений */
	SDL_Thread *m_Thread;
	/* Мьютекс для обновления экрана */
	SDL_mutex *m_MutexDraw;
	/* Время ожидания */
	int m_Delay;
	/* Условие для ожидания нажатия клавишы */
	SDL_cond *m_Cond;
	/* Условие завершения цикла (программа закрыта) */
	bool m_Quit;

	/* Функция для потока обработки сообщений */
	static int ThreadFunction(void *data);
	int ThreadProcess();

	CUI(const CUI &UI) = default;
public:
	CUI(const vprobot::presentation::CPresentationHandler &Handler,
			const Json::Value &PresentationObject);
	~CUI();

	/* Обновление данных */
	bool Update();
};

}

}

#endif
