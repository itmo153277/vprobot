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

#include "ui.h"

#include <SDL2/SDL2_framerate.h>

using namespace ::vprobot::presentation;
using namespace ::vprobot::ui;

/* CUI */

::vprobot::ui::CUI::CUI(const CPresentationHandler &Handler,
		const Json::Value &PresentationObject) :
		m_Handler(Handler), m_Quit(false) {
	m_Delay = PresentationObject["delay"].asInt();
	SDL_Init(SDL_INIT_EVERYTHING);
	m_Window = SDL_CreateWindow(PACKAGE_STRING, SDL_WINDOWPOS_UNDEFINED,
	SDL_WINDOWPOS_UNDEFINED, PresentationObject["width"].asInt(),
			PresentationObject["height"].asInt(), SDL_WINDOW_SHOWN);
	m_Surface = SDL_GetWindowSurface(m_Window);
	m_Thread = SDL_CreateThread(ThreadFunction, "GUIThread",
			static_cast<void *>(this));
	m_MutexDraw = SDL_CreateMutex();
	if (m_Delay == 0) {
		m_Cond = SDL_CreateCond();
	} else {
		m_Cond = NULL;
	}
}

::vprobot::ui::CUI::~CUI() {
	SDL_LockMutex(m_MutexDraw);
	m_Quit = true;
	SDL_UnlockMutex(m_MutexDraw);
	SDL_WaitThread(m_Thread, NULL);
	SDL_DestroyCond(m_Cond);
	SDL_DestroyMutex(m_MutexDraw);
	SDL_DestroyWindow(m_Window);
	SDL_Quit();
}

/* Функция для потока обработки сообщений */
int ::vprobot::ui::CUI::ThreadFunction(void *data) {
	return static_cast<CUI *>(data)->ThreadProcess();
}
int ::vprobot::ui::CUI::ThreadProcess() {
	FPSmanager manager;
	SDL_Event e;

	SDL_initFramerate(&manager);
	do {
		SDL_framerateDelay(&manager);
		SDL_LockMutex(m_MutexDraw);
		while (SDL_PollEvent(&e)) {
			switch (e.type) {
				case SDL_QUIT:
					m_Quit = true;
					SDL_CondSignal(m_Cond);
					break;
				case SDL_KEYDOWN:
					SDL_CondSignal(m_Cond);
					break;
			}
		}
		SDL_UpdateWindowSurface(m_Window);
		SDL_UnlockMutex(m_MutexDraw);
	} while (!m_Quit);
	return 0;
}

/* Обновление данных */
bool ::vprobot::ui::CUI::Update() {
	bool quit;

	SDL_LockMutex(m_MutexDraw);
	if (m_Delay == 0) {
		SDL_CondWait(m_Cond, m_MutexDraw);
	}
	quit = m_Quit;
	SDL_UnlockMutex(m_MutexDraw);
	if (quit)
		return false;
	if (m_Delay > 0)
		SDL_Delay(m_Delay);
	return true;
}
