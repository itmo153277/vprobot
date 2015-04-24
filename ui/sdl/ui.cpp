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
#include <SDL2/SDL2_gfxPrimitives.h>

using namespace ::vprobot::presentation;
using namespace ::vprobot::ui;

/* CSDLPresentationDriver */

::vprobot::ui::CSDLPresentationDriver::CSDLPresentationDriver(
		const Json::Value &ScreenObject) :
		m_Title(ScreenObject["title"].asString()), m_Rect() {
	m_Rect.x = ScreenObject["x"].asInt();
	m_Rect.y = ScreenObject["y"].asInt();
	m_Rect.w = ScreenObject["width"].asInt();
	m_Rect.h = ScreenObject["height"].asInt();
	m_ofsx = ScreenObject["ofsx"].asDouble();
	m_ofsy = ScreenObject["ofsy"].asDouble();
	m_zoom = ScreenObject["zoom"].asDouble();
	m_Surface = SDL_CreateRGBSurface(0, m_Rect.w, m_Rect.h, 32, 0, 0, 0, 0);
	m_Renderer = SDL_CreateSoftwareRenderer(m_Surface);
	Update();
}

::vprobot::ui::CSDLPresentationDriver::~CSDLPresentationDriver() {
	SDL_DestroyRenderer(m_Renderer);
	SDL_FreeSurface(m_Surface);
}

/* Функция для преобразования координат */
void ::vprobot::ui::CSDLPresentationDriver::TranslateCoord(double x, double y,
		int &d_x, int &d_y) {
	d_x = static_cast<int>((x + m_ofsx) * m_zoom);
	d_y = static_cast<int>((y + m_ofsy) * m_zoom);
}

/* Обновить экран */
void ::vprobot::ui::CSDLPresentationDriver::Update() {
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderClear(m_Renderer);
	stringRGBA(m_Renderer, 0, 0, m_Title.c_str(), 0, 0, 0, 255);
}

/* Нарисовать точку */
void ::vprobot::ui::CSDLPresentationDriver::DrawPoint(double x, double y, int R,
		int G, int B) {

}

/* Нарисовать элипс */
void ::vprobot::ui::CSDLPresentationDriver::DrawEllipse(double x, double y,
		double a, double b, double angle, int R, int G, int B) {

}

/* Нарисовать фигуру */
void ::vprobot::ui::CSDLPresentationDriver::DrawShape(double *x, double *y,
		int R, int G, int B) {

}

/* Проецировать на экран */
void ::vprobot::ui::CSDLPresentationDriver::ProjectToSurface(
		SDL_Renderer *Renderer) {
	SDL_Texture *i_Texture;

	SDL_RenderPresent(m_Renderer);
	i_Texture = SDL_CreateTextureFromSurface(Renderer, m_Surface);
	SDL_RenderCopy(Renderer, i_Texture, NULL, &m_Rect);
	Update();
}

/* CUI */

::vprobot::ui::CUI::CUI(CPresentationHandler &Handler,
		const Json::Value &PresentationObject) :
		m_Handler(Handler), m_ScreensSet(), m_Quit(false) {
	m_Delay = PresentationObject["delay"].asInt();
	SDL_Init(SDL_INIT_EVERYTHING);
	m_Window = SDL_CreateWindow(PACKAGE_STRING, SDL_WINDOWPOS_UNDEFINED,
	SDL_WINDOWPOS_UNDEFINED, PresentationObject["width"].asInt(),
			PresentationObject["height"].asInt(), SDL_WINDOW_SHOWN);
	m_Renderer = SDL_CreateRenderer(m_Window, 0, 0);
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderClear(m_Renderer);
	SDL_RenderPresent(m_Renderer);

	Json::ArrayIndex i;
	const Json::Value Screens = PresentationObject["screens"];

	for (i = 0; i < Screens.size(); i++) {
		m_ScreensSet.emplace_back(new SScreen(Screens[i]));
	}

	m_MutexDraw = SDL_CreateMutex();
	if (m_Delay == 0) {
		m_Cond = SDL_CreateCond();
	} else {
		m_Cond = NULL;
	}
	m_Thread = SDL_CreateThread(ThreadFunction, "GUIThread",
			static_cast<void *>(this));
}

::vprobot::ui::CUI::~CUI() {
	SDL_LockMutex(m_MutexDraw);
	m_Quit = true;
	SDL_UnlockMutex(m_MutexDraw);
	SDL_WaitThread(m_Thread, NULL);
	SDL_DestroyCond(m_Cond);
	SDL_DestroyMutex(m_MutexDraw);
	for (auto s : m_ScreensSet) {
		delete s;
	}
	SDL_DestroyRenderer(m_Renderer);
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
		SDL_UnlockMutex(m_MutexDraw);
	} while (!m_Quit);
	return 0;
}

/* Обновление данных */
bool ::vprobot::ui::CUI::Update() {
	bool quit;

	SDL_LockMutex(m_MutexDraw);
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderClear(m_Renderer);
	for (auto s : m_ScreensSet) {
		m_Handler.DrawPresentation(s->Driver, s->Name);
		s->Driver.ProjectToSurface(m_Renderer);
	}
	SDL_RenderPresent(m_Renderer);
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
