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
		const Json::Value &ScreenObject, SDL_Renderer *Renderer) :
		m_Title(ScreenObject["title"].asString()), m_Renderer(Renderer), m_Rect() {
	m_Rect.x = ScreenObject["x"].asInt();
	m_Rect.y = ScreenObject["y"].asInt();
	m_Rect.w = ScreenObject["width"].asInt();
	m_Rect.h = ScreenObject["height"].asInt();
	m_ofsx = ScreenObject["ofsx"].asDouble();
	m_ofsy = ScreenObject["ofsy"].asDouble();
	m_zoom = ScreenObject["zoom"].asDouble();
	m_Texture = SDL_CreateTexture(m_Renderer, SDL_PIXELFORMAT_ARGB8888,
			SDL_TEXTUREACCESS_TARGET, m_Rect.w, m_Rect.h);
	Update();
}

::vprobot::ui::CSDLPresentationDriver::~CSDLPresentationDriver() {
}

/* Функция для преобразования координат */
void ::vprobot::ui::CSDLPresentationDriver::TranslateCoord(double x, double y,
		Sint16 &d_x, Sint16 &d_y) {
	d_x = static_cast<Sint16>((x + m_ofsx) * m_zoom);
	d_y = static_cast<Sint16>((y + m_ofsy) * m_zoom);
}

/* Обновить экран */
void ::vprobot::ui::CSDLPresentationDriver::Update() {
	SDL_Rect i_Rect = {0, 0, m_Rect.w, m_Rect.h};

	SDL_SetRenderTarget(m_Renderer, m_Texture);
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderClear(m_Renderer);
	SDL_SetRenderDrawColor(m_Renderer, 0, 0, 0, 255);
	SDL_RenderDrawRect(m_Renderer, &i_Rect);
	stringRGBA(m_Renderer, 1, 1, m_Title.c_str(), 0, 0, 0, 255);
	SDL_SetRenderTarget(m_Renderer, NULL);
}

/* Нарисовать точку */
void ::vprobot::ui::CSDLPresentationDriver::DrawPoint(double x, double y, int R,
		int G, int B) {
	Sint16 r_x, r_y;

	TranslateCoord(x, y, r_x, r_y);
	SDL_SetRenderTarget(m_Renderer, m_Texture);
	pixelRGBA(m_Renderer, x, m_Rect.h - y, R, G, B, 255);
	SDL_SetRenderTarget(m_Renderer, NULL);
}

/* Нарисовать элипс */
void ::vprobot::ui::CSDLPresentationDriver::DrawEllipse(double x, double y,
		double a, double b, double angle, int R, int G, int B) {
	Sint16 r_x, r_y, r_a, r_b, r_w, r_h;
	SDL_Texture *aux;

	r_a = static_cast<Sint16>(a * m_zoom);
	r_b = static_cast<Sint16>(b * m_zoom);
	TranslateCoord(x, y, r_x, r_y);
	r_w = r_a * 2;
	r_h = r_b * 2;
	aux = SDL_CreateTexture(m_Renderer, SDL_PIXELFORMAT_ABGR8888,
			SDL_TEXTUREACCESS_TARGET, r_w, r_h);
	SDL_SetRenderTarget(m_Renderer, aux);
	ellipseRGBA(m_Renderer, r_a, r_b, r_a, r_b, R, G, B, 255);
	SDL_SetRenderTarget(m_Renderer, m_Texture);
	SDL_Point i_Center = {r_a, r_b};
	SDL_Rect i_Rect = {r_x, m_Rect.w - r_y, r_w, r_h};
	SDL_RenderCopyEx(m_Renderer, aux, NULL, &i_Rect, -angle, &i_Center,
			SDL_FLIP_NONE);
	SDL_SetRenderTarget(m_Renderer, NULL);
	SDL_DestroyTexture(aux);
}

/* Нарисовать фигуру */
void ::vprobot::ui::CSDLPresentationDriver::DrawShape(double *x, double *y,
		int count, int R, int G, int B) {
	Sint16 *r_x, *r_y;
	int i;

	r_x = new Sint16[count];
	r_y = new Sint16[count];
	for (i = 0; i < count; i++) {
		TranslateCoord(x[i], y[i], r_x[i], r_y[i]);
	}
	SDL_SetRenderTarget(m_Renderer, m_Texture);
	filledPolygonRGBA(m_Renderer, r_x, r_y, count, R, G, B, 255);
	SDL_SetRenderTarget(m_Renderer, NULL);
	delete[] r_x;
	delete[] r_y;
}

/* Проецировать на экран */
void ::vprobot::ui::CSDLPresentationDriver::ProjectToSurface() {
	SDL_RenderCopy(m_Renderer, m_Texture, NULL, &m_Rect);
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
	m_Renderer = SDL_CreateRenderer(m_Window, 0,
			SDL_RENDERER_ACCELERATED || SDL_RENDERER_TARGETTEXTURE);
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderClear(m_Renderer);
	SDL_RenderPresent(m_Renderer);

	Json::ArrayIndex i;
	const Json::Value Screens = PresentationObject["screens"];

	for (i = 0; i < Screens.size(); i++) {
		m_ScreensSet.push_back(new SScreen(Screens[i], m_Renderer));
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
		s->Driver.ProjectToSurface();
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
