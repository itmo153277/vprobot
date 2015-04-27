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

#include "../types.h"

using namespace ::vprobot;
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
	m_Texture = SDL_CreateTexture(m_Renderer, SDL_PIXELFORMAT_RGBA8888,
			SDL_TEXTUREACCESS_TARGET, m_Rect.w, m_Rect.h);
}

::vprobot::ui::CSDLPresentationDriver::~CSDLPresentationDriver() {
	SDL_DestroyTexture(m_Texture);
}

/* Функция для преобразования координат */
void ::vprobot::ui::CSDLPresentationDriver::TranslateCoord(double x, double y,
		Sint16 &d_x, Sint16 &d_y) {
	d_x = static_cast<Sint16>((x + m_ofsx) * m_zoom);
	d_y = m_Rect.h - static_cast<Sint16>((y + m_ofsy) * m_zoom);
}

/* Обновить экран */
void ::vprobot::ui::CSDLPresentationDriver::Update() {
	SDL_SetRenderTarget(m_Renderer, m_Texture);
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderFillRect(m_Renderer, NULL);
}

/* Нарисовать точку */
void ::vprobot::ui::CSDLPresentationDriver::DrawCircle(double x, double y,
		double r, int R, int G, int B, int A) {
	Sint16 r_x, r_y, r_r = static_cast<Uint16>(m_zoom * r);

	TranslateCoord(x, y, r_x, r_y);
	if (r_r > 0 && A > 0)
		filledCircleRGBA(m_Renderer, r_x, r_y, r_r, R, G, B, A);
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
	SDL_Rect i_Rect = {r_x, r_y, r_w, r_h};

	SDL_RenderCopyEx(m_Renderer, aux, NULL, &i_Rect, -angle, &i_Center,
			SDL_FLIP_NONE);
	SDL_DestroyTexture(aux);
}

/* Нарисовать фигуру */
void ::vprobot::ui::CSDLPresentationDriver::DrawShape(double *x, double *y,
		int count, int R, int G, int B, int A, int f_R, int f_G, int f_B,
		int f_A) {
	Sint16 *r_x, *r_y;
	int i;

	r_x = new Sint16[count];
	r_y = new Sint16[count];
	for (i = 0; i < count; i++) {
		TranslateCoord(x[i], y[i], r_x[i], r_y[i]);
	}
	if (f_A > 0)
		filledPolygonRGBA(m_Renderer, r_x, r_y, count, f_R, f_G, f_B, f_A);
	if (A > 0)
		polygonRGBA(m_Renderer, r_x, r_y, count, R, G, B, A);
	delete[] r_x;
	delete[] r_y;
}

/* Нарисовать линию */
void ::vprobot::ui::CSDLPresentationDriver::DrawLine(double x0, double y0,
		double xf, double yf, int R, int G, int B, int A) {
	Sint16 r_x0, r_y0, r_xf, r_yf;

	TranslateCoord(x0, y0, r_x0, r_y0);
	TranslateCoord(xf, yf, r_xf, r_yf);
	if (A > 0)
		lineRGBA(m_Renderer, r_x0, r_y0, r_xf, r_yf, R, G, B, A);
}

/* Написать текст */
void ::vprobot::ui::CSDLPresentationDriver::PutText(double x, double y,
		const char *Text, int R, int G, int B, int A) {
	Sint16 r_x, r_y;

	TranslateCoord(x, y, r_x, r_y);
	if (A > 0)
		stringRGBA(m_Renderer, r_x, r_y, Text, R, G, B, A);
}

/* Проецировать на экран */
void ::vprobot::ui::CSDLPresentationDriver::ProjectToSurface() {
	SDL_RenderCopy(m_Renderer, m_Texture, NULL, &m_Rect);
	SDL_SetRenderDrawColor(m_Renderer, 0, 0, 0, 255);
	SDL_RenderDrawRect(m_Renderer, &m_Rect);
	stringRGBA(m_Renderer, m_Rect.x + 1, m_Rect.y + 1, m_Title.c_str(), 0, 0, 0,
			255);
}

/* CUI */

::vprobot::ui::CUI::CUI(CPresentationHandler &Handler,
		const Json::Value &PresentationObject) :
		m_Handler(Handler), m_ScreensSet(), m_Quit(false), m_Redraw(true), m_FirstStart(
				true), m_HandlerFunction(
		NULL) {
	int i_w = PresentationObject["width"].asInt(), i_h =
			PresentationObject["height"].asInt();

	m_Delay = PresentationObject["delay"].asInt();
	m_QuitOnStop = PresentationObject["quit"].asBool();
	SDL_Init(SDL_INIT_EVERYTHING);
	m_Window = SDL_CreateWindow(PACKAGE_STRING, SDL_WINDOWPOS_UNDEFINED,
	SDL_WINDOWPOS_UNDEFINED, i_w, i_h, SDL_WINDOW_SHOWN);
	std::cout << SDL_GetError() << std::endl;
	m_Renderer = SDL_CreateRenderer(m_Window, -1,
			SDL_RENDERER_ACCELERATED || SDL_RENDERER_PRESENTVSYNC
					|| SDL_RENDERER_TARGETTEXTURE);
	m_Texture = SDL_CreateTexture(m_Renderer, SDL_PIXELFORMAT_RGBA8888,
			SDL_TEXTUREACCESS_TARGET, i_w, i_h);
	SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
	SDL_RenderClear(m_Renderer);
	SDL_SetRenderTarget(m_Renderer, m_Texture);
	SDL_RenderFillRect(m_Renderer, NULL);
	SDL_SetRenderTarget(m_Renderer, NULL);
	SDL_RenderPresent(m_Renderer);

	Json::ArrayIndex i;
	const Json::Value Screens = PresentationObject["screens"];

	for (i = 0; i < Screens.size(); i++) {
		m_ScreensSet.push_back(new SScreen(Screens[i], m_Renderer));
	}

	m_MutexDraw = SDL_CreateMutex();
	m_Cond = SDL_CreateCond();
}

::vprobot::ui::CUI::~CUI() {
	SDL_DestroyCond(m_Cond);
	SDL_DestroyMutex(m_MutexDraw);
	for (auto s : m_ScreensSet) {
		delete s;
	}
	SDL_DestroyTexture(m_Texture);
	SDL_DestroyRenderer(m_Renderer);
	SDL_DestroyWindow(m_Window);
	SDL_Quit();
}

/* Функция для потока обработки сообщений */
int ::vprobot::ui::CUI::ThreadFunction(void *data) {
	return static_cast<CUI *>(data)->ThreadProcess();
}
int ::vprobot::ui::CUI::ThreadProcess() {
	bool quit = false, wait = false;

	for (;;) {
		quit = !(*m_HandlerFunction)();
		if (m_Delay > 0 && wait)
			SDL_Delay(m_Delay);
		wait = !wait;
		SDL_LockMutex(m_MutexDraw);
		if (quit && m_QuitOnStop)
			m_Quit = true;
		if (m_Quit) {
			SDL_UnlockMutex(m_MutexDraw);
			break;
		}
		m_Redraw = true;
		SDL_CondWait(m_Cond, m_MutexDraw);
		m_FirstStart = false;
		SDL_UnlockMutex(m_MutexDraw);
		if (quit)
			break;
	}
	return 0;
}

/* Обновление данных */
void ::vprobot::ui::CUI::Process(const HandlerFunction &Function) {
	bool wait = true, input = false, quit = false, wait2 = true;
	SDL_Thread *i_Thread;
	FPSmanager manager;
	SDL_Event e;

	m_Redraw = false;
	m_FirstStart = true;
	m_HandlerFunction = &Function;
	SDL_initFramerate(&manager);
	i_Thread = SDL_CreateThread(ThreadFunction, "GUIThread",
			static_cast<void *>(this));
	do {
		SDL_framerateDelay(&manager);
		SDL_LockMutex(m_MutexDraw);
		if (m_Redraw) {
			wait = true;
			wait2 = !wait2;
			SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
			SDL_RenderClear(m_Renderer);
			for (auto s : m_ScreensSet) {
				s->Driver.Update();
				m_Handler.DrawPresentation(s->Driver, s->Name);
				SDL_SetRenderTarget(m_Renderer, m_Texture);
				s->Driver.ProjectToSurface();
			}
			SDL_SetRenderTarget(m_Renderer, NULL);
			SDL_RenderCopy(m_Renderer, m_Texture, NULL, NULL);
			SDL_RenderPresent(m_Renderer);
			m_Redraw = false;
		} else {
			SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);
			SDL_RenderClear(m_Renderer);
			SDL_RenderCopy(m_Renderer, m_Texture, NULL, NULL);
			SDL_RenderPresent(m_Renderer);
		}
		while (SDL_PollEvent(&e)) {
			switch (e.type) {
				case SDL_QUIT:
					m_Quit = true;
					break;
				case SDL_KEYDOWN:
					input = true;
					break;
			}
		}
		if (wait) {
			if (m_FirstStart) {
				SDL_CondSignal(m_Cond);
			} else if (m_Quit) {
				SDL_CondSignal(m_Cond);
			} else if (m_Delay == 0 && wait2) {
				if (input) {
					wait = false;
					input = false;
					SDL_CondSignal(m_Cond);
				}
			} else {
				SDL_CondSignal(m_Cond);
			}
		}
		quit = m_Quit;
		SDL_UnlockMutex(m_MutexDraw);
	} while (!quit);
	SDL_WaitThread(i_Thread, NULL);
	m_HandlerFunction = NULL;
}
