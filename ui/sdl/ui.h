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

#include <string>
#include <vector>
#include <functional>
#include <SDL2/SDL.h>
#include <json/json.h>
#include "../../model/presentation.h"

namespace vprobot {

namespace ui {

/* Класс драйвера для презентации */
class CSDLPresentationDriver: public vprobot::presentation::CPresentationDriver {
private:
	/* Данные об экране */
	double m_ofsx;
	double m_ofsy;
	double m_zoom;
	std::string m_Title;
	/* Рендерер */
	SDL_Renderer *m_Renderer;
	/* Текстура */
	SDL_Texture *m_Texture;
	/* Место, куда будут записываться данные */
	SDL_Rect m_Rect;

	/* Функция для преобразования координат */
	void TranslateCoord(double x, double y, Sint16 &d_x, Sint16 &d_y);

	CSDLPresentationDriver(const CSDLPresentationDriver &Driver) = default;
public:
	CSDLPresentationDriver(const Json::Value &ScreenObject,
			SDL_Renderer *Renderer);
	~CSDLPresentationDriver();

	/* Нарисовать точку */
	void DrawCircle(double x, double y, double r, int R, int G, int B, int A);
	/* Нарисовать угол */
	void DrawPie(double x, double y, double r, double sa, double fa, int R,
			int G, int B, int A);
	/* Нарисовать элипс */
	void DrawEllipse(double x, double y, double a, double b, double angle,
			int R, int G, int B, int A);
	/* Нарисовать фигуру */
	void DrawShape(double *x, double *y, int count, int R, int G, int B, int A,
			int f_R, int f_G, int f_B, int f_A);
	/* Нарисовать линию */
	void DrawLine(double x0, double y0, double xf, double yf, int R, int G,
			int B, int A);
	/* Нарисовать квадрат */
	void DrawRectangle(double x0, double y0, double xf, double yf, int R, int G,
			int B, int A);
	/* Написать текст */
	void PutText(double x, double y, const char *Text, int R, int G, int B,
			int A);
	/* Проецировать на экран */
	void ProjectToSurface();
	/* Обновить экран */
	void Update();
};

/* Класс интерфейса пользователя */
class CUI {
public:
	/* Тип для обработчика */
	typedef std::function<void()> HandlerFunction;
private:
	/* Структура для экрана */
	struct SScreen {
		CSDLPresentationDriver Driver;
		std::string Name;
		SScreen(const Json::Value &ScreenObject, SDL_Renderer *ScreenRenderer) :
				Driver(ScreenObject, ScreenRenderer), Name(
						ScreenObject["name"].asString()) {
		}
	private:
		SScreen(const SScreen &Scren) = default;
	};
	typedef std::vector<SScreen *> ScreensSet;

	/* Ссылка на обработчик презентаций */
	vprobot::presentation::CPresentationHandler &m_Handler;
	/* Внутренние экраны */
	ScreensSet m_ScreensSet;
	/* Окно */
	SDL_Window *m_Window;
	/* Поверхность окна */
	SDL_Renderer *m_Renderer;
	/* Текстура */
	SDL_Texture *m_Texture;
	/* Мьютекс для обновления экрана */
	SDL_mutex *m_MutexDraw;
	/* Время ожидания */
	int m_Delay;
	/* Условие для ожидания нажатия клавишы */
	SDL_cond *m_Cond;
	/* Условие завершения цикла (программа закрыта) */
	bool m_Quit;
	/* Условие перерисовки */
	bool m_Redraw;
	/* Условие ожидания */
	bool m_Wait;
	/* Функция обработки */
	const HandlerFunction* m_HandlerFunction;
	/* Выходить по окончании симуляции */
	bool m_QuitOnStop;

	/* Функция для потока обработки сообщений */
	static int ThreadFunction(void *data);
	int ThreadProcess();

	CUI(const CUI &UI) = default;
public:
	CUI(vprobot::presentation::CPresentationHandler &Handler,
			const Json::Value &PresentationObject);
	~CUI();

	/* Обновление данных */
	void Process(const HandlerFunction &Handler);
};

}

}

#endif
