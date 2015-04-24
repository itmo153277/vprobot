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
	/* Внутреняя поверхность */
	SDL_Surface *m_Surface;
	/* Рендерер */
	SDL_Renderer *m_Renderer;
	/* Место, куда будет записываться данные */
	SDL_Rect m_Rect;

	/* Функция для преобразования координат */
	void TranslateCoord(double x, double y, int &d_x, int &d_y);
	/* Обновить экран */
	void Update();

	CSDLPresentationDriver(const CSDLPresentationDriver &Driver) = default;
public:
	CSDLPresentationDriver(const Json::Value &ScreenObject);
	~CSDLPresentationDriver();

	/* Нарисовать точку */
	void DrawPoint(double x, double y, int R, int G, int B);
	/* Нарисовать элипс */
	void DrawEllipse(double x, double y, double a, double b, double angle,
			int R, int G, int B);
	/* Нарисовать фигуру */
	void DrawShape(double *x, double *y, int R, int G, int B);
	/* Проецировать на экран */
	void ProjectToSurface(SDL_Renderer *Renderer);
};

/* Класс интерфейса пользователя */
class CUI {
private:
	/* Структура для экрана */
	struct SScreen {
		CSDLPresentationDriver Driver;
		std::string Name;
		SScreen(const Json::Value &ScreenObject) :
				Driver(ScreenObject), Name(ScreenObject["name"].asString()) {
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
	CUI(vprobot::presentation::CPresentationHandler &Handler,
			const Json::Value &PresentationObject);
	~CUI();

	/* Обновление данных */
	bool Update();
};

}

}

#endif
