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

#ifndef __PRESENTATION_H_
#define __PRESENTATION_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string>
#include <vector>
#include <json/json.h>

namespace vprobot {

namespace presentation {

/* Интерфейс для отрисовки данных */
class CPresentationDriver {
private:
	CPresentationDriver(const CPresentationDriver &Driver) = default;
public:
	CPresentationDriver() = default;
	virtual ~CPresentationDriver() = default;

	/* Нарисовать точку */
	virtual void DrawPoint(double x, double y, int Colour) = 0;
	/* Нарисовать элипс */
	virtual void DrawEllipse(double x, double y, double a, double b,
			double angle, int Colour) = 0;
	/* Нарисовать фигуру */
	virtual void DrawShape(double *x, double *y, int Colour) = 0;

};

/* Интерфейс для классов с данными */
struct SPresentationParameters {
	virtual ~SPresentationParameters() = default;
};

/* Класс для провайдера */
class CPresentationProvider {
private:
	CPresentationProvider(const CPresentationProvider &Provider) = default;
protected:
	/* Парсинг параметров для экрана */
	virtual SPresentationParameters *ParsePresentation(
			const Json::Value &PresentationObject) = 0;
	/* Отображаем данные */
	virtual void DrawPresentation(const SPresentationParameters &Params,
			CPresentationDriver &Driver) = 0;
public:
	CPresentationProvider(const Json::Value &PresentationObject);
	virtual ~CPresentationProvider();

	/* Инициализация дополнительных данных */
	void InitPresentations(const Json::Value &PresentationObject);
};

}

}

#endif
