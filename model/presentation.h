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
#include <Eigen/Dense>
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
	virtual void DrawCircle(double x, double y, double r, int R, int G, int B,
			int A) = 0;
	/* Нарисовать угол */
	virtual void DrawPie(double x, double y, double r, double sa, double fa,
			int R, int G, int B, int A) = 0;
	/* Нарисовать эллипс */
	virtual void DrawEllipse(double x, double y, double a, double b,
			double angle, int R, int G, int B, int A) = 0;
	/* Написовать эллипс по матрице */
	void DrawEllipse(const Eigen::Vector2d &x, const Eigen::Matrix2d &sx, int R,
			int G, int B, int A);
	/* Нарисовать фигуру */
	virtual void DrawShape(double *x, double *y, int count, int R, int G, int B,
			int A, int f_R, int f_G, int f_B, int f_A) = 0;
	/* Нарисовать линию */
	virtual void DrawLine(double x0, double y0, double xf, double yf, int R,
			int G, int B, int A) = 0;
	/* Нарисовать квадрат */
	virtual void DrawRectangle(double x0, double y0, double xf, double yf,
			int R, int G, int B, int A) = 0;
	/* Написать текст */
	virtual void PutText(double x, double y, const char *Text, int R, int G,
			int B, int A) = 0;

};

/* Интерфейс для классов с данными */
struct SPresentationParameters {
	virtual ~SPresentationParameters() = default;
};

/* Класс для провайдера */
class CPresentationProvider {
private:
	/* Данные о вывода */
	struct SPresentationData {
		SPresentationParameters *Parameters;
		std::string Name;
		double IndicatorZoom;

		SPresentationData(SPresentationParameters *i_Parameters,
				const std::string &i_Name, double i_IndicatorZoom) :
				Parameters(i_Parameters), Name(i_Name), IndicatorZoom(
						i_IndicatorZoom) {
		}
	};
	/* Набор параметров */
	typedef std::vector<SPresentationData> DataSet;

	DataSet m_DataSet;

	CPresentationProvider(const CPresentationProvider &Provider) = default;
protected:
	/* Парсинг параметров для экрана */
	virtual SPresentationParameters *ParsePresentation(
			const Json::Value &PresentationObject) {
		return NULL;
	}
	virtual void DrawPresentation(const SPresentationParameters *Params,
			double IndicatorZoom, CPresentationDriver &Driver) {
	}
public:
	CPresentationProvider();
	virtual ~CPresentationProvider();

	/* Инициализация дополнительных данных */
	void InitPresentations(const Json::Value &PresentationObject);
	/* Нарисовать */
	void UpdatePresentation(CPresentationDriver &Driver,
			const std::string &Name);
};

/* Обработчик презентаций */
class CPresentationHandler {
public:
	enum SimulationState {
		SimulationEnd, SimulationWait, SimulationWorking
	};
protected:
	SimulationState m_sState;
private:
	CPresentationHandler(const CPresentationHandler &Handler) = default;
public:
	CPresentationHandler() :
			m_sState(SimulationEnd) {
	}
	;
	virtual ~CPresentationHandler() = default;

	/* Нарисовать */
	virtual void DrawPresentation(CPresentationDriver &Driver,
			const std::string &Name) = 0;
	/* Статус */
	inline const SimulationState GetSimlationState() const {
		return m_sState;
	}
};

}

}

#endif
