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

#include "map.h"
#include "../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;
using namespace ::vprobot::line;
using namespace ::vprobot::map;

/* CPointMap */

vprobot::map::CPointMap::CPointMap(const Json::Value &MapObject) :
		CMap() {
	const Json::Value MapArray = MapObject["points"];

	/* Загружаем данные */
	Json::ArrayIndex i;

	for (i = 0; i < MapArray.size(); i++) {
		const Json::Value &p = MapArray[i];

		m_List.emplace_back(p["x"].asDouble(), p["y"].asDouble());
	}
}

vprobot::map::CPointMap::~CPointMap() {
}

/* Произвести измерение из точки по направлению */
double vprobot::map::CPointMap::GetDistance(const Point &p, double angle) {
	/* Только точки, нет пересечений с препятствиями */
	return 0;
}

/* Произвести измерение из точки до нужного маяка */
double vprobot::map::CPointMap::GetDistance(const Point &p, size_t index) {
	return (p - m_List[index]).norm();
}

/* Отображаем данные */
void vprobot::map::CPointMap::DrawPresentation(
		const SPresentationParameters *Params, double IndicatorZoom,
		CPresentationDriver &Driver) {
	for (auto p : m_List) {
		Driver.DrawCircle(p[0], p[1], 0.3 * IndicatorZoom, 0, 0, 255, 255);
	}
}

/* CLineMap */

vprobot::map::CLineMap::CLineMap(const Json::Value &MapObject) :
		CMap() {
	const Json::Value MapArray = MapObject["lines"];

	/* Загружаем данные */
	Json::ArrayIndex i;

	for (i = 0; i < MapArray.size(); i++) {
		const Json::Value &l = MapArray[i];
		MapList::reverse_iterator rl;
		Json::ArrayIndex j;

		m_List.emplace_back();
		rl = m_List.rbegin();
		for (j = 0; j < l.size(); j++) {
			const Json::Value &p = l[j];

			rl->emplace_back(p["x"].asDouble(), p["y"].asDouble());
		}
	}
}

vprobot::map::CLineMap::~CLineMap() {
}

/* Произвести измерение из точки по направлению */
double vprobot::map::CLineMap::GetDistance(const Point &p, double angle) {
	double d = 0;

	for (auto l : m_List) {
		double l_d = Measure(l, p, angle);

		if (LessOrEqualsZero(l_d)) {
			continue;
		}
		if (LessThan(l_d, d) || EqualsZero(d)) {
			d = l_d;
		}
	}
	return d;
}

/* Произвести измерение из точки до нужного маяка */
double vprobot::map::CLineMap::GetDistance(const Point &p, size_t index) {
	/* Нет маяков, невозможно измерить */
	return 0;
}

/* Отображаем данные */
void vprobot::map::CLineMap::DrawPresentation(
		const SPresentationParameters *Params, double IndicatorZoom,
		CPresentationDriver &Driver) {
	for (auto l : m_List) {
		size_t i;
		Line::const_iterator cl = l.begin();
		double *mx, *my;

		mx = new double[l.size()];
		my = new double[l.size()];
		for (i = 0; i < l.size(); i++, cl++) {
			mx[i] = (*cl)[0];
			my[i] = (*cl)[1];
		}
		Driver.DrawShape(mx, my, l.size(), 0, 0, 0, 255, 0, 0, 0, 0);
		delete[] mx;
		delete[] my;
	}
}
