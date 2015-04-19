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

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot::line;
using namespace ::vprobot::map;

/* CPointMap */

vprobot::map::CPointMap::CPointMap(const Json::Value &MapObject) {
	if (MapObject.isArray()) {
		/* Загружаем данные */
		int i;

		for (i = 0; i < MapObject.size(); i++) {
			const Json::Value p = MapObject[i];

			m_List.push_back(Point(p["x"].asDouble(), p["y"].asDouble()));
		}
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
double vprobot::map::CPointMap::GetDistance(const Point &p, int index) {
	return (p - m_List[index]).norm();
}
