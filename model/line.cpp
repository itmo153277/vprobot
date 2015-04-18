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

#include "line.h"
#include "../types.h"

using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::line;

/* Функция поворота точки на угол */
Point vprobot::line::Rotate(const Point &r, double angle) {
	Matrix2d T;

	T << cos(angle), -sin(angle), sin(angle), cos(angle);
	return T * r;
}

/* Функция для замера расстояния от центра до отрезка по направлению */
double vprobot::line::Measure(const Point &x1, const Point &x2, double angle) {
	Point x1r = Rotate(x1, -angle);
	Point x2r = Rotate(x2, -angle);
	bool x1r_zero = EqualsZero(x1r[1]);
	bool x2r_zero = EqualsZero(x2r[1]);

	if (x1r_zero && x2r_zero) {
		return min(x1r[0], x2r[0]);
	} else if (x1r_zero) {
		return x1r[0];
	} else if (x2r_zero) {
		return x2r[0];
	}

	bool x1r_lzero = LessThanZero(x1r[1]);
	bool x2r_lzero = LessThanZero(x2r[1]);
	if ((x1r_lzero && x2r_lzero) || (!x1r_lzero && !x2r_lzero)) {
		return 0;
	}
	double Rate = - (x1r[1]) / (x2r[1]);

	return (x1r[0] + Rate * x2r[0]) / (1 + Rate);
}

/* Функция для замера расстояния от точки до линии по направлению */
double vprobot::line::Measure(const Line &x, const Point &p, double angle) {
	double d = 0;

	if (x.size() > 0) {
		Line::const_iterator x1 = x.begin();
		Line::const_iterator x2 = x1 + 1;

		for (;;) {
			double cd = Measure(*x1 - p, *x2 - p, angle);

			if (LessOrEqualsZero(cd)) {
				continue;
			}
			if (LessThan(cd, d) || EqualsZero(d)) {
				d = cd;
			}
			x1++;
			if (x1 == x.end())
				break;
			x2++;
			if (x2 == x.end())
				x2 = x.begin();
		}
	}
	return d;
}

