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

#ifndef __LINE_H_
#define __LINE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <Eigen/Dense>

namespace vprobot {

namespace line {

/* Тип для точки */
typedef Eigen::Vector2d Point;
/* Тип для линии */
typedef std::vector<Point> Line;

/* Функция поворота точки на угол */
Point Rotate(const Point &r, double angle);
/* Функция для замера расстояния от центра до отрезка по направлению */
double Measure(const Point &x1, const Point &x2, double angle);
/* Функция для замера расстояния от точки до линии по направлению */
double Measure(const Line &x, const Point &p, double angle);

}

}

#endif
