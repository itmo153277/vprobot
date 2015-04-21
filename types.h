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

#ifndef __TYPES_H_
#define __TYPES_H_

namespace vprobot {

const double PI = 3.14159265358979;

const double ErrorDomain = 0.00001;

/* Функции минимума, максиммума и модуля */

/* Минимум */
template<typename T>
T &min(T &x, T &y) {
	if (x < y)
		return x;
	return y;
}

/* Максимум */
template<typename T>
T &max(T &x, T &y) {
	if (x < y)
		return y;
	return x;
}

/* Модуль */
template<typename T>
T abs(T &x) {
	if (x < 0)
		return -x;
	return x;
}

/* Функции для сравнения дробных чисел */

/* Сравнение на ноль */
template<typename T>
bool EqualsZero(const T &x) {
	return abs(x) < ErrorDomain;
}

/* Равенство двух чисел */
template<typename T>
bool Equals(const T &x, const T &y) {
	return EqualsZero(x - y);
}

/* Отношения числа к нулю */
template<typename T>
bool LessThanZero(const T &x) {
	return x <= -ErrorDomain;
}
template<typename T>
bool LessOrEqualsZero(const T &x) {
	return x < ErrorDomain;
}
template<typename T>
bool GreaterThanZero(const T &x) {
	return x >= ErrorDomain;
}
template<typename T>
bool GreaterOrEqualsZero(const T &x) {
	return x > -ErrorDomain;
}

/* Отношения двух чисел */
template<typename T>
bool LessThan(const T &x, const T &y) {
	return LessThanZero(x - y);
}
template<typename T>
bool LessOrEquals(const T &x, const T &y) {
	return LessOrEqualsZero(x - y);
}
template<typename T>
bool GreaterOrEquals(const T &x, const T &y) {
	return GreaterOrEqualsZero(x - y);
}

/* Проверка угла */
template<typename T>
T CorrectAngle(const T &angle) {
	if (angle > PI)
		return angle - 2 * PI;
	else if (angle < -PI)
		return angle + 2 * PI;
	return angle;
}

/* Обход идиотских преудпрждений CDT */
#ifndef MatrixConvert
#define MatrixConvert(matrix) ((matrix).finished())
#endif

}

#endif
