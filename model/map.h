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

#ifndef __MAP_H_
#define __MAP_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstddef>
#include <vector>
#include <Eigen/Dense>
#include <json/json.h>
#include "line.h"

namespace vprobot {

namespace map {

/* Класс карты */
class CMap {
private:
	CMap(const CMap &Map) = default;
public:
	CMap() = default;
	virtual ~CMap() = default;

	/* Произвести измерение из точки по направлению */
	virtual double GetDistance(const line::Point &p, double angle) = 0;
	/* Произвести измерение из точки до нужного маяка */
	virtual double GetDistance(const line::Point &p, std::size_t index) = 0;
};

/* Карта, содержащая маяки */
class CPointMap: public CMap {
private:
	typedef std::vector<line::Point> MapList;

	/* Содержание карты */
	MapList m_List;

	CPointMap(const CPointMap &Map) = default;
public:
	CPointMap(const Json::Value &MapObject);
	~CPointMap();

	/* Произвести измерение из точки по направлению */
	double GetDistance(const line::Point &p, double angle);
	/* Произвести измерение из точки до нужного маяка */
	double GetDistance(const line::Point &p, std::size_t index);
};

/* Карта, содержащая линии */
class CLineMap: public CMap {
private:
	typedef std::vector<line::Line> MapList;

	/* Содержание карты */
	MapList m_List;

	CLineMap(const CLineMap &Map) = default;
public:
	CLineMap(const Json::Value &MapObject);
	~CLineMap();

	/* Произвести измерение из точки по направлению */
	double GetDistance(const line::Point &p, double angle);
	/* Произвести измерение из точки до нужного маяка */
	double GetDistance(const line::Point &p, std::size_t index);
};

}

}

#endif
