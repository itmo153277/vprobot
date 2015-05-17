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

#include "presentation.h"

#include <cmath>
#include <Eigen/Eigenvalues>

#include "../types.h"

using namespace ::std;
using namespace ::Eigen;
using namespace ::vprobot;
using namespace ::vprobot::presentation;

/* CPresentationDriver */

/* Написовать эллипс по матрице */
void vprobot::presentation::CPresentationDriver::DrawEllipse(const Vector2d &x,
		const Matrix2d &sx, int R, int G, int B, int A) {
	EigenSolver<Matrix2d> es;

	es.compute(sx, true);

	Vector2cd ev = es.eigenvalues(), ev1 = es.eigenvectors().col(0);

	DrawEllipse(x[0], x[1], 3 * sqrt(ev[0].real()), 3 * sqrt(ev[1].real()),
			CorrectAngle(atan2(ev1[1].real(), ev1[0].real())), R, G, B, A);
}

/* CPresentationProvider */

vprobot::presentation::CPresentationProvider::CPresentationProvider() :
		m_DataSet() {
}

vprobot::presentation::CPresentationProvider::~CPresentationProvider() {
	for (auto p : m_DataSet) {
		delete p.Parameters;
	}
}

/* Инициализация дополнительных данных */
void vprobot::presentation::CPresentationProvider::InitPresentations(
		const Json::Value &PresentationObject) {
	Json::ArrayIndex i;

	for (i = 0; i < PresentationObject.size(); i++) {
		const Json::Value po = PresentationObject[i];

		m_DataSet.emplace_back(ParsePresentation(po), po["name"].asString(),
				po["indicator_zoom"].asDouble());
	}
}

/* Нарисовать */
void vprobot::presentation::CPresentationProvider::UpdatePresentation(
		CPresentationDriver &Driver, const std::string &Name) {
	for (auto d : m_DataSet) {
		if (Name == d.Name) {
			DrawPresentation(d.Parameters, d.IndicatorZoom, Driver);
			break;
		}
	}
}
