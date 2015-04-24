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

using namespace ::std;
using namespace ::vprobot::presentation;

/* CPresentationProvider */

vprobot::presentation::CPresentationProvider::CPresentationProvider(
		const Json::Value &PresentationObject) {
	InitPresentations(PresentationObject);
}

vprobot::presentation::CPresentationProvider::~CPresentationProvider() {
}

/* Инициализация дополнительных данных */
void vprobot::presentation::CPresentationProvider::InitPresentations(
		const Json::Value &PresentationObject) {
}