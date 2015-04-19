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

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <json/json.h>
#include <cstdlib>
#include <check.h>

#ifdef fail
#undef fail
#endif

Json::Value data;

START_TEST(json_array_check) {
	int arrayLen = 0,
		nullCount = 0,
		stringCount = 0,
		objectCount = 0,
		intCount = 0,
		doubleCount = 0,
		arrayTarget = data["arrayLength"].asInt(),
		nullCountTarget = data["nullCount"].asInt(),
		stringCountTarget = data["stringCount"].asInt(),
		objectCountTarget = data["objectCount"].asInt(),
		intCountTarget = data["intCount"].asInt(),
		doubleCountTarget = data["doubleCount"].asInt(),
		i;
	const Json::Value arrayData = data["arrayData"];
	for (i = 0; i < arrayData.size(); i++) {
		const Json::Value arrayItem = arrayData[i];

		arrayLen++;
		if (arrayItem.isNull())
			nullCount++;
		else if (arrayItem.isString())
			stringCount++;
		else if (arrayItem.isObject())
			objectCount++;
		else if (arrayItem.isInt())
			intCount++;
		else if (arrayItem.isDouble())
			doubleCount++;
	}
	ck_assert_int_eq(arrayLen, arrayTarget);
	ck_assert_int_eq(nullCount, nullCountTarget);
	ck_assert_int_eq(stringCount, stringCountTarget);
	ck_assert_int_eq(objectCount, objectCountTarget);
	ck_assert_int_eq(intCount, intCountTarget);
	ck_assert_int_eq(doubleCount, doubleCountTarget);
}
END_TEST

START_TEST(json_tree_check) {
	int targetCode = data["targetCode"].asInt(),
		resultCode = data["arrayData"][4]["resultCode"].asInt();
	ck_assert_int_eq(targetCode, resultCode);
}
END_TEST

Suite *RobotTests(const char *in_file) {
	std::ifstream inp(in_file);
	std::stringstream json;

	if (inp.fail())
		return NULL;
	json << inp.rdbuf();

	Json::Reader reader;
	Json::Value root;

	if (!reader.parse(json.str(), root))
		return NULL;

	const std::string test_case = root["test"].asString();
	data = root["data"];
	Suite *s = NULL;
	TCase *tc_core = NULL;

	if (test_case == "json_parse") {
		s = suite_create("json_parse");
		tc_core = tcase_create("Core");

		tcase_add_test(tc_core, json_array_check);
		tcase_add_test(tc_core, json_tree_check);
	}
	if (s == NULL)
		return NULL;
	if (tc_core != NULL)
		suite_add_tcase(s, tc_core);
	return s;
}

int main(int argc, char *argv[]) {
	if (argc < 2)
		return EXIT_FAILURE;
	int number_failed;
	SRunner *sr = srunner_create(RobotTests(argv[1]));

	srunner_run_all(sr, CK_NORMAL);
	number_failed = srunner_ntests_failed(sr);
	srunner_free(sr);
	return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
