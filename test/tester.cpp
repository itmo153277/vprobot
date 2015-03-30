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

#include <stdlib.h>
#include <check.h>

START_TEST(simple_test) {
	ck_assert_int_eq(5, 5);
}
END_TEST

Suite *RobotTests() {
	Suite *s = suite_create("RobotTests");
	TCase *tc_core = tcase_create("Core");

	tcase_add_test(tc_core, simple_test);
	suite_add_tcase(s, tc_core);
	return s;
}

int main(int argc, char *argv[]) {
	int number_failed;
	SRunner *sr = srunner_create(RobotTests());

	srunner_run_all(sr, CK_NORMAL);
	number_failed = srunner_ntests_failed(sr);
	srunner_free(sr);
	return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}