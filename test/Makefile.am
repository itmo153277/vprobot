TEST_FILES = movement.test
EXTRA_DIST = $(TEST_FILES)

tester_CXXFLAGS = @CHECK_CFLAGS@
tester_LDADD = @CHECK_LIBS@
tester_SOURCES = tester.cpp

if CHECK
TESTS = $(TEST_FILES)
check_PROGRAMS = tester
LOG_COMPILER = $(top_builddir)/test/tester

else

TESTS = 
check_PROGRAMS =
LOG_COMPILER =

endif
