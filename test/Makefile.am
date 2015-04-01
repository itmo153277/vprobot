TEST_FILES = jsonparse.test
EXTRA_DIST = $(TEST_FILES)

tester_CXXFLAGS = @CHECK_CFLAGS@
tester_LDADD = @CHECK_LIBS@
tester_SOURCES = tester.cpp

TEST_LOG_COMPILER = $(top_builddir)/test/tester

if CHECK
TESTS = $(TEST_FILES)
check_PROGRAMS = tester

else

TESTS = 
check_PROGRAMS =

endif
