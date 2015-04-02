TEST_FILES = jsonparse.test
EXTRA_DIST = $(TEST_FILES)

tester_SOURCES = tester.cpp
tester_CXXFLAGS = @CHECK_CFLAGS@
tester_LDADD = @CHECK_LIBS@ $(top_builddir)/model/libvprmodel.a $(top_builddir)/ui/libvprui.a

TEST_LOG_COMPILER = $(top_builddir)/test/tester

if CHECK
TESTS = $(TEST_FILES)
check_PROGRAMS = tester

else

TESTS = 
check_PROGRAMS =

endif
