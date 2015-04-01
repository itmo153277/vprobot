AC_PREREQ([2.69])
AC_INIT([vprobot], [0.1], [viktprog@gmail.com])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([-Wall -Werror])
AC_CONFIG_HEADERS([config.h])

AC_COPYRIGHT([Copyright (c) 2015Ivanov Viktor])
AC_REVISION([rev1])

AC_CANONICAL_HOST

host_os_windows=no
case ${host_os} in
	*cygwin* | *mingw* )
		host_os_windows=yes
		;;
esac
AM_CONDITIONAL([WINDOWS], [test "x$host_os_windows" = "xyes"])

dnl Should add -mwin32 to enable _WIN32 macro
case ${host_os} in
	*cygwin* )
	CFLAGS="-mwin32 $CFLAGS"
	CXXFLAGS="-mwin32 $CXXFLAGS"
esac

# Checks for programs.
AC_PROG_CC
AC_PROG_CXX

# Checks for libraries.

PKG_CHECK_MODULES([JSONCPP], [jsoncpp >= 0.6.0], [],
	[AC_MSG_ERROR([couldn't find jsoncpp])]);
LIBS="$JSONCPP_LIBS $LIBS"
CFLAGS="$JSONCPP_CFLAGS $CFLAGS"
CXXFLAGS="$JSONCPP_CFLAGS $CXXFLAGS"

PKG_CHECK_MODULES([CHECK], [check >= 0.9.0], [can_use_check="yes"],
	[AC_MSG_WARN([Check not found; cannot run tests])])
AM_CONDITIONAL([CHECK], [test "x$can_use_check" = "xyes"])

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_CONFIG_FILES([Makefile
                 test/Makefile])
AC_OUTPUT
