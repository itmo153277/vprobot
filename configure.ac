AC_PREREQ([2.69])
AC_INIT([vprobot], [0.1], [viktprog@gmail.com])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([-Wall -Werror subdir-objects])
AC_CONFIG_HEADERS([config.h])

AC_COPYRIGHT([Copyright (c) 2015 Ivanov Viktor])
AC_REVISION([rev4])

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

AC_PROG_RANLIB
AC_PROG_CC
AC_PROG_CXX
AM_PROG_AR

AC_LANG([C++])

AX_CXX_COMPILE_STDCXX_11([], [mandatory])

# Checks for libraries.

PKG_CHECK_MODULES([jsoncpp], [jsoncpp >= 0.6.0], [],
	[AC_MSG_ERROR([couldn't find jsoncpp])]);
LIBS="$jsoncpp_LIBS $LIBS"
CPPFLAGS="$jsoncpp_CFLAGS $CPPFLAGS"
CFLAGS="$jsoncpp_CFLAGS $CFLAGS"
CXXFLAGS="$jsoncpp_CFLAGS $CXXFLAGS"

PKG_CHECK_MODULES([Eigen3], [eigen3 >= 3.2.0], [],
	[AC_MSG_ERROR([couldn't find Eigen3])]);
LIBS="$Eigen3_LIBS $LIBS"
CPPFLAGS="$Eigen3_CFLAGS $CPPFLAGS"
CXXFLAGS="$Eigen3_CFLAGS $CXXFLAGS"

PKG_CHECK_MODULES([CHECK], [check >= 0.9.0], [can_use_check="yes"],
	[AC_MSG_WARN([Check not found; cannot run tests])])
AM_CONDITIONAL([CHECK], [test "x$can_use_check" = "xyes"])

can_use_sdl2="yes"
AC_ARG_WITH(sdl2, AS_HELP_STRING([--with-sdl2], [use sdl2 for ui [default=yes]]), [can_use_sdl2=$withval])
if test "x$can_use_sdl2" = "xyes" ; then
	PKG_CHECK_MODULES([SDL2], [sdl2 >= 2.0.0], [], [can_use_sdl2="no"])
	SDL2_LIBS="$SDL2_LIBS -lSDL2_gfx"
	if test "x$can_use_sdl2" = "xyes" ; then
		AC_LANG_PUSH([C])
		LIBS_SAVE=$LIBS
		CFLAGS_SAVE=$CFLAGS
		CPPFLAGS_SAVE=$CPPFLAGS
		LIBS="$SDL2_LIBS $LIBS"
		CLAGS="$SDL2_CFLAGS $CFLAGS"
		CPPLAGS="$SDL2_CFLAGS $CPPFLAGS"
		AC_CHECK_HEADERS([SDL2/SDL2_gfxPrimitives.h \
		                  SDL2/SDL2_rotozoom.h \
		                  SDL2/SDL2_framerate.h], [], [can_use_sdl2="no"], [])
		LIBS=$LIBS_SAVE
		CFLAGS=$CFLAGS_SAVE
		CPPFLAGS=$CPPFLAGS_SAVE
		AC_LANG_POP
	fi
	if test "x$can_use_sdl2" = "xno" ; then
		AC_MSG_WARN([Couldn't find SDL2; cli will be used instead])
	else
		AC_DEFINE([HAVE_SDL], 1, [Define to 1 to enable SDL])
	fi
fi
AM_CONDITIONAL([SDL2], [test "x$can_use_sdl2" = "xyes"])

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_CONFIG_FILES([Makefile
                 model/Makefile
                 test/Makefile
                 ui/Makefile])
AC_OUTPUT
