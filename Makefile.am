ACLOCAL_AMFLAGS = -I m4
SUBDIRS = model ui test
bin_PROGRAMS = vprobot
vprobot_SOURCES = main.cpp
noinst_HEADERS = types.h

if SDL2

vprobot_LDADD = model/libvprmodel.a ui/libvprui.a @SDL2_LIBS@
vprobot_CXXFLAGS = @SDL2_CFLAGS@

else

vprobot_LDADD = model/libvprmodel.a ui/libvprui.a

endif
