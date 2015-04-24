noinst_LIBRARIES = libvprui.a
noinst_HEADERS = ui.h cli/ui.h sdl/ui.h
EXTRA_DIST = sdl/ui.cpp

if SDL2

libvprui_a_CXXFLAGS = @SDL2_CFLAGS@
UI_SOURCES = sdl/ui.cpp

else

UI_SOURCES = 

endif

libvprui_a_SOURCES = $(UI_SOURCES)
