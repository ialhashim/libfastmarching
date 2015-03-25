TARGET = libfastmarching
TEMPLATE = lib
CONFIG += staticlib

SOURCES += libfastmarching.cpp
HEADERS += libfastmarching.h

# Only part of boost used is 'heap' module
INCLUDEPATH += . boost_heap/

# Build flag
CONFIG(debug, debug|release) {CFG = debug} else {CFG = release}

# LIB output folder
DESTDIR = $$PWD/$$CFG/lib
