
### You should not need to change anything below.
LINUX=1

# Compilers
CC=gcc
CXX=g++

# prefix for the library name
LIB_PREFIX=hogman_

# Paths
ROOTDIR=/home/nuechter/slam6d/trunk/3rdparty/hogman
LIBDIR=/home/nuechter/slam6d/trunk/3rdparty/hogman/lib
BINDIR=/home/nuechter/slam6d/trunk/3rdparty/hogman/bin

# Build tools
PRETTY=/home/nuechter/slam6d/trunk/3rdparty/hogman/build_tools/pretty_compiler
MESSAGE=/home/nuechter/slam6d/trunk/3rdparty/hogman/build_tools/message
TESTLIB=/home/nuechter/slam6d/trunk/3rdparty/hogman/build_tools/testlib

# Generic makefiles
MAKEFILE_GENERIC=/home/nuechter/slam6d/trunk/3rdparty/hogman/build_tools/Makefile.generic-shared-object
MAKEFILE_APP=/home/nuechter/slam6d/trunk/3rdparty/hogman/build_tools/Makefile.app
MAKEFILE_SUBDIRS=/home/nuechter/slam6d/trunk/3rdparty/hogman/build_tools/Makefile.subdirs

# Flags
CPPFLAGS+=-DLINUX -I/home/nuechter/slam6d/trunk/3rdparty/hogman  
CXXFLAGS+=-fPIC
LDFLAGS+= -Wl,-rpath,/home/nuechter/slam6d/trunk/3rdparty/hogman/lib
CARMENSUPPORT=

# QT support
MOC=/usr/bin/moc-qt4
UIC=/usr/bin/uic-qt4
QT_LIB=-L/usr/share/qt4/lib/ -lQtCore -lQtGui -lQtDesigner -lQtOpenGL -lQtXml
QT_INCLUDE=-I/usr/include/qt4 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtXml -I/usr/include/qt4/QtOpenGL



include /home/nuechter/slam6d/trunk/3rdparty/hogman/manual.mk

