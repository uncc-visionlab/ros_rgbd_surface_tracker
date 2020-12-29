## -*- Makefile -*-
##
## User: arwillis
## Time: Jun 26, 2020 12:34:23 PM
## Makefile created by Oracle Developer Studio.
##
## This file is generated automatically.
##
# 	libatlas-base-dev
# libgdal
CFLAGS += $(HDF5_CFLAGS)
CFLAGS += $(GEOTIFF_CFLAGS)
CFLAGS += $(HDF5_CFLAGS)
# Makefile for libasf_proj library
JAVA_HOME = /usr/lib/jvm/default-java

CFLAGS += $(W_ERROR) 

LIBDIR = ../devel/lib

# libs for the test program
LIBS =	${LIBDIR}/libros_rgbd_surface_tracker 

PYTHON_CFLAGS = $(shell python-config --cflags)
PYTHON_LDFLAGS = $(shell python-config --ldflags)

all: swig
	
## Target: swig
swig: .FORCE
	rm -rf libobjs
	mkdir -p libobjs
	mkdir -p ./Python
	echo $(PYTHON_CFLAGS)
	swig -python -outdir Python uncc_rgbd.i
	touch Python/__init__.py
	gcc -fPIC -c uncc_rgbd_wrap.c $(PYTHON_CFLAGS) -I../include 
	echo $(PYTHON_LDFLAGS)
	#gcc -shared -o Python/_asf_mapready.so asf_mapready_wrap.o $(PYTHON_LDFLAGS) -L../lib -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lasf -lasf_meta -lasf_import -lasf_insar -lasf_proj -lasf_raster -lhdf5 -lmpi -lgsl -lcblas -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp -lgdal -lglib-2.0 -lstdc++ -lm -lgtk-x11-2.0 -lgdk-x11-2.0 -lpangocairo-1.0 -latk-1.0 -lcairo -lgdk_pixbuf-2.0 -lgio-2.0 -lpangoft2-1.0 -lpango-1.0 -lgobject-2.0 -lglib-2.0 -lfontconfig -lfreetype -lproj
	gcc -shared -o Python/_uncc_rgbd.so asf_mapready_wrap.o $(PYTHON_LDFLAGS) -Wl,--whole-archive ../lib/asf.a ../lib/asf_meta.a ../lib/libasf_import.a ../lib/libasf_metadata.a ../lib/libasf_proj.a ../lib/libasf_raster.a ../lib/libasf_sar.a  ../lib/asf_fft.a -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp -lcblas -lgdal -lglib-2.0	
	#gcc -shared -o Python/_asf_mapready.so asf_mapready_wrap.o $(PYTHON_LDFLAGS) -Wl,--whole-archive ../lib/asf.a ../lib/asf_meta.a ../lib/libasf_export.a ../lib/libasf_geocode.a ../lib/libasf_import.a ../lib/libasf_insar.a ../lib/libasf_metadata.a ../lib/libasf_proj.a ../lib/libasf_raster.a ../lib/libasf_vector.a ../lib/libifm.a -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lcblas -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp -lgdal -lglib-2.0
	#gcc -shared -o Python/_asf_mapready.so asf_mapready_wrap.o -Wl,--whole-archive ../lib/asf.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/asf_meta.a  -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lcblas -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp -lgdal $(PYTHON_LDFLAGS)
	#gcc -shared -o Python/_asf_mapready.so asf_mapready_wrap.o -Wl,--whole-archive ../lib/asf.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/asf_meta.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_import.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_metadata.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_proj.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_sar.a  -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp 
	mkdir -p ./Java/src/swig/asf
	swig -java -outdir Java/src/swig/asf -package swig.asf asf_mapready.i
	gcc -fPIC -c asf_mapready_wrap.c -I$(JAVA_HOME)/include -I$(JAVA_HOME)/include/linux -I../include -I/usr/include/geotiff -I/usr/include/hdf5/serial -I/usr/include/libxml2
	#gcc -shared -o Java/libasf_mapready.so asf_mapready_wrap.o -Wl,--whole-archive ../lib/asf.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/asf_meta.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_import.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_proj.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_sar.a  -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp 
	gcc -shared -o Java/src/native_libs/libasf_mapready.so asf_mapready_wrap.o -Wl,--whole-archive ../lib/asf.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/asf_meta.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_import.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_metadata.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_proj.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_sar.a  -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp 
	#gcc -shared -o Python/libasf_mapready.so asf_mapready_wrap.o -Wl,--whole-archive ../lib/asf.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/asf_meta.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_import.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_proj.a  -Wl,--no-whole-archive -Wl,--whole-archive ../lib/libasf_sar.a  -Wl,--no-whole-archive -L/usr/lib/x86_64-linux-gnu/hdf5/openmpi -L/usr/lib/openmpi/lib -lhdf5 -lmpi -lgsl -lxml2 -lproj -ljpeg -ltiff -lgeotiff -lshp 
	#javac Java/swig/asf/*.java
	#javac runme.java -classpath ./Java
	
.FORCE:
	
#### Clean target deletes all generated files ####c
clean:
	rm -rf Java/src/swig/asf/*
	rm -rf Python/asf_mapready
	rm asf_mapready_wrap.o libasf_mapready.so || true



