CC=g++
CFLAGS=-g -D__STDC_CONSTANT_MACROS -D_MV_DEBUG_ --std=c++11
CFLAGS+=-O3 -D__STDC_CONSTANT_MACROS -fexceptions -D_MV_DEBUG_
#CFLAGS=-g -D__STDC_CONSTANT_MACROS
LDFLAGS=-lswscale -lavdevice -lavformat -lavcodec -lswresample -lavutil -lpthread -lbz2 -lz -lc -lrt
OPENCV_DEPS= -L/usr/local/lib -lopencv_highgui -lopencv_videoio -lopencv_imgproc -lopencv_imgcodecs -lopencv_core -lpng -lopencv_calib3d -lopencv_photo

all: mv_detector_test
.PHONY: all

.cpp.o:
	$(CC) -c $(CFLAGS) $(CPPFLAGS) -o $@ $<

objects = mv_detector_test.o mv_detector.o
mv_detector_test: $(objects)
	g++ $^ -o $@ $(CFLAGS) $(LDFLAGS) $(OPENCV_DEPS) $(INSTALLED_DEPS)

mv_detector_test.exe : mv_detector_test.cpp  mv_detector.cpp mv_detector.h
	cl $? /MT /EHsc /I$(FFMPEG_DIR)\include /link avcodec.lib avformat.lib avutil.lib swscale.lib swresample.lib /LIBPATH:$(FFMPEG_DIR)\lib /OUT:$@
	for %I in ($(FFMPEG_DIR:dev=shared)\bin\avutil-*.dll $(FFMPEG_DIR:dev=shared)\bin\avformat-*.dll $(FFMPEG_DIR:dev=shared)\bin\avcodec-*.dll $(FFMPEG_DIR:dev=shared)\bin\swresample-*.dll) do copy %I $(MAKEDIR)

clean:
	$(OS:Windows_NT=del) rm -f mv_detector_test $(objects) *.exe *.obj *.dll
