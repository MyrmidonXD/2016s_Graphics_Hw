CC=g++
CFLAGS=-std=c++11 -O3 -lm -lGL -lGLU -lglut -I ./Eigen -I . -I ./bitmap

OBJ=parser.o spline.o models.o bitmap/EasyBMP.o hw5.o
DEPS=parser.h spline.h models.h bitmap/EasyBMP.h

.PHONY: clean run

all: _hw5

_hw5: $(OBJ)
	$(CC) $(CFLAGS) $^ -o $@

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

run: _hw5
	./_hw5

clean:
	rm -rf _hw5 *.o
