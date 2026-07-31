CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I include -I include/include
SRC = src/frontend/main.cpp

ifeq ($(OS),Windows_NT)
	CXX = C:/msys64/ucrt64/bin/g++.exe
	TARGET = main.exe
	LDFLAGS = -L include/lib -lraylibdll -lopengl32 -lgdi32 -lwinmm
	COPY_DLL = cmd /C "if not exist raylib.dll copy include\lib\raylib.dll raylib.dll >NUL"
	RUN_TARGET = .\$(TARGET)
	CLEAN_CMD = cmd /C "if exist $(TARGET) del /Q $(TARGET) & if exist raylib.dll del /Q raylib.dll"
else
	TARGET = main
	LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
	COPY_DLL =
	RUN_TARGET = ./$(TARGET)
	CLEAN_CMD = rm -f $(TARGET)
endif

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(CXXFLAGS) $(LDFLAGS)
	$(COPY_DLL)

run: all
	$(RUN_TARGET)

clean:
	$(CLEAN_CMD)
