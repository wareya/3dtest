#!bash
g++ --std=c++17 -Wall -Wextra -pedantic -Wno-unused-parameter main.cpp depends/gl3w.c depends/libglfw3.a -o test.exe -Iinclude -lopengl32 -lgdi32 -static -lstdc++fs -O0 -ggdb -mconsole -mwindows -fdata-sections -ffunction-sections -Wl,--gc-sections #-Wl,--strip-all
