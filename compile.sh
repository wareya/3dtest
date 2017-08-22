#!bash
g++ -Wfatal-errors -msse -msse2 --std=c++17 -Wall -Wextra -pedantic -Wno-narrowing -Wno-unused-parameter main.cpp depends/gl3w.c depends/libglfw3.a -o test.exe -Iinclude -lopengl32 -lgdi32 -static -lstdc++fs -O3 -ggdb -mconsole -mwindows -fdata-sections -ffunction-sections -Wl,--gc-sections #-Wl,--strip-all
