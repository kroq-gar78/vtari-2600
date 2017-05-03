#!/bin/bash -ux

gcc -o graphics_test graphics_test.c $(sdl2-config --cflags) $(sdl2-config --libs)
