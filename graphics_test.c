#ifndef ATARI_2600

#include <stdlib.h>
#include <stdio.h>
#include <SDL.h>

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

int main(int argc, char* argv[])
{
    SDL_Window* window = NULL;

    SDL_Surface* screenSurface = NULL;

    if(SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
        return 1;
    }

    window = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if(window == NULL)
    {
        SDL_Log("Unable to initialize SDL window: %s", SDL_GetError());
        return 1;
    }

    screenSurface = SDL_GetWindowSurface(window);
    SDL_FillRect(screenSurface, NULL, SDL_MapRGB(screenSurface->format, 0xff, 0xff, 0xff));

    SDL_UpdateWindowSurface(window);

    SDL_Delay(2000);

    SDL_DestroyWindow(window);

    SDL_Quit();

    return 0;
}
#endif
