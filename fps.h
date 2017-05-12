#ifndef FPS_H
#define FPS_H

#include <SDL.h>

//#include "cpu.h"

#define FRAME_VALUES 10

#ifdef RENDER_FPS
#include <SDL2/SDL_ttf.h>
TTF_Font* font;
SDL_Texture* fps_texture;

extern void get_text_and_rect(SDL_Renderer *renderer, int x, int y, char *text,
        TTF_Font *font, SDL_Texture **texture, SDL_Rect *rect);
#endif

extern float framespersecond;
extern void fpsinit();
extern void fpsthink();

#endif
