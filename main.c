#include <stdio.h>
#include <sys/mman.h> // mmap
#include <sys/stat.h> // filesize
#include <unistd.h> // check file perms

#include <fcntl.h> // file control

#include <SDL.h>

#include "cmdline.h"
#include "cpu.h"
#include "fps.h"
#include "mem.h"
#include "tia.h"

struct gengetopt_args_info args;

bool graphics_on;
bool verbose_on;

byte* mmap_p; // pointer to mmap'd file

// SDL/graphics-related vars
SDL_Renderer* renderer;
SDL_Window* window;
unsigned int frame_num = 0;
unsigned int frame_rate = 0;
unsigned int frame_sleep = 0;

// account for the blanks, overscan, etc.
int v_start = DISPLAY_V_START;
int v_end = DISPLAY_V_END;
int h_start = DISPLAY_H_START;
int h_end = DISPLAY_H_END;

// loader; mmap the file into memory
struct stat st;
byte* ldr_mmap_file(char *filename)
{
    if(access(filename, F_OK) == -1)
    {
        fprintf(stderr, "ROM '%s' does not exist\n", filename);
        exit(1);
    }
    if(access(filename, R_OK) == -1)
    {
        fprintf(stderr, "ROM '%s' has no read permissions\n", filename);
        exit(1);
    }

    // get stats for the file
    stat(filename, &st);

    int fd = open(filename, O_RDONLY);

    byte* p = mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);

    // pick the minimum of the max cartrige size, and the filesize
#ifdef MOS_6502
    cart_size = st.st_size;
#endif
    cart_mem = (byte*)malloc(cart_size*sizeof(byte));
    for(int i = 0; i < cart_size; i++)
    {
        cart_mem[i] = p[i];
    }

#ifdef ATARI_2600
    // mirror onto second half if it's <=2K
    // TODO: make an option for this
    if(st.st_size <= CART_SIZE_ATARI/2)
    {
        for(int i = 0; i < st.st_size; i++)
        {
            cart_mem[i+2048] = p[i];
        }
    }
#endif

    if(p == MAP_FAILED)
    {
        fprintf(stderr, "mmap failed\n");
        perror("mmap");
    }
    return p;
}

// render single frame; should be callable anywhere
void draw_frame()
{
    // clear the screen before drawing anything
    // but, this might not actually be necessary (and might be slower)
    //SDL_RenderClear(renderer);

    // go through the grid and set colors
    for(int y = v_start; y < v_end; y++)
    {
        for(int x = h_start; x < h_end; x++)
        {
            int color = ntsc_rgb[tia_display[y][x]>>1];
            SDL_SetRenderDrawColor(renderer, color&0xff, (color>>8)&0xff, (color>>16)&0xff, 0);
            // rectangle from: http://stackoverflow.com/a/21903973
            SDL_Rect r;
            r.x = (x-h_start)*WINDOW_ZOOM*COLOR_CLOCK_WIDTH;
            r.y = (y-v_start)*WINDOW_ZOOM;
            r.w = WINDOW_ZOOM*COLOR_CLOCK_WIDTH;
            r.h = WINDOW_ZOOM;
            SDL_RenderFillRect(renderer, &r);
        }
    }

#ifdef RENDER_FPS
    // draw the FPS to top-left corner if it's enabled
    SDL_Rect text_rect;
    char fps_str[20];
    snprintf(fps_str, 20, "FPS: %.2f", framespersecond);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    get_text_and_rect(renderer, 0, 0, fps_str, font, &fps_texture, &text_rect);
    SDL_RenderCopy(renderer, fps_texture, NULL, &text_rect);
#endif

    SDL_RenderPresent(renderer);
}


int main(int argc, char* argv[])
{
    if(cmdline_parser(argc, argv, &args) != 0)
    {
        exit(1);
    }
    if(args.inputs_num != 1)
    {
        fprintf(stderr, "One ROM file necessary\n");
        exit(1);
    }

    graphics_on = !(args.no_graphics_flag);
    verbose_on = args.verbose_flag;

    char* rom_path = args.inputs[0];

    char* window_title_base = "Vtari 2600 - ";
    char* window_title = malloc(strlen(window_title_base) + strlen(rom_path) + 1);
    sprintf(window_title, "%s%s", window_title_base, rom_path);

    // http://stackoverflow.com/a/35989490
    if(graphics_on)
    {
        SDL_Init(SDL_INIT_VIDEO);
        //SDL_CreateWindowAndRenderer((h_end-h_start)*WINDOW_ZOOM*COLOR_CLOCK_WIDTH, (v_end-v_start)*WINDOW_ZOOM, 0, &window, &renderer);
        window = SDL_CreateWindow(window_title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                (h_end-h_start)*WINDOW_ZOOM*COLOR_CLOCK_WIDTH, // width
                (v_end-v_start)*WINDOW_ZOOM, // height
                0); // flags
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    }

    mmap_p = ldr_mmap_file(rom_path);

    // initialize each module
    // there might be dependencies that force the initialization order
    mem_init();
    tia_init();
    fpsinit();
    cpu_init();

    cpu_exec();

    // dump RAM to file (if CLI option given)
    if(args.dump_ram_given)
    {
        write_ram_path(args.dump_ram_arg);
    }

    /*int getwindowsize_h;
    int getwindowsize_w;
    SDL_GetWindowSize(window, &getwindowsize_w, &getwindowsize_h);
    printfv("Screen size: %dh %dw\n", getwindowsize_h, getwindowsize_w);
    printfv("should be:   %dh %dw\n", (v_end-v_start), (h_end-h_start));*/

    if(graphics_on)
    {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    free(cart_mem);
    free(window_title);
    cmdline_parser_free(&args);

    if(munmap(mmap_p, st.st_size) == -1)
    {
        perror("mmap");
        exit(1);
    }

    return 0;
}
