/**
 * PS3EYEDriver Simple SDL 2 example, using OpenGL where available.
 * Thomas Perl <m@thp.io>; 2014-01-10
 * Joseph Howse <josephhowse@nummist.com>; 2014-12-26
 **/

#include "SDL.h"
#include "ps3eyedriver.h"
#include <memory.h>
#include <stdlib.h>
#include <stdio.h>

struct ps3eye_context {
    ps3eye_context(int width, int height, int fps)
        : eye(0)
        , running(true)
        , last_ticks(0)
        , last_frames(0)
		, frame_width(width)
		, frame_height(height)
		, frame_rate(fps)
    {
		ps3eye_init();
    }

	~ps3eye_context()
	{
		ps3eye_uninit();
	}

    bool
    hasDevices()
    {
		return (ps3eye_count_connected() > 0);
    }

	ps3eye_t *eye;

    bool running;
    Uint32 last_ticks;
    Uint32 last_frames;
	Uint32 frame_width;
	Uint32 frame_height;
	Uint32 frame_rate;
};

void
print_renderer_info(SDL_Renderer *renderer)
{
    SDL_RendererInfo renderer_info;
    SDL_GetRendererInfo(renderer, &renderer_info);
    printf("Renderer: %s\n", renderer_info.name);
}

int
main(int argc, char *argv[])
{
    ps3eye_context ctx(640, 480, 30);
    if (!ctx.hasDevices()) {
        printf("No PS3 Eye camera connected\n");
        return EXIT_FAILURE;
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    SDL_Window *window = SDL_CreateWindow(
            "PS3 Eye - SDL 2", SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED, 640, 480, 0);
    if (window == NULL) {
        printf("Failed to create window: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1,
                                                SDL_RENDERER_PRESENTVSYNC);
    if (renderer == NULL) {
        printf("Failed to create renderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        return EXIT_FAILURE;
    }
    SDL_RenderSetLogicalSize(renderer, ctx.frame_width, ctx.frame_height);
    print_renderer_info(renderer);

    SDL_Texture *video_tex = SDL_CreateTexture(
            renderer, SDL_PIXELFORMAT_YUY2, SDL_TEXTUREACCESS_STREAMING,
			ctx.frame_width, ctx.frame_height);
    if (video_tex == NULL) {
        printf("Failed to create video texture: %s\n", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        return EXIT_FAILURE;
    }

	ctx.eye = ps3eye_open(0, ctx.frame_width, ctx.frame_height, ctx.frame_rate);
	if (ctx.eye == NULL)
	{
		printf("Failed to initialize camera: %s\n");
		SDL_DestroyRenderer(renderer);
		SDL_DestroyWindow(window);
		return EXIT_FAILURE;
	}

	ps3eye_set_flip(ctx.eye, true, false);

	printf("Camera mode: %dx%d@%d\n", ctx.frame_width, ctx.frame_height, ctx.frame_rate);


    SDL_Event e;
    void *video_tex_pixels;
    int pitch;
    while (ctx.running) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                ctx.running = false;
            }
        }

        // TODO: Proper thread signalling to wait for next available buffer
        SDL_Delay(10);
		int row_bytes = 0;
		uint8_t *frame_data = ps3eye_grab_frame(ctx.eye, &row_bytes);
		int frame_data_size = row_bytes * ctx.frame_height;

		if (frame_data != NULL)
		{
			SDL_LockTexture(video_tex, NULL, &video_tex_pixels, &pitch);
			memcpy(video_tex_pixels, frame_data, frame_data_size);
			SDL_UnlockTexture(video_tex);
		}

        SDL_RenderCopy(renderer, video_tex, NULL, NULL);
        SDL_RenderPresent(renderer);
    }

	ps3eye_close(ctx.eye);

    SDL_DestroyTexture(video_tex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return EXIT_SUCCESS;
}
