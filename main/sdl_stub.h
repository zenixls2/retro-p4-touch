#ifndef SDL_STUB_H
#define SDL_STUB_H

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t Uint32;
typedef uint8_t Uint8;

typedef struct SDL_Window {
    int dummy;
} SDL_Window;

typedef struct SDL_Renderer {
    int dummy;
} SDL_Renderer;

typedef struct SDL_Texture {
    int width;
    int height;
    int pitch;
    Uint32 *pixels;
    bool locked;
} SDL_Texture;

typedef struct SDL_FRect {
    float x;
    float y;
    float w;
    float h;
} SDL_FRect;

#define SDL_INIT_VIDEO 0x00000020
#define SDL_PIXELFORMAT_RGBA32 0x16462004u
#define SDL_TEXTUREACCESS_STREAMING 1

int SDL_Init(Uint32 flags);
void SDL_Quit(void);
const char *SDL_GetError(void);

SDL_Window *SDL_CreateWindow(const char *title, int w, int h, Uint32 flags);
void SDL_DestroyWindow(SDL_Window *window);

SDL_Renderer *SDL_CreateRenderer(SDL_Window *window, const char *name);
void SDL_DestroyRenderer(SDL_Renderer *renderer);

int SDL_SetRenderDrawColor(SDL_Renderer *renderer, Uint8 r, Uint8 g, Uint8 b, Uint8 a);
int SDL_RenderClear(SDL_Renderer *renderer);
void SDL_RenderPresent(SDL_Renderer *renderer);

SDL_Texture *SDL_CreateTexture(SDL_Renderer *renderer, Uint32 format, int access, int w, int h);
void SDL_DestroyTexture(SDL_Texture *texture);

int SDL_UpdateTexture(SDL_Texture *texture, const void *rect, const void *pixels, int pitch);
int SDL_LockTexture(SDL_Texture *texture, const void *rect, void **pixels, int *pitch);
void SDL_UnlockTexture(SDL_Texture *texture);

int SDL_RenderTexture(SDL_Renderer *renderer, SDL_Texture *texture, const void *srcrect, const SDL_FRect *dstrect);

Uint32 *SDL_GetTexturePixels(SDL_Texture *texture, int *width, int *height, int *pitch);

#endif
