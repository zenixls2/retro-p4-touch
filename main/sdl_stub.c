#include "sdl_stub.h"

#include <stdlib.h>
#include <string.h>

static const char *sdl_stub_error = "SDL stub";

int SDL_Init(Uint32 flags)
{
    (void)flags;
    return 0;
}

void SDL_Quit(void)
{
}

const char *SDL_GetError(void)
{
    return sdl_stub_error;
}

SDL_Window *SDL_CreateWindow(const char *title, int w, int h, Uint32 flags)
{
    (void)title;
    (void)w;
    (void)h;
    (void)flags;
    return calloc(1, sizeof(SDL_Window));
}

void SDL_DestroyWindow(SDL_Window *window)
{
    free(window);
}

SDL_Renderer *SDL_CreateRenderer(SDL_Window *window, const char *name)
{
    (void)window;
    (void)name;
    return calloc(1, sizeof(SDL_Renderer));
}

void SDL_DestroyRenderer(SDL_Renderer *renderer)
{
    free(renderer);
}

int SDL_SetRenderDrawColor(SDL_Renderer *renderer, Uint8 r, Uint8 g, Uint8 b, Uint8 a)
{
    (void)renderer;
    (void)r;
    (void)g;
    (void)b;
    (void)a;
    return 0;
}

int SDL_RenderClear(SDL_Renderer *renderer)
{
    (void)renderer;
    return 0;
}

void SDL_RenderPresent(SDL_Renderer *renderer)
{
    (void)renderer;
}

SDL_Texture *SDL_CreateTexture(SDL_Renderer *renderer, Uint32 format, int access, int w, int h)
{
    (void)renderer;
    (void)format;
    (void)access;

    if ((w <= 0) || (h <= 0)) {
        return NULL;
    }

    SDL_Texture *texture = calloc(1, sizeof(SDL_Texture));
    if (texture == NULL) {
        return NULL;
    }

    texture->width = w;
    texture->height = h;
    texture->pitch = (int)(sizeof(Uint32) * (size_t)w);
    texture->pixels = calloc((size_t)w * (size_t)h, sizeof(Uint32));

    if (texture->pixels == NULL) {
        free(texture);
        return NULL;
    }

    return texture;
}

void SDL_DestroyTexture(SDL_Texture *texture)
{
    if (texture == NULL) {
        return;
    }
    free(texture->pixels);
    free(texture);
}

int SDL_UpdateTexture(SDL_Texture *texture, const void *rect, const void *pixels, int pitch)
{
    (void)rect;

    if ((texture == NULL) || (pixels == NULL) || (pitch <= 0)) {
        return -1;
    }

    const int copy_pitch = (pitch < texture->pitch) ? pitch : texture->pitch;
    const uint8_t *src = (const uint8_t *)pixels;
    uint8_t *dst = (uint8_t *)texture->pixels;

    for (int row = 0; row < texture->height; row++) {
        memcpy(dst + (size_t)row * (size_t)texture->pitch,
               src + (size_t)row * (size_t)pitch,
               (size_t)copy_pitch);
    }

    return 0;
}

int SDL_LockTexture(SDL_Texture *texture, const void *rect, void **pixels, int *pitch)
{
    (void)rect;

    if ((texture == NULL) || (pixels == NULL) || (pitch == NULL)) {
        return -1;
    }

    texture->locked = true;
    *pixels = texture->pixels;
    *pitch = texture->pitch;
    return 0;
}

void SDL_UnlockTexture(SDL_Texture *texture)
{
    if (texture != NULL) {
        texture->locked = false;
    }
}

int SDL_RenderTexture(SDL_Renderer *renderer, SDL_Texture *texture, const void *srcrect, const SDL_FRect *dstrect)
{
    (void)renderer;
    (void)texture;
    (void)srcrect;
    (void)dstrect;
    return 0;
}

Uint32 *SDL_GetTexturePixels(SDL_Texture *texture, int *width, int *height, int *pitch)
{
    if (texture == NULL) {
        return NULL;
    }

    if (width != NULL) {
        *width = texture->width;
    }
    if (height != NULL) {
        *height = texture->height;
    }
    if (pitch != NULL) {
        *pitch = texture->pitch;
    }

    return texture->pixels;
}
