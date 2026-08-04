#ifndef FLOWENGINE_COMPAT_WIN_LIBGEN_H
#define FLOWENGINE_COMPAT_WIN_LIBGEN_H

#include <string.h>

static inline char* dirname(char* path) {
    if (!path || !path[0]) return (char*)".";
    char* slash = strrchr(path, '/');
    char* bslash = strrchr(path, '\\');
    if (!slash || (bslash && bslash > slash)) slash = bslash;
    if (!slash) return (char*)".";
    if (slash == path) {
        slash[1] = '\0';
    } else {
        *slash = '\0';
    }
    return path;
}

#endif /* FLOWENGINE_COMPAT_WIN_LIBGEN_H */
