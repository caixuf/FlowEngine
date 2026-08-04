#ifndef FLOWENGINE_COMPAT_WIN_DLFCN_H
#define FLOWENGINE_COMPAT_WIN_DLFCN_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <stdio.h>
#include <string.h>

#define RTLD_LAZY   0
#define RTLD_NOW    0
#define RTLD_GLOBAL 0
#define RTLD_LOCAL  0

static char flow_dlerror_buf[512];

static inline void flow_set_dlerror_from_win(const char* what) {
    DWORD e = GetLastError();
    snprintf(flow_dlerror_buf, sizeof(flow_dlerror_buf), "%s failed (winerr=%lu)", what, (unsigned long)e);
}

static inline void flow_make_dll_candidate(const char* path, char* out, size_t out_sz) {
    snprintf(out, out_sz, "%s", path);
    size_t n = strlen(out);
    if (n > 3 && strcmp(out + n - 3, ".so") == 0) {
        out[n - 3] = '\0';
        const char* base = strrchr(out, '/');
        const char* base2 = strrchr(out, '\\');
        if (!base || (base2 && base2 > base)) base = base2;
        char* mutable_base = base ? (char*)base + 1 : out;
        if (strncmp(mutable_base, "lib", 3) == 0) {
            memmove(mutable_base, mutable_base + 3, strlen(mutable_base + 3) + 1);
        }
        strncat(out, ".dll", out_sz - strlen(out) - 1);
    }
}

static inline void* dlopen(const char* path, int flags) {
    (void)flags;
    if (!path) return NULL;
    HMODULE h = LoadLibraryA(path);
    if (!h) {
        char alt[512];
        flow_make_dll_candidate(path, alt, sizeof(alt));
        if (strcmp(alt, path) != 0) h = LoadLibraryA(alt);
    }
    if (!h) flow_set_dlerror_from_win("LoadLibraryA");
    return (void*)h;
}

static inline void* dlsym(void* handle, const char* symbol) {
    FARPROC p = GetProcAddress((HMODULE)handle, symbol);
    if (!p) flow_set_dlerror_from_win("GetProcAddress");
    return (void*)p;
}

static inline int dlclose(void* handle) {
    if (!handle) return 0;
    if (!FreeLibrary((HMODULE)handle)) {
        flow_set_dlerror_from_win("FreeLibrary");
        return -1;
    }
    return 0;
}

static inline char* dlerror(void) {
    return flow_dlerror_buf[0] ? flow_dlerror_buf : NULL;
}

#endif /* FLOWENGINE_COMPAT_WIN_DLFCN_H */
