#ifndef FLOWENGINE_COMPAT_WIN_SYS_UN_H
#define FLOWENGINE_COMPAT_WIN_SYS_UN_H

#include <winsock2.h>

#ifndef AF_UNIX
#define AF_UNIX AF_INET
#endif

struct sockaddr_un {
    ADDRESS_FAMILY sun_family;
    char sun_path[108];
};

#endif /* FLOWENGINE_COMPAT_WIN_SYS_UN_H */
