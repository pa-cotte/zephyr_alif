/**
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WIFI_WINC1500_NATIVE_H_
#define WIFI_WINC1500_NATIVE_H_

/* Include WINC1500 headers from the HAL module */
#include <driver/include/m2m_types.h>
#include <socket/include/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Wrapper functions with unique names to avoid conflicts with Zephyr socket API */
static inline void winc1500_socketInit(void) { 
    socketInit(); 
}

static inline void winc1500_registerSocketCallback(tpfAppSocketCb socket_cb, tpfAppResolveCb resolve_cb) { 
    registerSocketCallback(socket_cb, resolve_cb); 
}

static inline SOCKET winc1500_socket_native(uint16 u16Domain, uint8 u8Type, uint8 u8Flags) { 
    return socket(u16Domain, u8Type, u8Flags); 
}

static inline sint8 winc1500_bind_native(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen) { 
    return bind(sock, pstrAddr, u8AddrLen); 
}

static inline sint8 winc1500_listen_native(SOCKET sock, uint8 backlog) { 
    return listen(sock, backlog); 
}

static inline sint8 winc1500_accept_native(SOCKET sock, struct sockaddr *addr, uint8 *addrlen) { 
    return accept(sock, addr, addrlen); 
}

static inline sint8 winc1500_connect_native(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen) { 
    return connect(sock, pstrAddr, u8AddrLen); 
}

static inline sint16 winc1500_recv_native(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec) { 
    return recv(sock, pvRecvBuf, u16BufLen, u32Timeoutmsec); 
}

static inline sint16 winc1500_send_native(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 u16Flags) { 
    return send(sock, pvSendBuffer, u16SendLength, u16Flags); 
}

static inline sint16 winc1500_sendto_native(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 flags, struct sockaddr *pstrDestAddr, uint8 u8AddrLen) { 
    return sendto(sock, pvSendBuffer, u16SendLength, flags, pstrDestAddr, u8AddrLen); 
}

static inline sint8 winc1500_close_native(SOCKET sock) { 
    return winc1500_close(sock); 
}

static inline sint8 winc1500_gethostbyname_native(uint8 *pu8HostName) { 
    return gethostbyname(pu8HostName); 
}

#ifdef __cplusplus
}
#endif

#endif /* WIFI_WINC1500_NATIVE_H_ */