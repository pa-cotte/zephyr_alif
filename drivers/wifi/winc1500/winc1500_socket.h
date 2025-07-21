/*
 * Copyright (c) 2017 IpTronix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WIFI_WINC1500_WINC1500_SOCKET_H_
#define ZEPHYR_DRIVERS_WIFI_WINC1500_WINC1500_SOCKET_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_pkt.h>

/* Forward declarations */
struct net_context;
struct sockaddr;

/* Include HAL types (SOCKET is defined as sint8 in HAL) */
#ifndef __SOCKET_H__
/* Forward declaration if socket.h not included */
typedef signed char SOCKET;
#endif

/* Timeouts from main driver */
#define WINC1500_BIND_TIMEOUT   K_MSEC(500)
#define WINC1500_LISTEN_TIMEOUT K_MSEC(500)
#define WINC1500_BUF_TIMEOUT    K_MSEC(100)

/* Forward declarations of structures defined in main driver */
struct socket_data;
struct winc1500_data;

/* Hardware API functions (from HAL) - declared in main driver */

/* Common socket functions */
int __winc1500_socket_new(sa_family_t family, enum net_sock_type type, 
			  enum net_ip_protocol ip_proto, struct net_context **context);
int __winc1500_bind(int sock, const struct sockaddr *addr, socklen_t addrlen);
int __winc1500_listen(int sock, int backlog);
int __winc1500_connect(int sock, const struct sockaddr *addr, socklen_t addrlen);
int __winc1500_accept(int sock);
int __winc1500_send(int sock, const void *buf, size_t len, int flags);
int __winc1500_sendto(int sock, const void *buf, size_t len, int flags,
		      const struct sockaddr *dest_addr, socklen_t addrlen);
int __winc1500_recv(int sock, void *buf, size_t max_len, int32_t timeout);
int __winc1500_close(int sock);

/* Wrappers pour compatibilité avec l'ancien code */
/* Les types HAL ne sont disponibles que si les headers HAL sont inclus */
#ifdef NM_BSP_H
/* HAL types already included */
SOCKET winc1500_socket(uint16 u16Domain, uint8 u8Type, uint8 u8Flags);
sint8 winc1500_socket_bind(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
sint8 winc1500_socket_listen(SOCKET sock, uint8 backlog);
sint8 winc1500_socket_accept(SOCKET sock, struct sockaddr *addr, uint8 *addrlen);
sint8 winc1500_socket_connect(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
sint16 winc1500_socket_recv(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen, uint32 u32Timeoutmsec);
sint16 winc1500_socket_send(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength, uint16 u16Flags);
sint16 winc1500_socket_sendto(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength,
			      uint16 flags, struct sockaddr *pstrDestAddr, uint8 u8AddrLen);
#endif /* NM_BSP_H */

#endif /* ZEPHYR_DRIVERS_WIFI_WINC1500_WINC1500_SOCKET_H_ */