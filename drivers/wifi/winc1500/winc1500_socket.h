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

#endif /* ZEPHYR_DRIVERS_WIFI_WINC1500_WINC1500_SOCKET_H_ */