/*
 * Copyright (c) 2017 IpTronix
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(wifi_winc1500, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <errno.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_context.h>

/* We need to define these before including HAL to avoid conflicts */
#define __SOCKET_H__
#define HOSTNAME_MAX_SIZE (64)

/* Include basic HAL types */
#include <bsp/include/nm_bsp.h>
#include <common/include/nm_common.h>
#include <socket/include/m2m_socket_host_if.h>

#include "winc1500_socket.h"

/* Socket data structure (redefined here to avoid circular dependencies) */
struct socket_data {
	struct net_context *context;
	net_context_connect_cb_t connect_cb;
	net_tcp_accept_cb_t accept_cb;
	net_context_recv_cb_t recv_cb;
	void *connect_user_data;
	void *recv_user_data;
	void *accept_user_data;
	struct net_pkt *rx_pkt;
	struct net_buf *pkt_buf;
	int ret_code;
	struct k_sem wait_sem;
	bool is_offload_socket; /* true si socket offload, false si net_context */
};

/* Global data structure (redefined here) */
struct winc1500_data {
	struct socket_data socket_data[CONFIG_WIFI_WINC1500_OFFLOAD_MAX_SOCKETS];
	struct net_if *iface;
	unsigned char mac[6];
	void *scan_cb; /* scan_result_cb_t */
	uint8_t scan_result;
	bool connecting;
	bool connected;
};

/* External reference to global data from main driver */
extern struct winc1500_data w1500_data;

/* Hardware API function declarations (mapped to HAL functions) */
NMI_API void socketInit(void);
typedef void (*tpfAppSocketCb)(SOCKET sock, uint8 u8Msg, void *pvMsg);
typedef void (*tpfAppResolveCb)(uint8 *pu8DomainName, uint32 u32ServerIP);
NMI_API void registerSocketCallback(tpfAppSocketCb socket_cb, tpfAppResolveCb resolve_cb);
NMI_API SOCKET winc1500_socket(uint16 u16Domain, uint8 u8Type, uint8 u8Flags);
NMI_API sint8 winc1500_socket_bind(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
NMI_API sint8 winc1500_socket_listen(SOCKET sock, uint8 backlog);
NMI_API sint8 winc1500_socket_accept(SOCKET sock, struct sockaddr *addr, uint8 *addrlen);
NMI_API sint8 winc1500_socket_connect(SOCKET sock, struct sockaddr *pstrAddr, uint8 u8AddrLen);
NMI_API sint16 winc1500_socket_recv(SOCKET sock, void *pvRecvBuf, uint16 u16BufLen,
				    uint32 u32Timeoutmsec);
NMI_API sint16 winc1500_socket_send(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength,
				    uint16 u16Flags);
NMI_API sint16 winc1500_socket_sendto(SOCKET sock, void *pvSendBuffer, uint16 u16SendLength,
				      uint16 flags, struct sockaddr *pstrDestAddr, uint8 u8AddrLen);
NMI_API sint8 winc1500_close(SOCKET sock);

int __winc1500_socket_new(sa_family_t family, enum net_sock_type type,
			  enum net_ip_protocol ip_proto, struct net_context **context)
{
	struct socket_data *sd;
	SOCKET sock;

	if (family != AF_INET) {
		LOG_ERR("Only AF_INET is supported!");
		return -EAFNOSUPPORT;
	}

	/* winc1500 atmel uses AF_INET 2 instead of zephyrs AF_INET 1
	 * we have checked if family is AF_INET so we can hardcode this
	 * for now.
	 */
	sock = winc1500_socket(2, type, 0);
	if (sock < 0) {
		LOG_ERR("socket error!");
		return -ENOMEM;
	}

	if (context) {
		(*context)->offload_context = (void *)(intptr_t)sock;
	}
	sd = &w1500_data.socket_data[sock];

	k_sem_init(&sd->wait_sem, 0, 1);
	if (context) {
		sd->context = *context;
		sd->is_offload_socket = false; /* Socket créé via net_context API */
	} else {
		sd->is_offload_socket = true; /* Socket créé via socket offload API */
	}

	return sock;
}

int __winc1500_bind(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	int ret;

	/* FIXME atmel winc1500 don't support bind on null port */
	if (net_sin(addr)->sin_port == 0U) {
		return 0;
	}

	ret = winc1500_socket_bind(sock, (struct sockaddr *)addr, addrlen);
	if (ret) {
		LOG_ERR("bind error %d!", ret);
		return ret;
	}

	if (k_sem_take(&w1500_data.socket_data[sock].wait_sem, WINC1500_BIND_TIMEOUT)) {
		LOG_ERR("bind error timeout expired");
		return -ETIMEDOUT;
	}

	return w1500_data.socket_data[sock].ret_code;
}

int __winc1500_listen(int sock, int backlog)
{
	int ret;

	ret = winc1500_socket_listen(sock, backlog);
	if (ret) {
		LOG_ERR("listen error %d!", ret);
		return ret;
	}

	if (k_sem_take(&w1500_data.socket_data[sock].wait_sem, WINC1500_LISTEN_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	return w1500_data.socket_data[sock].ret_code;
}

int __winc1500_connect(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	int ret;

	ret = winc1500_socket_connect(sock, (struct sockaddr *)addr, addrlen);
	if (ret) {
		LOG_ERR("connect error %d!", ret);
		return ret;
	}

	return 0;
}

int __winc1500_accept(int sock)
{
	int ret;

	ret = winc1500_socket_accept(sock, NULL, 0);
	if (ret) {
		LOG_ERR("accept error %d!", ret);
		return ret;
	}

	return w1500_data.socket_data[sock].ret_code;
}

int __winc1500_send(int sock, const void *buf, size_t len, int flags)
{
	int ret;

	ret = winc1500_socket_send(sock, (void *)buf, len, flags);
	if (ret) {
		LOG_ERR("send error %d!", ret);
		return ret;
	}

	return ret;
}

int __winc1500_sendto(int sock, const void *buf, size_t len, int flags,
		      const struct sockaddr *dest_addr, socklen_t addrlen)
{
	int ret;

	ret = winc1500_socket_sendto(sock, (void *)buf, len, flags, (struct sockaddr *)dest_addr,
				     addrlen);
	if (ret) {
		LOG_ERR("sendto error %d!", ret);
		return ret;
	}

	return ret;
}

int __winc1500_recv(int sock, void *buf, size_t max_len, int32_t timeout)
{
	int ret;

	ret = winc1500_socket_recv(sock, buf, max_len, timeout);
	if (ret) {
		LOG_ERR("recv error %d!", ret);
		return ret;
	}

	return ret;
}

int __winc1500_close(int sock)
{
	int ret;
	struct socket_data *sd = &w1500_data.socket_data[sock];

	ret = winc1500_close(sock);

	if (sd->rx_pkt) {
		net_pkt_unref(sd->rx_pkt);
	}

	memset(sd, 0, sizeof(struct socket_data));

	return ret;
}