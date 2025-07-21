#ifndef ZEPHYR_DRIVERS_WIFI_WINC1500_WIFI_WINC1500_SOCKET_OFFLOAD_H_
#define ZEPHYR_DRIVERS_WIFI_WINC1500_WIFI_WINC1500_SOCKET_OFFLOAD_H_

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_offload.h>

enum winc1500_socket_state {
	WINC1500_SOCKET_STATE_NONE,
	WINC1500_SOCKET_STATE_CONNECTING,
	WINC1500_SOCKET_STATE_CONNECTED,
	WINC1500_SOCKET_STATE_ACCEPTING,
	WINC1500_SOCKET_STATE_LISTENING,
};

struct winc1500_off_socket {
	int winc_sock;
	enum winc1500_socket_state state;
	struct sockaddr peer_addr;
	struct sockaddr local_addr;
	uint16_t local_port;
	bool is_server;
	int usage;
	int family;  /* Famille d'adresses (AF_INET) */
	int type;    /* Type de socket (SOCK_STREAM, SOCK_DGRAM) */
	int proto;   /* Protocole (IPPROTO_TCP, IPPROTO_UDP) */
	struct k_sem read_sem;
	struct k_sem accept_sem;
	struct k_fifo recv_fifo;
	struct k_work connect_work;
	struct k_work_delayable read_work;
};

#define SD_TO_OBJ(sd) ((struct winc1500_off_socket *)(sd))
#define OBJ_TO_SD(obj) ((int)(obj))

int winc1500_socket_offload_init(void);
int winc1500_socket_off_create(int family, int type, int proto);

#endif /* ZEPHYR_DRIVERS_WIFI_WINC1500_WIFI_WINC1500_SOCKET_OFFLOAD_H_ */