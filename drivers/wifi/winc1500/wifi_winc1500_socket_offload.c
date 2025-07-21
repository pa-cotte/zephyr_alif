#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(winc1500_socket_offload, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_offload.h>
#include <zephyr/net/net_pkt.h>
#include <errno.h>

#include "../../../subsys/net/lib/sockets/sockets_internal.h"
#include "wifi_winc1500_socket_offload.h"
#include "winc1500_socket.h"

static struct winc1500_off_socket winc1500_sockets[CONFIG_WIFI_WINC1500_OFFLOAD_MAX_SOCKETS];
static const struct socket_op_vtable winc1500_socket_fd_op_vtable;

static bool winc1500_socket_is_supported(int family, int type, int proto)
{
	if (family != AF_INET) {
		return false;
	}

	if (type != SOCK_STREAM && type != SOCK_DGRAM) {
		return false;
	}

	return true;
}

int winc1500_socket_off_create(int family, int type, int proto)
{
	int fd;
	struct winc1500_off_socket *sock;
	int winc_sock;

	fd = z_reserve_fd();
	if (fd < 0) {
		return -1;
	}

	sock = &winc1500_sockets[fd];
	memset(sock, 0, sizeof(*sock));
	
	/* Stocker les paramètres du socket */
	sock->family = family;
	sock->type = type;
	sock->proto = proto;
	sock->state = WINC1500_SOCKET_STATE_NONE;
	sock->usage = 1;

	/* Créer immédiatement le socket WINC1500 natif avec les bons paramètres */
	winc_sock = __winc1500_socket_new(family, type, proto, NULL);
	if (winc_sock < 0) {
		LOG_ERR("Failed to create WINC1500 native socket: %d", winc_sock);
		return -1;
	}
	
	sock->winc_sock = winc_sock;
	LOG_DBG("Created WINC1500 socket offload: fd=%d, winc_sock=%d, family=%d, type=%d, proto=%d", 
		fd, winc_sock, family, type, proto);

	z_finalize_fd(fd, sock, (const struct fd_op_vtable *)&winc1500_socket_fd_op_vtable);

	return fd;
}

static int winc1500_socket_off_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct winc1500_off_socket *sock = (struct winc1500_off_socket *)obj;
	const struct sockaddr_in *addr_in = (const struct sockaddr_in *)addr;
	int ret;

	if (!addr || addrlen < sizeof(struct sockaddr_in)) {
		LOG_ERR("Invalid bind address");
		return -EINVAL;
	}

	if (addr->sa_family != AF_INET) {
		LOG_ERR("Only AF_INET family supported");
		return -EAFNOSUPPORT;
	}

	/* Le socket WINC1500 natif a déjà été créé dans winc1500_socket_off_create */
	if (sock->winc_sock == -1) {
		LOG_ERR("WINC1500 native socket should have been created during socket creation");
		return -EINVAL;
	}

	/* Utiliser la fonction d'abstraction existante */
	ret = __winc1500_bind(sock->winc_sock, addr, addrlen);
	if (ret < 0) {
		LOG_ERR("__winc1500_bind failed: %d", ret);
		return ret;
	}

	/* Sauvegarder l'adresse locale liée */
	memcpy(&sock->local_addr, addr, addrlen);
	sock->local_port = ntohs(addr_in->sin_port);

	LOG_DBG("WINC1500 socket offload bound to port %d", sock->local_port);
	return 0;
}

static int winc1500_socket_off_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct winc1500_off_socket *sock = (struct winc1500_off_socket *)obj;
	const struct sockaddr_in *addr_in = (const struct sockaddr_in *)addr;
	int ret;

	if (!addr || addrlen < sizeof(struct sockaddr_in)) {
		LOG_ERR("Invalid connect address");
		return -EINVAL;
	}

	if (addr->sa_family != AF_INET) {
		LOG_ERR("Only AF_INET family supported");
		return -EAFNOSUPPORT;
	}

	/* Le socket WINC1500 natif a déjà été créé dans winc1500_socket_off_create */
	if (sock->winc_sock == -1) {
		LOG_ERR("WINC1500 native socket should have been created during socket creation");
		return -EINVAL;
	}

	/* Utiliser la fonction d'abstraction existante */
	ret = __winc1500_connect(sock->winc_sock, addr, addrlen);
	if (ret < 0) {
		LOG_ERR("__winc1500_connect failed: %d", ret);
		return ret;
	}

	/* Sauvegarder l'adresse distante connectée */
	memcpy(&sock->peer_addr, addr, addrlen);
	sock->state = WINC1500_SOCKET_STATE_CONNECTED;

	LOG_DBG("WINC1500 socket offload connected to %d.%d.%d.%d:%d",
		addr_in->sin_addr.s_addr & 0xFF, (addr_in->sin_addr.s_addr >> 8) & 0xFF,
		(addr_in->sin_addr.s_addr >> 16) & 0xFF, (addr_in->sin_addr.s_addr >> 24) & 0xFF,
		ntohs(addr_in->sin_port));

	return 0;
}

static int winc1500_socket_off_listen(void *obj, int backlog)
{
	return -ENOTSUP;
}

static int winc1500_socket_off_accept(void *obj, struct sockaddr *addr, socklen_t *addrlen)
{
	return -ENOTSUP;
}

static ssize_t winc1500_socket_off_sendto(void *obj, const void *buf, size_t len, int flags,
					  const struct sockaddr *dest_addr, socklen_t addrlen)
{
	struct winc1500_off_socket *sock = (struct winc1500_off_socket *)obj;
	int ret;

	if (!buf || len == 0) {
		LOG_ERR("Invalid send parameters");
		return -EINVAL;
	}

	LOG_DBG("sendto called: len=%zu, winc_sock=%d, state=%d", len, sock->winc_sock, sock->state);

	/* Le socket WINC1500 natif a déjà été créé dans winc1500_socket_off_create */
	if (sock->winc_sock == -1) {
		LOG_ERR("WINC1500 native socket should have been created during socket creation");
		return -EINVAL;
	}

	/* Utiliser la fonction d'abstraction existante */
	if (dest_addr && addrlen > 0) {
		/* sendto avec adresse de destination */
		ret = __winc1500_sendto(sock->winc_sock, buf, len, flags, dest_addr, addrlen);
	} else {
		/* send simple (socket connecté) */
		ret = __winc1500_send(sock->winc_sock, buf, len, flags);
	}

	if (ret < 0) {
		LOG_ERR("__winc1500_send/sendto failed: %d", ret);
		return ret;
	}

	/* Les fonctions WINC1500 retournent 0 en cas de succès (style code d'erreur),
	 * mais l'API socket attend le nombre d'octets envoyés */
	if (ret == 0) {
		LOG_DBG("WINC1500 socket offload sent %zu bytes (winc_sock=%d)", len, sock->winc_sock);
		return len;  /* Retourner le nombre d'octets demandés */
	}

	LOG_DBG("WINC1500 socket offload sent %d bytes", ret);
	return ret;
}

static ssize_t winc1500_socket_off_recvfrom(void *obj, void *buf, size_t max_len, int flags,
					    struct sockaddr *src_addr, socklen_t *addrlen)
{
	return -ENOTSUP;
}

static int winc1500_socket_off_setsockopt(void *obj, int level, int optname, const void *optval,
					  socklen_t optlen)
{
	struct winc1500_off_socket *sock = (struct winc1500_off_socket *)obj;

	ARG_UNUSED(sock);

	if (level == SOL_SOCKET) {
		switch (optname) {
		case SO_BINDTODEVICE:
			/* SO_BINDTODEVICE is handled by the dispatcher normally,
			 * but if we get here, just return success as the socket
			 * is already bound to the WINC1500 interface */
			LOG_DBG("SO_BINDTODEVICE on WINC1500 socket - already bound");
			return 0;

		case SO_REUSEADDR:
			/* WINC1500 doesn't have explicit reuse addr support,
			 * but we can safely ignore it */
			LOG_DBG("SO_REUSEADDR on WINC1500 socket - ignored");
			return 0;

		default:
			LOG_DBG("Unsupported socket option: level=%d, optname=%d", level, optname);
			break;
		}
	}

	return -ENOTSUP;
}

static ssize_t winc1500_socket_off_read(void *obj, void *buffer, size_t count)
{
	return winc1500_socket_off_recvfrom(obj, buffer, count, 0, NULL, 0);
}

static ssize_t winc1500_socket_off_write(void *obj, const void *buffer, size_t count)
{
	return winc1500_socket_off_sendto(obj, buffer, count, 0, NULL, 0);
}

static int winc1500_socket_off_close(void *obj)
{
	struct winc1500_off_socket *sock = (struct winc1500_off_socket *)obj;

	if (sock->usage > 0) {
		sock->usage--;
	}

	if (sock->usage == 0) {
		/* Fermer le socket WINC1500 natif */
		if (sock->winc_sock != -1) {
			LOG_DBG("Closing WINC1500 native socket %d", sock->winc_sock);
			__winc1500_close(sock->winc_sock);
			sock->winc_sock = -1;
		}
		sock->state = WINC1500_SOCKET_STATE_NONE;
	}

	return 0;
}

static int winc1500_socket_off_ioctl(void *obj, unsigned int request, va_list args)
{
	return -ENOTSUP;
}

static const struct socket_op_vtable winc1500_socket_fd_op_vtable = {
	.fd_vtable =
		{
			.read = winc1500_socket_off_read,
			.write = winc1500_socket_off_write,
			.close = winc1500_socket_off_close,
			.ioctl = winc1500_socket_off_ioctl,
		},
	.bind = winc1500_socket_off_bind,
	.connect = winc1500_socket_off_connect,
	.listen = winc1500_socket_off_listen,
	.accept = winc1500_socket_off_accept,
	.sendto = winc1500_socket_off_sendto,
	.recvfrom = winc1500_socket_off_recvfrom,
	.setsockopt = winc1500_socket_off_setsockopt,
};

#ifdef CONFIG_NET_SOCKETS_OFFLOAD
NET_SOCKET_OFFLOAD_REGISTER(winc1500, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, AF_UNSPEC,
			    winc1500_socket_is_supported, winc1500_socket_off_create);
#endif

int winc1500_socket_offload_init(void)
{
	LOG_DBG("WINC1500 socket offload initialized");
	return 0;
}
