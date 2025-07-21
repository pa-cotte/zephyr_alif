# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_WINC1500)
  zephyr_include_directories(./)

  zephyr_library_sources(
    wifi_winc1500_nm_bsp.c
    wifi_winc1500_nm_bus_wrapper.c
    wifi_winc1500.c
    winc1500_socket.c
    )
  
  # Socket offload implementation
  zephyr_library_sources_ifdef(CONFIG_WIFI_WINC1500_SOCKET_OFFLOAD wifi_winc1500_socket_offload.c)
endif()
