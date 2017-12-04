/* Protocol level parameter IDs. */

#pragma once

enum api_errno {
	ERRNO_EOK = 0,
	ERRNO_ENOENT = 2,
	ERRNO_EIO = 5,
	ERRNO_EBADF = 9,
	ERRNO_EINVAL = 22,
	ERRNO_ENOTSUP = 35,
};


enum api_irq_source {
	IRQ_SOURCE_INVALID = 0,
	IRQ_SOURCE_SOCKET_READY = 1,
};




enum api_command {
	COMMAND_CMD_INVALID = 0,
	COMMAND_DMA_WRITE = 193,
	COMMAND_DMA_READ = 194,
	COMMAND_SINGLE_WRITE = 201,
	COMMAND_SINGLE_READ = 202,
	COMMAND_RESET = 207,
	COMMAND_INTERNAL_WRITE = 195,
	COMMAND_INTERNAL_READ = 196,
	COMMAND_TERMINATE = 197,
	COMMAND_REPEAT = 198,
	COMMAND_DMA_EXT_WRITE = 199,
	COMMAND_DMA_EXT_READ = 200,
	COMMAND_MASK = 15,
};

enum api_data {
	DATA_DATA_INVALID = 0,
	DATA_START = 240,
	DATA_FIRST_PACKET = 1,
	DATA_MID_PACKET = 2,
	DATA_LAST_PACKET = 3,
};

enum api_response {
	RESPONSE_OK = 0,
	RESPONSE_UNSUPPORTED_COMMAND = 1,
	RESPONSE_UNEXPECTED_DATA_PACKET = 2,
	RESPONSE_CRC7_ERROR = 3,
	RESPONSE_CRC16_ERROR = 4,
	RESPONSE_GENERAL_ERROR = 5,
};

enum api_connect_status {
	CONNECT_STATUS_IDLE = 0,
	CONNECT_STATUS_CONNECTING = 1,
	CONNECT_STATUS_WRONG_PASSWORD = 2,
	CONNECT_STATUS_NO_AP_FOUND = 3,
	CONNECT_STATUS_CONNECT_FAIL = 4,
	CONNECT_STATUS_GOT_IP = 5,
};

enum api_phy_mode {
	PHY_MODE_INVALID = 0,
	PHY_MODE_M11B = 1,
	PHY_MODE_M11G = 2,
	PHY_MODE_M11N = 3,
};

enum api_family {
	FAMILY_UNSPEC = 0,
	FAMILY_INET = 2,
	FAMILY_INET6 = 10,
};

enum api_sock_type {
	SOCK_TYPE_SOCK_TYPE_INVALID = 0,
	SOCK_TYPE_STREAM = 1,
	SOCK_TYPE_DGRAM = 2,
};

enum api_ip_protocol {
	IP_PROTOCOL_INVALID = 0,
	IP_PROTOCOL_ICMP = 1,
	IP_PROTOCOL_TCP = 6,
	IP_PROTOCOL_UDP = 17,
	IP_PROTOCOL_ICMPV6 = 58,
};

enum api_socket_event {
	SOCKET_EVENT_NONE = 0,
	SOCKET_EVENT_CONNECTED = 1,
	SOCKET_EVENT_READ = 2,
	SOCKET_EVENT_WRITE = 4,
	SOCKET_EVENT_EXCEPT = 8,
	SOCKET_EVENT_CLOSED = 16,
};




enum api_msg_id {
	/* API */
	/* Link */
	API_LINK_PROTOCOL = 101,
	API_LINK_MAGIC = 102,
	API_LINK_ERR = 103,
	API_LINK_STATUS = 105,
	API_LINK_IRQ_MASK = 106,

	/* System */
	API_SYSTEM_API = 201,
	API_SYSTEM_SDK = 202,
	API_SYSTEM_SW = 203,
	API_SYSTEM_HW = 204,
	API_SYSTEM_CHIP_ID = 205,
	API_SYSTEM_UPTIME = 206,
	API_SYSTEM_FREE_HEAP = 207,

	/* Wifi */
	API_WIFI_SSID = 302,
	API_WIFI_SSID_LEN = 303,
	API_WIFI_PASSWORD = 304,
	API_WIFI_MAC_ADDRESS = 306,
	API_WIFI_MAC_ADDRESS_LEN = 307,
	API_WIFI_CONNECTED = 308,
	API_WIFI_CONNECT_STATUS = 309,
	API_WIFI_RSSI = 310,
	API_WIFI_PHY_MODE = 311,
	API_WIFI_IP = 312,
	API_WIFI_IP_LEN = 313,

	/* GPIO */
	API_GPIO_DIRECTION = 401,
	API_GPIO_PINS = 402,

	/* Sockets */
	API_SOCKETS_ERR = 1001,
	API_SOCKETS_EVENT_FDS = 1002,
	API_SOCKETS_GET = 1010,

	/* Socket */
	API_SOCKET_STRIDE = 32,
	API_SOCKET_ERR = 1101,
	API_SOCKET_FD = 1102,
	API_SOCKET_EVENTS = 1103,
	API_SOCKET_BIND = 1110,
	API_SOCKET_LISTEN = 1111,
	API_SOCKET_CONNECT = 1112,
	API_SOCKET_ACCEPT = 1113,
	API_SOCKET_PUT = 1114,
	API_SOCKET_RECV_AVAIL = 1115,
	API_SOCKET_RECV = 1116,
	API_SOCKET_SEND = 1117,
	API_SOCKET_SENDTO = 1118,


};
