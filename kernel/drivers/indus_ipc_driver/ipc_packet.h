#ifndef __IPC_PACKET_H
#define __IPC_PACKET_H

enum hostId
{
	IPC_HOST_DOMX,
	IPC_HOST_PVSMC,
	IPC_HOST_DOMI,
	IPC_HOST_PVSMC2,
	IPC_HOST_LAST = 4,
};

struct ipc_message
{
	unsigned int destinationId;
	struct ipc_payload
	{
		unsigned int	msgId;
		unsigned char	sourceId;
		unsigned char	destinationId;
		unsigned char	portId;
		unsigned char	payloadsize;
		unsigned char	payload[48];
		unsigned char	CRC[4];
		unsigned char	rsrd[4];
	} ipc_payload;
};

#endif /* __IPC_PACKET_H */
