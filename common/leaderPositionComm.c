/*
 * leaderPositionComm.c
 *
 * See leaderPositionComm.h.
 */

#include "comm.h"
#include "commands.h"
#include "system.h"
#include "leaderPositionComm.h"
#include <string.h>

// COMM_CMD_GSP_PACKET (commands.h) is the generic command code meant for
// custom GSP-defined payloads - exactly this kind of use case.
#define LEADER_POS_CMD COMM_CMD_GSP_PACKET

static float lastLeaderPos[3] = {0.0f, 0.0f, 0.0f};
static unsigned char leaderPosReceived = 0;

void leaderPositionBroadcast(const float leaderPos[3])
{
	default_comm_payload payload;

	memset(payload, 0, sizeof(payload));
	memcpy(payload, leaderPos, sizeof(float) * 3);

	// commSendPacket(channel, to, from, command, source, priority)
	// -> expands to commSendRFMPacket(channel, to, command, source, priority)
	commSendPacket(COMM_CHANNEL_STS, BROADCAST_HWID, sysIdentityGet(),
				   LEADER_POS_CMD, payload, (COMM_LOW_PRIORITY | COMM_NO_ACK));
}

void leaderPositionProcessPacket(default_rfm_packet packet)
{
	if (packet[PKT_CM] == LEADER_POS_CMD) {
		memcpy(lastLeaderPos, &packet[PKT_DATA], sizeof(float) * 3);
		leaderPosReceived = 1;
	}
}

unsigned char leaderPositionGet(float leaderPos[3])
{
	if (!leaderPosReceived) {
		return 0;
	}
	leaderPos[0] = lastLeaderPos[0];
	leaderPos[1] = lastLeaderPos[1];
	leaderPos[2] = lastLeaderPos[2];
	return 1;
}
