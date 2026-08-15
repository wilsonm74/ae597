/*
 * leaderPositionComm.h
 *
 * Lets the leader (SPHERE1) broadcast its current position over the
 * sphere-to-sphere (STS) RF channel, and lets the viewer (SPHERE2) receive
 * and cache the most recent value, for use with calculateTrackingError().
 *
 * Verified against the project's comm.h / commands.h:
 *   - default_rfm_packet is a raw byte array (default_comm_packet =
 *     unsigned char[COMM_PACKET_SIZE]), indexed via the PKT_* offsets
 *     defined in commands.h.
 *   - Packets are sent with the commSendPacket(...) macro (comm.h), which
 *     wraps commSendRFMPacket().
 *   - COMM_CMD_GSP_PACKET (commands.h) is the generic command code meant
 *     for custom GSP-defined payloads like this one.
 */

#ifndef LEADER_POSITION_COMM_H
#define LEADER_POSITION_COMM_H

#include "comm.h"

// Call once per control cycle from the LEADER (SPHERE1) to broadcast its
// current [X,Y,Z] position to the other vehicle(s).
void leaderPositionBroadcast(const float leaderPos[3]);

// Call from gspProcessRXData() (on all vehicles) to let this module inspect
// an incoming packet. If it's a leader-position packet, the cached leader
// position is updated.
void leaderPositionProcessPacket(default_rfm_packet packet);

// Call from the VIEWER (SPHERE2) to fetch the most recently received
// leader position.
//   Returns 1 and fills in leaderPos, if at least one broadcast has been
//            received since program start.
//   Returns 0 and leaves leaderPos untouched, if nothing has been received
//            yet (e.g. very start of the test, before SPHERE1's first
//            broadcast has arrived).
unsigned char leaderPositionGet(float leaderPos[3]);

#endif /* LEADER_POSITION_COMM_H */
