import { RoboCup } from "@proto/message/input/RoboCup";
import { Overview } from "@proto/message/support/nusight/Overview";

import { hashType } from "../../shared/nuclearnet/hash_type";
import { NUClearNetPeerWithType } from "../../shared/nuclearnet/nuclearnet_client";

import { DecodedUDPMessage } from "./udp_side_channel_client";
import { UDPSideChannelClient } from "./udp_side_channel_client";
import { UDPSideChannelClientOptions } from "./udp_side_channel_client";

/** The fully qualified Protobuf type name for the Overview message. */
const OVERVIEW_TYPE = "message.support.nusight.Overview";

/** The peer type presented for connections made via the {@link RoboCupUDPClient}. */
export const ROBOCUP_UDP_PEER_TYPE: NUClearNetPeerWithType["type"] = "robocup-udp-peer";

export type RoboCupUDPClientOptions = UDPSideChannelClientOptions;

/**
 * A receive-only {@link NUClearNetClient} that listens on a UDP port for serialised RoboCup team
 * communication packets (i.e. the official RoboCup Standard Message format used to talk to and
 * about other teams' robots) and re-presents the fields they share with {@link Overview} as an
 * Overview packet, so they can be displayed using NUsight's existing Overview visualisation.
 *
 * This is intended to be pointed at the same port configured for team communication in the
 * `RobotCommunication` module (`send_port`/`receive_port` in `RobotCommunication.yaml`, which
 * defaults to `10000 + team_id`).
 */
export class RoboCupUDPClient extends UDPSideChannelClient {
  private readonly hash = hashType(OVERVIEW_TYPE);

  constructor(options: RoboCupUDPClientOptions) {
    super(options, ROBOCUP_UDP_PEER_TYPE);
  }

  static of(options: RoboCupUDPClientOptions): RoboCupUDPClient {
    return new RoboCupUDPClient(options);
  }

  /** Decode an incoming UDP datagram containing a serialised RoboCup message. */
  protected decode(payload: Buffer, address: string, port: number): DecodedUDPMessage | undefined {
    let robocup: RoboCup;
    try {
      robocup = RoboCup.fromBinary(new Uint8Array(payload));
    } catch (error) {
      console.warn(`RoboCup UDP side channel: failed to decode packet from ${address}:${port}`, error);
      return undefined;
    }

    const playerId = robocup.currentPose?.playerId ?? 0;
    const ballPosition = robocup.ball?.position;
    const ballCovariance = robocup.ball?.covariance;

    const overview = new Overview({
      timestamp: robocup.timestamp,
      robotId: playerId,
      robotPosition: robocup.currentPose?.position,
      robotPositionCovariance: robocup.currentPose?.covariance,
      walkCommand: robocup.walkCommand,
      kickTarget: robocup.kickTarget,
      ballPosition: ballPosition && { x: ballPosition.x, y: ballPosition.y },
      ballPositionCovariance: ballCovariance && {
        x: { x: ballCovariance.x?.x ?? 0, y: ballCovariance.x?.y ?? 0 },
        y: { x: ballCovariance.y?.x ?? 0, y: ballCovariance.y?.y ?? 0 },
      },
    });

    return {
      key: `${address}:${playerId}`,
      name: `robocup-${playerId}`,
      packets: [{ type: OVERVIEW_TYPE, hash: this.hash, payload: Buffer.from(overview.toBinary()) }],
    };
  }
}
