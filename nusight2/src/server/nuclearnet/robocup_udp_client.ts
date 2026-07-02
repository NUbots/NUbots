import dgram from "node:dgram";

import { RoboCup } from "@proto/message/input/RoboCup";
import { NUClearNetOptions } from "nuclearnet.js";
import { NUClearNetPacket } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";

import { hashType } from "../../shared/nuclearnet/hash_type";
import { NUClearEventListener } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearNetPeerWithType } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearPacketListener } from "../../shared/nuclearnet/nuclearnet_client";

import { parseEventString } from "./parse_event_string";

/** The fully qualified Protobuf type name for the RoboCup message. */
const ROBOCUP_TYPE = "message.input.RoboCup";

/** The peer type presented for connections made via the {@link RoboCupUDPClient}. */
export const ROBOCUP_UDP_PEER_TYPE: NUClearNetPeerWithType["type"] = "robocup-udp-peer";

export interface RoboCupUDPClientOptions {
  /** The UDP port to listen on for serialised RoboCup packets. */
  port: number;
  /**
   * The list of source IP addresses to accept RoboCup packets from. When empty or omitted,
   * packets from any address are accepted.
   */
  allowedAddresses?: string[];
  /**
   * The number of milliseconds to wait for a new RoboCup packet from a source before
   * considering that robot to have left the network. Defaults to 5000ms.
   */
  timeout?: number;
}

interface RoboCupPeer {
  peer: NUClearNetPeerWithType;
  timer: NodeJS.Timeout;
}

/**
 * A receive-only {@link NUClearNetClient} that listens on a UDP port for serialised RoboCup team
 * communication packets (i.e. the official RoboCup Standard Message format used to talk to and
 * about other teams' robots). Each unique sender is presented as its own peer (tagged as
 * {@link ROBOCUP_UDP_PEER_TYPE}, distinct from regular NUClearNet peers) that delivers the
 * RoboCup packets it sends.
 *
 * This is intended to be pointed at the same port configured for team communication in the
 * `RobotCommunication` module (`send_port`/`receive_port` in `RobotCommunication.yaml`, which
 * defaults to `10000 + team_id`).
 */
export class RoboCupUDPClient implements NUClearNetClient {
  private readonly hash = hashType(ROBOCUP_TYPE);
  private readonly allowedAddresses: Set<string>;
  private readonly timeout: number;

  /** The peers we have seen, keyed by a stable identity of `${address}:${playerId}`. */
  private readonly peers = new Map<string, RoboCupPeer>();

  /** Listeners registered for peers joining the network. */
  private readonly joinListeners = new Set<NUClearEventListener>();

  /** Listeners registered for peers leaving the network. */
  private readonly leaveListeners = new Set<NUClearEventListener>();

  /** Packet listeners, keyed by the event type they are listening for. */
  private readonly packetListeners = new Map<string, Set<NUClearPacketListener>>();

  private socket?: dgram.Socket;

  constructor(private readonly options: RoboCupUDPClientOptions) {
    this.allowedAddresses = new Set(options.allowedAddresses ?? []);
    this.timeout = options.timeout ?? 5000;
  }

  static of(options: RoboCupUDPClientOptions): RoboCupUDPClient {
    return new RoboCupUDPClient(options);
  }

  connect(_options: NUClearNetOptions): () => void {
    const socket = dgram.createSocket({ type: "udp4", reuseAddr: true });
    this.socket = socket;

    socket.on("message", (message, remote) => this.onMessage(message, remote.address, remote.port));
    socket.on("error", (error) => {
      console.error(`RoboCup UDP side channel error:`, error);
    });
    socket.bind(this.options.port, () => {
      console.info(`RoboCup UDP side channel listening on port ${this.options.port}`);
    });

    return () => {
      // Signal that all side channel peers have left
      for (const { peer, timer } of this.peers.values()) {
        clearTimeout(timer);
        for (const cb of this.leaveListeners) {
          cb(peer);
        }
      }
      this.peers.clear();

      socket.close();
      this.socket = undefined;
    };
  }

  onJoin(cb: NUClearEventListener): () => void {
    this.joinListeners.add(cb);
    return () => this.joinListeners.delete(cb);
  }

  onLeave(cb: NUClearEventListener): () => void {
    this.leaveListeners.add(cb);
    return () => this.leaveListeners.delete(cb);
  }

  on(event: string, cb: NUClearPacketListener): () => void {
    const { type } = parseEventString(event, { throwOnInvalidSubtype: false });
    let listeners = this.packetListeners.get(type);
    if (!listeners) {
      listeners = new Set();
      this.packetListeners.set(type, listeners);
    }
    listeners.add(cb);
    return () => listeners!.delete(cb);
  }

  onPacket(cb: NUClearPacketListener): () => void {
    return this.on("nuclear_packet", cb);
  }

  send(_options: NUClearNetSend): void {
    // The RoboCup side channel is receive-only, so sending is a no-op.
  }

  /** Handle an incoming UDP datagram containing a serialised RoboCup message. */
  private onMessage(payload: Buffer, address: string, port: number): void {
    // Filter out packets from addresses that aren't allowed, if a filter is configured
    if (this.allowedAddresses.size > 0 && !this.allowedAddresses.has(address)) {
      return;
    }

    // Decode enough of the RoboCup message to identify the sending robot
    let playerId = 0;
    try {
      const robocup = RoboCup.fromBinary(new Uint8Array(payload));
      playerId = robocup.currentPose?.playerId ?? 0;
    } catch (error) {
      console.warn(`RoboCup UDP side channel: failed to decode packet from ${address}:${port}`, error);
      return;
    }

    const key = `${address}:${playerId}`;
    const name = `robocup-${playerId}`;
    const peer: NUClearNetPeerWithType = { name, address, port, type: ROBOCUP_UDP_PEER_TYPE };

    // Announce the peer joining the first time we see it
    const existing = this.peers.get(key);
    if (existing === undefined) {
      for (const cb of this.joinListeners) {
        cb(peer);
      }
    } else {
      clearTimeout(existing.timer);
    }

    // (Re)arm the watchdog that removes the peer when it stops sending
    const timer = setTimeout(() => this.onPeerTimeout(key), this.timeout);
    this.peers.set(key, { peer, timer });

    // Deliver the packet to any listeners registered for the RoboCup type
    const packet: NUClearNetPacket = { hash: this.hash, payload, reliable: true, peer };
    for (const cb of this.packetListeners.get(ROBOCUP_TYPE) ?? []) {
      cb(packet);
    }
    for (const cb of this.packetListeners.get("nuclear_packet") ?? []) {
      cb(packet);
    }
  }

  /** Remove a peer that has stopped sending RoboCup packets. */
  private onPeerTimeout(key: string): void {
    const entry = this.peers.get(key);
    if (entry === undefined) {
      return;
    }
    clearTimeout(entry.timer);
    this.peers.delete(key);
    for (const cb of this.leaveListeners) {
      cb(entry.peer);
    }
  }
}
