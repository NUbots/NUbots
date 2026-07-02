import dgram from "node:dgram";

import { NUClearNetOptions } from "nuclearnet.js";
import { NUClearNetPacket } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";

import { NUClearEventListener } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearNetPeerWithType } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearPacketListener } from "../../shared/nuclearnet/nuclearnet_client";

import { parseEventString } from "./parse_event_string";

export interface UDPSideChannelClientOptions {
  /** The UDP port to listen on for incoming packets. */
  port: number;
  /**
   * The list of source IP addresses to accept packets from. When empty or omitted, packets from
   * any address are accepted.
   */
  allowedAddresses?: string[];
  /**
   * The number of milliseconds to wait for a new packet from a source before considering that
   * robot to have left the network. Defaults to 5000ms.
   */
  timeout?: number;
}

/** A single decoded, NUClearNet-shaped packet extracted from a UDP datagram. */
export interface DecodedUDPPacket {
  /** The fully qualified Protobuf type name of the packet's payload. */
  type: string;
  /** The NUClear hash for `type` (see {@link ../../shared/nuclearnet/hash_type.hashType}). */
  hash: Buffer;
  /** The serialised payload to deliver to listeners of `type`. */
  payload: Buffer;
}

/** The result of successfully decoding an incoming UDP datagram. */
export interface DecodedUDPMessage {
  /** A stable identity for the sender, e.g. `${address}:${robotId}`. Used to track join/leave. */
  key: string;
  /** The name to present this peer as (e.g. shown in NUsight's robot list). */
  name: string;
  /** The packet(s) to deliver to registered listeners. */
  packets: DecodedUDPPacket[];
}

interface SideChannelPeer {
  peer: NUClearNetPeerWithType;
  timer: NodeJS.Timeout;
}

/**
 * A receive-only {@link NUClearNetClient} that listens on a UDP port and presents each unique
 * sender as a NUClearNet peer, delivering whatever packet(s) {@link decode} extracts from each
 * datagram.
 *
 * This provides a low bandwidth "side channel" for robots that cannot join the NUClear network
 * directly (e.g. across a restrictive network) but can still send a single UDP packet, and for
 * receiving packets in other UDP-based formats (e.g. the official RoboCup team communication
 * protocol) that can be translated into a NUClearNet-style packet.
 */
export abstract class UDPSideChannelClient implements NUClearNetClient {
  private readonly allowedAddresses: Set<string>;
  private readonly timeout: number;

  /** The peers we have seen, keyed by the `key` returned from {@link decode}. */
  private readonly peers = new Map<string, SideChannelPeer>();

  /** Listeners registered for peers joining the network. */
  private readonly joinListeners = new Set<NUClearEventListener>();

  /** Listeners registered for peers leaving the network. */
  private readonly leaveListeners = new Set<NUClearEventListener>();

  /** Packet listeners, keyed by the event type they are listening for. */
  private readonly packetListeners = new Map<string, Set<NUClearPacketListener>>();

  private socket?: dgram.Socket;

  protected constructor(
    private readonly options: UDPSideChannelClientOptions,
    /** The peer type to present connections received by this client as. */
    private readonly peerType: NUClearNetPeerWithType["type"],
  ) {
    this.allowedAddresses = new Set(options.allowedAddresses ?? []);
    this.timeout = options.timeout ?? 5000;
  }

  connect(_options: NUClearNetOptions): () => void {
    const socket = dgram.createSocket({ type: "udp4", reuseAddr: true });
    this.socket = socket;

    socket.on("message", (message, remote) => this.onMessage(message, remote.address, remote.port));
    socket.on("error", (error) => {
      console.error(`UDP side channel error:`, error);
    });
    socket.bind(this.options.port, () => {
      console.info(`UDP side channel (${this.peerType}) listening on port ${this.options.port}`);
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
    // UDP side channels are receive-only, so sending is a no-op.
  }

  /**
   * Decode an incoming UDP datagram into the packet(s) it represents. Return `undefined` to
   * ignore the datagram, e.g. because it failed to decode.
   */
  protected abstract decode(payload: Buffer, address: string, port: number): DecodedUDPMessage | undefined;

  /** Handle an incoming UDP datagram. */
  private onMessage(payload: Buffer, address: string, port: number): void {
    // Filter out packets from addresses that aren't allowed, if a filter is configured
    if (this.allowedAddresses.size > 0 && !this.allowedAddresses.has(address)) {
      return;
    }

    const decoded = this.decode(payload, address, port);
    if (decoded === undefined) {
      return;
    }

    const { key, name, packets } = decoded;
    const peer: NUClearNetPeerWithType = { name, address, port, type: this.peerType };

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

    // Deliver the decoded packet(s) to any listeners registered for their type
    for (const { type, hash, payload: packetPayload } of packets) {
      const packet: NUClearNetPacket = { hash, payload: packetPayload, reliable: true, peer };
      for (const cb of this.packetListeners.get(type) ?? []) {
        cb(packet);
      }
      for (const cb of this.packetListeners.get("nuclear_packet") ?? []) {
        cb(packet);
      }
    }
  }

  /** Remove a peer that has stopped sending packets. */
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
