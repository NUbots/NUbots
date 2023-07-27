import { EventEmitter } from "events";
import { NUClearNetSend } from "nuclearnet.js";

import { createSingletonFactory } from "../../shared/base/create_singleton_factory";

import { FakeNUClearNetClient } from "./fake_nuclearnet_client";
import { hashType } from "./hash_type";

/**
 * A fake in-memory NUClearNet 'server' which routes messages between each FakeNUClearNetClient.
 *
 * All messages are 'reliable' in that nothing is intentionally dropped.
 * Targeted messages are supported.
 */
export class FakeNUClearNetServer {
  private events: EventEmitter;
  private clients: FakeNUClearNetClient[];

  constructor() {
    this.events = new EventEmitter();
    this.clients = [];
  }

  /**
   * Designed to only have a single instance during runtime. This singleton is mimicking the behaviour of
   * the real NUClearNet, where a single global TCP/IP network is an implicit non-injectable dependency. This is why
   * FakeNUClearNetClient.of() does not take a given server and instead uses the singleton instance.
   *
   * Avoid using this singleton factory in tests though, as you'll introduce cross-contamination between tests.
   * Simply use the constructor of both FakeNUClearNetServer and FakeNUClearNetClient instead.
   */
  static of = createSingletonFactory(() => {
    return new FakeNUClearNetServer();
  });

  connect(client: FakeNUClearNetClient): () => void {
    this.events.emit("nuclear_join", client.peer);
    this.clients.push(client);

    for (const otherClient of this.clients) {
      // This ensures that the newly connected client receives a nuclear_join event from everyone else on the network.
      otherClient.peer && client.fakeJoin(otherClient.peer);

      // This conditional avoids sending nuclear_join twice to the newly connected client.
      if (otherClient !== client) {
        // Send a single nuclear_join to everyone on the network about the newly connected client.
        client.peer && otherClient.fakeJoin(client.peer);
      }
    }

    return () => {
      this.events.emit("nuclear_leave", client.peer);
      this.clients.splice(this.clients.indexOf(client), 1);

      for (const otherClient of this.clients) {
        client.peer && otherClient.fakeLeave(client.peer);
      }
    };
  }

  send(client: FakeNUClearNetClient, opts: NUClearNetSend) {
    if (!client.peer) {
      throw new Error("Cannot send a packaet to a client who has not connected to the network");
    }

    const hash: Buffer = typeof opts.type === "string" ? hashType(opts.type) : opts.type;
    const packet = {
      peer: client.peer,
      hash,
      payload: opts.payload,
      reliable: !!opts.reliable,
    };

    /*
     * This list intentionally includes the sender unless explicitly targeting another peer. This matches the real
     * NUClearNet behaviour.
     */
    const targetClients =
      opts.target === undefined
        ? this.clients
        : this.clients.filter((otherClient) => otherClient.peer && otherClient.peer.name === opts.target);

    const hashString = hash.toString("hex");
    for (const client of targetClients) {
      client.fakePacket(hashString, packet);
    }
  }
}
