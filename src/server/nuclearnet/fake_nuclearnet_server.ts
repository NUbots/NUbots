import * as EventEmitter from 'events'
import { NUClearNetSend } from 'nuclearnet.js'
import { createSingletonFactory } from '../../shared/base/create_singleton_factory'
import { FakeNUClearNetClient } from './fake_nuclearnet_client'

/**
 * A fake in-memory NUClearNet 'server' which routes messages between each FakeNUClearNetClient.
 *
 * All messages are 'reliable' in that nothing is intentially dropped.
 * Targetted messages are supported.
 */
export class FakeNUClearNetServer {
  private events: EventEmitter
  private clients: FakeNUClearNetClient[]

  public constructor() {
    this.events = new EventEmitter()
    this.clients = []
  }

  /**
   * Designed to only have a single instance during runtime. This singleton is mimicking the behaviour of
   * the real NUClearNet, where a single global TCP/IP network is an implicit non-injectable dependency. This is why
   * FakeNUClearNetClient.of() does not take a given server and instead uses the singleton instance.
   *
   * Avoid using this singleton factory in tests though, as you'll introduce cross-contamination between tests.
   * Simply use the constructor of both FakeNUClearNetServer and FakeNUClearNetClient instead.
   */
  public static of = createSingletonFactory(() => {
    return new FakeNUClearNetServer()
  })

  public connect(client: FakeNUClearNetClient): () => void {
    this.events.emit('nuclear_join', client.peer)
    this.clients.push(client)

    for (const otherClient of this.clients) {
      // This ensures that the newly connected client receives a nuclear_join event from everyone else on the network.
      client.fakeJoin(otherClient.peer)

      // This conditional avoids sending nuclear_join twice to the newly connected client.
      if (otherClient !== client) {
        // Send a single nuclear_join to everyone on the network about the newly connected client.
        otherClient.fakeJoin(client.peer)
      }
    }

    return () => {
      this.events.emit('nuclear_leave', client.peer)
      this.clients.splice(this.clients.indexOf(client), 1)

      for (const otherClient of this.clients) {
        otherClient.fakeLeave(client.peer)
      }
    }
  }

  public send(client: FakeNUClearNetClient, opts: NUClearNetSend) {
    if (typeof opts.type === 'string') {
      const packet = {
        peer: client.peer,
        payload: opts.payload,
        reliable: !!opts.reliable,
      }

      /*
       * This list intentially includes the sender unless explicitly targeting another peer. This matches the real
       * NUClearNet behaviour.
       */
      const targetClients = opts.target === undefined
        ? this.clients
        : this.clients.filter(otherClient => otherClient.peer.name === opts.target)

      for (const client of targetClients) {
        client.fakePacket(opts.type, packet)
      }
    }
  }
}
