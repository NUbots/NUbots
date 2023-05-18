import { NUClearNetOptions } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";
import { NUClearNetPacket } from "nuclearnet.js";

import { NUClearPacketListener } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearEventListener } from "../../shared/nuclearnet/nuclearnet_client";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";

import { DirectWebSocketClient } from "./direct_web_socket_client";
import { WebSocketClient } from "./web_socket_client";

type PacketListener = (packet: NUClearNetPacket, ack?: (time?: number) => void) => void;

/**
 * A client-side interface for interacting with NUClearNet. Allows a browser to connect transparently connect to
 * NUClearNet, using web sockets at the transport layer. Supports automatic reconnection, which rebinds all NUClearNet
 * event listeners.
 */
export class WebSocketProxyNUClearNetClient implements NUClearNetClient {
  private nextRequestToken: number;

  // Store all listeners so that we can restore them after a socket disconnection/reconnection.
  private joinListeners: Set<NUClearEventListener>;
  private leaveListeners: Set<NUClearEventListener>;
  private packetListeners: Map<string, Set<{ requestToken: string; listener: PacketListener }>>;

  constructor(private socket: WebSocketClient) {
    this.nextRequestToken = 0;
    this.joinListeners = new Set();
    this.leaveListeners = new Set();
    this.packetListeners = new Map();
  }

  static of() {
    const uri = `${document.location!.origin}/nuclearnet`;
    return new WebSocketProxyNUClearNetClient(
      DirectWebSocketClient.of(uri, {
        upgrade: false,
        transports: ["websocket"],
      } as any),
    );
  }

  connect(options: NUClearNetOptions): () => void {
    this.socket.on("reconnect", this.onReconnect.bind(this, options));
    this.socket.connect();

    return () => this.socket.disconnect();
  }

  onJoin(listener: NUClearEventListener): () => void {
    this.socket.on("nuclear_join", listener);
    this.joinListeners.add(listener);
    return () => {
      this.socket.off("nuclear_join", listener);
      this.joinListeners.delete(listener);
    };
  }

  onLeave(listener: NUClearEventListener): () => void {
    this.socket.on("nuclear_leave", listener);
    this.leaveListeners.add(listener);
    return () => {
      this.socket.off("nuclear_leave", listener);
      this.leaveListeners.delete(listener);
    };
  }

  on(event: string, cb: NUClearPacketListener): () => void {
    /*
     * This one is a bit more complicated than the others, mostly for performance purposes.
     *
     * The intent is to avoid WebSocketProxyNUClearNetServer sending all packets to the client, regardless of whether
     * they subscribed or not. Ideally we only send packets to a client that they are actively interested in listening
     * to that type of message. So we first request the type of message, with a 'listen' command. Then we listen to
     * that type of message. On unsubscribe, we send an unlisten command, to prevent listening.
     *
     * We use a request token to uniquely identify each 'listen' call, and so that we can 'unlisten' that same
     * subscription later.
     *
     * You can see this in practice when the client starts/stops receiving localisation events (e.g. Sensors) when
     * they focus/unfocus the localisation tab.
     */
    const requestToken = String(this.nextRequestToken++);
    this.socket.send("listen", event, requestToken);

    const onEventPacket: PacketListener = async (packet, ack) => {
      // Any async work in the packet callback should be awaited, to have an
      // accurate measurement of the time it took to process the packet.
      await cb(packet);

      if (ack) {
        // We acknowledge the packet with the current time, so the server can calculate
        // the time spent processing and rendering a packet.
        ack(Date.now());
      }
    };
    this.socket.on(event, onEventPacket);

    let existingListeners = this.packetListeners.get(event);
    if (!existingListeners) {
      existingListeners = new Set();
      this.packetListeners.set(event, existingListeners);
    }

    const newListener = { requestToken, listener: onEventPacket };
    existingListeners.add(newListener);

    return () => {
      this.socket.send("unlisten", requestToken);
      this.socket.off(event, onEventPacket);

      const packetListenersForEvent = this.packetListeners.get(event);
      if (packetListenersForEvent) {
        packetListenersForEvent.delete(newListener);
      }
    };
  }

  onPacket(cb: NUClearPacketListener): () => void {
    return this.on("nuclear_packet", cb);
  }

  send(options: NUClearNetSend): void {
    this.socket.send("packet", options);
  }

  private onReconnect = (_options: NUClearNetOptions) => {
    // We assume the server could have crashed during a reconnection. We need to restore all our listeners so that
    // a browser refresh is not necessary.
    for (const joinListener of this.joinListeners) {
      this.socket.on("nuclear_join", joinListener);
    }

    for (const leaveListener of this.leaveListeners) {
      this.socket.on("nuclear_leave", leaveListener);
    }

    for (const [event, packetListeners] of this.packetListeners.entries()) {
      for (const packetListener of packetListeners) {
        this.socket.send("listen", event, packetListener.requestToken);
        this.socket.on(event, packetListener.listener);
      }
    }
  };

  /**
   * Provide the given callback access to the underlying websocket transport
   */
  useSocket(cb: (socket: WebSocketClient) => void) {
    cb(this.socket);
  }
}
