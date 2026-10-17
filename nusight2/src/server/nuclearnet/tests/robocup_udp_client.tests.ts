import { RoboCup } from "@proto/message/input/RoboCup";
import { NUClearNetPacket } from "nuclearnet.js";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { RoboCupUDPClient } from "../robocup_udp_client";

/**
 * A minimal fake `dgram.Socket` that lets tests register the "message" handler `RoboCupUDPClient`
 * installs and trigger it directly with fabricated datagrams, without doing any real networking.
 */
function createFakeSocket() {
  const listeners = new Map<string, (...args: any[]) => void>();

  const socket = {
    on: vi.fn((event: string, cb: (...args: any[]) => void) => {
      listeners.set(event, cb);
      return socket;
    }),
    bind: vi.fn((_port: number, cb?: () => void) => {
      cb?.();
      return socket;
    }),
    close: vi.fn(),
  };

  return {
    socket,
    /** Simulate an incoming UDP datagram from the given address/port. */
    receive(payload: Uint8Array, remote: { address: string; port: number }) {
      listeners.get("message")?.(Buffer.from(payload), remote);
    },
  };
}

function robocupPacket(playerId: number): Uint8Array {
  return new RoboCup({ currentPose: { playerId } }).toBinary();
}

vi.mock("node:dgram", () => {
  const createSocket = vi.fn();
  return { default: { createSocket }, createSocket };
});

describe("RoboCupUDPClient", () => {
  let fakeSocket: ReturnType<typeof createFakeSocket>;

  beforeEach(async () => {
    vi.useFakeTimers();
    fakeSocket = createFakeSocket();
    const dgram = await import("node:dgram");
    vi.mocked(dgram.createSocket).mockReturnValue(fakeSocket.socket as any);
  });

  afterEach(() => {
    vi.useRealTimers();
    vi.clearAllMocks();
  });

  it("presents a sender as a peer and delivers its packets to on() and onPacket() listeners", () => {
    const client = RoboCupUDPClient.of({ port: 10001 });
    const onJoin = vi.fn();
    const onEvent = vi.fn();
    const onPacket = vi.fn();
    client.onJoin(onJoin);
    client.on("message.input.RoboCup", onEvent);
    client.onPacket(onPacket);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });

    expect(onJoin).toHaveBeenCalledTimes(1);
    expect(onJoin).toHaveBeenCalledWith(
      expect.objectContaining({ name: "robocup-1", address: "10.1.1.5", type: "robocup-udp-peer" }),
    );
    expect(onEvent).toHaveBeenCalledTimes(1);
    expect(onPacket).toHaveBeenCalledTimes(1);
  });

  it("delivers packets marked as unreliable, so they're subject to normal client-side throttling", () => {
    const client = RoboCupUDPClient.of({ port: 10001 });
    const onPacket = vi.fn<(packet: NUClearNetPacket) => void>();
    client.onPacket(onPacket);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });

    expect(onPacket).toHaveBeenCalledWith(expect.objectContaining({ reliable: false }));
  });

  it("uses a stable peer port across packets, regardless of the sender's source port", () => {
    const client = RoboCupUDPClient.of({ port: 10001 });
    const onPacket = vi.fn<(packet: NUClearNetPacket) => void>();
    client.onPacket(onPacket);
    client.connect({ name: "nusight" });

    // Simulate the sender using a different ephemeral source port for each datagram
    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 11111 });
    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 22222 });

    const peers = onPacket.mock.calls.map(([packet]) => packet.peer);
    expect(peers[0].port).toBe(10001);
    expect(peers[1].port).toBe(10001);
  });

  it("ignores packets from addresses that aren't in the allow-list", () => {
    const client = RoboCupUDPClient.of({ port: 10001, allowedAddresses: ["10.1.1.5"] });
    const onJoin = vi.fn();
    const onPacket = vi.fn();
    client.onJoin(onJoin);
    client.onPacket(onPacket);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.99", port: 34567 });

    expect(onJoin).not.toHaveBeenCalled();
    expect(onPacket).not.toHaveBeenCalled();
  });

  it("accepts packets from addresses that are in the allow-list", () => {
    const client = RoboCupUDPClient.of({ port: 10001, allowedAddresses: ["10.1.1.5"] });
    const onPacket = vi.fn();
    client.onPacket(onPacket);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });

    expect(onPacket).toHaveBeenCalledTimes(1);
  });

  it("only emits one join event per peer, even across repeated packets", () => {
    const client = RoboCupUDPClient.of({ port: 10001 });
    const onJoin = vi.fn();
    client.onJoin(onJoin);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });

    expect(onJoin).toHaveBeenCalledTimes(1);
  });

  it("emits a leave event after the configured timeout without a further packet", () => {
    const client = RoboCupUDPClient.of({ port: 10001, timeout: 1000 });
    const onLeave = vi.fn();
    client.onLeave(onLeave);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    expect(onLeave).not.toHaveBeenCalled();

    vi.advanceTimersByTime(1000);

    expect(onLeave).toHaveBeenCalledTimes(1);
    expect(onLeave).toHaveBeenCalledWith(expect.objectContaining({ name: "robocup-1", address: "10.1.1.5" }));
  });

  it("does not emit a leave event if another packet arrives before the timeout", () => {
    const client = RoboCupUDPClient.of({ port: 10001, timeout: 1000 });
    const onLeave = vi.fn();
    client.onLeave(onLeave);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    vi.advanceTimersByTime(600);
    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    vi.advanceTimersByTime(600);

    expect(onLeave).not.toHaveBeenCalled();
  });

  it("treats robots with different player ids as different peers", () => {
    const client = RoboCupUDPClient.of({ port: 10001 });
    const onJoin = vi.fn();
    client.onJoin(onJoin);
    client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    fakeSocket.receive(robocupPacket(2), { address: "10.1.1.5", port: 34567 });

    expect(onJoin).toHaveBeenCalledTimes(2);
  });

  it("emits leave events for all peers when disconnected", () => {
    const client = RoboCupUDPClient.of({ port: 10001 });
    const onLeave = vi.fn();
    client.onLeave(onLeave);
    const disconnect = client.connect({ name: "nusight" });

    fakeSocket.receive(robocupPacket(1), { address: "10.1.1.5", port: 34567 });
    disconnect();

    expect(onLeave).toHaveBeenCalledTimes(1);
    expect(fakeSocket.socket.close).toHaveBeenCalledTimes(1);
  });
});
