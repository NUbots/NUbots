import { beforeEach, describe, expect, it } from "vitest";
import { beforeEach, describe, expect, it, vi } from "vitest";

import { FakeNUClearNetClient } from "../fake_nuclearnet_client";
import { FakeNUClearNetServer } from "../fake_nuclearnet_server";

describe("FakeNUClearNetClient", () => {
  let server: FakeNUClearNetServer;
  let bob: FakeNUClearNetClient;
  let alice: FakeNUClearNetClient;
  let eve: FakeNUClearNetClient;

  beforeEach(() => {
    server = new FakeNUClearNetServer();
    bob = new FakeNUClearNetClient(server);
    alice = new FakeNUClearNetClient(server);
    eve = new FakeNUClearNetClient(server);
  });

  it("receives own join event", () => {
    const onJoin = vi.fn();
    bob.onJoin(onJoin);

    bob.connect({ name: "bob" });

    expect(onJoin).toHaveBeenCalledTimes(1);
    expect(onJoin).toHaveBeenLastCalledWith(expect.objectContaining({ name: "bob" }));
  });

  it("does not receive own leave event", () => {
    const onLeave = vi.fn();
    bob.onLeave(onLeave);

    const disconnect = bob.connect({ name: "bob" });
    disconnect();

    expect(onLeave).not.toHaveBeenCalled();
  });

  it("receives join events from other clients", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });

    const bobOnJoin = vi.fn();
    bob.onJoin(bobOnJoin);

    const aliceOnJoin = vi.fn();
    alice.onJoin(aliceOnJoin);

    eve.connect({ name: "eve" });

    expect(bobOnJoin).toHaveBeenCalledTimes(1);
    expect(bobOnJoin).toHaveBeenLastCalledWith(expect.objectContaining({ name: "eve" }));

    expect(aliceOnJoin).toHaveBeenCalledTimes(1);
    expect(aliceOnJoin).toHaveBeenLastCalledWith(expect.objectContaining({ name: "eve" }));
  });

  it("does not receive join events from other clients after unsubscribing", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });

    const bobOnJoin = vi.fn();
    const bobOffJoin = bob.onJoin(bobOnJoin);
    bobOffJoin();

    const aliceOnJoin = vi.fn();
    alice.onJoin(aliceOnJoin);

    eve.connect({ name: "eve" });

    expect(bobOnJoin).not.toHaveBeenCalled();

    expect(aliceOnJoin).toHaveBeenCalledTimes(1);
    expect(aliceOnJoin).toHaveBeenLastCalledWith(expect.objectContaining({ name: "eve" }));
  });

  it("receives leave events from other clients", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });

    const bobOnLeave = vi.fn();
    bob.onLeave(bobOnLeave);

    const aliceOnLeave = vi.fn();
    alice.onLeave(aliceOnLeave);

    const disconnectEve = eve.connect({ name: "eve" });
    disconnectEve();

    expect(bobOnLeave).toHaveBeenCalledTimes(1);
    expect(bobOnLeave).toHaveBeenLastCalledWith(expect.objectContaining({ name: "eve" }));

    expect(aliceOnLeave).toHaveBeenCalledTimes(1);
    expect(aliceOnLeave).toHaveBeenLastCalledWith(expect.objectContaining({ name: "eve" }));
  });

  it("does not receive leave events from other clients after unsubscribing", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });

    const bobOnLeave = vi.fn();
    const bobOffLeave = bob.onLeave(bobOnLeave);
    bobOffLeave();

    const aliceOnLeave = vi.fn();
    alice.onLeave(aliceOnLeave);

    const disconnectEve = eve.connect({ name: "eve" });
    disconnectEve();

    expect(bobOnLeave).not.toHaveBeenCalled();

    expect(aliceOnLeave).toHaveBeenCalledTimes(1);
    expect(aliceOnLeave).toHaveBeenLastCalledWith(expect.objectContaining({ name: "eve" }));
  });

  it("receives join events for all other connected clients on connect", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });

    const onJoin = vi.fn();
    eve.onJoin(onJoin);
    eve.connect({ name: "eve" });

    expect(onJoin).toHaveBeenCalledTimes(3);
    expect(onJoin).toHaveBeenCalledWith(expect.objectContaining({ name: "bob" }));
    expect(onJoin).toHaveBeenCalledWith(expect.objectContaining({ name: "alice" }));
    expect(onJoin).toHaveBeenCalledWith(expect.objectContaining({ name: "eve" }));
  });

  it("receives own messages", () => {
    bob.connect({ name: "bob" });

    const onSensors = vi.fn();
    bob.on("sensors", onSensors);

    const payload = Buffer.alloc(8);
    bob.send({ type: "sensors", payload });

    expect(onSensors).toHaveBeenCalledTimes(1);
    expect(onSensors).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "bob" }),
      }),
    );
  });

  it("receives specific messages sent from other clients", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });
    eve.connect({ name: "eve" });

    const bobOnSensors = vi.fn();
    bob.on("sensors", bobOnSensors);

    const aliceOnSensors = vi.fn();
    alice.on("sensors", aliceOnSensors);

    const eveOnSensors = vi.fn();
    eve.on("sensors", eveOnSensors);

    const payload = Buffer.alloc(8);
    eve.send({ type: "sensors", payload });

    expect(bobOnSensors).toHaveBeenCalledTimes(1);
    expect(bobOnSensors).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(aliceOnSensors).toHaveBeenCalledTimes(1);
    expect(aliceOnSensors).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(eveOnSensors).toHaveBeenCalledTimes(1);
    expect(eveOnSensors).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );
  });

  it("receives general packets sent from other clients", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });
    eve.connect({ name: "eve" });

    const bobOnPacket = vi.fn();
    bob.onPacket(bobOnPacket);

    const aliceOnPacket = vi.fn();
    alice.onPacket(aliceOnPacket);

    const eveOnSensors = vi.fn();
    eve.on("sensors", eveOnSensors);

    const payload = Buffer.alloc(8);
    eve.send({ type: "sensors", payload });

    expect(bobOnPacket).toHaveBeenCalledTimes(1);
    expect(bobOnPacket).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(aliceOnPacket).toHaveBeenCalledTimes(1);
    expect(aliceOnPacket).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(eveOnSensors).toHaveBeenCalledTimes(1);
    expect(eveOnSensors).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );
  });

  it("only receives targetted messages if they are the target", () => {
    bob.connect({ name: "bob" });
    alice.connect({ name: "alice" });
    eve.connect({ name: "eve" });

    const bobOnSensors = vi.fn();
    bob.on("sensors", bobOnSensors);

    const aliceOnSensors = vi.fn();
    alice.on("sensors", aliceOnSensors);

    const eveOnSensors = vi.fn();
    eve.on("sensors", eveOnSensors);

    const payload = Buffer.alloc(8);
    eve.send({ type: "sensors", payload, target: "bob" });

    expect(bobOnSensors).toHaveBeenCalledTimes(1);
    expect(bobOnSensors).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(aliceOnSensors).not.toHaveBeenCalled();
    expect(eveOnSensors).not.toHaveBeenCalled();
  });

  it("will receive targetted messages even if there are other clients with the same name", () => {
    const bob1 = new FakeNUClearNetClient(server);
    const bob2 = new FakeNUClearNetClient(server);

    bob1.connect({ name: "bob" });
    bob2.connect({ name: "bob" });
    eve.connect({ name: "eve" });

    const bobOnSensors1 = vi.fn();
    bob1.on("sensors", bobOnSensors1);

    const bobOnSensors2 = vi.fn();
    bob2.on("sensors", bobOnSensors2);

    const eveOnSensors = vi.fn();
    eve.on("sensors", eveOnSensors);

    const payload = Buffer.alloc(8);
    eve.send({ type: "sensors", payload, target: "bob" });

    expect(bobOnSensors1).toHaveBeenCalledTimes(1);
    expect(bobOnSensors1).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(bobOnSensors2).toHaveBeenCalledTimes(1);
    expect(bobOnSensors2).toHaveBeenLastCalledWith(
      expect.objectContaining({
        payload,
        peer: expect.objectContaining({ name: "eve" }),
      }),
    );

    expect(eveOnSensors).not.toHaveBeenCalled();
  });
});
