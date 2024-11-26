import { describe, expect, it, vi } from "vitest";
import { FakeClock } from "../../../shared/time/fake_clock";
import { ScrubberPacketListenerCallback, ScrubberSet } from "../scrubber_set";
import { nanosToTimestampObject, timestampObjectToNanos } from "../utils";

import {
  computeMessageSetForTimestamp,
  computeTimestampForMessageSet,
  makePacket,
  makeScrubberStatePacket,
  sampleFileA,
  sampleFileB,
  scrubberPeerName,
  tick,
} from "./test_utils";

describe("ScrubberSet and Scrubber", () => {
  it("throws when attempting to load a scrubber with invalid files", () => {
    const scrubberSet = ScrubberSet.of();

    expect(() => {
      scrubberSet.load({ files: ["/file/that/does/not/exist.nbs"] });
    }).toThrow();
  });

  it("loads scrubbers with unique ids when given valid files", () => {
    // Given we have a scrubber set, ...
    const scrubberSet = ScrubberSet.of();

    // When we load a scrubber, ...
    const scrubber1 = scrubberSet.load({ files: [sampleFileA] });

    // We should get a scrubber with a unique id and the right files
    expect(scrubber1.data.id).toBe(1);
    expect(scrubber1.decoder.getTimestampRange()).toMatchObject([
      { seconds: 1000, nanos: 0 }, // sampleFileA timestamp range is 1000 to 1299 seconds
      { seconds: 1299, nanos: 0 },
    ]);

    // When we load another scrubber, ...
    const scrubber2 = scrubberSet.load({ files: [sampleFileB] });

    // We should get another scrubber with a unique id and the right files
    expect(scrubber2.data.id).toBe(2);
    expect(scrubber2.decoder.getTimestampRange()).toMatchObject([
      { seconds: 1300, nanos: 0 }, // sampleFileB timestamp range is 1300 to 1599 seconds
      { seconds: 1599, nanos: 0 },
    ]);
  });

  it("calls new listeners with matching packets from all scrubbers currently loaded in the set", async () => {
    const scrubberSet = ScrubberSet.of();

    // Load two scrubbers
    const [scrubberA, scrubberB] = [
      loadScrubber(scrubberSet, { name: "scrubberA" }),
      loadScrubber(scrubberSet, { name: "scrubberB" }),
    ];

    const makePacketFromA = scrubberA.makePacketFromScrubber;
    const makePacketFromB = scrubberB.makePacketFromScrubber;

    // Seek scrubber B to a different timestamp (before adding listeners)
    scrubberSet.update({ id: scrubberB.id, type: "seek", timestamp: scrubberB.halfwayTimestamp });

    // Add listeners for Ping and Pong messages
    const onPing = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Ping", onPing);

    const onPong = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pong", onPong);

    const startMessageSet = 0;
    const halfwayMessageSet = computeMessageSetForTimestamp(scrubberB.halfwayTimestamp);

    // Check that we got Ping messages from both scrubbers at their current timestamps
    expect(onPing).toHaveBeenCalledTimes(2);
    expect(onPing).toHaveBeenNthCalledWith(1, makePacketFromA("message.Ping", `ping.${startMessageSet}`, "scrubberA"));
    expect(onPing).toHaveBeenNthCalledWith(
      2,
      makePacketFromB("message.Ping", `ping.${halfwayMessageSet}`, "scrubberB"),
    );

    // Check that we got Pong messages from both scrubbers at their current timestamps
    expect(onPong).toHaveBeenCalledTimes(2);
    // There's no Pong at the start of the file so we get a packet with empty payload
    expect(onPong).toHaveBeenNthCalledWith(1, makePacketFromA("message.Pong", undefined, "scrubberA"));
    expect(onPong).toHaveBeenNthCalledWith(
      2,
      makePacketFromB("message.Pong", `pong.${halfwayMessageSet}`, "scrubberB"),
    );
  });

  it("calls event listeners for types in the scrubbed files on seek", async () => {
    const scrubberSet = ScrubberSet.of();

    // Set listeners for Ping, Pong, and Pang messages
    const onPing = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Ping", onPing);

    const onPong = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pong", onPong);

    const onPang = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pang", onPang);

    // Load a scrubber
    const { id, makePacketFromScrubber } = loadScrubber(scrubberSet);

    // Seek to the 10th message set (ping.10, pong.10, pang.10)
    const timestamp = computeTimestampForMessageSet(10);
    scrubberSet.update({ id, type: "seek", timestamp });

    // Check that we got the message from the initial load
    expect(onPing).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Ping", "ping.0"));
    expect(onPong).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Pong", undefined)); // No Pong at start of file, payload empty
    expect(onPang).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Pang", undefined)); // No Pang at start of file, payload empty
    expect(onPang).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Pang", undefined)); // Pang appears twice for both subtypes

    // Wait for the messages from the initial load to be acked
    await tick();

    // Check that we got the messages from the seek
    expect(onPing).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Ping", "ping.10"));
    expect(onPong).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Pong", "pong.10"));
    // Packet for Pang's first subtype
    expect(onPang).toHaveBeenNthCalledWith(3, makePacketFromScrubber("message.Pang", "pang.10"));
    // Packet for Pang's second subtype, different payload as Pang payloads change across subtypes in the sample NBS files
    expect(onPang).toHaveBeenNthCalledWith(4, makePacketFromScrubber("message.Pang", "pang.9"));
  });

  it("calls event listeners for types registered after loading a scrubber", async () => {
    const scrubberSet = ScrubberSet.of();

    // Load the scrubber first
    const { id, makePacketFromScrubber } = loadScrubber(scrubberSet);

    // Add Ping and Pong listeners after loading scrubber
    const onPing = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Ping", onPing);

    const onPong = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pong", onPong);

    // Check that we got the messages from adding the listeners
    expect(onPing).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Ping", "ping.0"));
    expect(onPong).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Pong", undefined)); // No Pong at start of file, payload empty

    // Wait for the messages from the listeners to be acked
    await tick();

    // Seek to the 10th message set (ping.10, pong.10)
    const timestamp = computeTimestampForMessageSet(10);
    scrubberSet.update({ id, type: "seek", timestamp });

    // Check that we got the messages from the seek
    expect(onPing).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Ping", "ping.10"));
    expect(onPong).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Pong", "pong.10"));
  });

  it("supports subtypes when listening for events", async () => {
    const scrubberSet = ScrubberSet.of();

    // Register a listener for Pang#100 that acks immediately
    const onPangSubtype100 = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pang#100", onPangSubtype100);

    // Load the scrubber and wait for ack of initial message
    const { id, makePacketFromScrubber } = loadScrubber(scrubberSet);
    await tick();

    // Seek to where we know there is a Pang message with subtype 100 and payload "pang.0"
    scrubberSet.update({ id, type: "seek", timestamp: { seconds: 1002, nanos: 0 } });

    // Check the message from the initial load: no Pang at start of file, payload undefined
    expect(onPangSubtype100).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Pang", undefined));

    // Check the message from the seek
    expect(onPangSubtype100).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Pang", "pang.0"));
  });

  it("automatically creates separate listeners for each known subtype of events that don't explicitly specify a subtype", async () => {
    const scrubberSet = ScrubberSet.of();

    // Register a listener for Pang that acks immediately
    const onPang = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pang", onPang);

    // Load a scrubber, and wait for ack of initial messages
    const { id, makePacketFromScrubber } = loadScrubber(scrubberSet);
    await tick();

    // Check that the listener was called twice, for each subtype
    expect(onPang).toHaveBeenCalledTimes(2);

    // Check that we got the messages from the initial load
    expect(onPang).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Pang", undefined)); // Subtype 100
    expect(onPang).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Pang", undefined)); // Subtype 200

    // Seek to where we know there is a Pang message with payload "pang.0"
    scrubberSet.update({ id, type: "seek", timestamp: { seconds: 1002, nanos: 0 } });

    // Check that we got the messages for the seek: subtype 100 and 200
    // Subtype 200 is not available at timestamp requested, so the payload is empty
    expect(onPang).toHaveBeenNthCalledWith(3, makePacketFromScrubber("message.Pang", "pang.0"));
    expect(onPang).toHaveBeenNthCalledWith(4, makePacketFromScrubber("message.Pang", undefined));
  });

  it("throttles emits based on promises returned from event listeners", async () => {
    const scrubberSet = ScrubberSet.of();

    // Keep track of resolves for acknowledging message sends
    let resolves: Array<() => void> = [];

    // Register a Ping listener that stores the resolve callback for each call
    const onPing = vi.fn(() => new Promise<void>((resolve) => resolves.push(resolve)));
    scrubberSet.on("message.Ping", onPing);

    // Load a scrubber
    const { id, makePacketFromScrubber } = loadScrubber(scrubberSet);

    // Ack the message from the load, and reset the resolves array
    resolves[0]();
    await tick();
    resolves = [];

    // Seek to every message set up until the 10th message set
    for (let i = 0; i <= 10; i++) {
      scrubberSet.update({ id, type: "seek", timestamp: computeTimestampForMessageSet(i) });
    }

    // Ack the first seek message
    resolves[0]();

    // Wait for the ack to be handled
    await tick();

    // The listener should have been called only three times:
    //    - on load
    //    - on the first seek
    //    - on the last seek
    // All other seeks in between should have not resulted in an emit, since the
    // intermediate requests overwrote each other while the listener was waiting
    // to ack a previously sent message.
    expect(onPing).toHaveBeenCalledTimes(3);

    // Check the that we got the messages for the first and last seeks
    expect(onPing).toHaveBeenNthCalledWith(2, makePacketFromScrubber("message.Ping", "ping.0"));
    expect(onPing).toHaveBeenNthCalledWith(3, makePacketFromScrubber("message.Ping", "ping.10"));
  });

  it("throttles emits separately by type and callback", (done) => {
    const scrubberSet = ScrubberSet.of();

    // Keep track of resolves for acknowledging message sends
    let onPingResolves: Array<() => void> = [];

    // Register a Ping listener that stores the resolve callback for each call
    const onPing = vi.fn(() => new Promise<void>((resolve) => onPingResolves.push(resolve)));
    scrubberSet.on("message.Ping", onPing);

    // Register a Pong listener that resolves immediately
    const onPong = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Pong", onPong);

    // Load a scrubber
    const { id } = loadScrubber(scrubberSet);

    // Ack the packet from the load, and reset the resolves array
    onPingResolves[0]();
    onPingResolves = [];

    // Wait for the ack to be handled
    setImmediate(() => {
      // Seek to every message set up until the 10th message set (ping.10, pong.10, pang.10)
      for (let i = 0; i <= 10; i++) {
        // The first seek should be synchronous, so the resolve following this loop works
        if (i === 0) {
          scrubberSet.update({ id, type: "seek", timestamp: computeTimestampForMessageSet(i) });
          continue;
        }

        // Do the other seeks asynchronously with setImmediate(), so they don't all run in the
        // same tick, to allow the listener promises to have the chance to run in between.
        setImmediate(() => {
          scrubberSet.update({ id, type: "seek", timestamp: computeTimestampForMessageSet(i) });
        });
      }

      // Ack the Ping message from the first seek
      onPingResolves[0]();

      // Wait for ack to be handled
      setImmediate(() => {
        try {
          // onPing should have been called only three times:
          // on load, first seek, last seek
          expect(onPing).toHaveBeenCalledTimes(3);
          expect(onPing.mock.calls).toMatchSnapshot();

          // onPong should have been called for the initial load
          // and for every seek, since it resolved each seek right away
          expect(onPong).toHaveBeenCalledTimes(12);

          done();
        } catch (error) {
          done(error);
        }
      });
    });
  });

  it("supports removing event listeners", async () => {
    const scrubberSet = ScrubberSet.of();

    // Register a Ping listener
    const onPing = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Ping", onPing);

    // Load a scrubber and wait for the resulting emit to be acked
    const { id, makePacketFromScrubber } = loadScrubber(scrubberSet, { files: [sampleFileA], name: "myScrubber" });
    await tick();

    // Check the message sent on load
    expect(onPing).toHaveBeenNthCalledWith(1, makePacketFromScrubber("message.Ping", "ping.0"));

    // Remove the listener
    scrubberSet.off("message.Ping", onPing);

    // Seek the scrubber
    scrubberSet.update({ id, type: "seek", timestamp: computeTimestampForMessageSet(20) });
    await tick();

    // The listener should have been called only once (for the initial load),
    // since the listener was removed before the second seek
    expect(onPing).toHaveBeenCalledTimes(1);
  });

  it("does not trigger next emit if the listener is offed while the current emit is pending", async () => {
    const scrubberSet = ScrubberSet.of();

    // Keep track of the ack for the first send to the Ping listener
    let ackFirstSend: (() => Promise<void>) | undefined = undefined;

    // Add a Ping listener stores an ack callback for the first send
    const onPing = vi.fn<ReturnType<ScrubberPacketListenerCallback>, Parameters<ScrubberPacketListenerCallback>>(
      () =>
        new Promise((resolve) => {
          if (!ackFirstSend) {
            ackFirstSend = async () => {
              resolve();
              await tick();
            };
          }
        }),
    );
    scrubberSet.on("message.Ping", onPing);

    // Load a scrubber
    const scrubber = scrubberSet.load({ files: [sampleFileA] });

    // Seek twice to trigger an emit for the first, and queue up an emit for the second
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(0) });
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });

    // Remove the listener while the first emit's ack is pending
    scrubberSet.off("message.Ping", onPing);

    await ackFirstSend!();

    // The listener should have been called only once, for the first seek
    expect(onPing).toHaveBeenCalledTimes(1);
  });

  it("allows for closing loaded scrubbers", async () => {
    const scrubberSet = ScrubberSet.of();

    // Register a Ping listener that resolves immediately
    const onPing = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Ping", onPing);

    // Load a scrubber and wait for the resulting emit to be acked
    const scrubber = scrubberSet.load({ files: [sampleFileA], name: "myScrubber" });
    await tick();

    // Close the scrubber
    scrubberSet.close(scrubber.data.id);

    // Attempting to seek the closed scrubber should throw an error
    expect(() => {
      scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });
    }).toThrowError(`scrubber ${scrubber.data.id} not found for update (seek)`);

    // The listener should have been called only once (for the initial load),
    // since the scrubber was closed before the second seek
    expect(onPing).toHaveBeenCalledTimes(1);
  });

  it("destroys all scrubbers and clears event listeners when destroyed", async () => {
    const scrubberSet = ScrubberSet.of();

    // Register a Ping listener that resolves immediately
    const onPing = vi.fn(() => Promise.resolve());
    scrubberSet.on("message.Ping", onPing);

    // Load two scrubbers and wait for the resulting emits to be acked
    const scrubberA = scrubberSet.load({ files: [sampleFileA], name: "ScrubberA" });
    const scrubberB = scrubberSet.load({ files: [sampleFileB], name: "ScrubberB" });
    await tick();

    // Seek both scrubbers and wait for the resulting emits to be acked
    scrubberSet.update({ id: scrubberA.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });
    scrubberSet.update({ id: scrubberB.data.id, type: "seek", timestamp: computeTimestampForMessageSet(20) });
    await tick();

    // Destroy the scrubber set
    scrubberSet.destroy();

    // Attempting to seek the closed scrubbers should throw an error
    expect(() => {
      scrubberSet.update({ id: scrubberA.data.id, type: "seek", timestamp: computeTimestampForMessageSet(15) });
    }).toThrowError(`scrubber ${scrubberA.data.id} not found for update (seek)`);

    expect(() => {
      scrubberSet.update({ id: scrubberB.data.id, type: "seek", timestamp: computeTimestampForMessageSet(25) });
    }).toThrowError(`scrubber ${scrubberB.data.id} not found for update (seek)`);

    // The listener should have been called only four times (for the initial loads and seeks)
    expect(onPing).toHaveBeenCalledTimes(4);
  });

  it("supports scrubber playback", async () => {
    const { clock, scrubberSet } = createScrubberSet();

    // Keep track of resolves for acknowledging message sends
    const resolves: Record<string, () => void> = {};
    const ack = async () => {
      await tick();
      for (const resolve of Object.values(resolves)) {
        resolve();
      }
    };

    // Save the resolve callback when we get Ping and ScrubberState messages
    const onPing = vi.fn(() => new Promise<void>((resolve) => (resolves["Ping"] = resolve)));
    const onScrubberState = vi.fn(() => new Promise<void>((resolve) => (resolves["ScrubberState"] = resolve)));

    // Listen for Ping and ScrubberState messages
    scrubberSet.on("message.Ping", onPing);
    scrubberSet.on("message.eye.ScrubberState", onScrubberState);

    // Load a scrubber and ack the resulting emits
    const { scrubber, id, defaultState, makePacketFromScrubber } = loadScrubber(scrubberSet);
    await ack();

    // Start playback
    scrubberSet.update({ id, type: "play" });

    // 1 for the initial load, 1 for the play
    const nthCallOffset = 2;

    for (let i = 0; i < 10; i++) {
      const messageSet = i * 10; // * 10 to go through more of the sample file
      const nextMessageSet = (i + 1) * 10;
      const nthCall = i + nthCallOffset;

      const { delta, timestamp } = getMessageSetTimestampDelta(messageSet, nextMessageSet);

      // Check that we got the expected Ping message
      expect(onPing).toHaveBeenCalledTimes(nthCall);
      expect(onPing).toHaveBeenNthCalledWith(nthCall, makePacketFromScrubber("message.Ping", `ping.${messageSet}`));

      // Check that we got the expected ScrubberState message
      expect(onScrubberState).toHaveBeenCalledTimes(nthCall);
      expect(onScrubberState).toHaveBeenNthCalledWith(
        nthCall,
        makeScrubberStatePacket({ ...defaultState, timestamp, playbackState: "playing", peer: scrubber.peer }),
      );

      // Advance the clock by the delta between the message sets
      clock.tick(delta);

      // Ack the previous message to trigger the next send
      await ack();
    }
  });

  it("supports pausing of playback", async () => {
    const { clock, scrubberSet } = createScrubberSet();

    // Keep track of resolve callbacks for acknowledging message sends
    const resolves: Record<string, () => void> = {};
    const ack = async () => {
      await tick();
      for (const resolve of Object.values(resolves)) {
        resolve();
      }
    };

    // Save the resolve callback when we get Ping and ScrubberState messages
    const onPing = vi.fn(() => new Promise<void>((resolve) => (resolves["Ping"] = resolve)));
    const onScrubberState = vi.fn(() => new Promise<void>((resolve) => (resolves["ScrubberState"] = resolve)));

    // Listen for Ping and ScrubberState messages
    scrubberSet.on("message.Ping", onPing);
    scrubberSet.on("message.eye.ScrubberState", onScrubberState);

    // Load a scrubber and ack the resulting emits
    const { scrubber, id, startTimestamp, defaultState, makePacketFromScrubber } = loadScrubber(scrubberSet);
    await ack();

    // Start playback
    scrubberSet.update({ id, type: "play" });

    const { delta, nextTimestamp } = getMessageSetTimestampDelta(0, 10);

    // Advance the clock by the delta between our message sets
    clock.tick(delta);

    // Pause playback
    scrubberSet.update({ id, type: "pause" });

    // Advance the clock after pausing
    clock.tick(getMessageSetTimestampDelta(10, 20).delta);

    // Ack the packet from play, to trigger the next send
    await ack();

    // Check that onPing was called thrice: once for initial load, once for play, and once for pause
    expect(onPing).toHaveBeenCalledTimes(3);

    // The emit from pause should have the message at the time of the pause, not after advancing the clock
    expect(onPing).toHaveBeenNthCalledWith(3, makePacketFromScrubber("message.Ping", "ping.10"));

    // onScrubberState should have been called thrice with the expected packets: on load, on play, and on pause
    expect(onScrubberState).toHaveBeenCalledTimes(3);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      2,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "playing",
        timestamp: startTimestamp,
        peer: scrubber.peer,
      }),
    );
    expect(onScrubberState).toHaveBeenNthCalledWith(
      3,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "paused",
        timestamp: nextTimestamp,
        peer: scrubber.peer,
      }),
    );
  });

  it("supports restarting playback after ended state", async () => {
    const { clock, scrubberSet } = createScrubberSet();

    // Keep track of resolve callbacks for acknowledging message sends
    let resolves: Record<string, () => void> = {};
    const ack = async () => {
      await tick();
      for (const resolve of Object.values(resolves)) {
        resolve();
      }
    };

    // Save the ack function when we get Ping and ScrubberState messages
    const onPing = vi.fn(() => new Promise<void>((resolve) => (resolves["Ping"] = resolve)));
    const onScrubberState = vi.fn(() => new Promise<void>((resolve) => (resolves["ScrubberState"] = resolve)));

    // Listen for Ping and ScrubberState messages
    scrubberSet.on("message.Ping", onPing);
    scrubberSet.on("message.eye.ScrubberState", onScrubberState);

    // Load a scrubber
    const {
      scrubber,
      id,
      defaultState,
      startNanos,
      startTimestamp,
      halfwayNanos,
      halfwayTimestamp,
      endNanos,
      endTimestamp,
      makePacketFromScrubber,
    } = loadScrubber(scrubberSet);

    // Ack the emits from load and reset resolves
    await ack();
    resolves = {};

    // Start playback and ack the resulting emits
    scrubberSet.update({ id, type: "play" });

    const packetPing0 = makePacketFromScrubber("message.Ping", "ping.0");

    // Offset by 1 for the initial load
    const nthCallOffset = 1;

    // Check we got the first Ping packet
    expect(onPing).toHaveBeenCalledTimes(1 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(1 + nthCallOffset, packetPing0);

    // Check we got the first ScrubberState packet
    expect(onScrubberState).toHaveBeenCalledTimes(1 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      1 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "playing",
        timestamp: startTimestamp,
        peer: scrubber.peer,
      }),
    );

    const halfwayMessageSet = computeMessageSetForTimestamp(nanosToTimestampObject(halfwayNanos));

    // Advance clock to halfway through the scrubber
    const deltaSeconds = Number(halfwayNanos - startNanos) * 1e-9;
    clock.tick(deltaSeconds);

    // Ack play packet, to send halfway packet
    await ack();

    // Check we got the halfway Ping packet
    expect(onPing).toHaveBeenCalledTimes(2 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(
      2 + nthCallOffset,
      makePacketFromScrubber("message.Ping", `ping.${halfwayMessageSet}`),
    );

    // Check we got the halfway ScrubberState packet
    expect(onScrubberState).toHaveBeenCalledTimes(2 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      2 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "playing",
        timestamp: halfwayTimestamp,
        peer: scrubber.peer,
      }),
    );

    // Advance clock to exceed the end timestamp
    clock.tick(Number(endNanos - startNanos) * 1e-9);

    // Ack the halfway packet, to send the end packet
    await ack();

    // Check we got the end packet
    expect(onPing).toHaveBeenCalledTimes(3 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(
      3 + nthCallOffset,
      makePacketFromScrubber("message.Ping", `ping.${computeMessageSetForTimestamp(endTimestamp)}`),
    );

    // Check we got the end ScrubberState packet
    expect(onScrubberState).toHaveBeenCalledTimes(3 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      3 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "ended",
        timestamp: endTimestamp,
        peer: scrubber.peer,
      }),
    );

    // Tick the clock by some amount, to go beyond the end timestamp
    clock.tick(Number(endNanos - startNanos) * 1e-9);

    // Ack the end packet, to check we don't get a new packet past the end
    await ack();

    // Check that we got no new packets since we're ended,
    // even though time has progressed past the end timestamp
    expect(onPing).toHaveBeenCalledTimes(3 + nthCallOffset);
    expect(onScrubberState).toHaveBeenCalledTimes(3 + nthCallOffset);

    // Send play
    scrubberSet.update({ id, type: "play" });

    // Check we started playing again from the start
    expect(onPing).toHaveBeenCalledTimes(4 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(4 + nthCallOffset, packetPing0);
  });

  it("handles seeking past end of file", async () => {
    const { scrubberSet } = createScrubberSet();

    // Keep track of resolve callbacks for acknowledging message sends
    let resolves: Record<string, () => void> = {};
    const ack = async () => {
      await tick();
      for (const resolve of Object.values(resolves)) {
        resolve();
      }
    };

    // Save the ack function when we get Ping and ScrubberState messages
    const onPing = vi.fn(() => new Promise<void>((resolve) => (resolves["Ping"] = resolve)));
    const onScrubberState = vi.fn(() => new Promise<void>((resolve) => (resolves["ScrubberState"] = resolve)));

    // Listen for Ping and ScrubberState messages
    scrubberSet.on("message.Ping", onPing);
    scrubberSet.on("message.eye.ScrubberState", onScrubberState);

    // Load a scrubber
    const {
      scrubber,
      id,
      defaultState,
      startTimestamp,
      halfwayNanos,
      halfwayTimestamp,
      endTimestamp,
      length,
      makePacketFromScrubber,
    } = loadScrubber(scrubberSet);

    // Ack the emits from the initial load and reset resolves
    await ack();
    resolves = {};

    // ----- Test wrapped seek when playback repeats -----------------------------------------------

    // Enable playback repeat and ack resulting emits
    scrubberSet.update({ id, type: "set-repeat", repeat: true });
    await ack();

    // Offset by 1 for the initial load
    const nthCallOffset = 1;

    // Check that we got a Ping packet from where we were when toggling repeat (at the start)
    expect(onPing).toHaveBeenCalledTimes(1 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(1 + nthCallOffset, makePacketFromScrubber("message.Ping", "ping.0"));

    // Check that we got a packet for the state change
    expect(onScrubberState).toHaveBeenCalledTimes(1 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      1 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "paused",
        playbackRepeat: true,
        timestamp: startTimestamp,
        peer: scrubber.peer,
      }),
    );

    // Seek to 1.5x the length of the file
    scrubberSet.update({ id, type: "seek", timestamp: nanosToTimestampObject(halfwayNanos + length) });

    // Check that the seek wrapped around, emitting the halfway packet
    expect(onPing).toHaveBeenCalledTimes(2 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(
      2 + nthCallOffset,
      makePacketFromScrubber("message.Ping", `ping.${computeMessageSetForTimestamp(halfwayTimestamp)}`),
    );

    // Check that we got a packet for the state change
    expect(onScrubberState).toHaveBeenCalledTimes(2 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      2 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "paused",
        playbackRepeat: true,
        timestamp: halfwayTimestamp,
        peer: scrubber.peer,
      }),
    );

    // ----- Test seek doesn't wrap without repeat -------------------------------------------------

    // Disable playback repeat and ack the resulting emits
    scrubberSet.update({ id, type: "set-repeat", repeat: false });
    await ack();

    // Note: No Ping packet is sent on repeat toggle + ack,
    // since the timestamp remains the same on toggle

    // Check that we got a packet for the state change
    expect(onScrubberState).toHaveBeenCalledTimes(3 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      3 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "paused",
        playbackRepeat: false,
        timestamp: halfwayTimestamp,
        peer: scrubber.peer,
      }),
    );

    // Seek to 1.5x the length of the file and ack the resulting emits
    scrubberSet.update({ id, type: "seek", timestamp: nanosToTimestampObject(halfwayNanos + length) });
    await ack();

    // Check that the seek sent the end packet without wrapping, since repeat is disabled
    expect(onPing).toHaveBeenCalledTimes(3 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(
      3 + nthCallOffset,
      makePacketFromScrubber("message.Ping", `ping.${computeMessageSetForTimestamp(endTimestamp)}`),
    );

    // Check that we got a packet for the state change
    expect(onScrubberState).toHaveBeenCalledTimes(4 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      4 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "ended",
        playbackRepeat: false,
        timestamp: endTimestamp,
        peer: scrubber.peer,
      }),
    );

    // ----- Test seek after ended state -----------------------------------------------------------

    // Seek to halfway through the file and ack the sent packets
    scrubberSet.update({ id, type: "seek", timestamp: halfwayTimestamp });
    await ack();

    // Check that the halfway packet was sent
    expect(onPing).toHaveBeenCalledTimes(4 + nthCallOffset);
    expect(onPing).toHaveBeenNthCalledWith(
      4 + nthCallOffset,
      makePacketFromScrubber("message.Ping", `ping.${computeMessageSetForTimestamp(halfwayTimestamp)}`),
    );

    // Check that we got a packet for the state change
    expect(onScrubberState).toHaveBeenCalledTimes(5 + nthCallOffset);
    expect(onScrubberState).toHaveBeenNthCalledWith(
      5 + nthCallOffset,
      makeScrubberStatePacket({
        ...defaultState,
        playbackState: "paused",
        playbackRepeat: false,
        timestamp: halfwayTimestamp,
        peer: scrubber.peer,
      }),
    );
  });

  it("supports changing playback speed", async () => {
    const { clock, scrubberSet } = createScrubberSet();

    // Keep track of resolve callbacks for acknowledging message sends
    let resolves: Record<string, () => void> = {};
    const ack = async () => {
      await tick();
      for (const resolve of Object.values(resolves)) {
        resolve();
      }
    };

    // Save the ack function when we get Ping and ScrubberState messages
    const onPing = vi.fn(() => new Promise<void>((resolve) => (resolves["Ping"] = resolve)));
    const onScrubberState = vi.fn(() => new Promise<void>((resolve) => (resolves["ScrubberState"] = resolve)));

    // Listen for Ping and ScrubberState messages
    scrubberSet.on("message.Ping", onPing);
    scrubberSet.on("message.eye.ScrubberState", onScrubberState);

    // Load a scrubber
    const { scrubber, id, defaultState, makePacketFromScrubber } = loadScrubber(scrubberSet);

    // Ack the emits from the load and reset the resolves
    await ack();
    resolves = {};

    // Start playback
    scrubberSet.update({ id, type: "play" });

    const stepsTillEndOfFile = 7;
    const { delta } = getMessageSetTimestampDelta(0, 1);

    for (let i = 0; i < stepsTillEndOfFile; i++) {
      // Generates the Mersenne numbers: 0, 1, 3, 7, 15, 31, 63, ...
      // You get this when doubling and subtracting 1, which is what we do:
      // playback speed is doubled below and we subtract 1 since i starts at 0
      const expected = 2 ** i - 1;
      const nthCall = i + 1 + 1; // + 2 for load, play

      const timestamp = computeTimestampForMessageSet(expected, 0);

      // Check that we got the Ping packet expected at this point given the playback speed
      expect(onPing).toHaveBeenCalledTimes(nthCall);
      expect(onPing).toHaveBeenNthCalledWith(nthCall, makePacketFromScrubber("message.Ping", `ping.${expected}`));

      // Check that we got a packet for the state change
      expect(onScrubberState).toHaveBeenCalledTimes(nthCall);
      expect(onScrubberState).toHaveBeenNthCalledWith(
        nthCall,
        makeScrubberStatePacket({
          ...defaultState,
          playbackState: "playing",
          // The first playback speed (0) will occur in two state updates.
          // Once for play above and the other for the first iteration of this loop.
          playbackSpeed: Math.max(0, i - 1),
          timestamp,
          peer: scrubber.peer,
        }),
      );

      // Double our current playback speed
      scrubberSet.update({ id, type: "set-playback-speed", playbackSpeed: i });

      // Advance the clock
      clock.tick(delta);

      // Ack the previous message to trigger the next send
      await ack();
    }
  });
});

/** Create a scrubber set with a fake clock */
function createScrubberSet() {
  const clock = new FakeClock(0);
  const scrubberSet = new ScrubberSet({ clock });
  return { clock, scrubberSet };
}

/**
 * Load a scrubber with the given options into the given scrubber set,
 * and compute common values that are useful for tests
 */
function loadScrubber(scrubberSet: ScrubberSet, opts: { files?: string[]; name?: string } = {}) {
  const scrubber = scrubberSet.load({ files: [sampleFileA], name: scrubberPeerName, ...opts });

  const length = scrubber.end - scrubber.start;
  const halfwayNanos = scrubber.start + length / 2n;

  const makePacketFromScrubber: typeof makePacket = (event, message) => {
    return { ...makePacket(event, message), peer: scrubber.peer };
  };

  return {
    scrubber,
    id: scrubber.data.id,
    startNanos: scrubber.start,
    startTimestamp: scrubber.data.start,
    endNanos: scrubber.end,
    endTimestamp: scrubber.data.end,
    halfwayNanos,
    halfwayTimestamp: nanosToTimestampObject(halfwayNanos),
    length,
    defaultState: { ...scrubber.data },
    makePacketFromScrubber,
  };
}

/** Compute the timestamp for the given message sets and the delta between them */
function getMessageSetTimestampDelta(setA: number, setB: number, offset: number = 0) {
  const timestamp = computeTimestampForMessageSet(setA, offset);
  const nextTimestamp = computeTimestampForMessageSet(setB, offset);

  const delta = Number(timestampObjectToNanos(nextTimestamp) - timestampObjectToNanos(timestamp)) * 1e-9;

  return { delta, timestamp, nextTimestamp };
}
