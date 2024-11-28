import { describe, expect, it, vi } from "vitest";

import { FakeClock } from "../../../shared/time/fake_clock";
import { NbsPacketProcessor, PacketSend } from "../packet_processor";
import { ScrubberSet } from "../scrubber_set";

import { computeTimestampForMessageSet, makePacket, sampleFileA, tick } from "./test_utils";

// Fake timings for acknowledging packets sent to a client
const clientAcknowledgedAt = 0;
const clientReceivedAt = 0;

describe("NbsPacketProcessor", () => {
  it("listens for and forwards packets from scrubbers", async () => {
    const scrubberSet = ScrubberSet.of();

    let ack = async () => {};
    const send: jest.Mock<ReturnType<PacketSend>, Parameters<PacketSend>> = vi.fn((event, packet, ackSend) => {
      ack = async () => {
        ackSend(clientAcknowledgedAt, clientReceivedAt);
        await tick();
      };
    });

    // Create the packet processor and listen for Ping messages
    const processor = NbsPacketProcessor.of(scrubberSet, send);
    processor.listenFor("message.Ping");

    // Load a scrubber for seeking and ack the resulting packet
    const scrubber = scrubberSet.load({ files: [sampleFileA], name: "myScrubber" });
    await ack();

    // Seek to the point where we have the 10th Ping message
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });

    // Seek again to the 20th Ping message
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(20) });

    // Offset send() calls by 1 to account for the initial packet sent on load
    const sendCallsOffset = 1;

    const makePacketFromScrubber: typeof makePacket = (event, message) => {
      return { ...makePacket(event, message), peer: scrubber.peer };
    };

    // Check that the packet processor sent the first seek message. At this point we haven't ack'ed
    // the first message yet, so we should not have sent the second seek message.
    expect(send).toHaveBeenCalledTimes(1 + sendCallsOffset);
    expect(send).toHaveBeenNthCalledWith(
      1 + sendCallsOffset,
      "message.Ping",
      makePacketFromScrubber("message.Ping", "ping.10"),
      expect.any(Function),
    );

    // Ack the first seek message, to send the second one
    await ack();

    // Check that the packet processor sent the second seek message
    expect(send).toHaveBeenCalledTimes(2 + sendCallsOffset);
    expect(send).toHaveBeenNthCalledWith(
      2 + sendCallsOffset,
      "message.Ping",
      makePacketFromScrubber("message.Ping", "ping.20"),
      expect.any(Function),
    );

    // Ack the second seek to clear the ack timeout
    await ack();
  });

  it("times out if forwarded packets are not acknowledged within the configured timeout", async () => {
    const clock = FakeClock.of(0);
    const scrubberSet = ScrubberSet.of({ clock });

    let ack = async () => {};
    const send: jest.Mock<ReturnType<PacketSend>, Parameters<PacketSend>> = vi.fn((event, packet, ackSend) => {
      ack = async () => {
        ackSend(clientAcknowledgedAt, clientReceivedAt);
        await tick();
      };
    });

    // Create the packet processor and listen for Ping messages
    const processor = NbsPacketProcessor.of(scrubberSet, send, { clock, ackTimeout: 100 });
    processor.listenFor("message.Ping");

    // Load a scrubber for seeking and ack the resulting packet
    const scrubber = scrubberSet.load({ files: [sampleFileA], name: "myScrubber" });
    await ack();

    // Seek twice
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(20) });

    // Offset send() calls by 1 to account for the initial packet sent on load
    const sendCallsOffset = 1;

    const makePacketFromScrubber: typeof makePacket = (event, message) => {
      return { ...makePacket(event, message), peer: scrubber.peer };
    };

    // Check that the first seek sent a message
    expect(send).toHaveBeenCalledTimes(1 + sendCallsOffset);
    expect(send).toHaveBeenNthCalledWith(
      1 + sendCallsOffset,
      "message.Ping",
      makePacketFromScrubber("message.Ping", "ping.10"),
      expect.any(Function),
    );

    // Advance the clock to expire the ack timeout
    clock.tick(101);

    // Wait for the expired ack to be handled asynchronously
    await tick();

    // Check that the second seek sent a message after the first ack timed out
    expect(send).toHaveBeenCalledTimes(2 + sendCallsOffset);
    expect(send).toHaveBeenNthCalledWith(
      2 + sendCallsOffset,
      "message.Ping",
      makePacketFromScrubber("message.Ping", "ping.20"),
      expect.any(Function),
    );

    // Advance the clock to expire the second ack timeout
    clock.tick(101);
  });

  it("can stop listening for packets of a specific type", async () => {
    const scrubberSet = ScrubberSet.of();

    const send: jest.Mock<ReturnType<PacketSend>, Parameters<PacketSend>> = vi.fn((packet, timestamp, ack) => {
      // Ack immediately on send
      ack(clientAcknowledgedAt, clientReceivedAt);
    });

    // Create the packet processor and listen for Ping messages
    const processor = NbsPacketProcessor.of(scrubberSet, send);
    const stopListening = processor.listenFor("message.Ping");

    // Load a scrubber for seeking, and wait for the ack
    const scrubber = scrubberSet.load({ files: [sampleFileA], name: "myScrubber" });
    await tick();

    // Seek, and wait for the ack
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });
    await tick();

    // Offset send() calls by 1 to account for the initial packet sent on load
    const sendCallsOffset = 1;

    // Check that the first seek sent a message
    expect(send).toHaveBeenCalledTimes(1 + sendCallsOffset);

    // Stop listening for Ping messages
    stopListening();

    // Seek again, and wait for the ack
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(20) });
    await tick();

    // Check that the second seek did not send a message,
    // since we stopped listening for Ping messages
    expect(send).toHaveBeenCalledTimes(1 + sendCallsOffset);
  });

  it("stops listening for all packet types when destroyed", async () => {
    const scrubberSet = ScrubberSet.of();

    const send: jest.Mock<ReturnType<PacketSend>, Parameters<PacketSend>> = vi.fn((packet, timestamp, ack) => {
      // Ack immediately on send
      ack(clientAcknowledgedAt, clientReceivedAt);
    });

    // Create the packet processor and listen for Ping and Pong messages
    const processor = NbsPacketProcessor.of(scrubberSet, send);
    processor.listenFor("message.Ping");
    processor.listenFor("message.Pong");

    // Load a scrubber for seeking, and wait for the ack
    const scrubber = scrubberSet.load({ files: [sampleFileA], name: "myScrubber" });
    if (scrubber == undefined) {
      throw new Error("scrubber was not loaded successfully");
    }

    await tick();

    // Seek, and wait for the ack
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(10) });
    await tick();

    // Offset send() calls by 2 to account for the packets sent on load and seek
    const sendCallsOffset = 2;

    // Check that the first seek sent both message types
    expect(send).toHaveBeenCalledTimes(2 + sendCallsOffset);

    // Destroy the processor
    processor.destroy();

    // Seek again
    scrubberSet.update({ id: scrubber.data.id, type: "seek", timestamp: computeTimestampForMessageSet(20) });
    await tick();

    // Check that the second seek did not send any messages
    // since we destroyed the processor after the second ack
    expect(send).toHaveBeenCalledTimes(2 + sendCallsOffset);
  });
});
