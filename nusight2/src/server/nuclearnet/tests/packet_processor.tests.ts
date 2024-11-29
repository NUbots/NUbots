import { NUClearNetPacket } from "nuclearnet.js";
import { describe, expect, it, vi } from "vitest";

import { message } from "../../../shared/messages";
import { FakeClock } from "../../../shared/time/fake_clock";
import { NodeSystemClock } from "../../time/node_clock";
import { hashType } from "../hash_type";
import { LruPriorityQueue } from "../lru_priority_queue";
import { NUClearNetPacketProcessor } from "../packet_processor";

const DataPoint = message.eye.DataPoint;
const Test = message.support.nusight.Test;

function makePacket(typeName: string, opts: { payload: Uint8Array; reliable?: boolean }): NUClearNetPacket {
  return {
    hash: hashType(typeName),
    payload: Buffer.from(opts.payload),
    reliable: opts.reliable ?? false,
    peer: {
      name: "nusight",
      address: "127.0.0.1",
      port: 0,
    },
  };
}

type SendArgs = [event: string, packet: NUClearNetPacket, ack: () => void | undefined];

describe("NUClearNetPacketProcessor", () => {
  it("sends reliable packets through immediately without queuing or dropping when queue capacity is exceeded", () => {
    const typeName = "message.support.nusight.Test";
    const payload = Test.encode({ message: "Test" }).finish();
    const packet = makePacket(typeName, { payload, reliable: true });

    const send = vi.fn();

    const processor = new NUClearNetPacketProcessor(
      send,
      NodeSystemClock,
      new LruPriorityQueue({ capacityPerKey: 2 }),
      {
        outgoingLimit: 2, // Allow 2 packets to be sent before we start queueing and need to wait for an ack
        timeout: 5,
      },
    );

    // Send 4 packets, all reliable
    processor.onPacket(typeName, packet);
    processor.onPacket(typeName, packet);
    processor.onPacket(typeName, packet);
    processor.onPacket(typeName, packet);

    // Since we sent reliably, all packets should have been forwarded without queuing
    // dropping despite the queue limit being 2 per type
    expect(send).toHaveBeenCalledTimes(4);
  });

  it("sends unreliable packets immediately through the queue if the outgoing packet limit is not yet exceeded", () => {
    const typeName = "message.support.nusight.Test";
    const packetA = makePacket(typeName, {
      payload: Test.encode({ message: "Packet A" }).finish(),
      reliable: false,
    });
    const packetB = makePacket(typeName, {
      payload: Test.encode({ message: "Packet B" }).finish(),
      reliable: false,
    });

    const send = vi.fn<void, SendArgs>();

    const processor = new NUClearNetPacketProcessor(
      send,
      NodeSystemClock,
      new LruPriorityQueue({ capacityPerKey: 2 }),
      {
        outgoingLimit: 2, // Allow 2 packets to be sent before we need to wait for an ack
        timeout: 5,
      },
    );

    // Send the two packets
    processor.onPacket(typeName, packetA);
    processor.onPacket(typeName, packetB);

    // Both packets should have been sent immediately without waiting for an ack
    expect(send).toHaveBeenNthCalledWith(1, typeName, packetA, expect.any(Function));
    expect(send).toHaveBeenNthCalledWith(2, typeName, packetB, expect.any(Function));

    // Ack both sends to clear the timeouts and end the test gracefully
    send.mock.calls[0][2]();
    send.mock.calls[1][2]();
  });

  it("queues unreliable packets and waits for ack before sending more if the outgoing packet limit is exceeded", () => {
    const typeName = "message.support.nusight.Test";
    const packetA = makePacket(typeName, {
      payload: Test.encode({ message: "Packet A" }).finish(),
      reliable: false,
    });
    const packetB = makePacket(typeName, {
      payload: Test.encode({ message: "Packet B" }).finish(),
      reliable: false,
    });
    const packetC = makePacket(typeName, {
      payload: Test.encode({ message: "Packet C" }).finish(),
      reliable: false,
    });

    const send = vi.fn<void, SendArgs>();

    const processor = new NUClearNetPacketProcessor(
      send,
      NodeSystemClock,
      new LruPriorityQueue({ capacityPerKey: 2 }),
      {
        outgoingLimit: 2, // Allow 2 packets to be sent before we need to wait for an ack
        timeout: 5,
      },
    );

    // Send three packets
    processor.onPacket(typeName, packetA);
    processor.onPacket(typeName, packetB);
    processor.onPacket(typeName, packetC);

    // The first two packets should have been sent immediately without waiting for an ack
    expect(send).toHaveBeenCalledTimes(2);
    expect(send).toHaveBeenNthCalledWith(1, typeName, packetA, expect.any(Function));
    expect(send).toHaveBeenNthCalledWith(2, typeName, packetB, expect.any(Function));

    // Ack the first send to send the third packet
    send.mock.calls[0][2]();

    // The third packet should have been sent now after the ack
    expect(send).toHaveBeenNthCalledWith(3, typeName, packetC, expect.any(Function));

    // Ack the other two send to clear the timeouts and end the test gracefully
    send.mock.calls[1][2]();
    send.mock.calls[2][2]();
  });

  it("gives up on waiting for ack after the configured timeout", () => {
    const typeName = "message.support.nusight.Test";
    const packetA = makePacket(typeName, {
      payload: Test.encode({ message: "Packet A" }).finish(),
      reliable: false,
    });
    const packetB = makePacket(typeName, {
      payload: Test.encode({ message: "Packet B" }).finish(),
      reliable: false,
    });
    const packetC = makePacket(typeName, {
      payload: Test.encode({ message: "Packet C" }).finish(),
      reliable: false,
    });

    const send = vi.fn<void, SendArgs>();

    const clock = new FakeClock(0);

    const processor = new NUClearNetPacketProcessor(send, clock, new LruPriorityQueue({ capacityPerKey: 2 }), {
      outgoingLimit: 2, // Allow 2 packets to be sent before we need to wait for an ack
      timeout: 5, // Give up on waiting for an ack after 5s
    });

    // Send three packets
    processor.onPacket(typeName, packetA);
    processor.onPacket(typeName, packetB);
    processor.onPacket(typeName, packetC);

    // The first two packets should have been sent immediately without waiting for an ack
    expect(send).toHaveBeenCalledTimes(2);
    expect(send).toHaveBeenNthCalledWith(1, typeName, packetA, expect.any(Function));
    expect(send).toHaveBeenNthCalledWith(2, typeName, packetB, expect.any(Function));

    // Fast forward the clock to trigger the first send's ack timeout
    clock.tick(5);

    // The first send's ack timeout should have triggered a send of the third packet
    expect(send).toHaveBeenNthCalledWith(3, typeName, packetC, expect.any(Function));

    // Ack the other two sends to clear the remaining timeouts and end the test gracefully
    send.mock.calls[1][2]();
    send.mock.calls[2][2]();
  });

  it("uses an LRU queue to ensure high frequency packets don't dominate low frequency packets", () => {
    const dataPointType = "message.eye.DataPoint";
    const testType = "message.support.nusight.Test";

    const dataPointA = makePacket(dataPointType, {
      payload: DataPoint.encode({ label: "DataPoint A" }).finish(),
      reliable: false,
    });
    const dataPointB = makePacket(dataPointType, {
      payload: DataPoint.encode({ label: "DataPoint B" }).finish(),
      reliable: false,
    });
    const dataPointC = makePacket(dataPointType, {
      payload: DataPoint.encode({ label: "DataPoint C" }).finish(),
      reliable: false,
    });
    const dataPointD = makePacket(dataPointType, {
      payload: DataPoint.encode({ label: "DataPoint D" }).finish(),
      reliable: false,
    });

    const testA = makePacket(testType, {
      payload: Test.encode({ message: "Test A" }).finish(),
      reliable: false,
    });
    const testB = makePacket(testType, {
      payload: Test.encode({ message: "Test B" }).finish(),
      reliable: false,
    });

    const send = vi.fn<void, SendArgs>();

    const clock = new FakeClock(0);

    const processor = new NUClearNetPacketProcessor(send, clock, new LruPriorityQueue({ capacityPerKey: 4 }), {
      outgoingLimit: 2, // Allow 2 packets to be sent before we need to wait for an ack
      timeout: 5, // Give up on waiting for an ack after 5s
    });

    // Send all the packets, with the data points first
    processor.onPacket(dataPointType, dataPointA);
    processor.onPacket(dataPointType, dataPointB);
    processor.onPacket(dataPointType, dataPointC);
    processor.onPacket(dataPointType, dataPointD);
    processor.onPacket(testType, testA);
    processor.onPacket(testType, testB);

    // Ack all the sends except the last one which won't need an ack,
    // since by the time the last send is done the queue will be empty
    for (let i = 0; i < 5; i++) {
      send.mock.calls[i][2]();
    }

    // All 6 packets should have been sent
    expect(send).toHaveBeenCalledTimes(6);

    // Check the send order:

    // The first two data points should have been sent immediately without waiting for acks
    // (since 2 is our outgoing limit)
    expect(send).toHaveBeenNthCalledWith(1, dataPointType, dataPointA, expect.any(Function));
    expect(send).toHaveBeenNthCalledWith(2, dataPointType, dataPointB, expect.any(Function));

    // The remaining packets should have been sent from the queue after the first ack,
    // starting with a DataPoint packet ...
    expect(send).toHaveBeenNthCalledWith(3, dataPointType, dataPointC, expect.any(Function));

    // ... then a Test packet ...
    expect(send).toHaveBeenNthCalledWith(4, testType, testA, expect.any(Function));

    // ... then another DataPoint packet ...
    expect(send).toHaveBeenNthCalledWith(5, dataPointType, dataPointD, expect.any(Function));

    // ... then the last Test packet
    expect(send).toHaveBeenNthCalledWith(6, testType, testB, expect.any(Function));
  });
});
