import { NUClearNetPacket } from "nuclearnet.js";
import { describe, expect, it } from "vitest";

import { message } from "../../../shared/messages";
import { hashType } from "../../../shared/nuclearnet/hash_type";
import { decodePacketId } from "../decode_packet_id";

const DataPoint = message.eye.DataPoint;
const Test = message.network.Test;

function makePacket(typeName: string, payload: Uint8Array): NUClearNetPacket {
  return {
    hash: hashType(typeName),
    payload: Buffer.from(payload),
    reliable: true,
    peer: {
      name: "nusight",
      address: "127.0.0.1",
      port: 0,
    },
  };
}

describe("decodePacketId()", () => {
  it("decodes the id for messages with an id field", () => {
    const payload = DataPoint.encode({ id: 123 }).finish();

    const packet = makePacket("message.eye.DataPoint", payload);

    expect(decodePacketId("message.eye.DataPoint", packet)).toBe(123);
  });

  it("returns a default id of 0 for messages without an id field", () => {
    const payload = Test.encode({ message: "This is a test" }).finish();

    const packet = makePacket("message.network.Test", payload);

    expect(decodePacketId("message.network.Test", packet)).toBe(0);
  });
});
