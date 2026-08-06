import { BinaryReader } from "@bufbuild/protobuf/wire";
import { NUClearNetPacket } from "nuclearnet.js";

import { messageFieldsIndex } from "../../shared/messages/generated/fields_index";

/**
 * Decode and retrieve the id field from the given packet.
 * Returns a default id of 0 if the packet does not have an id field.
 */
export function decodePacketId(typeName: string, packet: NUClearNetPacket) {
  const defaultId = 0;

  const idField = messageFieldsIndex[typeName]?.id;
  if (idField === undefined) {
    return defaultId;
  }

  const reader = new BinaryReader(packet.payload as Uint8Array);
  const end = reader.len;

  while (reader.pos < end) {
    // Tag is of the format: (fieldNumber << 3) | wireType
    // See https://developers.google.com/protocol-buffers/docs/encoding#structure
    const [fieldNumber, wireType] = reader.tag();

    if (fieldNumber === idField.id && wireType === idField.wireType) {
      // Assumes id fields are always 32 or 64-bit unsigned ints
      const id = idField.type === "uint32" ? reader.uint32() : toBigInt(reader.uint64());
      return id;
    }

    reader.skip(wireType, fieldNumber);
  }

  return defaultId;
}

function toBigInt(value: bigint | string): bigint {
  if (typeof value === "bigint") {
    return value;
  }

  return BigInt(value);
}
