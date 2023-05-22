import { NUClearNetPacket } from "nuclearnet.js";
import { Reader } from "protobufjs/minimal";

import { messageFieldsIndex } from "../../shared/messages/fields_index";

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

  const reader = Reader.create(packet.payload);
  const end = reader.len;

  while (reader.pos < end) {
    // Tag is of the format: (fieldNumber << 3) | wireType
    // See https://developers.google.com/protocol-buffers/docs/encoding#structure
    const tag = reader.uint32();

    const fieldNumber = tag >>> 3;
    const wireType = tag & 7;

    if (fieldNumber === idField.id && wireType === idField.wireType) {
      // Assumes id fields are always 32 or 64-bit unsigned ints
      const id = idField.type === "uint32" ? reader.uint32() : reader.uint64();
      return id;
    }

    reader.skipType(wireType);
  }

  return defaultId;
}
