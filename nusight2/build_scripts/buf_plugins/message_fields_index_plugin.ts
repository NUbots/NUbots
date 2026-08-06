import { FieldDescriptorProto_Type } from "@bufbuild/protobuf/wkt";
import { createEcmaScriptPlugin, runNodeJs, Schema } from "@bufbuild/protoplugin";

import { getAllTypes } from "./get_types";

const plugin = createEcmaScriptPlugin({
  name: "message-fields-plugin",
  version: "v1",
  generateTs,
});

// Data types to protobuf wire types
export const protobufWireTypes: Record<string, number> = {
  // Varint fields
  int32: 0,
  int64: 0,
  uint32: 0,
  uint64: 0,
  sint32: 0,
  sint64: 0,
  bool: 0,
  enum: 0,

  // 64-bit fields
  fixed64: 1,
  sfixed64: 1,
  double: 1,

  // Length-delimited fields
  string: 2,
  bytes: 2,

  // Note: wire types for composite fields (i.e. embedded messages, groups, packed repeated fields)
  // are omitted from this list

  // 32-bit fields
  fixed32: 5,
  sfixed32: 5,
  float: 5,
};

interface MessageField {
  wireType: number;
  type: string;
  id: number;
}

interface MessageFieldsIndex {
  [messageName: string]: {
    [fieldName: string]: MessageField;
  };
}

function generateTs(schema: Schema) {
  const f = schema.generateFile("fields_index.ts");
  const { messages } = getAllTypes(schema);

  const index: MessageFieldsIndex = {};
  const fieldsToIndex = ["id"];

  for (const message of messages) {
    for (const field of message.fields) {
      if (fieldsToIndex.includes(field.name)) {
        const indexEntry = index[field.parent.typeName] ?? {};

        indexEntry[field.name] = {
          type: FieldDescriptorProto_Type[field.proto.type].toLowerCase(),
          id: field.proto.number,
          wireType: protobufWireTypes[FieldDescriptorProto_Type[field.proto.type].toLowerCase()],
        } as MessageField;

        index[field.parent.typeName] = indexEntry;
      }
    }
  }

  const types = `
export interface MessageField {
  wireType: number
  type: string
  id: number
}

export interface MessageFieldsIndex {
  [messageName: string]: {
    [fieldName: string]: MessageField
  }
}
  `.trim();

  const file =
    types + "\n\n" + "export const messageFieldsIndex: MessageFieldsIndex = " + JSON.stringify(index, null, "  ");

  f.print(file);
}

runNodeJs(plugin);
