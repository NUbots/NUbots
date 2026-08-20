import { DescEnum, DescMessage } from "@bufbuild/protobuf";
import { Schema } from "@bufbuild/protoplugin";

type Types = { messages: DescMessage[]; enums: DescEnum[] };

/**
 * Get all types (messages and enums) in the given schema in a flat list grouped by kind.
 */
export function getAllTypes(schema: Schema) {
  const types: Types = {
    messages: [],
    enums: [],
  };

  for (const file of schema.files) {
    // Add top-level enums in the file
    for (const enumVar of file.enums) {
      types.enums.push(enumVar);
    }

    // Recursively add top-level messages and nested messages/enums
    for (const message of file.messages) {
      addNestedTypes(types, message);
    }
  }

  return types;
}

function addNestedTypes(types: Types, messageSchema: DescMessage) {
  types.messages.push(messageSchema);

  // Add enums nested in the current message
  for (const enumVar of messageSchema.nestedEnums) {
    types.enums.push(enumVar);
  }

  // Recursively add messages and enums nested in the current message
  for (const message of messageSchema.nestedMessages) {
    addNestedTypes(types, message);
  }
}

interface TypeGroup {
  /**
   * The path of the file containing the types (without the extension), relative to the protobuf message root folder.
   * E.g. for `message/eye/labeller/LabelFile.proto`, the location is `eye/labeller/LabelFile`.
   */
  location: string;

  /**
   * The file name without the path or extension.
   * E.g. for `message/eye/labeller/LabelFile.proto`, the fileName is `LabelFile`.
   */
  fileName: string;

  /** The messages in the group */
  messages: {
    /** The name of the message, without the package name. E.g. `ScrubberPlayRequest` */
    name: string;
    /** The full type name of the message, including the package name. E.g. `message.eye.ScrubberPlayRequest` */
    typeName: string;
  }[];

  /** The enums in the group */
  enums: {
    /** The name of the enum, without the package name. E.g. `ScrubberState_State` */
    name: string;
    /** The full type name of the enum, including the package name. E.g. `message.eye.ScrubberState_State` */
    typeName: string;
    /** The numeric values of the enum cases, keyed by the case name */
    values: Record<string, number>;
  }[];
}

/**
 * Get all types (messages and enums) in the given schema grouped by the location (i.e. file) they are defined in.
 */
export function getTypesGroupedByLocation(schema: Schema) {
  const groups: TypeGroup[] = [];
  const { messages, enums } = getAllTypes(schema);

  // Add messages, grouped by location
  for (const message of messages) {
    const group = groups.find((group) => group.location === message.file.name);
    if (group) {
      group.messages.push(getNameAndTypeName(message));
    } else {
      groups.push({
        location: message.file.name,
        fileName: message.file.name.substring(message.file.name.lastIndexOf("/") + 1, message.file.name.length),
        messages: [getNameAndTypeName(message)],
        enums: [],
      });
    }
  }

  // Add enums, grouped by location
  for (const enumDesc of enums) {
    const enumVar = {
      ...getNameAndTypeName(enumDesc),
      values: getEnumValues(enumDesc),
    };

    const group = groups.find((group) => group.location === enumDesc.file.name);

    if (group) {
      group.enums.push(enumVar);
    } else {
      groups.push({
        location: enumDesc.file.name,
        fileName: enumDesc.file.name.substring(enumDesc.file.name.lastIndexOf("/") + 1, enumDesc.file.name.length),
        messages: [],
        enums: [enumVar],
      });
    }
  }

  return groups;
}

function getNameAndTypeName(message: DescMessage | DescEnum) {
  return {
    name:
      // Get the message name from the typeName by removing the package name and replacing dots with underscores to
      // account for nested messages. For example:
      //   - Top-level message with typeName `message.eye.ScrubberPlayRequest` will become `ScrubberPlayRequest`
      //   - Nested message with typeName `message.eye.ScrubberPlayRequest.Response` will become `ScrubberPlayRequest_Response`
      message.typeName.replace(`${message.file.proto.package}.`, "").split(".").join("_"),
    typeName: message.typeName,
  };
}

function getEnumValues(enumVar: DescEnum) {
  return Object.fromEntries(enumVar.values.map((value) => [value.name, value.number]));
}
