import { GenMessage } from "@bufbuild/protobuf/codegenv2";

/** An instance of a Protobuf message with a known type name and a method for conversion to binary */
export type MessageInstance<TName extends string = string> = {
  $typeName: TName;
  toBinary: () => Uint8Array;
};

/** A class that describes a Protobuf message with a known type name and the ability to create a new instance from binary data */
export interface MessageType<T extends MessageInstance> {
  new (...args: any[]): T;
  typeName: T["$typeName"];
  schema: GenMessage<any>;
  fromBinary(data: Uint8Array): T;
}
