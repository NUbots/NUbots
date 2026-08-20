import { DescField, isMessage, ScalarType } from "@bufbuild/protobuf";
import { isReflectMessage, reflect, ReflectList, ReflectMap, ReflectMessage } from "@bufbuild/protobuf/reflect";
import { MessageTypeId } from "@components/advanced_scrubber/model";
import { InternalNode, LeafNode, Node } from "@components/collapsible_tree/types";
import { messageNameToType } from "@shared/messages/generated/type_converters";
import { MessageInstance } from "@shared/messages/types";
import { action, IObservableValue, observable } from "mobx";

export interface MessageNodeAttributes {
  value: unknown;
  fieldType: DescField["fieldKind"] | ScalarType;
  typeName: string;
  showRawData?: IObservableValue<boolean>;
}

export type MessageNode = Node<MessageNodeAttributes>;
export type MessageLeafNode = LeafNode<MessageNodeAttributes>;
export type MessageInternalNode = InternalNode<MessageNodeAttributes>;

function createLeafNode(value: unknown, fieldName: string, fieldType: ScalarType | "enum"): MessageLeafNode {
  return {
    type: "leaf",
    name: fieldName,
    value,
    fieldType: fieldType,
    typeName: "scalar",
  };
}

function createMapNode(map: ReflectMap, existingNode?: MessageNode): MessageInternalNode {
  const children: MessageNode[] = [];
  const mapDesc = map.field();

  const keyTypeName = ScalarType[mapDesc.mapKey].toLowerCase();

  let valueTypeName;
  if (mapDesc.mapKind === "message") {
    valueTypeName = mapDesc.message.typeName;
  } else if (mapDesc.mapKind === "enum") {
    valueTypeName = mapDesc.enum.typeName;
  } else {
    valueTypeName = ScalarType[mapDesc.scalar].toLowerCase();
  }

  const typeName = "map" + "<" + keyTypeName + ", " + valueTypeName + ">[" + map.size + "]";

  for (const [key, item] of map.entries()) {
    const childFieldName = String(key);

    // Determine existing child node if available
    const existingChild =
      existingNode?.type === "internal"
        ? existingNode.children.find((child) => child.name === childFieldName)
        : undefined;

    if (mapDesc.mapKind === "enum") {
      const enumString = mapDesc.enum.values[item as number]?.name ?? item;
      children.push(createLeafNode(enumString, childFieldName, "enum"));
    } else if (mapDesc.mapKind === "scalar") {
      children.push(createLeafNode(item, childFieldName, mapDesc.scalar));
    } else if (isReflectMessage(item)) {
      children.push(createMessageNode(item, childFieldName, existingChild));
    }
  }

  return {
    type: "internal",
    name: map.field().localName,
    fieldType: "map",
    typeName,
    value: map,
    isExpanded: existingNode?.type === "internal" && existingNode.name === typeName && existingNode.isExpanded,
    showRawData: observable.box(
      existingNode?.type === "internal" && existingNode.name === typeName && existingNode.showRawData?.get(),
    ),
    children,
  };
}

function createListNode(list: ReflectList, existingNode?: MessageNode): MessageInternalNode {
  const children: MessageNode[] = [];
  const listDesc = list.field();

  let typeName;
  if (listDesc.listKind === "message") {
    typeName = listDesc.message.typeName;
  } else if (listDesc.listKind === "enum") {
    typeName = listDesc.enum.typeName;
  } else {
    typeName = ScalarType[listDesc.scalar].toLowerCase();
  }
  typeName += "[" + list.size + "]";

  for (const [index, item] of list.entries()) {
    // Determine existing child node if available
    const existingChild = existingNode?.type === "internal" ? existingNode?.children[index] : undefined;

    const fieldName = "[" + index.toString() + "]";

    if (listDesc.listKind === "enum") {
      const enumString = listDesc.enum.values[item as number]?.name ?? item;
      children.push(createLeafNode(enumString, fieldName, "enum"));
    } else if (listDesc.listKind === "scalar") {
      children.push(createLeafNode(item, fieldName, listDesc.scalar));
    } else if (isReflectMessage(item)) {
      children.push(createMessageNode(item, fieldName, existingChild));
    }
  }

  return {
    type: "internal",
    fieldType: "list",
    typeName,
    name: list.field().localName,
    value: list,
    isExpanded: existingNode?.type === "internal" && existingNode.name === typeName && existingNode.isExpanded,
    showRawData: observable.box(
      existingNode?.type === "internal" && existingNode.name === typeName && existingNode.showRawData?.get(),
    ),
    children,
  };
}

function createMessageNode(
  message: ReflectMessage,
  fieldName: string,
  existingNode?: MessageNode,
): MessageInternalNode {
  const children: MessageNode[] = [];

  for (const field of message.fields) {
    const existingChild =
      existingNode?.type === "internal"
        ? existingNode.children.find((child) => child.name === field.localName)
        : undefined;

    if (message.isSet(field) === false && message.oneofs.some((oo) => oo.fields.includes(field))) {
      children.push(createLeafNode(null, field.localName, ScalarType.STRING));
    } else if (field.fieldKind === "message") {
      const childMessage = message.get(field);
      children.push(createMessageNode(childMessage, field.localName, existingChild));
    } else if (field.fieldKind === "list") {
      const childList = message.get(field);
      children.push(createListNode(childList, existingChild));
    } else if (field.fieldKind === "map") {
      const childMap = message.get(field);
      children.push(createMapNode(childMap, existingChild));
    } else if (field.fieldKind === "enum") {
      const enumValue = message.get(field);
      const enumString = field.enum.values[enumValue]?.name ?? enumValue;
      children.push(createLeafNode(enumString, field.localName, "enum"));
    } else {
      const leafValue = message.get(field);
      children.push(createLeafNode(leafValue, field.localName, field.scalar));
    }
  }

  const name = fieldName;

  return {
    type: "internal",
    value: message.message,
    fieldType: "message",
    typeName: message.desc.typeName,
    name,
    isExpanded: existingNode?.type === "internal" && existingNode.name === name && existingNode.isExpanded,
    showRawData: observable.box(
      existingNode?.type === "internal" && existingNode.name === name && existingNode.showRawData?.get(),
    ),
    children,
  };
}

export class MessageNodeModel {
  root: MessageInternalNode;

  /**
   * Creates the root node from a message or raw bytes
   */
  private createRootNode(
    message: MessageInstance | Uint8Array,
    type: MessageTypeId,
    existingNode?: MessageInternalNode,
  ): MessageInternalNode {
    if (isMessage(message)) {
      const schema = messageNameToType(message.$typeName).schema;
      const reflectMessage = reflect(schema, message);
      return createMessageNode(reflectMessage, "message", existingNode);
    } else {
      return {
        type: "internal",
        name: "message",
        value: message,
        fieldType: "message",
        typeName: "unknown (" + type.typeHash + ")",
        isExpanded: true,
        showRawData: observable.box(false),
        children: [createLeafNode(message, "bytes", ScalarType.BYTES)],
      };
    }
  }

  constructor(message: MessageInstance | Uint8Array, type: MessageTypeId) {
    this.root = this.createRootNode(message, type);
    // Expand the root node by default
    this.root.isExpanded = true;
  }

  /**
   * Update the tree with a new message while keeping the expansion state
   * of matching fields in the new message.
   */
  @action
  update(message: MessageInstance | Uint8Array, type: MessageTypeId) {
    this.root = this.createRootNode(message, type, this.root);
  }
}
