import React, { useCallback } from "react";
import { ScalarType } from "@bufbuild/protobuf";
import { siUnit } from "@client/base/si_unit";
import { CollapsibleTree } from "@components/collapsible_tree/collapsible_tree";
import { InternalNode, NodeContentProps, NodeLabelProps } from "@components/collapsible_tree/types";
import { Icon } from "@components/icon/view";
import { useTimedState } from "@hooks/use_timed_state";
import { useUpdatable } from "@hooks/use_updatable";
import { Timestamp } from "@shared/time/timestamp";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import { MessageViewModel } from "../model";

import { BytesView } from "./custom_formats/bytes";
import { formatValue } from "./custom_formats/custom_formats";
import { MessageNode, MessageNodeAttributes, MessageNodeModel } from "./model";

const ColorPalette = {
  key: "text-[#0F0E7D] dark:text-[#9CDCFE]", // The color of keys of the message fields
  type: "text-[#795E26] dark:text-[#DCDCAA]", // The color of the type name of a message field if it is not a primitive
  string: "text-[#a31515] dark:text-[#ce9178]", // The color of a string value
  number: "text-[#098658] dark:text-[#b5cea8]", // The color of a number value
  boolean: "text-[#0000FF] dark:text-[#569CD6]", // The color of a boolean value
  enum: "text-[#267F99] dark:text-[#4EC9B0]", // The color of an enum value
  error: "text-[#CD3131] dark:text-[#C586C0]", // The color of an unknown value
  undefined: "text-[#6c6c6c] dark:text-[#939393]", // The color of an undefined value
} as const;

/** Stringify a value, converting BigInt to a string ending in "n" */
function stringifyWithBigInt(object: unknown): string {
  return JSON.stringify(object, (_, val) => (typeof val === "bigint" ? val.toString() + "n" : val), 4);
}

/** Display for a primitive value of a message */
function MessageFieldPrimitive(props: Readonly<{ value: unknown; type: MessageNodeAttributes["fieldType"] }>) {
  const { value } = props;

  if (value === null || value === undefined) {
    return <span className={ColorPalette.undefined}>undefined</span>;
  }

  switch (props.type) {
    case ScalarType.STRING:
      return <span className={ColorPalette.string}>{`"${value}"`}</span>;
    case ScalarType.BOOL:
      return <span className={ColorPalette.boolean}>{String(value)}</span>;
    case "enum":
      return <span className={ColorPalette.enum}>{String(value)}</span>;
    case ScalarType.DOUBLE:
    case ScalarType.FLOAT:
    case ScalarType.INT64:
    case ScalarType.UINT64:
    case ScalarType.INT32:
    case ScalarType.UINT32:
    case ScalarType.FIXED64:
    case ScalarType.FIXED32:
    case ScalarType.SFIXED32:
    case ScalarType.SFIXED64:
    case ScalarType.SINT32:
    case ScalarType.SINT64:
      return (
        <>
          <span className={ColorPalette.type}>{ScalarType[props.type].toLowerCase()} = </span>
          <span className={ColorPalette.number}>{Number(value)}</span>
        </>
      );
    case ScalarType.BYTES:
      if (value instanceof Uint8Array) {
        return (
          <div style={{ paddingLeft: "15px" }}>
            <BytesView value={value} />
          </div>
        );
      } else {
        return <span className={ColorPalette.error}>{String(value)}</span>;
      }
    default:
      return <span className={ColorPalette.error}>{String(value)}</span>;
  }
}

const MessageFieldType = observer(function MessageFieldType(props: {
  node: InternalNode<MessageNodeAttributes>;
  isExpanded: boolean;
}) {
  const { typeName, value, showRawData } = props.node;
  const showingRawData = showRawData?.get();

  // Whether the value was copied, used to show feedback when copying
  const [copied, setIsCopied] = useTimedState(false, 2000);
  const copyValue = useCallback(() => {
    navigator.clipboard.writeText(stringifyWithBigInt(value)).then(() => {
      setIsCopied(true);
    });
  }, [value, setIsCopied]);

  const hasCustomDisplay = formatValue(value) !== null;
  return (
    <>
      <span className={classNames(ColorPalette.type, "whitespace-nowrap pr-2")}>{typeName}</span>
      {props.isExpanded ? (
        <button
          title="Copy JSON"
          className={classNames(
            "w-4 h-4 mt-0.5 rounded flex justify-center cursor-pointer select-none active:ring-1 ring-inset ring-auto",
            copied ? "text-green-800 dark:text-green-300" : "text-auto-icon-button",
          )}
          onClick={copyValue}
        >
          <Icon className="text-sm -mt-0.5">{copied ? "check" : "content_copy"}</Icon>
        </button>
      ) : null}
      {props.isExpanded && hasCustomDisplay ? (
        <button
          title={showingRawData ? "Show formatted data" : "Show raw data"}
          className={classNames(
            "w-4 h-4 mt-0.5 rounded flex justify-center text-auto-icon-button cursor-pointer select-none ml-0.5 ring-inset ring-auto",
            showingRawData ? "bg-transparent active:ring-1" : "bg-auto-contrast-3 ring-1",
          )}
          onClick={action(() => showRawData?.set(!showingRawData))}
        >
          <Icon className="text-sm -mt-0.5">data_object</Icon>
        </button>
      ) : null}
    </>
  );
});

/**
 * Displays the field key and value for primitives, and the field key and type for non-primitives. For non-primitives,
 * the value will be displayed separately using MessageFieldChildren.
 */
function MessageField(props: NodeLabelProps<MessageNode>) {
  const { level, node, toggleButton } = props;
  return (
    <div
      style={{ paddingLeft: `${15 * level}px` }}
      className="hover:bg-auto-contrast-2 cursor-default w-full whitespace-pre-wrap flex leading-5"
    >
      {toggleButton}
      {/* For object nodes which display a (key, type) pair, use flex so they don't wrap */}
      <div className={node.type === "leaf" ? "block break-all" : "flex"}>
        <span className="pr-2">
          <span className={ColorPalette.key}>{node.name}</span>:
        </span>
        {node.type === "leaf" ? (
          <MessageFieldPrimitive value={node.value} type={node.fieldType} />
        ) : (
          <MessageFieldType node={node} isExpanded={node.isExpanded} />
        )}
      </div>
    </div>
  );
}

/** Displays the nested child content of a non-primitive message field */
const MessageFieldChildren = observer(function MessageFieldChildren(props: NodeContentProps<MessageNode>) {
  const { node, level, children } = props;
  if (node.type === "leaf") {
    return null;
  }

  // Pad the content depending on the level in the tree
  const padding = 15 * level + 40;

  // Display a message if the node is an empty object
  if (node.children.length === 0) {
    return (
      <div style={{ paddingLeft: padding + "px" }} className="hover:bg-auto-contrast-2 text-auto-hint">
        (empty)
      </div>
    );
  }

  // Get a custom display for the node's value if it has one
  const formatted = node.showRawData?.get() ? null : formatValue(node.value);

  return formatted ? (
    <>
      <div style={{ paddingLeft: padding + "px" }} className="hover:bg-auto-contrast-2 py-1">
        {formatted.content}
      </div>
      {formatted.keepOriginal ? children() : null}
    </>
  ) : (
    <>{children()}</>
  );
});

function MessageStat(props: { name: string; children: React.ReactNode }) {
  return (
    <div className="flex justify-between gap-4 w-full text-sm py-0.5">
      <span className="text-auto-secondary">{props.name}</span>
      {props.children}
    </div>
  );
}

export function MessageView({ viewModel }: { viewModel: MessageViewModel }) {
  const [bytes, units] = siUnit(viewModel.size, "b");
  const content = useUpdatable(
    () => new MessageNodeModel(viewModel.message, viewModel.type),
    (instance, [message]) => instance.update(message, viewModel.type),
    [viewModel.message],
  );

  const timestampNanos = Timestamp.toNanos(viewModel.timestamp);

  return (
    <div className="p-4 flex flex-col h-full gap-2 bg-auto-surface-2">
      <div className="relative h-full border border-auto rounded overflow-hidden">
        <div className="absolute w-full h-full top-0 right-0 py-1 overflow-y-auto text-xs font-mono">
          <CollapsibleTree root={content.root} NodeLabel={MessageField} NodeContent={MessageFieldChildren} />
        </div>
      </div>
      <div className="divide-y">
        <MessageStat name="Timestamp">
          <span className="truncate">{Timestamp.toFormattedDate(timestampNanos)}</span>
        </MessageStat>
        <MessageStat name="Size">
          <span>{bytes.toFixed(2) + units}</span>
        </MessageStat>
      </div>
    </div>
  );
}
