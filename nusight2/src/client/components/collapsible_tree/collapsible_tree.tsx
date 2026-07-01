import React, { useCallback, useState } from "react";
import { Icon } from "@components/icon/view";

import { NodeContentComponentType } from "./types";
import { NodeContentProps } from "./types";
import { NodeLabelComponentType } from "./types";
import { NodeLabelProps } from "./types";
import { InternalNode } from "./types";

function DefaultNodeLabel(props: NodeLabelProps) {
  const { node, level, toggleButton } = props;
  return (
    <div className="flex items-center" style={{ paddingLeft: 16 * level + "px" }}>
      {toggleButton}
      <span>{node.name}</span>
    </div>
  );
}

function DefaultNodeContent(props: NodeContentProps) {
  return <>{props.children()}</>;
}

export interface CollapsibleTreeProps<T = {}> {
  root: InternalNode<T>;
  NodeLabel?: NodeLabelComponentType<T>;
  NodeContent?: NodeContentComponentType<T>;
}

/** A collapsible tree, supporting custom node types */
export function CollapsibleTree<T>(props: CollapsibleTreeProps<T>) {
  const { root } = props;

  // Use the default components if they aren't given
  const NodeLabel = props.NodeLabel ?? DefaultNodeLabel;
  const NodeContent = props.NodeContent ?? DefaultNodeContent;

  return <InternalNodeView node={root} level={0} path={root.name} NodeLabel={NodeLabel} NodeContent={NodeContent} />;
}

interface InternalNodeViewProps {
  node: InternalNode;
  path: string;
  level: number;
  NodeLabel: NodeLabelComponentType;
  NodeContent: NodeContentComponentType;
}

/** An internal node in the collapsible tree */
function InternalNodeView(props: InternalNodeViewProps) {
  const { node, path, level, NodeLabel, NodeContent } = props;
  const [isExpanded, setIsExpanded] = useState(node.isExpanded);

  const toggleExpanded = useCallback(() => {
    setIsExpanded((value) => {
      node.isExpanded = !value;
      return !value;
    });
  }, [setIsExpanded, node]);

  return (
    <div className="flex flex-col">
      <NodeLabel
        path={path}
        level={level}
        node={node}
        toggleButton={
          <button className="flex items-center w-5 h-5 shrink-0" onClick={toggleExpanded}>
            <Icon size="20" rotate={isExpanded ? "90" : "0"} className="transition-transform text-auto-hint">
              chevron_right
            </Icon>
          </button>
        }
      />
      {isExpanded ? (
        <NodeContent level={level} node={node} path={path}>
          {() => (
            <NodeContentView node={node} path={path} level={level} NodeLabel={NodeLabel} NodeContent={NodeContent} />
          )}
        </NodeContent>
      ) : null}
    </div>
  );
}

interface NodeChildrenViewProps {
  node: InternalNode;
  path: string;
  level: number;
  NodeLabel: NodeLabelComponentType;
  NodeContent: NodeContentComponentType;
}

/** An unordered list of a node's children */
function NodeContentView(props: NodeChildrenViewProps) {
  const { node, path, level, NodeLabel, NodeContent } = props;
  return (
    <ul>
      {node.children.map((node) => {
        return (
          <li key={node.name}>
            {node.type === "internal" ? (
              <InternalNodeView
                path={path + "." + node.name}
                level={level + 1}
                node={node}
                NodeLabel={NodeLabel}
                NodeContent={NodeContent}
              />
            ) : (
              <NodeLabel
                path={path + "." + node.name}
                level={level + 1}
                node={node}
                toggleButton={<div className="w-5 h-5 shrink-0" />}
              />
            )}
          </li>
        );
      })}
    </ul>
  );
}
