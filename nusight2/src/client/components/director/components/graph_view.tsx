import React, { useMemo } from "react";
import ReactFlow, {
  Background,
  Edge,
  Node,
  Position,
  ReactFlowProvider,
  useEdgesState,
  useNodesState,
  Handle,
} from "reactflow";
import "reactflow/dist/style.css";
import dagre from "@dagrejs/dagre";

import { DirectorGraph, GroupModel } from "../model";
import { MarkerType } from "reactflow";
import { ProviderGroupView } from "./provider_group_view";

// Fixed box dimensions for ProviderGroupView (should match CSS)
const NODE_WIDTH = 260;

function estimateHeight(g: GroupModel): number {
  const header = 40; // title + padding
  const providerH = 140; // approx per provider view
  const taskRow = g.subtasks.length ? 32 : 0;
  return header + g.providers.length * providerH + taskRow;
}

/**
 * Convert DirectorGraph into React Flow nodes & edges and run Dagre (top-bottom) layout.
 */
function graphToFlow(graph: DirectorGraph): { nodes: Node[]; edges: Edge[] } {
  const nodes: Node[] = [];
  const edges: Edge[] = [];

  // create nodes with size estimation
  for (const g of Object.values(graph.groupsById)) {
    nodes.push({
      id: g.id,
      type: "providerGroup",
      data: g,
      position: { x: 0, y: 0 }, // dagre will update
      sourcePosition: Position.Bottom,
      targetPosition: Position.Top,
    });
  }

  // edges based on parentProvider (tree that is actually running)
  for (const g of Object.values(graph.groupsById)) {
    if (g.parentProvider && g.parentProvider.group) {
      const srcId = g.parentProvider.group.id;
      const tgtId = g.id;
      const id = `${srcId}->${tgtId}`;
      edges.push({ id, source: srcId, target: tgtId, markerEnd: { type: MarkerType.ArrowClosed } });
    }
  }

  // dagre layout
  const gDagre = new dagre.graphlib.Graph();
  gDagre.setGraph({ rankdir: "TB", nodesep: 40, ranksep: 80 });
  gDagre.setDefaultEdgeLabel(() => ({}));

  nodes.forEach((n) => {
    const h = estimateHeight(n.data as GroupModel);
    gDagre.setNode(n.id, { width: NODE_WIDTH, height: h });
  });
  edges.forEach((e) => gDagre.setEdge(e.source, e.target));

  dagre.layout(gDagre);

  nodes.forEach((n) => {
    const nodeWithPos = gDagre.node(n.id);
    const h = estimateHeight(n.data as GroupModel);
    n.position = { x: nodeWithPos.x - NODE_WIDTH / 2, y: nodeWithPos.y - h / 2 };
  });

  return { nodes, edges };
}

// Custom node renderer that reuses existing ProviderGroupView
function ProviderGroupNode({ data }: { data: GroupModel }) {
  return (
    <div className="relative">
      {/* handles for reactflow connections */}
      <Handle type="target" id="t" position={Position.Top} style={{ opacity: 0 }} />
      <Handle type="source" id="b" position={Position.Bottom} style={{ opacity: 0 }} />
      <ProviderGroupView group={data} />
    </div>
  );
}

const nodeTypes = { providerGroup: ProviderGroupNode };

export function GraphView({ graph }: { graph: DirectorGraph }) {
  const { nodes: initialNodes, edges: initialEdges } = useMemo(() => graphToFlow(graph), [graph]);

  const [nodes, , onNodesChange] = useNodesState(initialNodes);
  const [edges, , onEdgesChange] = useEdgesState(initialEdges);

  return (
    <ReactFlowProvider>
      <div className="w-full h-full relative bg-auto-surface-1">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          nodeTypes={nodeTypes}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          fitView
        >
          <Background gap={32} size={1} />
        </ReactFlow>
      </div>
    </ReactFlowProvider>
  );
}
