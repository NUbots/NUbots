import React from "react";
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
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [translateExtent, setTranslateExtent] = React.useState<[[number, number], [number, number]] | undefined>(
    undefined,
  );
  const [minZoom, setMinZoom] = React.useState(0.2);
  const wrapperRef = React.useRef<HTMLDivElement>(null);

  // recompute flow whenever graph observable changes
  React.useEffect(() => {
    const { nodes: newNodes, edges: newEdges } = graphToFlow(graph);
    setNodes(newNodes);
    setEdges(newEdges);

    // compute bounding box for pan limits
    if (newNodes.length) {
      let minX = Infinity,
        minY = Infinity,
        maxX = -Infinity,
        maxY = -Infinity;
      newNodes.forEach((n) => {
        minX = Math.min(minX, n.position.x);
        minY = Math.min(minY, n.position.y);
        maxX = Math.max(maxX, n.position.x + NODE_WIDTH);
        const h = estimateHeight(n.data as GroupModel);
        maxY = Math.max(maxY, n.position.y + h);
      });
      const padding = 200;
      const ext: [[number, number], [number, number]] = [
        [minX - padding, minY - padding],
        [maxX + padding, maxY + padding],
      ];
      setTranslateExtent(ext);

      // calculate minZoom so that entire extent fits in view
      if (wrapperRef.current) {
        const w = wrapperRef.current.clientWidth;
        const h = wrapperRef.current.clientHeight;
        const extW = ext[1][0] - ext[0][0];
        const extH = ext[1][1] - ext[0][1];
        if (extW > 0 && extH > 0 && w > 0 && h > 0) {
          const z = Math.min(w / extW, h / extH) * 0.9; // 10% margin
          setMinZoom(z);
        }
      }
    }
  }, [graph, setNodes, setEdges]);

  return (
    <ReactFlowProvider>
      <div ref={wrapperRef} className="w-full h-full relative bg-auto-surface-1">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          nodeTypes={nodeTypes}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          fitView
          minZoom={minZoom}
          maxZoom={2}
          zoomOnScroll
          zoomOnPinch
          panOnScroll
          nodesDraggable={false}
          nodesConnectable={false}
          elementsSelectable={true}
          translateExtent={translateExtent}
        >
          <Background gap={32} size={1} />
        </ReactFlow>
      </div>
    </ReactFlowProvider>
  );
}
