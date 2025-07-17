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
  useReactFlow,
} from "reactflow";
import "reactflow/dist/style.css";
import dagre from "@dagrejs/dagre";

import { DirectorGraph, GroupModel } from "../model";
import { MarkerType } from "reactflow";
import { ProviderGroupView } from "./provider_group_view";

/**
 * Convert DirectorGraph into React Flow nodes & edges.
 * The actual layout (positions) is performed later once the real
 * rendered size of each node is known.
 */
function graphToFlow(graph: DirectorGraph): { nodes: Node[]; edges: Edge[] } {
  const nodes: Node[] = [];
  const edges: Edge[] = [];

  // Create a node for every provider group (position is a placeholder for now)
  for (const g of Object.values(graph.groupsById)) {
    nodes.push({
      id: g.id,
      type: "providerGroup",
      data: g,
      position: { x: 0, y: 0 },
      sourcePosition: Position.Bottom,
      targetPosition: Position.Top,
      className: "transition-transform duration-300 ease-in-out",
    });
  }

  // Edges based on parentProvider (tree that is actually running)
  for (const g of Object.values(graph.groupsById)) {
    if (g.parentProvider && g.parentProvider.group) {
      const srcId = g.parentProvider.group.id;
      const tgtId = g.id;
      const id = `${srcId}->${tgtId}`;
      edges.push({ id, source: srcId, target: tgtId, markerEnd: { type: MarkerType.ArrowClosed } });
    }
  }

  // ---------------------------------------------------------------------------
  // Secondary "blocked" edges – tasks that request a target group that is not
  // currently controlled by this group (i.e. the target group's parentProvider
  // is *not* from this source group). These are rendered as dashed grey lines.
  // ---------------------------------------------------------------------------

  for (const g of Object.values(graph.groupsById)) {
    for (const task of g.subtasks ?? []) {
      const target = task.targetGroup;
      if (!target) continue;

      // A task is considered "blocked" when the target group's parentProvider
      // is undefined or originates from a *different* group than the source.
      const controlledBySource = target.parentProvider?.group?.id === g.id;
      if (controlledBySource) continue; // not blocked – handled by control edge

      const edgeId = `blocked:${g.id}->${target.id}`;

      // Prevent duplicates in case multiple tasks create the same blocked edge
      if (edges.some((e) => e.id === edgeId)) continue;

      edges.push({
        id: edgeId,
        source: g.id,
        target: target.id,
        markerEnd: { type: MarkerType.ArrowClosed },
        style: { strokeDasharray: "4 4", stroke: "#9ca3af" }, // tailwind gray-400
      });
    }
  }

  // ---------------------------------------------------------------------------
  // Needs edges – overlay or create edges based on provider.needs relationships.
  // If an edge already exists between source and target (control/blocked), we
  // recolour it to indicate a "needs" relationship. Otherwise, for groups that
  // currently have NO active provider, we add a new edge (solid/dashed based on
  // control) provided the provider's `when` conditions are satisfied.
  // ---------------------------------------------------------------------------

  const NEED_COLOR = "#8b5cf6"; // Tailwind violet-500

  // Helper to key by src->tgt regardless of edge id prefix
  function key(src: string, tgt: string) {
    return `${src}->${tgt}`;
  }

  const edgeMap = new Map<string, Edge>();
  edges.forEach((e) => edgeMap.set(key(e.source, e.target), e));

  for (const provider of Object.values(graph.providersById)) {
    const srcGroup = provider.group;

    // Only consider providers whose when conditions are met (or have none)
    const whenOk = (provider.when ?? []).every((w) => w.current !== false);
    if (!whenOk) continue;

    for (const tgtGroup of provider.needs ?? []) {
      if (!tgtGroup) continue;

      const k = key(srcGroup.id, tgtGroup.id);
      const controlsTarget = tgtGroup.parentProvider?.group?.id === srcGroup.id;

      if (edgeMap.has(k)) {
        // Transform existing edge to needs styling
        const e = edgeMap.get(k)!;
        e.style = {
          ...(e.style ?? {}),
          stroke: NEED_COLOR,
        };
        // Keep dash pattern if it existed (for blocked)
        e.markerEnd = { type: MarkerType.ArrowClosed, color: NEED_COLOR } as any;
      } else {
        // Only introduce new edges for groups with no active provider (fallback)
        if (srcGroup.activeProvider) continue;

        const newId = `need:${srcGroup.id}->${tgtGroup.id}`;
        const newEdge: Edge = {
          id: newId,
          source: srcGroup.id,
          target: tgtGroup.id,
          markerEnd: { type: MarkerType.ArrowClosed, color: NEED_COLOR } as any,
          style: {
            stroke: NEED_COLOR,
            ...(controlsTarget ? {} : { strokeDasharray: "4 4" }),
          },
        };
        edges.push(newEdge);
        edgeMap.set(k, newEdge);
      }
    }
  }

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
  // new global style injection
  // Inject CSS once to animate edge path 'd' attribute changes
  React.useEffect(() => {
    const style = document.createElement("style");
    style.innerHTML = `.react-flow__edge-path { transition: d 0.3s ease-in-out; }`;
    document.head.appendChild(style);
    return () => {
      document.head.removeChild(style);
    };
  }, []);
  const [minZoom, setMinZoom] = React.useState(0.2);
  const wrapperRef = React.useRef<HTMLDivElement>(null);
  // refs to compare values and avoid endless updates
  const translateExtentRef = React.useRef<[[number, number], [number, number]] | undefined>(undefined);
  const minZoomRef = React.useRef<number>(0.2);

  // ---------------------------------------------------------------------------
  // Build initial flow graph whenever the underlying DirectorGraph changes
  // ---------------------------------------------------------------------------

  // recompute flow whenever graph observable changes
  React.useEffect(() => {
    const { nodes: baseNodes, edges: newEdges } = graphToFlow(graph);

    // Reuse previous positions (and measured sizes) where possible to avoid flicker
    setNodes((prev) => {
      const posMap = new Map<string, { x: number; y: number }>(prev.map((n) => [n.id, n.position]));
      const sizeMap = new Map<string, { width?: number; height?: number }>(
        prev.map((n) => [n.id, { width: n.width ?? undefined, height: n.height ?? undefined }]),
      );

      const merged = baseNodes.map((n) => {
        const pos = posMap.get(n.id);
        const size = sizeMap.get(n.id);
        return pos || size ? { ...n, position: pos ?? n.position, ...size } : n;
      });

      return merged;
    });

    // update edges directly (no prior state to preserve)
    setEdges(newEdges);
  }, [graph]);

  // ---------------------------------------------------------------------------
  // LayoutUpdater: measures real node sizes, performs dagre layout, and
  // updates viewport limits.
  // ---------------------------------------------------------------------------

  function LayoutUpdater() {
    const { getNodes } = useReactFlow();

    React.useLayoutEffect(() => {
      const rfNodes = getNodes();
      if (!rfNodes.length) return;

      // Ensure all nodes have measured dimensions
      if (rfNodes.some((n: any) => n.width == null || n.height == null)) return;

      // Dagre layout with real sizes
      const gDagre = new dagre.graphlib.Graph();
      gDagre.setGraph({ rankdir: "TB", nodesep: 40, ranksep: 80 });
      gDagre.setDefaultEdgeLabel(() => ({}));

      rfNodes.forEach((n: Node) => {
        gDagre.setNode(n.id, { width: n.width!, height: n.height! });
      });
      edges.forEach((e) => gDagre.setEdge(e.source, e.target));

      dagre.layout(gDagre);

      // Build new node positions and update state only if something actually changed
      const nextNodes = nodes.map((n) => {
        const pos = gDagre.node(n.id);
        if (!pos) return n;
        const referenceNode = rfNodes.find((r: Node) => r.id === n.id);
        const width = n.width ?? referenceNode?.width ?? 0;
        const height = n.height ?? referenceNode?.height ?? 0;
        const newX = pos.x - width / 2;
        const newY = pos.y - height / 2;
        if (n.position.x !== newX || n.position.y !== newY) {
          return { ...n, position: { x: newX, y: newY } };
        }
        return n;
      });

      const changed = nextNodes.some((n, idx) => {
        const o = nodes[idx];
        return o.position.x !== n.position.x || o.position.y !== n.position.y;
      });

      if (changed) {
        setNodes(nextNodes);
      }

      // Compute translate extent & minZoom based on laid-out nodes
      let minX = Infinity,
        minY = Infinity,
        maxX = -Infinity,
        maxY = -Infinity;
      rfNodes.forEach((n: Node) => {
        minX = Math.min(minX, n.position.x);
        minY = Math.min(minY, n.position.y);
        maxX = Math.max(maxX, n.position.x + (n.width ?? 0));
        maxY = Math.max(maxY, n.position.y + (n.height ?? 0));
      });
      const padding = 200;
      const ext: [[number, number], [number, number]] = [
        [minX - padding, minY - padding],
        [maxX + padding, maxY + padding],
      ];
      // Update translateExtent only if it changed (shallow compare numbers)
      const prevExt = translateExtentRef.current;
      const extChanged =
        !prevExt ||
        prevExt[0][0] !== ext[0][0] ||
        prevExt[0][1] !== ext[0][1] ||
        prevExt[1][0] !== ext[1][0] ||
        prevExt[1][1] !== ext[1][1];
      if (extChanged) {
        translateExtentRef.current = ext;
        setTranslateExtent(ext);
      }

      // Fit-to-view zoom
      if (wrapperRef.current) {
        const w = wrapperRef.current.clientWidth;
        const h = wrapperRef.current.clientHeight;
        const extW = ext[1][0] - ext[0][0];
        const extH = ext[1][1] - ext[0][1];
        if (extW > 0 && extH > 0 && w > 0 && h > 0) {
          const z = Math.min(w / extW, h / extH) * 0.9; // 10% margin
          if (Math.abs(z - minZoomRef.current) > 0.0001) {
            minZoomRef.current = z;
            setMinZoom(z);
          }
        }
      }
    }, [edges, getNodes]);

    return null;
  }

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
          nodesDraggable={true}
          nodesConnectable={false}
          elementsSelectable={true}
          translateExtent={translateExtent}
        >
          <Background gap={32} size={1} />
          {/* Perform dynamic layout & viewport calculations */}
          <LayoutUpdater />
        </ReactFlow>
      </div>
    </ReactFlowProvider>
  );
}
