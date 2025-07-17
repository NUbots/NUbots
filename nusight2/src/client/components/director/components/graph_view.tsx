import React, { useEffect, useLayoutEffect, useRef, useState } from "react";

import { DirectorGraph, GroupModel } from "../model";
import { ProviderGroupView } from "./provider_group_view";

interface Point {
  x: number;
  y: number;
}

interface Edge {
  id: string;
  source: string; // group id or task id string
  target: string; // group id
  from: Point;
  to: Point;
}

export function GraphView({ graph }: { graph: DirectorGraph }) {
  // Refs
  const containerRef = useRef<HTMLDivElement | null>(null);
  const groupRefs = useRef<Record<string, HTMLDivElement | null>>({});

  const [edges, setEdges] = useState<Edge[]>([]);
  const [canvasSize, setCanvasSize] = useState<{ w: number; h: number }>({ w: 0, h: 0 });
  const edgesJsonRef = useRef<string>("[]");

  const recalculateEdges = () => {
    if (!containerRef.current) return;
    const containerRect = containerRef.current.getBoundingClientRect();

    const positions: Record<string, DOMRect> = {};
    Object.entries(groupRefs.current).forEach(([gid, el]) => {
      if (el) positions[gid] = el.getBoundingClientRect();
    });

    const newEdges: Edge[] = [];

    const addEdge = (srcId: string, tgtId: string, from: Point, to: Point) => {
      const id = `${srcId}->${tgtId}`;
      if (newEdges.some((e) => e.id === id)) return;
      newEdges.push({ id, source: srcId, target: tgtId, from, to });
    };

    for (const group of Object.values(graph.groupsById)) {
      const srcRect = positions[group.id];
      if (!srcRect) continue;

      const centerSrc: Point = {
        x: srcRect.left - containerRect.left + srcRect.width / 2,
        y: srcRect.top - containerRect.top + srcRect.height / 2,
      };

      // Subtask edges
      for (const task of group.subtasks) {
        const tgtRect = positions[task.targetGroup.id];
        if (!tgtRect) continue;
        const centerTgt: Point = {
          x: tgtRect.left - containerRect.left + tgtRect.width / 2,
          y: tgtRect.top - containerRect.top + tgtRect.height / 2,
        };
        addEdge(group.id, task.targetGroup.id, centerSrc, centerTgt);
      }

      // Needs edges
      for (const provider of group.providers) {
        for (const needGroup of provider.needs) {
          const tgtRect = positions[needGroup.id];
          if (!tgtRect) continue;
          const centerTgt: Point = {
            x: tgtRect.left - containerRect.left + tgtRect.width / 2,
            y: tgtRect.top - containerRect.top + tgtRect.height / 2,
          };
          addEdge(group.id, needGroup.id, centerSrc, centerTgt);
        }
      }
    }

    const json = JSON.stringify(newEdges);
    if (json !== edgesJsonRef.current) {
      edgesJsonRef.current = json;
      setEdges(newEdges);
    }
  };

  // Recalculate when data or DOM changes
  useLayoutEffect(recalculateEdges);

  // Recalculate on scroll and resize
  useEffect(() => {
    const handleResize = () => recalculateEdges();
    window.addEventListener("resize", handleResize);

    const container = containerRef.current;
    if (container) container.addEventListener("scroll", recalculateEdges, { passive: true });

    // Update canvas size
    if (containerRef.current) {
      setCanvasSize({ w: containerRef.current.scrollWidth, h: containerRef.current.scrollHeight });
    }

    return () => {
      window.removeEventListener("resize", handleResize);
      if (container) container.removeEventListener("scroll", recalculateEdges);
    };
  }, []);

  return (
    <div className="relative" ref={containerRef}>
      {/* SVG edges overlay */}
      <svg className="absolute inset-0 pointer-events-none" width={canvasSize.w} height={canvasSize.h}>
        {edges.map((e) => (
          <line
            key={e.id}
            x1={e.from.x}
            y1={e.from.y}
            x2={e.to.x}
            y2={e.to.y}
            stroke="currentColor"
            strokeWidth={1}
            className="text-gray-400" // override via tailwind colour
          />
        ))}
      </svg>

      {/* Group layout */}
      <div className="flex flex-row gap-4 p-4">
        {getRootGroups(graph).map((g) => (
          <GraphNode key={g.id} group={g} />
        ))}
      </div>
    </div>
  );

  // ----- helper components/funcs -----

  function GraphNode({ group }: { group: GroupModel }) {
    // dedupe child groups and preserve order
    const childGroups: GroupModel[] = Array.from(new Set(group.subtasks.map((t) => t.targetGroup)));

    return (
      <div className="flex flex-col items-center">
        <ProviderGroupView
          group={group}
          ref={(el) => {
            groupRefs.current[group.id] = el;
          }}
        />

        {childGroups.length > 0 && (
          <div className="flex flex-row gap-4 mt-4">
            {childGroups.map((c) => (
              <GraphNode key={c.id} group={c} />
            ))}
          </div>
        )}
      </div>
    );
  }

  function getRootGroups(gr: DirectorGraph): GroupModel[] {
    // If canonical root (-1) exists, use its subtasks
    const canonical = gr.groupsById["-1"];
    if (canonical) {
      return Array.from(new Set(canonical.subtasks.map((t) => t.targetGroup)));
    }
    // Fallback: groups without parentProvider
    return Object.values(gr.groupsById).filter((g) => !g.parentProvider && g.id !== "-1");
  }
}
