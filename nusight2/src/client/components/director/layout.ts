import { DirectorGraph, GroupModel, ProviderClassification } from "./model";

export interface GraphLayoutRow {
  groups: GroupModel[];
}

/**
 * Produce rows of groups for a simple hierarchical layout.
 * Children appear one row lower than their parent, based on subtask target groups.
 */
export function layoutGraph(graph: DirectorGraph): GraphLayoutRow[] {
  const rows: GroupModel[][] = [];
  const depthMap: Record<string, number> = {};

  const visited = new Set<string>();

  // Identify root groups: exactly one provider and that provider.classification === ROOT (4)
  const roots = Object.values(graph.groupsById).filter(
    (g) => g.providers.length === 1 && g.providers[0].classification === ProviderClassification.ROOT,
  );

  // Sort roots by priority
  roots.sort(compareGroups);

  // Traverse from each root
  roots.forEach((r) => traverse(r, 0));

  // Any groups not placed yet (disconnected) go to last row
  const unplaced = Object.values(graph.groupsById).filter((g) => depthMap[g.id] === undefined);
  if (unplaced.length) {
    // Place unconnected groups starting on a new row after placed ones
    let row = rows.length;
    unplaced.forEach((g) => {
      if (!rows[row]) rows[row] = [];
      rows[row].push(g);
      // distribute evenly by creating new rows every few groups if desired (simple: one row)
    });
  }

  return rows.map((groups) => ({ groups }));

  function traverse(group: GroupModel, depth: number) {
    if (visited.has(group.id)) return;
    visited.add(group.id);
    setDepth(group, depth);

    const children = group.subtasks.map((t) => t.targetGroup).filter(Boolean) as GroupModel[];
    // dedupe children
    const unique = Array.from(new Set(children));
    // sort by priority
    unique.sort(compareGroups);
    unique.forEach((child) => traverse(child, depth + 1));
  }

  function setDepth(group: GroupModel, d: number) {
    depthMap[group.id] = d;
    if (!rows[d]) rows[d] = [];
    rows[d].push(group);
  }
}

function firstPriority(g: GroupModel): number {
  const task = g.subtasks.find((t) => !t.optional) || g.subtasks[0];
  return task ? task.priority : 0;
}

function compareGroups(a: GroupModel, b: GroupModel): number {
  const [oa, pa] = groupKey(a);
  const [ob, pb] = groupKey(b);
  if (oa !== ob) return oa - ob; // non-optional (0) before optional (1)
  return pa - pb;
}

function groupKey(g: GroupModel): [number, number] {
  const mandatory = g.subtasks.find((t) => !t.optional);
  if (mandatory) return [0, mandatory.priority];
  const first = g.subtasks[0];
  return [1, first ? first.priority : 0];
}
