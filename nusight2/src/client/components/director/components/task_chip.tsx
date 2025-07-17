import React from "react";

import { TaskModel } from "../model";

export interface TaskChipProps {
  task: TaskModel;
}

/**
 * Chip representing a subtask in the director graph.
 * Optional tasks are styled differently from mandatory tasks.
 */
export function TaskChip({ task }: TaskChipProps) {
  let displayName = task.name as string | undefined;
  if (!displayName) {
    displayName = task.targetGroup?.type ?? task.targetGroup?.id ?? "Unnamed";
  }

  const { priority, optional } = task as any;
  return (
    <div
      className={`inline-flex items-center px-2 py-0.5 rounded text-xs font-medium border select-none whitespace-nowrap ${
        optional
          ? "bg-gray-200 dark:bg-gray-700 text-gray-800 dark:text-gray-200 border-dashed"
          : "bg-blue-200 dark:bg-blue-700 text-blue-900 dark:text-blue-100 border-solid"
      }`}
    >
      {priority !== undefined && (
        <span className="mr-1 px-1 rounded bg-white/30 dark:bg-black/30 text-[0.6rem] font-semibold">{priority}</span>
      )}
      {displayName || "Unnamed task"}
    </div>
  );
}
