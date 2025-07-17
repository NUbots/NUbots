import React from "react";

import { GroupModel } from "../model";

function shortName(s: string): string {
  return s.split("::").pop() ?? s;
}

export function NeedChip({ group }: { group: GroupModel }) {
  const display = shortName(group.type);
  return (
    <div
      title={group.type}
      className="inline-flex items-center px-2 py-0.5 rounded text-xs font-medium bg-purple-200 dark:bg-purple-700 text-purple-900 dark:text-purple-100 border border-purple-300 dark:border-purple-600 select-none whitespace-nowrap"
    >
      {display}
    </div>
  );
}
