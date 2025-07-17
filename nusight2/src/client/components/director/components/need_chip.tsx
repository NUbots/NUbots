import React from "react";

import { GroupModel } from "../model";

export function NeedChip({ group }: { group: GroupModel }) {
  return (
    <div className="inline-flex items-center px-2 py-0.5 rounded text-xs font-medium bg-purple-200 dark:bg-purple-700 text-purple-900 dark:text-purple-100 border border-purple-300 dark:border-purple-600 select-none whitespace-nowrap">
      {group.type}
    </div>
  );
}
