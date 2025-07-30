import React from "react";

import { WhenCondition } from "../model";

const comparatorSymbol: Record<string, string> = {
  "std::equal_to<int>": "==",
  "std::not_equal_to<int>": "≠",
  "std::less<int>": "<",
  "std::greater<int>": ">",
  "std::less_equal<int>": "≤",
  "std::greater_equal<int>": "≥",
};

function prettyComparator(raw: string): string {
  return comparatorSymbol[raw] ?? raw.replace(/^std::/, "");
}

function shortName(s: string): string {
  return s.split("::").pop() ?? s;
}

export function WhenChip({ condition }: { condition: WhenCondition }) {
  const { type, comparator, expectedState, current } = condition;

  const bg = current
    ? "bg-green-200 dark:bg-green-700 text-green-900 dark:text-green-100 border-green-300 dark:border-green-600"
    : "bg-red-200 dark:bg-red-700 text-red-900 dark:text-red-100 border-red-300 dark:border-red-600";

  const comparatorDisplay = prettyComparator(comparator);

  // Shorten the type by keeping only the last component after '::'
  const shortType = type.split("::").pop() ?? type;

  const valueDisplay = expectedState?.name ? shortName(expectedState.name) : "";

  return (
    <div
      title={`${type} ${comparator} ${expectedState?.name ?? ""}`}
      className={`inline-flex items-center px-2 py-0.5 rounded text-xs font-medium border ${bg} select-none whitespace-nowrap`}
    >
      {shortType} {comparatorDisplay} {valueDisplay}
    </div>
  );
}
