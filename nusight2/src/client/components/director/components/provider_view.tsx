import React from "react";
import { observer } from "mobx-react";

import { ProviderModel, ProviderClassification } from "../model";
import { NeedChip } from "./need_chip";

export interface ProviderViewProps {
  provider: ProviderModel;
}

export const ProviderView = observer(function ProviderView({ provider }: ProviderViewProps) {
  const { id, classification, when, causing, needs } = provider;
  const unmetWhen = (when ?? []).filter((w) => w.current === false);
  const allWhenMet = unmetWhen.length === 0;
  const active = provider === provider.group?.activeProvider;

  return (
    <div
      className={`border rounded p-2 text-xs space-y-1 bg-auto-surface-1 dark:bg-auto-surface-2 ${
        active ? "border-green-500" : "border-auto"
      } ${allWhenMet ? "opacity-100" : "opacity-50"}`}
    >
      <div className="flex justify-between">
        <span className="font-semibold">Provider {id}</span>
        {classification !== undefined && classification !== null && (
          <span className="italic text-gray-500">{ProviderClassification[classification as any]}</span>
        )}
      </div>
      {when && when.length > 0 && (
        <div>
          <span className="font-medium">When:</span>
          <ul className="list-disc ml-4">
            {when.map((w, i) => (
              <li key={i} className={w.current ? "" : "line-through"}>
                {w.type} {w.comparator} {w.expectedState?.name}
              </li>
            ))}
          </ul>
        </div>
      )}
      {causing && Object.keys(causing).length > 0 && (
        <div>
          <span className="font-medium">Causing:</span>
          <ul className="list-disc ml-4">
            {Object.entries(causing).map(([k, v]) => (
              <li key={k}>
                {k}: {v.name}
              </li>
            ))}
          </ul>
        </div>
      )}
      {needs && needs.length > 0 && (
        <div>
          <span className="font-medium">Needs:</span>
          <div className="flex flex-wrap gap-1 mt-1">
            {needs.map((g) => (
              <NeedChip key={g.id} group={g} />
            ))}
          </div>
        </div>
      )}
    </div>
  );
});
