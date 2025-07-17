import React from "react";
import { observer } from "mobx-react";

import { ProviderModel, ProviderClassification } from "../model";
import { NeedChip } from "./need_chip";
import { WhenChip } from "./when_chip";

export interface ProviderViewProps {
  provider: ProviderModel;
}

function lastComponent(str: string): string {
  return str.split("::").pop() ?? str;
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
          <div className="flex flex-wrap gap-1 mt-1">
            {when.map((w, i) => (
              <WhenChip key={i} condition={w} />
            ))}
          </div>
        </div>
      )}
      {causing && Object.keys(causing).length > 0 && (
        <div>
          <span className="font-medium">Causing:</span>
          <div className="flex flex-wrap gap-1 mt-1">
            {Object.entries(causing).map(([k, v]) => {
              const displayKey = lastComponent(k);
              const displayVal = lastComponent(v.name);
              return (
                <div
                  key={k}
                  title={`${k}: ${v.name}`}
                  className="inline-flex items-center px-2 py-0.5 rounded text-xs font-medium border bg-yellow-200 dark:bg-yellow-700 text-yellow-900 dark:text-yellow-100 border-yellow-300 dark:border-yellow-600 select-none whitespace-nowrap"
                >
                  {displayKey}: {displayVal}
                </div>
              );
            })}
          </div>
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
