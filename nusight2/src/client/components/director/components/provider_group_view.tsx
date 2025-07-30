import React, { forwardRef } from "react";
import { observer } from "mobx-react";

import { GroupModel, ProviderClassification } from "../model";

import { ProviderView } from "./provider_view";
import { TaskChip } from "./task_chip";

function shortName(s: string): string {
  return s.split("::").pop() ?? s;
}

export interface ProviderGroupViewProps {
  group: GroupModel;
}

/**
 * Renders a provider group: a box containing provider views and subtasks.
 */
export const ProviderGroupView = observer(
  forwardRef<HTMLDivElement, ProviderGroupViewProps>(function ProviderGroupView({ group }, ref) {
    const providerIds = (group.providers ?? []).map((p) => p.id);
    const subtasks = group.subtasks ?? [];

    const hasParent = !!group.parentProvider;
    const isRoot = group.providers.some((p) => p.classification === ProviderClassification.ROOT);
    const isRunning = !!group.activeProvider && !isRoot;
    const isOrphan = !hasParent && !isRoot;

    return (
      <div
        ref={ref}
        className={`relative rounded p-2 bg-auto-surface-2 space-y-2 flex-none ${(() => {
          if (hasParent) return "border";
          if (isRoot) return "border-2 border-blue-500";
          // Orphan: dashed border with default colour
          return "border-2 border-dashed";
        })()} ${isOrphan ? "opacity-25" : ""}`}
        style={{ minWidth: "16rem" }}
        data-testid="provider-group"
      >
        {/* badge */}
        {!hasParent && (
          <span
            className={`absolute -top-2 right-2 text-white text-[10px] font-semibold px-1.5 py-0.5 rounded ${
              isRoot ? "bg-blue-500" : "bg-red-500"
            }`}
          >
            {isRoot ? "ROOT" : "ORPHAN"}
          </span>
        )}
        {/* running badge */}
        {isRunning && (
          <span className="absolute -top-2 right-2 bg-green-500 text-white text-[10px] font-semibold px-1.5 py-0.5 rounded">
            RUNNING
          </span>
        )}
        <div className="font-semibold text-sm mb-1" title={group.type}>
          {group.type ? shortName(group.type) : `Group ${group}`}
        </div>

        {/* Providers */}
        <div className="space-y-1">
          {providerIds.map((pid) => {
            const p = group.providers.find((pr) => pr.id === pid.toString());
            if (!p) return null;
            return <ProviderView key={p.id} provider={p} />;
          })}
        </div>

        {/* Subtasks */}
        {subtasks.length > 0 && (
          <div className="flex flex-wrap items-center pt-2 border-t border-auto">
            {(() => {
              const mandatory = subtasks.filter((t) => !t.optional);
              const optional = subtasks.filter((t) => t.optional);

              const chips: JSX.Element[] = [];

              // Group of mandatory tasks (if any)
              if (mandatory.length) {
                chips.push(
                  <div key="mandatory" className="flex flex-wrap gap-1">
                    {mandatory.map((task, idx) => (
                      <TaskChip key={`m-${idx}`} task={task} />
                    ))}
                  </div>,
                );
              }

              // Append each optional task with divider before it
              optional.forEach((task, idx) => {
                // Add divider before optional chip (also between optionals)
                if (chips.length) {
                  chips.push(<div key={`div-${idx}`} className="w-px h-4 bg-gray-400 dark:bg-gray-500 mx-2" />);
                }
                chips.push(<TaskChip key={`o-${idx}`} task={task} />);
              });

              return chips;
            })()}
          </div>
        )}
      </div>
    );
  }),
);
