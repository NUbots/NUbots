import { observable } from "mobx";

import { message } from "../../../shared/messages";
import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";

export type ProviderClassification = message.behaviour.DirectorState.Provider.Classification;
export const ProviderClassification = message.behaviour.DirectorState.Provider.Classification;

/** Local representation of an enum value coming from DirectorState */
export interface EnumValue {
  name: string;
  value: number;
}

/** Representation of a `when` condition */
export interface WhenCondition {
  type: string;
  comparator: string;
  expectedState: EnumValue;
  current: boolean;
}

export interface GroupModel {
  id: string;
  type: string;
  providers: ProviderModel[];
  activeProvider?: ProviderModel;
  parentProvider?: ProviderModel;
  subtasks: TaskModel[];
}

export interface ProviderModel {
  id: string;
  classification: ProviderClassification;
  group: GroupModel;
  when: WhenCondition[];
  causing: Record<string, EnumValue>;
  needs: GroupModel[];
}

export interface TaskModel {
  name?: string;
  priority: number; // defaults to 0 when not provided
  optional: boolean;
  targetGroup: GroupModel;
}

export interface DirectorGraph {
  groupsById: Record<string, GroupModel>;
  providersById: Record<string, ProviderModel>;
}

function isRootGroup(
  group: message.behaviour.DirectorState.IGroup,
  providers: Record<string, message.behaviour.DirectorState.IProvider>,
): boolean {
  return (
    group.providerIds?.length === 1 && providers[group.providerIds[0]]?.classification === ProviderClassification.ROOT
  );
}

/**
 * Convert a raw protobuf DirectorState into an enriched graph that the UI can work with.
 */
export function transformDirectorState(state: message.behaviour.DirectorState): DirectorGraph {
  const groupsById: Record<string, GroupModel> = {};
  const providersById: Record<string, ProviderModel> = {};

  // Create canonical root group with a single provider (id -1)
  const canonicalRoot: GroupModel = {
    id: "-1",
    type: "Root",
    providers: [],
    activeProvider: undefined,
    parentProvider: undefined,
    subtasks: [],
  };

  const rootProvider: ProviderModel = {
    id: "-1",
    classification: message.behaviour.DirectorState.Provider.Classification.ROOT,
    group: canonicalRoot,
    when: [],
    causing: {},
    needs: [],
  };

  canonicalRoot.providers.push(rootProvider);
  canonicalRoot.activeProvider = rootProvider;

  groupsById[canonicalRoot.id] = canonicalRoot;
  providersById[rootProvider.id] = rootProvider;

  // Identify root groups from raw message
  const rootProviderIds = new Set<string>();
  const rootGroupIds = new Set<string>();
  const rootSubtasks: message.behaviour.DirectorState.IDirectorTask[] = [];

  if (state.groups) {
    // Create all the group models/update root group pointers
    for (const [gid, g] of Object.entries(state.groups)) {
      if (isRootGroup(g, state.providers)) {
        rootGroupIds.add(gid);
        rootProviderIds.add(g.providerIds?.[0] ?? "");
        rootSubtasks.push(...(g.subtasks ?? []));
      } else {
        // Create the group model
        groupsById[gid] = {
          id: gid,
          type: g.type!,
          providers: [],
          activeProvider: undefined,
          parentProvider: undefined,
          subtasks: [],
        };
      }
    }
  }

  // Treat the synthetic canonical root as a root group too so later passes skip it
  rootGroupIds.add(canonicalRoot.id);

  // Sort collected root subtasks so that highest priority appears first
  // Then sort by name as the javascript uses a hash map which breaks ordering of objects
  rootSubtasks.sort((a, z) => (z.priority ?? 0) - (a.priority ?? 0) || a.targetGroup - z.targetGroup);

  // Make providers for non-root groups
  if (state.providers) {
    for (const [pid, p] of Object.entries(state.providers).filter(([pid]) => !rootProviderIds.has(pid))) {
      const group = groupsById[p.group!.toString()];
      if (!group) continue;
      const provider: ProviderModel = {
        id: pid,
        classification: p.classification!,
        group,
        when: (p.when ?? []).map((w) => ({
          type: w.type ?? "",
          comparator: w.comparator ?? "",
          expectedState: {
            name: w.expectedState?.name ?? "",
            value: w.expectedState?.value ?? 0,
          },
          current: w.current ?? false,
        })),
        causing: Object.fromEntries(
          Object.entries(p.causing ?? {}).map(([k, v]) => [k, { name: v.name ?? "", value: v.value ?? 0 }]),
        ),
        needs: [],
      };
      providersById[pid] = provider;
      group.providers.push(provider);
    }
  }

  // Resolve needs links
  for (const provider of Object.values(providersById).filter((p) => !rootProviderIds.has(p.id))) {
    provider.needs = (state.providers?.[provider.id]?.needs ?? []).map((gid) => groupsById[gid]).filter(Boolean);
  }

  // Resolve the provider pointers
  for (const [gid, g] of Object.entries(groupsById).filter(([gid]) => !rootGroupIds.has(gid))) {
    const { activeProvider, parentProvider } = state.groups[gid]!;
    g.activeProvider = rootProviderIds.has(activeProvider) ? rootProvider : providersById[activeProvider];
    g.parentProvider = rootProviderIds.has(parentProvider) ? rootProvider : providersById[parentProvider];
  }

  // Resolve the subtasks
  for (const [gid, group] of Object.entries(groupsById)) {
    const subtasks = group === canonicalRoot ? rootSubtasks : (state.groups?.[gid]?.subtasks ?? []);
    group.subtasks = (subtasks ?? []).map((t): TaskModel => {
      const target = groupsById[t.targetGroup!.toString()];
      return {
        name: t.name ?? undefined,
        priority: t.priority ?? 0,
        optional: t.optional ?? false,
        targetGroup: target,
      };
    });
  }

  return { groupsById, providersById };
}

/**
 * MobX model for the Director UI.
 */
export class DirectorModel {
  private appModel: AppModel;

  constructor(appModel: AppModel) {
    this.appModel = appModel;
  }

  static of = memoize((appModel: AppModel) => new DirectorModel(appModel));

  /** Most recently received enriched graph */
  @observable.ref graph?: DirectorGraph;
}
