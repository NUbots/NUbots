import { observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { message } from "../../../shared/messages";

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

export enum ProviderClassification {
  UNKNOWN = 0,
  PROVIDE = 1,
  START = 2,
  STOP = 3,
  ROOT = 4,
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

/**
 * Convert a raw protobuf DirectorState into an enriched graph that the UI can work with.
 */
export function transformDirectorState(state: message.behaviour.DirectorState): DirectorGraph {
  const groupsById: Record<string, GroupModel> = {};
  const providersById: Record<string, ProviderModel> = {};

  // 1. Build groups map
  if (state.groups) {
    for (const [gid, g] of Object.entries(state.groups)) {
      groupsById[gid] = {
        id: gid,
        type: g.type ?? "unknown",
        providers: [],
        activeProvider: undefined,
        parentProvider: undefined,
        subtasks: [],
      };
    }
  }

  // 2. Build providers and attach to groups
  if (state.providers) {
    for (const [pid, p] of Object.entries(state.providers)) {
      const group = groupsById[p.group!.toString()];
      if (!group) continue;
      const provider: ProviderModel = {
        id: pid,
        classification: (p.classification ?? 0) as ProviderClassification,
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

  // 3. Resolve needs links
  for (const provider of Object.values(providersById)) {
    const raw = state.providers![provider.id];
    const needs = raw.needs ?? [];
    provider.needs = needs.map((gid) => groupsById[gid.toString()]).filter(Boolean);
  }

  // 4. Resolve group provider pointers and subtasks
  for (const group of Object.values(groupsById)) {
    const raw = state.groups![group.id];
    if (raw.activeProvider) {
      group.activeProvider = providersById[raw.activeProvider.toString()];
    }
    if (raw.parentProvider) {
      group.parentProvider = providersById[raw.parentProvider.toString()];
    }
    group.subtasks = (raw.subtasks ?? []).map((t): TaskModel => {
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
