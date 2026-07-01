import { useCallback, useEffect, useMemo } from "react";
import { autorun } from "mobx";
import { useSearchParams } from "react-router-dom";

function useSearchParam(key: string) {
  const [searchParams] = useSearchParams();
  return useMemo(() => searchParams.get(key)?.trim(), [searchParams, key]);
}

/**
 * Get the selected robot ID and name from the current URL parameters.
 * Returns undefined if no robot parameters are present in the URL.
 *
 * URL params: `robotId` (string) and/or `robotName` (string).
 */
export function useUrlSelectedDataSource(): { id?: string; name?: string } | undefined {
  const id = useSearchParam("robotId");
  const name = useSearchParam("robotName");
  return useMemo(() => (id || name ? { id: id ?? undefined, name: name ?? undefined } : undefined), [id, name]);
}

/**
 * Automatically select the first data source (robot) for which the given callback returns true.
 * If no callback is given, the first available (connected) robot is selected.
 *
 * The given `model` should have observable `robots` and `selectedRobot` properties.
 */
export function useAutoSelectMatchingDataSource<T extends { connected: boolean }>(
  model: { robots: T[]; selectedRobot?: T },
  setSelected: (dataSource?: T) => void,
  matchDataSource?: (dataSource: T) => boolean,
) {
  useEffect(() => {
    return autorun(() => {
      // Do nothing if there is already a valid (connected) selection
      if (model.selectedRobot && model.selectedRobot.connected) {
        return;
      }

      const matched: T | undefined = matchDataSource
        ? model.robots.find(matchDataSource)
        : model.robots.find((r) => r.connected);
      setSelected(matched);
    });
  }, [model, setSelected, matchDataSource]);
}

/**
 * Automatically select a robot as follows (in order of precedence):
 * - if there's a `robotId` parameter in the current URL, select the robot with that ID
 * - if there's a `robotName` parameter in the current URL, select the robot with that name
 * - if neither parameter is present, select the first connected robot unless `opts.fallbackToFirst` is false
 *
 * The given `model` should have observable `robots` and `selectedRobot` properties.
 *
 * @return The URL selected data source, if any
 */
export function useAutoSelectDataSourceFromUrl<T extends { id: string; name: string; connected: boolean }>(
  model: { robots: T[]; selectedRobot?: T },
  setSelected: (dataSource?: T) => void,
  opts?: { fallbackToFirst?: boolean },
) {
  const expectedDataSource = useUrlSelectedDataSource();

  const matchDataSource = useCallback(
    (dataSource: T) => {
      if (!dataSource.connected) {
        return false;
      }

      if (expectedDataSource?.id !== undefined) {
        return dataSource.id === expectedDataSource.id;
      }

      if (expectedDataSource?.name) {
        return dataSource.name === expectedDataSource.name;
      }

      return opts?.fallbackToFirst ?? true;
    },
    [expectedDataSource?.id, expectedDataSource?.name, opts?.fallbackToFirst],
  );

  useAutoSelectMatchingDataSource(model, setSelected, matchDataSource);

  return expectedDataSource;
}
