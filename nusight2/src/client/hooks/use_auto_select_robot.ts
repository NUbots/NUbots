import { useCallback, useEffect, useMemo } from "react";
import { autorun } from "mobx";
import { useSearchParams } from "react-router-dom";

function useSearchParam(key: string) {
  const [searchParams] = useSearchParams();
  return useMemo(() => searchParams.get(key)?.trim(), [searchParams, key]);
}

/**
 * Automatically select the first robot for which the given callback returns true.
 * If no callback is given, the first connected robot is selected.
 *
 * The given `model` should be an object with observable properties `robots` and `selectedRobot`
 * to enable automatic selection when a robot is added or removed.
 */
export function useAutoSelectFirstRobot<T extends { id: number }>(
  model: { robots: T[]; selectedRobot?: T },
  onSelectRobot: (robot: T) => void,
  matchRobot?: (robot: T) => boolean,
) {
  useEffect(() => {
    // If a robot is already selected, do nothing
    if (model.selectedRobot) {
      return;
    }

    const findAndSelectRobot = () => {
      const robot = matchRobot ? model.robots.find(matchRobot) : model.robots.length > 0 ? model.robots[0] : undefined;
      if (robot) {
        onSelectRobot(robot);
      }
      return robot;
    };

    // Try to select the robot immediately and exit if successful
    const robot = findAndSelectRobot();
    if (robot) {
      return;
    }

    // Otherwise, set up a reaction to select the robot when it connects
    return autorun((reaction) => {
      // Abort if a robot is already selected
      if (model.selectedRobot) {
        reaction.dispose();
        return;
      }

      // Try to select the robot and dispose the auto reaction if successful
      const robot = findAndSelectRobot();
      if (robot) {
        reaction.dispose();
      }
    });
  }, []);
}

/**
 * Automatically select the robot whose ID matches the `selectedRobot` URL parameter.
 *
 * The given `model` should be an object with observable properties `robots` and `selectedRobot`
 * to enable automatic selection when a robot is added or removed.
 */
export function useAutoSelectRobotFromUrl<T extends { id: number }>(
  model: { robots: T[]; selectedRobot?: T },
  onSelectRobot: (robot: T) => void,
) {
  const selectedRobotParam = useSearchParam("selectedRobot");
  const selectedRobotId = selectedRobotParam ? Number(selectedRobotParam) : undefined;

  if (typeof selectedRobotId !== "number" || Number.isNaN(selectedRobotId)) {
    throw new Error(`selectedRobot parameter in URL is missing or not a number: "${selectedRobotParam}"`);
  }

  const matchRobot = useCallback((robot: T) => robot.id === selectedRobotId, [selectedRobotId]);

  useAutoSelectFirstRobot(model, onSelectRobot, matchRobot);

  return selectedRobotId;
}
