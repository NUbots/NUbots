import { RoboCup } from "@proto/message/input/RoboCup";
import { Overview } from "@proto/message/support/nusight/Overview";
import { describe, expect, it } from "vitest";

import { createMockInstance } from "../../../../shared/base/testing/create_mock_instance";
import { Matrix4 } from "../../../../shared/math/matrix4";
import { Network } from "../../../network/network";
import { NUsightNetwork } from "../../../network/nusight_network";
import { RobotModel } from "../../robot/model";
import { DashboardRobotModel } from "../dashboard_components/dashboard_robot/model";
import { LocalisationModel } from "../model";
import { LocalisationNetwork } from "../network";
import { LocalisationRobotModel } from "../robot_model";

/**
 * Builds a `LocalisationNetwork` wired up to a fake `NUsightNetwork` that records the callback
 * registered for each message type (keyed by its protobuf type name), and returns a `trigger`
 * helper that invokes the matching callback directly, simulating a packet being received for the
 * given robot.
 */
function setup() {
  const handlers = new Map<string, (peer: any, message: any) => void>();

  const nusightNetwork = createMockInstance(NUsightNetwork);
  nusightNetwork.onNUClearMessage.mockImplementation((type: any, cb: any) => {
    const messageType = typeof type === "object" ? type.type : type;
    handlers.set(messageType.typeName, cb);
    return () => handlers.delete(messageType.typeName);
  });

  const network = Network.of(nusightNetwork);
  new LocalisationNetwork(network, {} as LocalisationModel);

  function trigger<T>(MessageType: { typeName: string }, robotModel: RobotModel, message: T) {
    const cb = handlers.get(MessageType.typeName);
    if (!cb) {
      throw new Error(`No handler registered for ${MessageType.typeName}`);
    }
    cb(robotModel, message);
  }

  return { trigger };
}

function createRobotModel(id = "1"): RobotModel {
  return RobotModel.of({
    id,
    name: `robocup-${id}`,
    address: "10.1.1.5",
    port: 10001,
    connected: true,
    enabled: true,
    type: "robocup-udp-peer",
  });
}

describe("LocalisationNetwork", () => {
  describe("onRoboCup", () => {
    it("populates the dashboard model from the fields RoboCup always carries", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();

      trigger(
        RoboCup,
        robotModel,
        new RoboCup({
          currentPose: { playerId: 3, position: { x: 1, y: 2, z: 0 } },
          ball: { position: { x: 4, y: 5, z: 0 } },
          kickTarget: { x: 6, y: 7 },
          walkCommand: { x: 0.1, y: 0.2, z: 0.3 },
        }),
      );

      const dashboardRobot = DashboardRobotModel.of(robotModel);
      expect(dashboardRobot.playerId).toBe(3);
      expect(dashboardRobot.robotPosition).toMatchObject({ x: 1, y: 2, z: 0 });
      expect(dashboardRobot.ballPosition).toMatchObject({ x: 4, y: 5 });
      expect(dashboardRobot.kickTarget).toMatchObject({ x: 6, y: 7 });
      expect(dashboardRobot.walkCommand).toMatchObject({ x: 0.1, y: 0.2, z: 0.3 });

      // Fields that RoboCup has no equivalent for (e.g. battery/voltage, which are Overview-only)
      // should be left at their defaults rather than overwritten with zeroes
      expect(dashboardRobot.battery).toBe(-1);
      expect(dashboardRobot.voltage).toBe(-1);
    });

    it("uses nubots.Hft directly for the field-frame pose when it's provided", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();

      trigger(
        RoboCup,
        robotModel,
        new RoboCup({
          currentPose: { playerId: 1, position: { x: 999, y: 999, z: 0 } },
          nubots: {
            Hft: {
              x: { x: 1, y: 0, z: 0, t: 0 },
              y: { x: 0, y: 1, z: 0, t: 0 },
              z: { x: 0, y: 0, z: 1, t: 0 },
              t: { x: 10, y: 20, z: 0.5, t: 1 },
            },
          },
        }),
      );

      const robot = LocalisationRobotModel.of(robotModel);
      const { translation } = robot.Hfw.decompose();
      expect(translation.x).toBeCloseTo(10);
      expect(translation.y).toBeCloseTo(20);
      expect(translation.z).toBeCloseTo(0.5);

      // Htw/Hrw should be identity, since Hft is already precombined
      expect(robot.Htw).toEqual(Matrix4.of());
      expect(robot.Hrw).toEqual(Matrix4.of());

      // The computed Hft getter (Hfw * Htw⁻¹) should reproduce the same pose we sent
      const { translation: HftTranslation } = robot.Hft.decompose();
      expect(HftTranslation.x).toBeCloseTo(10);
      expect(HftTranslation.y).toBeCloseTo(20);
      expect(HftTranslation.z).toBeCloseTo(0.5);
    });

    it("falls back to a flat pose built from current_pose.position when nubots.Hft isn't provided", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();

      trigger(
        RoboCup,
        robotModel,
        new RoboCup({
          currentPose: { playerId: 1, position: { x: 3, y: 4, z: 0 } },
        }),
      );

      const robot = LocalisationRobotModel.of(robotModel);
      const { translation } = robot.Hfw.decompose();
      expect(translation.x).toBeCloseTo(3);
      expect(translation.y).toBeCloseTo(4);
      expect(translation.z).toBeCloseTo(0);

      expect(robot.Htw).toEqual(Matrix4.of());
      expect(robot.Hrw).toEqual(Matrix4.of());
    });

    it("sets servo angles from nubots.servos when provided, in the same order as Sensors.servo", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();
      const servos = Array.from({ length: 20 }, (_, i) => i / 10);

      trigger(
        RoboCup,
        robotModel,
        new RoboCup({
          currentPose: { playerId: 1 },
          nubots: { servos },
        }),
      );

      const robot = LocalisationRobotModel.of(robotModel);
      expect(robot.motors.rightShoulderPitch.angle).toBeCloseTo(servos[0]);
      expect(robot.motors.leftShoulderPitch.angle).toBeCloseTo(servos[1]);
      expect(robot.motors.rightShoulderRoll.angle).toBeCloseTo(servos[2]);
      expect(robot.motors.leftShoulderRoll.angle).toBeCloseTo(servos[3]);
      expect(robot.motors.rightElbow.angle).toBeCloseTo(servos[4]);
      expect(robot.motors.leftElbow.angle).toBeCloseTo(servos[5]);
      expect(robot.motors.rightHipYaw.angle).toBeCloseTo(servos[6]);
      expect(robot.motors.leftHipYaw.angle).toBeCloseTo(servos[7]);
      expect(robot.motors.rightHipRoll.angle).toBeCloseTo(servos[8]);
      expect(robot.motors.leftHipRoll.angle).toBeCloseTo(servos[9]);
      expect(robot.motors.rightHipPitch.angle).toBeCloseTo(servos[10]);
      expect(robot.motors.leftHipPitch.angle).toBeCloseTo(servos[11]);
      expect(robot.motors.rightKnee.angle).toBeCloseTo(servos[12]);
      expect(robot.motors.leftKnee.angle).toBeCloseTo(servos[13]);
      expect(robot.motors.rightAnklePitch.angle).toBeCloseTo(servos[14]);
      expect(robot.motors.leftAnklePitch.angle).toBeCloseTo(servos[15]);
      expect(robot.motors.rightAnkleRoll.angle).toBeCloseTo(servos[16]);
      expect(robot.motors.leftAnkleRoll.angle).toBeCloseTo(servos[17]);
      expect(robot.motors.headPan.angle).toBeCloseTo(servos[18]);
      expect(robot.motors.headTilt.angle).toBeCloseTo(servos[19]);
    });

    it("leaves motor angles untouched when nubots.servos isn't provided", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();

      trigger(RoboCup, robotModel, new RoboCup({ currentPose: { playerId: 1 } }));

      const robot = LocalisationRobotModel.of(robotModel);
      expect(robot.motors.rightShoulderPitch.angle).toBe(0);
      expect(robot.motors.headTilt.angle).toBe(0);
    });

    it("does not touch the localisation pose when neither nubots.Hft nor current_pose.position is provided", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();

      trigger(RoboCup, robotModel, new RoboCup({}));

      const robot = LocalisationRobotModel.of(robotModel);
      expect(robot.Hfw).toEqual(Matrix4.of());
      expect(robot.Htw).toEqual(Matrix4.of());
      expect(robot.Hrw).toEqual(Matrix4.of());
    });
  });

  describe("onOverview", () => {
    it("populates the dashboard model from an Overview message", () => {
      const { trigger } = setup();
      const robotModel = createRobotModel();

      trigger(
        Overview,
        robotModel,
        new Overview({
          robotId: 2,
          roleName: "striker",
          battery: 0.5,
          voltage: 12,
          robotPosition: { x: 1, y: 2, z: 3 },
          ballPosition: { x: 4, y: 5 },
          kickTarget: { x: 6, y: 7 },
          walkCommand: { x: 0.1, y: 0.2, z: 0.3 },
        }),
      );

      const dashboardRobot = DashboardRobotModel.of(robotModel);
      expect(dashboardRobot.playerId).toBe(2);
      expect(dashboardRobot.roleName).toBe("striker");
      expect(dashboardRobot.battery).toBeCloseTo(0.5);
      expect(dashboardRobot.voltage).toBeCloseTo(12);
      expect(dashboardRobot.robotPosition).toMatchObject({ x: 1, y: 2, z: 3 });
      expect(dashboardRobot.ballPosition).toMatchObject({ x: 4, y: 5 });
      expect(dashboardRobot.kickTarget).toMatchObject({ x: 6, y: 7 });
      expect(dashboardRobot.walkCommand).toMatchObject({ x: 0.1, y: 0.2, z: 0.3 });
    });
  });
});
