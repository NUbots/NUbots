import { beforeEach, describe, expect, it } from "vitest";
import { beforeEach, describe, expect, it } from "vitest";

import { createMockInstance } from "../../../../shared/base/testing/create_mock_instance";
import { AppModel } from "../../app/model";
import { LocalisationController } from "../controller";
import { LocalisationModel } from "../model";
import { ViewMode } from "../model";

describe("LocalisationController", () => {
  let controller: LocalisationController;
  let model: LocalisationModel;

  beforeEach(() => {
    model = LocalisationModel.of(createMockInstance(AppModel));
    controller = new LocalisationController();
  });

  describe("clicking the hawk eye button", () => {
    beforeEach(() => {
      controller.onHawkEyeClick(model);
    });

    it("resets yaw to 0", () => {
      expect(model.camera.yaw).toEqual(Math.PI / 2);
    });

    it("resets pitch to looking vertically down", () => {
      expect(model.camera.pitch).toEqual(-Math.PI / 2);
    });

    it("moves camera above the field", () => {
      expect(model.camera.position).toEqual({ x: 0, y: 0, z: 5 });
    });

    it("resets viewing mode to no clip", () => {
      expect(model.viewMode).toEqual(ViewMode.FreeCamera);
    });
  });
});
