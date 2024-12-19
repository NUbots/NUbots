import { beforeEach, describe, expect, it, vi } from "vitest";

import { AppModel } from "../client/components/app/model";
import { AppNetwork } from "../client/components/app/network";
import { Network } from "../client/network/network";
import { NUsightNetwork } from "../client/network/nusight_network";
import { FakeNUClearNetClient } from "../server/nuclearnet/fake_nuclearnet_client";
import { FakeNUClearNetServer } from "../server/nuclearnet/fake_nuclearnet_server";
import { message } from "../shared/messages";
import Sensors = message.input.Sensors;

import CompressedImage = message.output.CompressedImage;
import Overview = message.support.nusight.Overview;

describe("Networking Integration", () => {
  let nuclearnetServer: FakeNUClearNetServer;
  let nusightNetwork: NUsightNetwork;
  let disconnectNusightNetwork: () => void;
  let sendMessages: () => void;

  beforeEach(() => {
    nuclearnetServer = new FakeNUClearNetServer();
    nusightNetwork = createNUsightNetwork();
    disconnectNusightNetwork = nusightNetwork.connect({ name: "nusight" });
    const network = new FakeNUClearNetClient(nuclearnetServer);
    network.connect({ name: "Robot #1" });
    sendMessages = () => {
      network.send({ type: "message.input.Sensors", payload: new Buffer(0) });
      network.send({ type: "message.support.nusight.Overview", payload: new Buffer(0) });
    };
  });

  function createNUsightNetwork() {
    const appModel = AppModel.of();
    const nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer);
    const nusightNetwork = new NUsightNetwork(nuclearnetClient, appModel);
    AppNetwork.of(nusightNetwork, appModel);
    return nusightNetwork;
  }

  describe("a single networked component", () => {
    let network: Network;

    beforeEach(() => {
      network = new Network(nusightNetwork);
    });

    it("receives a Sensors message after subscribing and a robot sending it", () => {
      const onSensors = vi.fn();
      network.on(Sensors, onSensors);

      sendMessages();

      expect(onSensors).toHaveBeenCalledWith(expect.objectContaining({ name: "Robot #1" }), expect.any(Sensors));
      expect(onSensors).toHaveBeenCalledTimes(1);
    });

    it("does not receive any messages after unsubscribing", () => {
      const onSensors1 = vi.fn();
      const onSensors2 = vi.fn();
      network.on(Sensors, onSensors1);
      network.on(Sensors, onSensors2);

      network.off();

      sendMessages();

      expect(onSensors1).not.toHaveBeenCalled();
      expect(onSensors2).not.toHaveBeenCalled();
    });

    it("does not receive message on specific unsubscribed callback", async () => {
      const onSensors1 = vi.fn();
      const onSensors2 = vi.fn();
      const off1 = network.on(Sensors, onSensors1);
      network.on(Sensors, onSensors2);

      off1();

      sendMessages();

      expect(onSensors1).not.toHaveBeenCalled();
      expect(onSensors2).toHaveBeenCalledWith(expect.objectContaining({ name: "Robot #1" }), expect.any(Sensors));
    });
  });

  describe("sessions", () => {
    let network: Network;

    beforeEach(() => {
      network = new Network(nusightNetwork);
    });

    it("handles reconnects", () => {
      const onSensors = vi.fn();
      network.on(Sensors, onSensors);

      disconnectNusightNetwork();

      nusightNetwork.connect({ name: "nusight" });

      sendMessages();

      expect(onSensors).toHaveBeenCalledWith(expect.objectContaining({ name: "Robot #1" }), expect.any(Sensors));
    });

    it("handles multiple sessions simultaneously", () => {
      const nusightNetwork2 = createNUsightNetwork();
      nusightNetwork2.connect({ name: "nusight" });
      const network2 = new Network(nusightNetwork2);

      const onSensors1 = vi.fn();
      network.on(Sensors, onSensors1);

      const onSensors2 = vi.fn();
      network2.on(Sensors, onSensors2);

      sendMessages();

      expect(onSensors1).toHaveBeenCalledWith(expect.objectContaining({ name: "Robot #1" }), expect.any(Sensors));
      expect(onSensors2).toHaveBeenCalledWith(expect.objectContaining({ name: "Robot #1" }), expect.any(Sensors));
    });
  });

  describe("multiple networked components", () => {
    let localisationNetwork: Network;
    let visionNetwork: Network;
    let dashboardNetwork: Network;

    beforeEach(() => {
      localisationNetwork = new Network(nusightNetwork);
      visionNetwork = new Network(nusightNetwork);
      dashboardNetwork = new Network(nusightNetwork);
    });

    it("subscribes and unsubscribes as expected when switching between components", () => {
      const onSensors = vi.fn();
      localisationNetwork.on(Sensors, onSensors);

      sendMessages();

      expect(onSensors).toHaveBeenCalledTimes(1);

      localisationNetwork.off();

      const onCompressedImage = vi.fn();
      visionNetwork.on(CompressedImage, onCompressedImage);

      sendMessages();

      expect(onCompressedImage).toHaveBeenCalledTimes(0);
      expect(onSensors).toHaveBeenCalledTimes(1);

      visionNetwork.off();

      const onOverview = vi.fn();
      dashboardNetwork.on(Overview, onOverview);

      expect(onOverview).toHaveBeenCalledTimes(0);
      expect(onCompressedImage).toHaveBeenCalledTimes(0);
      expect(onSensors).toHaveBeenCalledTimes(1);

      dashboardNetwork.off();
    });
  });
});
