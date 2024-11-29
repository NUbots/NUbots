import { NUClearNetSend } from "nuclearnet.js";
import { describe, expect, it, vi } from "vitest";

import { createMockInstance } from "../../../shared/base/testing/create_mock_instance";
import { message } from "../../../shared/messages";
import { Network } from "../network";
import { NUsightNetwork } from "../nusight_network";

import Sensors = message.input.Sensors;
import Test = message.support.nusight.Test;

describe("Network", () => {
  it("off() unregisters all callbacks", () => {
    const nusightNetwork = createMockInstance(NUsightNetwork);
    const network = new Network(nusightNetwork);

    const cb1 = vi.fn();
    const cb2 = vi.fn();

    const off1 = vi.fn();
    nusightNetwork.onNUClearMessage.mockReturnValue(off1);

    network.on(Sensors, cb1);
    expect(nusightNetwork.onNUClearMessage).toHaveBeenCalledWith(Sensors, cb1);

    const off2 = vi.fn();
    nusightNetwork.onNUClearMessage.mockReturnValue(off2);

    network.on(Sensors, cb2);
    expect(nusightNetwork.onNUClearMessage).toHaveBeenCalledWith(Sensors, cb2);

    network.off();
    expect(off1).toHaveBeenCalledTimes(1);
    expect(off2).toHaveBeenCalledTimes(1);
  });

  it("send() forwards the given message to NUsightNetwork", () => {
    const nusightNetwork = createMockInstance(NUsightNetwork);
    const network = new Network(nusightNetwork);

    const payload = Test.encode({ message: "hello world" }).finish();
    const packet: NUClearNetSend = {
      type: "message.support.nusight.Test",
      payload: payload as Buffer,
      reliable: true,
      target: "nusight",
    };

    network.send(packet);

    expect(nusightNetwork.send).toHaveBeenCalledWith(packet);
  });

  it("emit() forwards the given message and options to NUsightNetwork", () => {
    const nusightNetwork = Object.assign(createMockInstance(NUsightNetwork), {
      emit: vi.fn(),
    });
    const network = new Network(nusightNetwork);

    const data = { message: "hello world" };

    network.emit(new Test(data));
    expect(nusightNetwork.emit).toHaveBeenCalledWith(data, undefined);

    network.emit(new Test(data), { reliable: true });
    expect(nusightNetwork.emit).toHaveBeenCalledWith(data, { reliable: true });
  });
});
