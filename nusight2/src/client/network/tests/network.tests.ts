import { NUClearNetSend } from "nuclearnet.js";

import { createMockInstance } from "../../../shared/base/testing/create_mock_instance";
import { message } from "../../../shared/messages";
import { Network } from "../network";
import { NUsightNetwork } from "../nusight_network";

import Sensors = message.input.Sensors;
import Say = message.output.Say;

describe("Network", () => {
  it("off() unregisters all callbacks", () => {
    const nusightNetwork = createMockInstance(NUsightNetwork);
    const network = new Network(nusightNetwork);

    const cb1 = jest.fn();
    const cb2 = jest.fn();

    const off1 = jest.fn();
    nusightNetwork.onNUClearMessage.mockReturnValue(off1);

    network.on(Sensors, cb1);
    expect(nusightNetwork.onNUClearMessage).toHaveBeenCalledWith(Sensors, cb1);

    const off2 = jest.fn();
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

    const payload = Say.encode({ message: "hello world" }).finish();
    const packet: NUClearNetSend = {
      type: "message.output.Say",
      payload: payload as Buffer,
      reliable: true,
      target: "nusight",
    };

    network.send(packet);

    expect(nusightNetwork.send).toHaveBeenCalledWith(packet);
  });

  it("emit() forwards the given message and options to NUsightNetwork", () => {
    const nusightNetwork = Object.assign(createMockInstance(NUsightNetwork), {
      emit: jest.fn(),
    });
    const network = new Network(nusightNetwork);

    const data = { message: "hello world" };

    network.emit(new Say(data));
    expect(nusightNetwork.emit).toHaveBeenCalledWith(data, undefined);

    network.emit(new Say(data), { reliable: true });
    expect(nusightNetwork.emit).toHaveBeenCalledWith(data, { reliable: true });
  });
});
