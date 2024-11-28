import { NUClearNetSend } from "nuclearnet.js";
import { Mock, beforeEach, describe, expect, it, vi } from "vitest";

import { FakeNUClearNetClient } from "../../../server/nuclearnet/fake_nuclearnet_client";
import { FakeNUClearNetServer } from "../../../server/nuclearnet/fake_nuclearnet_server";
import { createMockEventEmitter } from "../../../shared/base/testing/create_mock_event_emitter";
import { message } from "../../../shared/messages";
import { NUClearNetClient } from "../../../shared/nuclearnet/nuclearnet_client";
import { AppModel } from "../../components/app/model";
import { NbsScrubbersModel } from "../../components/nbs_scrubbers/model";
import { NUsightNetwork } from "../nusight_network";

import Test = message.support.nusight.Test;

describe("NUsightNetwork", () => {
  let nuclearnetClient: ReturnType<typeof createMockNUClearNetClient>["nuclearnetClient"];
  let nusightNetwork: NUsightNetwork;

  beforeEach(() => {
    nuclearnetClient = createMockNUClearNetClient().nuclearnetClient;
    nusightNetwork = new NUsightNetwork(
      nuclearnetClient,
      new AppModel({ robots: [], scrubbersModel: NbsScrubbersModel.of() }),
    );
  });

  it("send() forwards the given packet to NUClearNet", () => {
    const payload = Test.encode({ message: "hello world" }).finish();
    const packet: NUClearNetSend = {
      type: "message.support.nusight.Test",
      payload: payload as Buffer,
      reliable: true,
      target: "nusight",
    };

    nusightNetwork.send(packet);

    expect(nuclearnetClient.send).toHaveBeenCalledWith(packet);
  });

  it("emit() creates a packet for the given message and forwards it to NUClearNet", () => {
    const data = { message: "hello world" };
    const expectedPayload = Test.encode(data).finish();

    nusightNetwork.emit(new Test(data));
    expect(nuclearnetClient.send).toHaveBeenCalledWith({
      type: "message.support.nusight.Test",
      payload: expectedPayload as Buffer,
      reliable: false,
      target: undefined,
    });

    nusightNetwork.emit(new Test(data), { reliable: true, target: "nusight" });
    expect(nuclearnetClient.send).toHaveBeenCalledWith({
      type: "message.support.nusight.Test",
      payload: expectedPayload as Buffer,
      reliable: true,
      target: "nusight",
    });
  });

  it("onNUClearMessage() registers a callback for the given message type", () => {
    const off = vi.fn();
    nuclearnetClient.on.mockReturnValue(off);

    expect(nusightNetwork.onNUClearMessage(Test, vi.fn())).toBe(off);
    expect(nuclearnetClient.on).toHaveBeenCalledWith("message.support.nusight.Test", expect.any(Function));

    const off2 = vi.fn();
    nuclearnetClient.on.mockReturnValue(off2);

    expect(nusightNetwork.onNUClearMessage({ type: Test, subtype: 1 }, vi.fn())).toBe(off2);
    expect(nuclearnetClient.on).toHaveBeenCalledWith("message.support.nusight.Test#1", expect.any(Function));
  });
});

/**
 * Create a fake NUClearNetClient connected to a fake server with mock implementations
 * of the `on()` and `send()` methods
 */
function createMockNUClearNetClient(): {
  nuclearnetClient: NUClearNetClient & {
    on: Mock<(event: string, callback: (...args: any[]) => void) => () => void>;
    send: Mock<(event: string, ...args: any[]) => void>;
  };
  nuclearnetMockEmit: (event: string, ...args: any[]) => void;
} {
  const nuclearnetServer = new FakeNUClearNetServer();
  const nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer);

  const mockEmitter = createMockEventEmitter();
  nuclearnetClient.on = mockEmitter.on;

  nuclearnetClient.send = vi.fn();

  return {
    nuclearnetClient: nuclearnetClient as any,
    nuclearnetMockEmit: mockEmitter.emit,
  };
}
