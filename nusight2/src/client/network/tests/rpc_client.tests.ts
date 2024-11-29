import { NUClearNetPacket } from "nuclearnet.js";
import { describe, expect, it, vi } from "vitest";

import { createMockEventEmitter } from "../../../shared/base/testing/create_mock_event_emitter";
import { message } from "../../../shared/messages";
import { NUClearNetClient } from "../../../shared/nuclearnet/nuclearnet_client";
import { AppModel } from "../../components/app/model";
import { RobotModel } from "../../components/robot/model";
import { Network } from "../network";
import { NUsightNetwork } from "../nusight_network";
import { nextRpcToken, RpcClient, RpcError } from "../rpc_client";

import Test = message.support.nusight.Test;
import ScrubberLoadRequest = message.eye.ScrubberLoadRequest;

describe("RpcClient", () => {
  it("call() throws on invalid arguments", async () => {
    const { network } = createNetwork();
    const client = new RpcClient(network);

    // @ts-expect-error
    const resultA = await client.call();
    if (resultA.ok) {
      throw new Error("Expected invalid RPC call to fail");
    } else {
      expect(resultA.error).toBeInstanceOf(RpcError);
      expect(resultA.error.cause).toBe("INVALID_REQUEST");
      expect(resultA.error.message).toBe("Request protobuf type for RPC call not found");
    }

    // @ts-expect-error
    const resultB = await client.call(new Test({ message: "hi" }));
    if (resultB.ok) {
      throw new Error("Expected invalid RPC call to fail");
    } else {
      expect(resultB.error).toBeInstanceOf(RpcError);
      expect(resultB.error.cause).toBe("INVALID_REQUEST");
      expect(resultB.error.message).toBe("Response protobuf type for RPC call not found");
    }
  });

  it("call() times out if no response is received for the request in the configured time", async () => {
    const { network, nuclearnetMockEmitter } = createNetwork();
    const client = new RpcClient(network);

    const request = new ScrubberLoadRequest({ files: ["a.nbs", "b.nbs"] });

    const result = await client.call(request, { timeout: 10 }); // 10ms timeout
    if (result.ok) {
      throw new Error("Expected RPC call to time out");
    } else {
      expect(result.error).toBeInstanceOf(RpcError);
      expect(result.error.cause).toBe("TIMEOUT");
      expect(result.error.message).toBe("RPC call timed out");
    }

    // Check that the response listener was removed after the timeout
    expect(nuclearnetMockEmitter.off).toHaveBeenCalledTimes(1);
  });

  it("call() throws a cancelled error if the call is cancelled and then times out", async () => {
    const { network, nuclearnetMockEmitter } = createNetwork();
    const client = new RpcClient(network);

    const request = new ScrubberLoadRequest({ files: ["a.nbs", "b.nbs"] });

    const promise = client.call(request, { timeout: 10 }); // 10ms timeout
    client.cancelAll();

    const result = await promise;
    if (result.ok) {
      throw new Error("Expected RPC call to be cancelled");
    } else {
      expect(result.error).toBeInstanceOf(RpcError);
      expect(result.error.cause).toBe("CANCELLED");
      expect(result.error.message).toBe("RPC call cancelled");
    }

    // Check that the response listener was removed after the timeout
    expect(nuclearnetMockEmitter.off).toHaveBeenCalledTimes(1);
  });

  it("call() throws a cancelled error if the call is cancelled and then gets a response", async () => {
    const robotModel = RobotModel.of({
      address: "127.0.0.1",
      connected: true,
      enabled: true,
      id: "robot1",
      name: "Robot 1",
      port: 0,
      type: "nusight-server",
    });
    const { network, nuclearnetMockEmitter } = createNetwork([robotModel]);
    const client = new RpcClient(network);

    const request = new ScrubberLoadRequest({ rpc: { token: 1 }, files: ["a.nbs", "b.nbs"] });

    const promise = client.call(request);
    client.cancelAll();

    const response = ScrubberLoadRequest.Response.encode({ rpc: { ok: true, token: nextRpcToken - 1 } });
    const packet: NUClearNetPacket = {
      hash: undefined as any,
      payload: response.finish() as Buffer,
      peer: {
        address: robotModel.address,
        name: robotModel.name,
        port: robotModel.port,
      },
      reliable: true,
    };

    nuclearnetMockEmitter.emit("message.eye.ScrubberLoadRequest.Response", packet);

    const result = await promise;
    if (result.ok) {
      throw new Error("Expected RPC call to be cancelled");
    } else {
      expect(result.error).toBeInstanceOf(RpcError);
      expect(result.error.cause).toBe("CANCELLED");
      expect(result.error.message).toBe("RPC call cancelled");
    }
  });

  it("call() resolves with the response data when making a call that succeeds", async () => {
    const robotModel = RobotModel.of({
      address: "127.0.0.1",
      connected: true,
      enabled: true,
      id: "robot1",
      name: "Robot 1",
      port: 0,
      type: "nusight-server",
    });
    const { network, nuclearnetMockEmitter } = createNetwork([robotModel]);
    const client = new RpcClient(network);

    const request = new ScrubberLoadRequest({ rpc: { token: 1 }, files: ["a.nbs", "b.nbs"] });

    const promise = client.call(request);

    // First emit a response for a different RPC call
    const unrelatedResponse = ScrubberLoadRequest.Response.encode({ rpc: { ok: true, token: 99 } });
    const unrelatedResponsePacket: NUClearNetPacket = {
      hash: undefined as any,
      payload: unrelatedResponse.finish() as Buffer,
      peer: {
        address: robotModel.address,
        name: robotModel.name,
        port: robotModel.port,
      },
      reliable: true,
    };
    nuclearnetMockEmitter.emit("message.eye.ScrubberLoadRequest.Response", unrelatedResponsePacket);

    // Then emit the response for the RPC call we're interested in
    const response = ScrubberLoadRequest.Response.encode({ rpc: { ok: true, token: nextRpcToken - 1 } });
    const packet: NUClearNetPacket = {
      hash: undefined as any,
      payload: response.finish() as Buffer,
      peer: {
        address: robotModel.address,
        name: robotModel.name,
        port: robotModel.port,
      },
      reliable: true,
    };
    nuclearnetMockEmitter.emit("message.eye.ScrubberLoadRequest.Response", packet);

    const result = await promise;
    if (result.ok) {
      expect(result.ok).toBe(true);
      expect(result.data).toEqual({
        robotModel,
        response: { rpc: { ok: true, token: nextRpcToken - 1 } },
      });
    } else {
      throw new Error("Expected RPC call to succeed");
    }

    // Check that the response listener was removed after the response was received
    expect(nuclearnetMockEmitter.off).toHaveBeenCalledTimes(1);
  });
});

/** Create a Network instance with the given robot models, with a mock NUClearNet implementation */
function createNetwork(robots: RobotModel[] = []) {
  const nuclearnetMockEmitter = createMockEventEmitter();
  const nuclearnetClient: NUClearNetClient = {
    on: nuclearnetMockEmitter.on,
    onJoin: vi.fn(),
    onLeave: vi.fn(),
    connect: vi.fn(),
    onPacket: vi.fn(),
    send: vi.fn(),
  };

  const nusightNetwork = new NUsightNetwork(nuclearnetClient, AppModel.of({ robots }));

  const network = new Network(nusightNetwork);

  return { network, nuclearnetMockEmitter };
}
