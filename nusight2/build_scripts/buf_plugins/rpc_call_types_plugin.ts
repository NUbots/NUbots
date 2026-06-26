import { createEcmaScriptPlugin, runNodeJs, Schema } from "@bufbuild/protoplugin";

import { getTypesGroupedByLocation } from "./get_types";

const plugin = createEcmaScriptPlugin({
  name: "rpc-call-types-plugin",
  version: "v1",
  generateTs,
});

function generateTs(schema: Schema) {
  const f = schema.generateFile("rpc_call.d.ts");
  const messagesGroupedByLocation = getTypesGroupedByLocation(schema);

  const imports: string[] = [];
  const callOverloads: string[] = [];
  const responseTypes: string[] = [];

  messagesGroupedByLocation.forEach((group) => {
    const groupImport: string[] = [];
    const requestMessagesList = group.messages.filter((message) => message.name.endsWith("Request"));
    // Iterate over list of requests and add to array if corresponding response exists
    requestMessagesList.forEach((request) => {
      if (group.messages.some((message) => message.name === request.name + "_Response")) {
        callOverloads.push(
          `function rpcCall<T = ${request.name}_Response>(request: ${request.name}, options?: RpcCallOptions<T>): Promise<RpcResult<T>>;`,
        );
        groupImport.push(request.name, request.name + "_Response");
        responseTypes.push("  | " + request.name + "_Response");
      }
    });
    if (groupImport.length > 0) {
      imports.push(`import { ${groupImport.join(", ")} } from "@proto/${group.location}";`);
    }
  });

  const file = `
/* eslint-disable simple-import-sort/imports */

${imports.join("\n")}
import type { RobotModel } from "@components/robot/model";

/** An RPC response message */
export type RpcResponseType =
${responseTypes.join("\n")};

/**
 * Represents an error with the response given by the remote side.
 */
export interface RpcResponseError extends Error {
  request: any;
  response: any;
  defaultHandler(): void;
}

/**
 * Represents an error with sending or handling an RPC request locally on the client side.
 */
export interface RpcLocalError extends Error {
  cause: "INVALID_REQUEST" | "TIMEOUT" | "CANCELLED";
  request: any;
  RequestType: any;
  ResponseType: any;
  defaultHandler(): void;
}

export interface RpcCallOptions<T> {
  /**
   * The name of the datasource peer to send the request to.
   * If not provided, defaults to "nusight", which sends to the server peer.
   */
  target?: string;
  /**
   * The maximum time to wait for a response before resolving with a timeout error,
   * in milliseconds. If not provided, defaults to 10000ms (10s).
   */
  timeout?: number;
  /**
   * The maximum time to wait between progress updates before resolving with a timeout error,
   * in milliseconds. If not provided, defaults to the value of \`timeout\`.
   */
  progressTimeout?: number;
  /**
   * An optional AbortSignal that can be used to cancel the call.
   * If the signal is aborted, the returned promise will be resolved with a cancelled error.
   */
  signal?: AbortSignal;
  /**
   * An optional callback that will be called with progress updates from the server.
   * Useful for streaming responses.
   */
  onProgress?: (robotModel: RobotModel, progress: T) => void;
}

export type RpcResult<T> =
  | { status: "ok"; sender: RobotModel; response: T }
  | { status: "error-response"; sender: RobotModel; response: T; error: RpcResponseError }
  | { status: "remote-error"; sender: RobotModel; error: RpcResponseError }
  | { status: "invalid-request" | "timeout" | "cancelled"; error: RpcLocalError };

${callOverloads.join("\n")}

export type RpcCall = typeof rpcCall;

  `.trim();
  f.print(file);
}

runNodeJs(plugin);
