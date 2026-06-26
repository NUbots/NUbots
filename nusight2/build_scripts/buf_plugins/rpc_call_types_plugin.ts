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

  messagesGroupedByLocation.forEach((group) => {
    const groupImport: string[] = [];
    const requestMessagesList = group.messages.filter((message) => message.name.endsWith("Request"));
    // Iterate over list of requests and add an overload if a corresponding Response message exists
    requestMessagesList.forEach((request) => {
      if (group.messages.some((message) => message.name === request.name + "_Response")) {
        callOverloads.push(
          `function rpcCall<T = ${request.name}_Response>(request: ${request.name}, options?: RpcCallOptions): Promise<RpcResult<T>>;`,
        );
        groupImport.push(request.name, request.name + "_Response");
      }
    });
    if (groupImport.length > 0) {
      imports.push(`import type { ${groupImport.join(", ")} } from "@proto/${group.location}";`);
    }
  });

  const file = `
/* eslint-disable simple-import-sort/imports */

${imports.join("\n")}
import type { RobotModel } from "@components/robot/model";

export type RpcErrorCause = "INVALID_REQUEST" | "TIMEOUT" | "CANCELLED" | "REMOTE_ERROR" | "UNKNOWN";

export interface RpcErrorOptions {
  cause: RpcErrorCause;
  request: any;
  RequestType: any;
  ResponseType: any;
}

export class RpcError extends Error {
  cause: RpcErrorOptions["cause"];
  request: RpcErrorOptions["request"];
  RequestType: RpcErrorOptions["RequestType"];
  ResponseType: RpcErrorOptions["ResponseType"];
  constructor(message: string, options: RpcErrorOptions);
  /** Determine if the error was caused by an error on the remote end */
  isRemoteError(): boolean;
  /** Determine if the error was caused by the request timing out */
  isTimeout(): boolean;
  /** Determine if the error was caused by the request being cancelled */
  isCancelled(): boolean;
  /** Handle the error based on its cause. Ignores errors caused by cancellation, and throws all other errors. */
  defaultHandler(): void;
}

export interface RpcCallOptions { target?: string; timeout?: number; }

export interface RpcResultData<T> { robotModel: RobotModel; response: T; }

export type RpcResult<T> = { ok: true; data: RpcResultData<T> } | { ok: false; error: RpcError };

${callOverloads.join("\n")}

export type RpcCall = typeof rpcCall;
  `.trim();
  f.print(file);
}

runNodeJs(plugin);
