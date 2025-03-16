import type {
  RpcCall,
  RpcCallOptions,
  RpcError as RpcErrorType,
  RpcErrorCause,
  RpcErrorOptions,
  RpcResult,
} from "../../shared/messages/rpc_call";

import { Network } from "./network";

/** Represents an error that occurred while making an RPC call */
export class RpcError extends Error implements RpcErrorType {
  cause: RpcErrorCause;
  request: any;
  RequestType: any;
  ResponseType: any;

  constructor(message: string, options: RpcErrorOptions) {
    super(message);
    this.cause = options.cause ?? "UNKNOWN";
    this.request = options.request;
    this.RequestType = options.RequestType;
    this.ResponseType = options.ResponseType;
  }

  isRemoteError() {
    return this.cause === "REMOTE_ERROR";
  }

  isTimeout() {
    return this.cause === "TIMEOUT";
  }

  isCancelled() {
    return this.cause === "CANCELLED";
  }

  defaultHandler() {
    if (this.cause === "CANCELLED") {
      return;
    }

    throw this;
  }
}

/** Used to generate unique tokens for RPC calls */
export let nextRpcToken = 0;

/**
 * Provides a client interface for making RPC calls. It keeps track of
 * calls that are in progress, to allow for cancellation.
 */
export class RpcClient {
  network: Network;

  /** Tokens for RPC calls currently in progress */
  tokensInFlight: Set<number>;

  /** Tokens for RPC calls that have been cancelled */
  tokensCancelled: Set<number>;

  constructor(network: Network) {
    this.network = network;
    this.tokensInFlight = new Set();
    this.tokensCancelled = new Set();
  }

  call: RpcCall = (request, options) => {
    // Generate a unique token for the request and add it to the set of tokens in flight
    const token = nextRpcToken++;
    this.tokensInFlight.add(token);

    // Make the call and remove the token from the set of tokens in flight after it completes,
    // returning the result or throwing the error
    return this.makeCall(request, options ?? {}, token)
      .then((result) => {
        this.tokensInFlight.delete(token);
        return result;
      })
      .catch((error) => {
        this.tokensInFlight.delete(token);
        throw error;
      });
  };

  private makeCall(request: any, options: RpcCallOptions, requestToken: number): Promise<RpcResult<any>> {
    return new Promise((resolve) => {
      // Get the request and response Protobuf types
      const RequestType = request ? Object.getPrototypeOf(request)?.constructor : undefined;
      const ResponseType = RequestType?.Response;

      // Ensure that the request type is valid
      if (!RequestType) {
        resolve({
          ok: false,
          error: new RpcError("Request protobuf type for RPC call not found", {
            cause: "INVALID_REQUEST",
            request,
            RequestType,
            ResponseType,
          }),
        });
        return;
      }

      // Ensure that the response type is valid
      if (!ResponseType) {
        resolve({
          ok: false,
          error: new RpcError("Response protobuf type for RPC call not found", {
            cause: "INVALID_REQUEST",
            request,
            RequestType,
            ResponseType,
          }),
        });
        return;
      }

      // Set the request token
      request.rpc = {
        token: requestToken,
      };

      // Set up a listener for the response
      const removeOn = this.network.on(
        ResponseType,
        (robotModel, response: { rpc: { token: number; ok: boolean; error?: string } }) => {
          // Ignore responses that don't match the request token
          if (response.rpc.token !== requestToken) {
            return;
          }

          // Clear the timeout timer and remove the listener
          clearTimeout(timeoutTimer);
          removeOn();

          // If the request was cancelled, resolve with a cancelled error
          if (this.tokensCancelled.has(requestToken)) {
            this.tokensCancelled.delete(requestToken);
            resolve({
              ok: false,
              error: new RpcError("RPC call cancelled", {
                cause: "CANCELLED",
                request,
                RequestType,
                ResponseType,
              }),
            });
          }
          // If there was an error from the remote end, resolve with an error
          else if (!response.rpc.ok) {
            resolve({
              ok: false,
              error: new RpcError(response.rpc.error ?? "Unknown error from remote", {
                cause: "REMOTE_ERROR",
                request,
                RequestType,
                ResponseType,
              }),
            });
          }
          // Otherwise, resolve with the response
          else {
            resolve({
              ok: true,
              data: { robotModel, response },
            });
          }
        },
      );

      // Set a timeout for the request
      const timeoutTimer = setTimeout(
        () => {
          // Remove the listener for the response
          removeOn();

          // If the request was cancelled, resolve with a cancelled error
          if (this.tokensCancelled.has(requestToken)) {
            this.tokensCancelled.delete(requestToken);
            resolve({
              ok: false,
              error: new RpcError("RPC call cancelled", {
                cause: "CANCELLED",
                request,
                RequestType,
                ResponseType,
              }),
            });
          }
          // Otherwise, resolve with a timeout error
          else {
            resolve({
              ok: false,
              error: new RpcError("RPC call timed out", {
                cause: "TIMEOUT",
                request,
                RequestType,
                ResponseType,
              }),
            });
          }
        },
        options.timeout ?? 10 * 1000,
      );

      // Send the request
      this.network.emit(new RequestType(request), {
        target: options.target ?? "nusight",
        reliable: true,
      });
    });
  }

  /** Cancel all RPC calls for this client that are currently in progress */
  cancelAll() {
    this.tokensCancelled = new Set([...this.tokensCancelled, ...this.tokensInFlight]);
  }
}
