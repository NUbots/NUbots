# NUsight client networking

The classes in this directory implement a network interface for interacting with NUsight's server and NUClearNet, from the browser.

## Using the `Network` class

The main class that should be used when a component needs networking is [`Network`](./network.ts), which allows for subscribing to multiple messages and unsubscribing from them all at once, as well as sending messages to the server and making RPC calls.

Tabs and other components in NUsight that need networking should create their own network class, taking an instance of the [`NUsightNetwork`](./nusight_network.ts) class and using that to create a new [`Network`](./network.ts) instance to use for networking.

For example, for a tab called **Log**, you would have a file at `src/client/components/log/network.ts` with code like the following:

```ts
import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { LogModel } from "./model";

export class LogNetwork {
  /** The network interface used for sending/receiving messages and making RPC calls */
  network: Network;

  /** The tab's data model which is updated when messages are received */
  model: LogModel;

  constructor(network: Network, model: LogModel) {
    this.network = network;
    this.model = model;
  }

  /** Used to create a new LogNetwork instance using the given NUsightNetwork and model */
  static of(nusightNetwork: NUsightNetwork, model: LogModel): LogNetwork {
    const network = Network.of(nusightNetwork);
    return new LogNetwork(network, model);
  }

  /**
   * Used to unsubscribe from all messages and cancel any RPC calls in progress.
   * Should be called when the component for the tab is unmounted.
   */
  destroy() {
    this.network.off();
  }
}
```

## Listening for messages from the server

To listen for messages from the server, the `Network.on()` method can be used, from the tab or component-specific network class.

This example adds a `onLog()` method to the `LogNetwork` class, which will be called when a `Log` message is received from the server:

```ts
// Assumes the following protobuf message is available:
// - message.output.Log

export class LogNetwork {
  // [...] (see above for class definition)

  constructor(network: Network, model: LogModel) {
    // [...] (see above for rest of constructor)

    // Listen for Log messages from the server
    this.network.on(Log, this.onLog);
  }

  /** Called when a Log message is received from the server */
  @action
  private onLog = (robotModel: RobotModel, log: Log) => {
    // Parameters are as follows:
    //   - `robotModel` has details about the peer that sent the message
    //   - `log` is the message that was sent
    // Use `log` to update the model here for rendering in the tab...
  };
}
```

## Sending messages to the server

To send a message to the server, the type-safe `Network.emit()` method can be used, from the tab or component-specific network class.

This example adds a `toggleLogging()` method to the `LogNetwork` class, which will send a `StartLogging` or `StopLogging` message to the server when called:

```ts
// Assumes the following protobuf messages are available:
//   - message.output.StartLogging
//   - message.output.StopLogging

export class LogNetwork {
  // [...] (see above for constructor, listeners, etc.)

  toggleLogging() {
    if (this.model.isLogging) {
      this.network.emit(new message.output.StopLogging());
    } else {
      this.network.emit(new message.output.StartLogging());
    }
  }
}
```

The `toggleLogging()` method can then be called from a controller or view, to send the appropriate message to the server.

## Making RPC calls

An RPC call can be used to send a message to the server when a response is expected. The type-safe `Network.call()` method can be used to make an RPC call, from the tab or component-specific network class.

This example adds a `searchLogs()` method to the `LogNetwork` class, which will send a `SearchLogsRequest` message to the server when called:

```ts
// Assumes the following protobuf messages are available:
//   - message.output.SearchLogsRequest
//   - message.output.SearchLogsRequest.Response
// Both messages must have an `rpc_token` field to be used with RPC calls.

export class LogNetwork {
  // [...] (see above for constructor, listeners, etc.)

  async searchLogs(query: string) {
    const result = await this.network.call(new message.output.SearchLogsRequest({ query: query }));

    // If the RPC call was successful, `result.ok` will be true, and `result.response` will be the response message
    if (result.ok) {
      // Use `result.response` here to update the model for rendering in the tab...
    }
    // Otherwise, `result.ok` will be false, and `result.error` will be an instance of `RpcError`,
    // which has a `cause` property that can be used to determine the cause of the error
    else {
      // Here we call the default error handler, which ignores cancellation errors and re-throws all other errors
      result.error.defaultHandler();
    }
  }
}
```

The `searchLogs()` method can then be called from a controller or view, to send the request to the server.
