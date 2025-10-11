# NetworkForwarder

A module that forwards messages between local and network NUClear scopes, configurable per NUClearNet client and per message type.

## Consumes

- `message.network.NetworkForwarderConfig` Configuration for message forwarding.

  This message is an alternative configuration method to the `NetworkForwarder.yaml` configuration file and allows for all the same configuration options.

- `message.network.MessageRequest` Request for message forwarding from network targets.

  When received from a network source, this message requests that a specific message type be forwarded to the requester.
  The module will only honour requests for message types that are already configured to be forwarded to the requesting target.

- `NUClear::message::NetworkJoin` Network join notification.

  When a target joins the network, this notification triggers the forwarder to send the latest version of all configured messages to that target.
  This ensures new network participants immediately receive up-to-date information.

- Any message type specified in the configuration will be consumed.

  The module dynamically creates reactions for any message type specified in its configuration.
  These messages are then forwarded to network targets according to the configured parameters.

## Emits

- `message.eye.RpcResponseMeta` Response to message requests.

  Emitted to indicate success or failure when processing a message request from a network target.
  Includes the original request token and error details if the request could not be fulfilled.

- Any message type specified in the configuration.

  The module will emit any configured message types to the specified network targets.
  The emission frequency and reliability are determined by the configuration settings.
