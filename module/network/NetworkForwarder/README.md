# NetworkForwarder

## Description

Forwards selected NUClear messages to named network targets over NUClearNet.

Forwarding is configured per target and per message type in `NetworkForwarder.yaml`.
Each type can be disabled, sent at full rate, or rate-limited to a maximum frequency.

If a message type has an `id` field, rate limiting is applied per-id. Otherwise, a single global rate limit is used for that message type/target.

## Usage

Configure targets and message types in `data/config/NetworkForwarder.yaml`:

- `false`: disable forwarding for this message type
- `true`: forward every message
- `<number>`: forward at most `<number>` messages per second (fps)

Example:

```yaml
targets:
	nusight:
		message.support.nusight.Overview: true
		message.output.CompressedImage: 10
		message.nuclear.LogMessage: false
```

Notes:

- Message names use protobuf-style dotted names (e.g. `message.support.nusight.Overview`)
- Unknown type names are ignored with a warning
- Reactions are enabled only when at least one target is subscribed to that message type

## Consumes

- `extension::Configuration` from `NetworkForwarder.yaml`
- Dynamically registered NUClear messages listed in `targets` (one reaction per known type)

## Emits

- Configured message types forwarded via `Scope::NETWORK` to each configured target

## Dependencies

- NUClear networking (`Scope::NETWORK`)
- Python 3 at build time (used to generate message handle registrations)
