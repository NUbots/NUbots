# NUsight2

A yarn-managed TypeScript project: client = React/MobX, server = Node/Express, `shared/` = code used by both
(including generated protobuf message bindings).

Scripts are usually run via `./b yarn <script>` so they execute inside the same Docker environment as the rest of
the repo, but plain `yarn <script>` works if you have the toolchain installed locally. See `package.json` for the
full script list; the ones worth knowing:

- `yarn dev` — dev server against virtual/simulated robots; `yarn start` — against real robots on the network.
- `yarn install` — also regenerates protobuf bindings via `postinstall`.

Protobuf messages shared between the C++ backend and NUsight2 are defined once (see `buf.yaml` at the repo root,
which spans `nuclear/message/proto`, `shared` and `nusight2/src/shared/messages/proto`) and generated via
`buf generate` (`yarn protobuf`) — never hand-edit the generated bindings.
