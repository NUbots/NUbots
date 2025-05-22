import { useEffect, useMemo } from "react";

import { useNUsightNetwork } from "../components/app/context";
import { Network } from "../network/network";
import { RpcClient } from "../network/rpc_client";

export interface RpcNetwork {
  call: typeof RpcClient.prototype.call;
}

/**
 * Create a memoized controller from the current (context-provided) NUsight network. Will create a
 * component-specific network and provide it to the given factory function to create the controller.
 *
 * The network is automatically cleaned up when the component is unmounted or a dependency changes.
 */
export function useRpcController<T>(createController: (network: RpcNetwork) => T, deps: React.DependencyList): T {
  const nusightNetwork = useNUsightNetwork();
  const network = useMemo(() => new Network(nusightNetwork), [nusightNetwork]);

  useEffect(() => {
    () => network.off();
  }, [network]);

  return useMemo(() => createController(network), [network, ...deps]);
}
