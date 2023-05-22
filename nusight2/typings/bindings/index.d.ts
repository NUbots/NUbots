declare module "bindings" {
  const bindings: (
    name:
      | string
      | {
          bindings: string;
          module_root: string;
        },
  ) => any;
  export = bindings;
}
