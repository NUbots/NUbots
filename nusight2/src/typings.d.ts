/// <reference types="vite/client" />

declare module "*.file.svg" {
  const value: string;
  export default value;
}

// Vite web worker import syntax
declare module "*?worker" {
  const workerConstructor: {
    new (): Worker;
  };
  export default workerConstructor;
}

declare module "*.frag" {
  const shader: string;
  export default shader;
}

declare module "*.vert" {
  const shader: string;
  export default shader;
}
