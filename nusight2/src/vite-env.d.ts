/// <reference types="vite-plugin-svgr/client" />

// Vite web workers
declare module '*?worker' {
  const workerConstructor: {
    new (): Worker
  }
  export default workerConstructor
}

// Vite import asset as URL string
declare module '*?url' {
  const src: string
  export default src
}
