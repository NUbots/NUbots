declare module '*.worker.ts' {
  type WorkerLoader = new() => Worker
  const content: WorkerLoader
  export = content
}
