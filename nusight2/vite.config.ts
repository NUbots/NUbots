/* eslint-env node */
import { defineConfig } from "vite";

import glslPlugin from "./build_scripts/glsl_plugin";

// See https://vitejs.dev/config/
export default defineConfig({
  publicDir: "./src/assets",
  optimizeDeps: {
    // These dependencies are not auto-detected in Vite's initial pre-bundling run.
    // Adding them here includes them in the first run, avoiding a second one.
    include: ["socket.io-client", "socket.io-parser"],
  },
  server: {
    port: 3000,
    hmr: {
      // Override the default HMR port so we can have a known port to expose
      // when running from Docker.
      port: 3010,
    },
  },
  esbuild: {
    supported: {
      decorators: false,
    }
  },
  build: {
    target: "es2020",
    emptyOutDir: false,
    // Raise the size limit for output chunk warnings (we have chunks bigger than the default 500kb limit)
    // Remove this option to see the warnings about the big chunks and the suggested fixes.
    chunkSizeWarningLimit: 1000,
    rollupOptions: {
      // The openlayers library (ol) which we use for the map imports geotiff.js,
      // which imports these node built-ins. We don't use the features in ol
      // that require geotiff, so we can safely mark these as external for
      // rollup to avoid bundling them.
      external: ["geotiff", "geotiff/src/compression", "fs", "http", "https", "url"],
    },
  },
  plugins: [
    glslPlugin(),
  ],
});
