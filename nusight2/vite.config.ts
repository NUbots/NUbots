import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'
import svgr from 'vite-plugin-svgr'

import glslPlugin from './build_scripts/glsl_vite_plugin'

// See https://vitejs.dev/config/
export default defineConfig({
  publicDir: './src/assets',
  optimizeDeps: {
    // These dependencies are not auto-detected in Vite's initial pre-bundling run.
    // Adding them here includes them in the first run, avoiding a second one.
    include: ['socket.io-client', 'socket.io-parser'],
  },
  server: {
    port: 3000,
    hmr: {
      // Override the default HMR port so we can have a known port to expose
      // when running from Docker.
      port: 3010,
    },
  },
  build: {
    target: 'es2020',
    emptyOutDir: false,
    // Raise the size limit for output chunk warnings (we have chunks bigger than the default 500kb limit)
    // Remove this option to see the warnings about the big chunks and the suggested fixes.
    chunkSizeWarningLimit: 1000,
  },
  plugins: [
    // SVG Loader
    svgr({ exportAsDefault: true }),
    react({
      babel: {
        parserOpts: {
          // Enable decorators, since we use them for mobx
          plugins: ['decorators-legacy'],
        },
        env: {
          development: {
            // Don't attempt to strip whitespace from generated code,
            // since Babel gives up on bundles > 500kb and complains.
            // Same for `production.compact` below.
            compact: false,
          },
          production: {
            compact: false,
          },
        },
      },
    }),
    // Our custom GLSL loading plugin with #include support
    glslPlugin(),
  ],
})
