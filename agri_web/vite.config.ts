import { fileURLToPath, URL } from 'node:url'
import { Agent as HttpAgent } from 'node:http'
import { Agent as HttpsAgent } from 'node:https'

import { defineConfig, loadEnv } from 'vite'
import vue from '@vitejs/plugin-vue'

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const backendTarget = env.AGRI_BACKEND_URL || 'http://127.0.0.1:8000'
  // Without an explicit agent the proxy sends Connection: close, forcing
  // another browser TCP handshake for every expiring control heartbeat.
  const ApiAgent = new URL(backendTarget).protocol === 'https:' ? HttpsAgent : HttpAgent
  const apiAgent = new ApiAgent({ keepAlive: true, maxFreeSockets: 32 })
  const proxy = {
    '/api': { target: backendTarget, changeOrigin: true, agent: apiAgent },
    '/video': {
      target: env.AGRI_VIDEO_URL || 'http://127.0.0.1:8080', changeOrigin: true,
      rewrite: (path: string) => path.replace(/^\/video/, ''),
      timeout: 0, proxyTimeout: 0,
    },
  }
  return {
  server: { proxy },
  preview: { proxy },
  plugins: [
    vue(),
  ],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  }
})
