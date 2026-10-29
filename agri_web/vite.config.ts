import { fileURLToPath, URL } from 'node:url'

import { defineConfig, loadEnv } from 'vite'
import vue from '@vitejs/plugin-vue'

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const proxy = {
    '/api': { target: env.AGRI_BACKEND_URL || 'http://127.0.0.1:8000', changeOrigin: true },
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
