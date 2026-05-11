import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/status': 'http://localhost:8000',
      '/pins': 'http://localhost:8000',
      '/reset': 'http://localhost:8000',
      '/state': 'http://localhost:8000',
      '/backend': 'http://localhost:8000',
      '/capture': 'http://localhost:8000',
    },
  },
})
