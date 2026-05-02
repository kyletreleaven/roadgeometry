import { useEffect, useRef } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'

const CAMBRIDGE_CENTER: [number, number] = [42.373, -71.109]
const DEFAULT_ZOOM = 14

export default function App() {
  const mapRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    if (!mapRef.current) return
    const map = L.map(mapRef.current).setView(CAMBRIDGE_CENTER, DEFAULT_ZOOM)
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
      maxZoom: 19,
    }).addTo(map)
    return () => { map.remove() }
  }, [])

  return <div ref={mapRef} style={{ width: '100%', height: '100vh' }} />
}
