import { useEffect, useRef, useState } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'

const CAMBRIDGE_CENTER: [number, number] = [42.373, -71.109]
const DEFAULT_ZOOM = 14

const PIN_COLOR = { supply: '#e03030', demand: '#3060e0' }
const TRAIL_STYLE = { color: '#444', weight: 3, opacity: 0.8 }

interface Pin {
  id: string
  kind: 'supply' | 'demand'
  lat: number
  lon: number
}

interface Trail {
  coordinates: [number, number][]
}

interface Matching {
  trails: Trail[]
}

export default function App() {
  const mapDiv = useRef<HTMLDivElement>(null)
  const map = useRef<L.Map | null>(null)
  const pinLayer = useRef<L.LayerGroup | null>(null)
  const trailLayer = useRef<L.LayerGroup | null>(null)
  const readyRef = useRef(false)
  const [ready, setReady] = useState(false)

  useEffect(() => {
    if (!map.current && mapDiv.current) {
      map.current = L.map(mapDiv.current).setView(CAMBRIDGE_CENTER, DEFAULT_ZOOM)
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
        maxZoom: 19,
      }).addTo(map.current)
      pinLayer.current = L.layerGroup().addTo(map.current)
      trailLayer.current = L.layerGroup().addTo(map.current)
    }

    let active = true

    const pollStatus = async () => {
      while (active && !readyRef.current) {
        try {
          const res = await fetch('/status')
          const data = await res.json()
          if (data.ready) {
            readyRef.current = true
            setReady(true)
            break
          }
        } catch {}
        await new Promise(r => setTimeout(r, 1000))
      }
    }
    pollStatus()

    const onClick = async (e: L.LeafletMouseEvent) => {
      if (!readyRef.current) return
      try {
        const res = await fetch('/pins', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ lat: e.latlng.lat, lon: e.latlng.lng }),
        })
        const data: { pin: Pin; matching: Matching } = await res.json()
        placeMarker(data.pin)
        renderTrails(data.matching.trails)
      } catch (err) {
        console.error('POST /pins failed:', err)
      }
    }
    map.current!.on('click', onClick)

    return () => {
      active = false
      map.current!.off('click', onClick)
    }
  }, [])

  function placeMarker(pin: Pin) {
    const color = PIN_COLOR[pin.kind]
    L.circleMarker([pin.lat, pin.lon], {
      radius: 8, color, fillColor: color, fillOpacity: 0.9, weight: 2,
    }).addTo(pinLayer.current!)
  }

  function renderTrails(trails: Trail[]) {
    trailLayer.current!.clearLayers()
    for (const trail of trails)
      L.polyline(trail.coordinates, TRAIL_STYLE).addTo(trailLayer.current!)
  }

  async function handleReset() {
    try {
      await fetch('/reset', { method: 'POST' })
    } catch {}
    pinLayer.current!.clearLayers()
    trailLayer.current!.clearLayers()
  }

  return (
    <div style={{ width: '100%', height: '100vh', position: 'relative' }}>
      <div ref={mapDiv} style={{ width: '100%', height: '100%' }} />
      {!ready && (
        <div style={{
          position: 'absolute', inset: 0, zIndex: 1000,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          background: 'rgba(255,255,255,0.65)', fontSize: 18, fontFamily: 'sans-serif',
        }}>
          Loading road network…
        </div>
      )}
      {/* Reset button — restore when backend is ready */}
    </div>
  )
}
