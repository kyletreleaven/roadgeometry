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

interface Timing {
  translate_ms: number
  flow_ms: number
  path_network_ms: number
  trails_ms: number
  matching_ms: number
  total_ms: number
}

interface Matching {
  trails: Trail[]
}

interface BackendEntry {
  current: string
  available: string[]
}

type BackendState = Record<string, BackendEntry>

export default function App() {
  const mapDiv = useRef<HTMLDivElement>(null)
  const map = useRef<L.Map | null>(null)
  const pinLayer = useRef<L.LayerGroup | null>(null)
  const trailLayer = useRef<L.LayerGroup | null>(null)
  const markers = useRef<Map<string, L.CircleMarker>>(new Map())
  const readyRef = useRef(false)
  const [ready, setReady] = useState(false)
  const [timing, setTiming] = useState<Timing | null>(null)
  const [backend, setBackend] = useState<BackendState | null>(null)

  useEffect(() => {
    if (!map.current && mapDiv.current) {
      map.current = L.map(mapDiv.current, { dragging: false, zoomControl: false, scrollWheelZoom: false, doubleClickZoom: false, touchZoom: false, boxZoom: false, keyboard: false }).setView(CAMBRIDGE_CENTER, DEFAULT_ZOOM)
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

    fetch('/reset', { method: 'POST' }).catch(() => {})
    fetch('/backend').then(r => r.json()).then(setBackend).catch(() => {})

    const onClick = async (e: L.LeafletMouseEvent) => {
      if (!readyRef.current) return
      try {
        const res = await fetch('/pins', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ lat: e.latlng.lat, lon: e.latlng.lng }),
        })
        const data: { pin: Pin; matching: Matching; timing: Timing } = await res.json()
        placeMarker(data.pin)
        renderTrails(data.matching.trails)
        setTiming(data.timing)
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

  async function selectBackend(key: string, value: string) {
    try {
      const res = await fetch('/backend', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ [key]: value }),
      })
      setBackend(await res.json())
    } catch (err) {
      console.error('POST /backend failed:', err)
    }
  }

  function placeMarker(pin: Pin) {
    const color = PIN_COLOR[pin.kind]
    const marker = L.circleMarker([pin.lat, pin.lon], {
      radius: 8, color, fillColor: color, fillOpacity: 0.9, weight: 2,
    }).addTo(pinLayer.current!)
    marker.on('click', (e) => {
      L.DomEvent.stopPropagation(e)
      removePin(pin.id, marker)
    })
    markers.current.set(pin.id, marker)
  }

  async function removePin(id: string, marker: L.CircleMarker) {
    try {
      const res = await fetch(`/pins/${id}`, { method: 'DELETE' })
      if (!res.ok) return
      const data: { matching: Matching } = await res.json()
      marker.remove()
      markers.current.delete(id)
      renderTrails(data.matching.trails)
    } catch (err) {
      console.error('DELETE /pins failed:', err)
    }
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
    markers.current.clear()
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
      {(timing || backend) && (
        <div style={{
          position: 'absolute', bottom: 12, left: 12, zIndex: 1000,
          background: 'rgba(255,255,255,0.85)', padding: '6px 8px',
          fontFamily: 'monospace', fontSize: 12, borderRadius: 3,
          border: '1px solid #ccc', lineHeight: 1.8,
        }}>
          {backend && Object.entries(backend).map(([key, { current, available }]) => (
            <div key={key} style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
              <span style={{ marginRight: 4, color: '#555' }}>{key}:</span>
              {available.map(v => (
                <button key={v} onClick={() => selectBackend(key, v)} style={{
                  padding: '0px 5px', fontSize: 11, cursor: 'pointer',
                  background: v === current ? '#333' : '#eee',
                  color: v === current ? '#fff' : '#555',
                  border: '1px solid #aaa', borderRadius: 2,
                  fontFamily: 'monospace',
                }}>{v}</button>
              ))}
            </div>
          ))}
          {timing && backend && <div style={{ borderTop: '1px solid #ddd', margin: '3px 0' }} />}
          {timing && <>
            {timing.translate_ms > 0 && <>translate: {timing.translate_ms.toFixed(1)}ms<br /></>}
            flow: {timing.flow_ms.toFixed(1)}ms<br />
            path network: {timing.path_network_ms.toFixed(1)}ms<br />
            trails: {timing.trails_ms.toFixed(1)}ms<br />
            matching: {timing.matching_ms.toFixed(1)}ms<br />
            total: {timing.total_ms.toFixed(1)}ms
          </>}
        </div>
      )}
    </div>
  )
}
