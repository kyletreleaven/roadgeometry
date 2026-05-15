from contextlib import asynccontextmanager
import time

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

import network
import matching as matching_module
from models import ClickRequest, AddPinResponse, PinResponse, Matching, Timing
from state import session


@asynccontextmanager
async def lifespan(app: FastAPI):
    network.start()
    yield

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=['http://localhost:5173'],
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.get('/status')
def status():
    return {'ready': network.is_ready()}


@app.get('/backend')
def get_backend():
    return matching_module.backend_state()


@app.post('/backend')
def update_backend(req: dict[str, str]):
    for key, value in req.items():
        try:
            matching_module.set_backend(key, value)
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
    return matching_module.backend_state()


@app.post('/pins', response_model=AddPinResponse)
def add_pin(req: ClickRequest):
    t0 = time.perf_counter()
    snapped = network.snap_pin(req.lat, req.lon)
    pin = session.add_pin(snapped['lat'], snapped['lon'])
    pin.road = snapped['road']
    pin.y = snapped['y']
    t1 = time.perf_counter()
    result = matching_module.run_matching(session)
    t2 = time.perf_counter()
    return AddPinResponse(
        pin=PinResponse(id=pin.id, kind=pin.kind, lat=pin.lat, lon=pin.lon),
        matching=Matching(**result),
        timing=Timing(**result['timing'], matching_ms=(t2 - t1) * 1000, total_ms=(t2 - t0) * 1000),
    )


@app.delete('/pins/{pin_id}')
def remove_pin(pin_id: str):
    if not session.remove_pin(pin_id):
        raise HTTPException(status_code=400, detail='Cannot remove: would exceed imbalance')
    result = matching_module.run_matching(session)
    return {'matching': Matching(**result)}


@app.post('/reset')
def reset():
    session.reset()
    return {}


@app.post('/capture')
def capture():
    from setiptah.roadgeometry.matching.io import save_pins

    supply_pins = [p for p in session.pins if p.kind == 'supply' and p.road is not None]
    demand_pins = [p for p in session.pins if p.kind == 'demand' and p.road is not None]
    n = min(len(supply_pins), len(demand_pins))

    P = [(p.road, p.y) for p in supply_pins[:n]]
    Q = [(p.road, p.y) for p in demand_pins[:n]]

    path = 'captured_pins.json'
    save_pins(P, Q, path)
    return {'saved': path, 'n_supply': len(P), 'n_demand': len(Q)}


@app.get('/state')
def get_state():
    return {
        'pins': [{'id': p.id, 'kind': p.kind, 'lat': p.lat, 'lon': p.lon}
                 for p in session.pins],
        'matching': Matching(pairs=[], trails=[]),
    }
