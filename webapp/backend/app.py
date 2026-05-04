from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

import network
from models import ClickRequest, AddPinResponse, PinResponse, Matching
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


@app.post('/pins', response_model=AddPinResponse)
def add_pin(req: ClickRequest):
    snapped = network.snap_pin(req.lat, req.lon)
    pin = session.add_pin(snapped['lat'], snapped['lon'])
    pin.road = snapped['road']
    pin.y = snapped['y']
    return AddPinResponse(
        pin=PinResponse(id=pin.id, kind=pin.kind, lat=pin.lat, lon=pin.lon),
        matching=Matching(pairs=[], trails=[]),
    )


@app.delete('/pins/{pin_id}')
def remove_pin(pin_id: str):
    if not session.remove_pin(pin_id):
        raise HTTPException(status_code=400, detail='Cannot remove: would exceed imbalance')
    return {'matching': Matching(pairs=[], trails=[])}


@app.post('/reset')
def reset():
    session.reset()
    return {}


@app.get('/state')
def get_state():
    return {
        'pins': [{'id': p.id, 'kind': p.kind, 'lat': p.lat, 'lon': p.lon}
                 for p in session.pins],
        'matching': Matching(pairs=[], trails=[]),
    }
