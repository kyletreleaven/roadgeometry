from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from models import ClickRequest, AddPinResponse, PinResponse, Matching
from state import session

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=['http://localhost:5173'],
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.get('/status')
def status():
    return {'ready': True}


@app.post('/pins', response_model=AddPinResponse)
def add_pin(req: ClickRequest):
    pin = session.add_pin(req.lat, req.lon)
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
