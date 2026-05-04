from pydantic import BaseModel
from typing import Literal


class ClickRequest(BaseModel):
    lat: float
    lon: float


class PinResponse(BaseModel):
    id: str
    kind: Literal['supply', 'demand']
    lat: float
    lon: float


class Trail(BaseModel):
    coordinates: list[tuple[float, float]]


class Pair(BaseModel):
    supply_id: str
    demand_id: str


class Matching(BaseModel):
    pairs: list[Pair]
    trails: list[Trail]


class AddPinResponse(BaseModel):
    pin: PinResponse
    matching: Matching
