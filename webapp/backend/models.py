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


class Timing(BaseModel):
    translate_ms: float        # building endpoints/lengths/is_oneway dicts (cpp path only)
    flow_ms: float             # compute_optimal (flow + matching)
    path_network_ms: float     # create_path_network_with_surplus
    trails_ms: float           # shortest path + coord extraction
    matching_ms: float         # total run_matching()
    total_ms: float            # full request


class AddPinResponse(BaseModel):
    pin: PinResponse
    matching: Matching
    timing: Timing
