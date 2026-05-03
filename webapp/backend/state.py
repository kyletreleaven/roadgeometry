from dataclasses import dataclass, field
from typing import Literal
import uuid


@dataclass
class Pin:
    id: str
    kind: Literal['supply', 'demand']
    lat: float
    lon: float


@dataclass
class Session:
    pins: list[Pin] = field(default_factory=list)

    def add_pin(self, lat: float, lon: float) -> Pin:
        supply = sum(1 for p in self.pins if p.kind == 'supply')
        demand = sum(1 for p in self.pins if p.kind == 'demand')
        kind: Literal['supply', 'demand'] = 'supply' if supply <= demand else 'demand'
        pin = Pin(id=str(uuid.uuid4()), kind=kind, lat=lat, lon=lon)
        self.pins.append(pin)
        return pin

    def remove_pin(self, pin_id: str) -> bool:
        pin = next((p for p in self.pins if p.id == pin_id), None)
        if pin is None:
            return False
        supply = sum(1 for p in self.pins if p.kind == 'supply')
        demand = sum(1 for p in self.pins if p.kind == 'demand')
        new_supply = supply - (1 if pin.kind == 'supply' else 0)
        new_demand = demand - (1 if pin.kind == 'demand' else 0)
        if abs(new_supply - new_demand) > 1:
            return False
        self.pins.remove(pin)
        return True

    def reset(self):
        self.pins.clear()


session = Session()
