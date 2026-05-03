import pytest
from state import Session


def make_session(*kinds):
    s = Session()
    for kind in kinds:
        lat, lon = 0.0, 0.0
        pin = s.add_pin(lat, lon)
        assert pin.kind == kind
    return s


# --- add_pin ---

def test_first_pin_is_supply():
    s = Session()
    assert s.add_pin(0, 0).kind == 'supply'

def test_alternates_supply_demand():
    s = Session()
    kinds = [s.add_pin(0, 0).kind for _ in range(4)]
    assert kinds == ['supply', 'demand', 'supply', 'demand']

def test_tie_broken_by_supply():
    # After one supply + one demand, next should be supply again
    s = make_session('supply', 'demand')
    assert s.add_pin(0, 0).kind == 'supply'


# --- remove_pin ---

def test_remove_only_pin():
    s = make_session('supply')
    pin_id = s.pins[0].id
    assert s.remove_pin(pin_id)
    assert s.pins == []

def test_remove_from_balanced_either_allowed():
    s = make_session('supply', 'demand')
    supply_id = s.pins[0].id
    demand_id = s.pins[1].id
    # removing supply from balanced: imbalance becomes 1 (demand excess) → ok
    s2 = make_session('supply', 'demand')
    assert s2.remove_pin(s2.pins[0].id)
    # removing demand from balanced: imbalance becomes 1 (supply excess) → ok
    s3 = make_session('supply', 'demand')
    assert s3.remove_pin(s3.pins[1].id)

def test_remove_minority_rejected_when_imbalanced():
    # supply=2, demand=1 → removing demand would give imbalance 2
    s = make_session('supply', 'demand', 'supply')
    demand_id = next(p.id for p in s.pins if p.kind == 'demand')
    assert not s.remove_pin(demand_id)
    assert len(s.pins) == 3  # unchanged

def test_remove_majority_allowed_when_imbalanced():
    # supply=2, demand=1 → removing a supply gives imbalance 0
    s = make_session('supply', 'demand', 'supply')
    supply_id = next(p.id for p in s.pins if p.kind == 'supply')
    assert s.remove_pin(supply_id)
    assert len(s.pins) == 2

def test_remove_nonexistent_pin():
    s = make_session('supply')
    assert not s.remove_pin('no-such-id')

def test_remove_second_of_same_type_rejected():
    # supply=2, demand=2 → remove one supply → supply=1, demand=2 (imbalance 1)
    # → remove another supply → supply=0, demand=2 (imbalance 2) → rejected
    s = make_session('supply', 'demand', 'supply', 'demand')
    supply_ids = [p.id for p in s.pins if p.kind == 'supply']
    assert s.remove_pin(supply_ids[0])
    assert not s.remove_pin(supply_ids[1])
    assert len(s.pins) == 3

def test_remove_preserves_order():
    s = make_session('supply', 'demand', 'supply', 'demand')
    middle_id = s.pins[1].id
    s.remove_pin(middle_id)
    assert [p.id for p in s.pins] == [s.pins[0].id, s.pins[1].id, s.pins[2].id]
