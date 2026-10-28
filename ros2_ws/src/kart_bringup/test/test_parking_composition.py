"""Portable parking configuration and ownership guards."""
from pathlib import Path
import pytest
from kart_bringup.parking import validate

ROOT=Path(__file__).resolve().parents[1]
SITE=ROOT.parent/'kart_parking/config/site.example.json'


def test_configuration_accepts_forward_parking():
    for slot in ('P1','P2','P3'):
        validate(ROOT/'config',str(SITE),slot,'off','false')


def test_recovery_and_live_map_edits_rejected():
    with pytest.raises(ValueError,match='reverse recovery'):
        validate(ROOT/'config',str(SITE),'P1','infra_imu','false')
    validate(ROOT/'config',str(SITE),'P1','off','true')  # Read-only map + parking controls.


def test_unknown_slot_is_not_silently_changed():
    with pytest.raises(KeyError): validate(ROOT/'config',str(SITE),'P4','off','false')
