"""Load the ROS-free shared HDMap parking contract from this checkout."""
import importlib.util
import importlib
import sys
from pathlib import Path
_source=Path(__file__).resolve().parents[3]/'ros2_ws/src/kart_hdmap/kart_hdmap'
_spec=importlib.util.spec_from_file_location('_kart_parking_contract',_source/'__init__.py',submodule_search_locations=[str(_source)])
sys.modules.setdefault(_spec.name,importlib.util.module_from_spec(_spec))
_module=importlib.import_module('_kart_parking_contract.parking')
validate=_module.validate
warnings=_module.warnings
