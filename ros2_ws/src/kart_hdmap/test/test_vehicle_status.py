import json
from types import SimpleNamespace as NS
from unittest.mock import patch
from kart_hdmap.vehicle_status import VehicleStatus


def recovery(state='reversing', stamp=10):
    return json.dumps(dict(schema='kart.recovery.v1', state=state, stamp=stamp,
                           attempts=2, reason='low_flow', settings={'large':'omitted'}))


def test_phase_freshness_and_invalid_payload_clear_old_state():
    cache = VehicleStatus()
    with patch('kart_hdmap.vehicle_status.time.monotonic', return_value=20):
        cache.recovery(recovery())
        result = cache.view(10.1, dict(bridge=0, recovery=1))
        assert result['bridge']['availability'] == 'absent'
        assert result['recovery']['state'] == 'reversing'
        assert 'settings' not in result['recovery']
        assert cache.view(11, dict(bridge=0, recovery=1))['recovery']['availability'] == 'stale'
        assert cache.view(9, dict(bridge=0, recovery=1))['recovery']['availability'] == 'stale'
        assert cache.view(10, dict(bridge=0, recovery=2))['recovery']['availability'] == 'conflict'
        cache.recovery('{bad')
        assert cache.view(10, dict(bridge=0, recovery=1))['recovery']['availability'] == 'waiting'
        for raw in ['null','[]',recovery('collision'),recovery(stamp=float('nan'))]:
            cache.recovery(raw)
            assert 'recovery' not in cache.samples
        cache.recovery(recovery('suspect'))
    with patch('kart_hdmap.vehicle_status.time.monotonic', return_value=21):
        assert cache.view(10, dict(bridge=0, recovery=1))['recovery']['availability'] == 'stale'


def test_bridge_filters_shared_diagnostics_and_preserves_current_fault_only():
    cache=VehicleStatus()
    def status(name='kart_bridge', level=0):
        return NS(name=name, hardware_id='JPBB-01', level=level, message='current',
                  values=[NS(key='steering_offset', value='0.02'),
                          NS(key='last_interlock', value='historic')])
    with patch('kart_hdmap.vehicle_status.time.monotonic', return_value=20):
        cache.bridge(10,[status('other')])
        assert 'bridge' not in cache.samples
        cache.bridge(10,[status(level=2)])
        b=cache.view(10.1,dict(bridge=1,recovery=0))['bridge']
        assert b['level']==2 and b['values']=={'steering_offset':'0.02'}
        cache.bridge(10,[status('other')])
        assert cache.view(10.1,dict(bridge=1,recovery=0))['bridge']['level']==2
        cache.bridge(10,[status(),status()])
        assert 'bridge' not in cache.samples


def test_bridge_accepts_integer_and_ros_byte_levels_without_stopping_updates():
    cache = VehicleStatus()
    with patch('kart_hdmap.vehicle_status.time.monotonic', return_value=20):
        for level in range(4):
            for raw in (level, bytes([level])):
                status = NS(name='kart_bridge', hardware_id='JPBB-01',
                            level=raw, message='current', values=[])
                cache.bridge(10, [status])
                result = cache.view(10.1, dict(bridge=1, recovery=0))['bridge']
                assert result['availability'] == 'fresh'
                assert type(result['level']) is int and result['level'] == level
                assert json.loads(json.dumps(result))['level'] == level


def test_invalid_bridge_level_clears_old_status_and_allows_next_valid_sample():
    cache = VehicleStatus()
    status = NS(name='kart_bridge', hardware_id='JPBB-01',
                level=b'\x00', message='current', values=[])
    with patch('kart_hdmap.vehicle_status.time.monotonic', return_value=20):
        for invalid in (b'', b'\x00\x01', b'\xff', -1, 4, None, '0', 0.0, True):
            status.level = b'\x00'
            cache.bridge(10, [status])
            assert 'bridge' in cache.samples
            status.level = invalid
            cache.bridge(10, [status])
            assert cache.view(10.1, dict(bridge=1, recovery=0))['bridge']['availability'] == 'waiting'
        status.level = b'\x02'
        cache.bridge(10, [status])
        assert cache.view(10.1, dict(bridge=1, recovery=0))['bridge']['level'] == 2
