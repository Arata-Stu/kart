import math
import pytest
from kart_studio.speed_sections import sections


def test_browser_endpoint_roundoff_is_clamped():
    points=[[0,0],[10,0],[10,13.8]]
    total=23.8
    result=sections([dict(start=0,end=math.nextafter(total,math.inf),speed=1)],points,False)
    assert result[0]['end']==total


@pytest.mark.parametrize('value',[23.801,float('nan'),float('inf'),-0.001])
def test_real_out_of_range_still_rejected(value):
    with pytest.raises(ValueError,match='開始・終了WP'):
        sections([dict(start=0,end=value,speed=1)],[[0,0],[23.8,0]],False)
