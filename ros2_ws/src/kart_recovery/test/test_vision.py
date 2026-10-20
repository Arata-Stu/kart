from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from kart_recovery.vision import FrameMotion, event_activity, grayscale, image_array


def texture():
    image = np.zeros((120, 212), np.uint8)
    for y in range(12, 112, 15):
        for x in range(12, 202, 15):
            cv2.rectangle(image, (x, y), (x + 4, y + 4), 255, -1)
    return image


def test_still_translation_and_no_texture():
    image = texture()
    m = FrameMotion()
    assert not m.update(image, 1)[0]
    valid, flow, count = m.update(image, 1.1)
    assert valid and flow < .1 and count >= 20
    shifted = cv2.warpAffine(image, np.float32([[1, 0, 2], [0, 1, 0]]), (212, 120))
    valid, flow, _ = m.update(shifted, 1.2)
    assert valid and 18 < flow < 22
    m = FrameMotion()
    blank = np.zeros_like(image)
    m.update(blank, 1)
    assert not m.update(blank, 1.1)[0]


def test_duplicate_stamp_gap_and_sparse_texture_are_unknown():
    image = texture()
    m = FrameMotion()
    m.update(image, 1)
    assert not m.update(image, 1)[0]
    assert not m.update(image, 2)[0]
    sparse = np.zeros_like(image)
    sparse[:35, :35] = image[:35, :35]
    m = FrameMotion(min_features=2, min_cells=4)
    m.update(sparse, 1)
    assert not m.update(sparse, 1.1)[0]


def test_event_image_empty_hot_pixel_and_supported_activity():
    image = np.full((40, 40, 3), 255, np.uint8)
    assert event_activity(image, 'bgr8') == 0
    image[10, 10] = (0, 0, 255)
    assert event_activity(image, 'bgr8') == 0
    image[20:30, 20:30] = (0, 0, 255)
    assert event_activity(image, 'bgr8') == pytest.approx(100 / 1600)
    with pytest.raises(ValueError):
        event_activity(image[:, :, 0], 'mono8')


def test_padded_rows_and_channel_conversion():
    msg = SimpleNamespace(width=2, height=2, step=4, encoding='mono8', data=bytes([1,2,99,99,3,4,99,99]))
    assert image_array(msg).tolist() == [[1, 2], [3, 4]]
    msg.data = b'bad'
    with pytest.raises(ValueError):
        image_array(msg)
    assert grayscale(np.zeros((120, 212, 3), np.uint8), 'rgb8', 106).shape == (60, 106)
