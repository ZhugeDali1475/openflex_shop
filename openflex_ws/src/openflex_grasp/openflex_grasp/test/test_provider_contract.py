import numpy as np
import pytest

from openflex_grasp.perception.providers.base import BoxRequest


def test_box_request_copies_and_freezes_rgb_and_keeps_prompt_fields():
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    request = BoxRequest(rgb, "农夫山泉", "a clear mineral water bottle", max_boxes=3)
    rgb[0, 0] = 255
    assert request.rgb[0, 0].tolist() == [0, 0, 0]
    assert not request.rgb.flags.writeable
    assert request.sku_name == "农夫山泉"
    assert request.owlv2_prompt == "a clear mineral water bottle"


@pytest.mark.parametrize("kwargs", [{"sku_name": ""}, {"max_boxes": 0}])
def test_box_request_rejects_invalid_contract(kwargs):
    values = {"rgb": np.zeros((2, 2, 3), dtype=np.uint8), "sku_name": "water"}
    values.update(kwargs)
    with pytest.raises(ValueError):
        BoxRequest(**values)
