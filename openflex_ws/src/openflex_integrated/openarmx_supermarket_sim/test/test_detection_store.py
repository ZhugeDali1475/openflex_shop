import pytest

from openarmx_supermarket_sim.detection_store import TruthDetectionStore


def test_truth_detection_is_single_consumer_and_arm_scoped():
    store = TruthDetectionStore(ttl_sec=5.0)
    detection_id = store.create("sku-1", "shelf-a", "right", now=10.0)

    assert detection_id == "truth:right:1"
    store.consume(detection_id, "sku-1", "shelf-a", "right", now=12.0)
    with pytest.raises(ValueError, match="already consumed"):
        store.consume(detection_id, "sku-1", "shelf-a", "right", now=12.1)


@pytest.mark.parametrize(
    ("object_id", "station_id", "arm_name", "message"),
    (
        ("sku-2", "shelf-a", "right", "object"),
        ("sku-1", "shelf-b", "right", "station"),
        ("sku-1", "shelf-a", "left", "arm"),
    ),
)
def test_truth_detection_rejects_contract_mismatch(
    object_id, station_id, arm_name, message
):
    store = TruthDetectionStore(ttl_sec=5.0)
    detection_id = store.create("sku-1", "shelf-a", "right", now=10.0)

    with pytest.raises(ValueError, match=message):
        store.validate(
            detection_id, object_id, station_id, arm_name, now=11.0
        )


def test_truth_detection_rejects_expired_or_unknown_id():
    store = TruthDetectionStore(ttl_sec=5.0)
    detection_id = store.create("sku-1", "shelf-a", "left", now=10.0)

    with pytest.raises(ValueError, match="expired"):
        store.validate(detection_id, "sku-1", "shelf-a", "left", now=15.1)
    with pytest.raises(ValueError, match="unknown"):
        store.validate("truth:left:999", "sku-1", "shelf-a", "left", now=11.0)
