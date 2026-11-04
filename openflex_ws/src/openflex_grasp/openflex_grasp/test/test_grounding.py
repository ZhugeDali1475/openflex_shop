from openflex_grasp.grounding import parse_detection, parse_detections


def test_parses_normalized_json_box():
    detection = parse_detection(
        '{"boxes":[{"label":"bottle","bbox_2d":[250,100,750,900],"confidence":0.8}]}',
        "water bottle",
        640,
        480,
    )
    assert detection.box_xyxy == (160, 48, 480, 432)
    assert detection.label == "bottle"
    assert detection.confidence == 0.8


def test_strips_thinking_and_fenced_json():
    response = '<think>hidden</think>\n```json\n{"bbox":[100,200,900,800]}\n```'
    detection = parse_detection(response, "box", 1000, 500)
    assert detection.box_xyxy == (100, 100, 900, 400)


def test_parses_all_instances_and_removes_duplicate_boxes():
    response = """{
      "boxes": [
        {"label":"bottle","bbox_2d":[100,100,300,900],"confidence":0.9},
        {"label":"bottle","bbox_2d":[105,105,305,895],"confidence":0.7},
        {"label":"bottle","bbox_2d":[600,120,800,880],"confidence":0.8}
      ]
    }"""
    detections = parse_detections(response, "water bottle", 1000, 500)
    assert len(detections) == 2
    assert detections[0].box_xyxy == (100, 50, 300, 450)
    assert detections[1].box_xyxy == (600, 60, 800, 440)
