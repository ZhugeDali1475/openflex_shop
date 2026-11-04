import pytest

from openarmx_task_executor.receipt_recognition import (
    build_receipt_prompt,
    normalize_items,
    parse_receipt_response,
    validate_order_items,
)


def test_prompt_contains_only_supplied_catalog_entries():
    prompt = build_receipt_prompt(["tea", "cola", "tea"])

    assert "- cola" in prompt
    assert prompt.count("- tea") == 1
    assert "只输出 JSON" in prompt


def test_normalize_merges_duplicates_and_rejects_invalid_quantities():
    items = normalize_items(
        [
            {"sku": "tea", "quantity": 1},
            {"sku": "tea", "quantity": "2"},
            {"sku": "cola", "quantity": True},
            {"sku": "unknown", "quantity": 3},
            {"sku": "tea", "quantity": 0},
        ],
        {"tea", "cola"},
    )

    assert items == [{"sku": "tea", "quantity": 3}]


def test_parser_accepts_fenced_or_prefixed_json():
    response = '识别结果如下：```json\n{"items":[{"sku":"tea","quantity":2}]}\n```'

    assert parse_receipt_response(response, {"tea"}) == [
        {"sku": "tea", "quantity": 2}
    ]


def test_parser_rejects_payload_without_valid_catalog_item():
    with pytest.raises(ValueError, match="unknown SKUs"):
        parse_receipt_response('{"items":[{"sku":"other","quantity":1}]}', {"tea"})


def test_parser_rejects_unknown_sku_even_when_expected_items_are_present():
    response = (
        '{"items":['
        '{"sku":"tea","quantity":1},'
        '{"sku":"cola","quantity":1},'
        '{"sku":"invented","quantity":1}'
        ']}'
    )

    with pytest.raises(ValueError, match="invented"):
        parse_receipt_response(response, {"tea", "cola"})


def test_validation_requires_expected_distinct_count_and_quantity_limit():
    with pytest.raises(ValueError, match="does not match expected"):
        validate_order_items(
            [{"sku": "tea", "quantity": 1}], expected_distinct_skus=2
        )
    with pytest.raises(ValueError, match="exceeds"):
        validate_order_items([{"sku": "tea", "quantity": 10}], max_quantity=9)
