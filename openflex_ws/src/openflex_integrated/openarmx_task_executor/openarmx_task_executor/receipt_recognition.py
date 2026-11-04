"""Pure shopping-ticket prompt and response validation helpers."""

from __future__ import annotations

import json
from typing import Any, Iterable


def build_receipt_prompt(allowed_skus: Iterable[str]) -> str:
    skus = sorted({str(value).strip() for value in allowed_skus if str(value).strip()})
    if not skus:
        raise ValueError("SKU catalog is empty")
    sku_list = "\n".join(f"- {sku}" for sku in skus)
    return (
        "图 1 是一张中文购物小票。读取表格中的商品名称、数量和规格，并映射为下列唯一允许的 SKU。\n"
        "小票商品名可能省略品牌或口味；结合商品名称和规格选择最匹配的一个 SKU。"
        "只能使用清单中的完全一致 SKU 名称；无法可靠映射的条目直接忽略，不要杜撰 SKU。"
        "相同 SKU 出现多次时合并数量。数量必须是正整数。\n\n"
        "允许的 SKU：\n"
        + sku_list
        + "\n\n"
        '只输出 JSON：{"items":[{"sku":"清单中的 SKU 名称","quantity":1}]}。'
        "不要输出 Markdown、解释、价格、日期或小票号。"
    )


def normalize_items(items: Any, allowed_skus: set[str]) -> list[dict[str, Any]]:
    """Keep exact catalog SKUs with positive integer quantities."""
    if not isinstance(items, list):
        return []
    quantities: dict[str, int] = {}
    for item in items:
        if not isinstance(item, dict):
            continue
        sku = item.get("sku")
        quantity = item.get("quantity")
        if isinstance(quantity, str) and quantity.strip().isdecimal():
            quantity = int(quantity.strip())
        if (
            not isinstance(sku, str)
            or sku not in allowed_skus
            or isinstance(quantity, bool)
            or not isinstance(quantity, int)
            or quantity <= 0
        ):
            continue
        quantities[sku] = quantities.get(sku, 0) + quantity
    return [
        {"sku": sku, "quantity": quantity}
        for sku, quantity in quantities.items()
    ]


def parse_receipt_response(
    response: str,
    allowed_skus: set[str],
) -> list[dict[str, Any]]:
    value = str(response).strip()
    if value.startswith("```") and value.endswith("```"):
        value = value.split("\n", 1)[-1].rsplit("```", 1)[0].strip()
    decoder = json.JSONDecoder()
    for index, character in enumerate(value):
        if character not in "{[":
            continue
        try:
            payload, _ = decoder.raw_decode(value[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict):
            raw_items = payload.get("items")
        elif isinstance(payload, list):
            raw_items = payload
        else:
            continue
        if isinstance(raw_items, list):
            unknown_skus = sorted({
                str(item.get("sku")).strip()
                for item in raw_items
                if isinstance(item, dict)
                and isinstance(item.get("sku"), str)
                and str(item.get("sku")).strip()
                and str(item.get("sku")).strip() not in allowed_skus
            })
            if unknown_skus:
                raise ValueError(
                    f"Qwen response contains unknown SKUs: {unknown_skus}"
                )
        items = normalize_items(raw_items, allowed_skus)
        if items:
            return items
    raise ValueError("Qwen response does not contain a valid receipt payload")


def validate_order_items(
    items: list[dict[str, Any]],
    *,
    expected_distinct_skus: int = 0,
    max_quantity: int = 9,
) -> list[dict[str, Any]]:
    if not items:
        raise ValueError("shopping ticket contains no recognized items")
    if expected_distinct_skus > 0 and len(items) != expected_distinct_skus:
        raise ValueError(
            "shopping ticket distinct SKU count "
            f"{len(items)} does not match expected {expected_distinct_skus}"
        )
    for item in items:
        quantity = int(item["quantity"])
        if quantity > max_quantity:
            raise ValueError(
                f"shopping-ticket quantity for {item['sku']} exceeds {max_quantity}"
            )
    return items
