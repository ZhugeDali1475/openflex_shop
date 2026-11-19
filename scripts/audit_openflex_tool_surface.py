#!/usr/bin/env python3
"""Audit the live MCP tool surface against the model-facing allow-list (P5.1).

Connects to the isolated delivery MCP server, lists the advertised tools and
checks three things:

1. the advertised set is exactly the model-facing allow-list plus the declared
   host-managed plumbing (so an accidental debug/coordinate tool is a failure);
2. no advertised tool accepts a free coordinate, arbitrary path or debug handle;
3. the model-facing projection drops host-owned request_id/timeout_sec, narrows
   every enum to the registered values, bounds free text and never offers the
   unvalidated right delivery table.

Read-only: it never executes a tool.
"""
import argparse
import asyncio
from datetime import datetime
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))

import httpx  # noqa: E402
from mcp import ClientSession  # noqa: E402
from mcp.client.streamable_http import streamable_http_client  # noqa: E402

from openflex_roboos_bridge.tool_surface import (  # noqa: E402
    HOST_MANAGED_TOOLS,
    MODEL_FACING_TOOLS,
    model_facing_tools,
    validate_advertised_surface,
)


async def audit(url, output):
    report = {'scope': 'live_MCP_tool_surface_audit_NOT_PHYSICAL', 'url': url,
              'physical_verification': False, 'topology': None, 'passed': False,
              'advertised': [], 'model_facing': list(MODEL_FACING_TOOLS),
              'host_managed': list(HOST_MANAGED_TOOLS), 'checks': {}, 'errors': []}
    async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
        async with streamable_http_client(url, http_client=client) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                await session.initialize()
                advertised = {tool.name: tool.inputSchema
                              for tool in (await session.list_tools()).tools}
    report['advertised'] = sorted(advertised)
    report['advertised_schemas'] = advertised
    expected = set(MODEL_FACING_TOOLS) | set(HOST_MANAGED_TOOLS)
    report['checks']['advertised_is_exactly_the_reviewed_surface'] = set(advertised) == expected
    if set(advertised) != expected:
        report['errors'].append({
            'unexpected_tools': sorted(set(advertised) - expected),
            'missing_tools': sorted(expected - set(advertised))})
    try:
        validate_advertised_surface(advertised)
        report['checks']['no_free_coordinate_or_debug_arguments'] = True
    except ValueError as error:
        report['checks']['no_free_coordinate_or_debug_arguments'] = False
        report['errors'].append({'advertised_surface': str(error)})
    try:
        surface = model_facing_tools(advertised)
        report['model_facing_schemas'] = surface
        report['checks']['projection_is_registered_and_bounded'] = True
        report['checks']['right_table_not_offered'] = 'sim_delivery_top_right' not in str(surface)
    except ValueError as error:
        report['checks']['projection_is_registered_and_bounded'] = False
        report['errors'].append({'projection': str(error)})
    report['passed'] = bool(report['checks']) and all(report['checks'].values())
    report['audited_at'] = datetime.now().astimezone().isoformat()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:18556/mcp')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'local_dependencies/openflex_thesis/multiskill_execution_20261114'
                                      '/tool_surface_audit/tool_surface.json')
    arguments = parser.parse_args()
    report = asyncio.run(audit(arguments.url, arguments.output))
    print(json.dumps({key: report[key] for key in
                      ('passed', 'checks', 'advertised', 'errors')}, ensure_ascii=False, indent=2))
    print(arguments.output)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
