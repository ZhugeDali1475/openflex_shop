import asyncio
import json

from jsonschema import validate


def check_stop(output):
    if (output / 'STOP').exists():
        raise asyncio.CancelledError('Operator requested a safe stop; no new tool may execute')


def validate_selection(tools, name, arguments):
    schema = next((tool['function']['parameters'] for tool in tools if tool['function']['name'] == name), None)
    if schema is None:
        raise ValueError('Unregistered model-selected skill')
    validate(arguments, schema)


async def cancel_and_audit(client, request_id, output, attempts=40, interval=0.25):
    def save(name, value):
        (output / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

    audit = {'request_id': request_id, 'stopped_confirmed': False}
    try:
        response = await client.post('http://127.0.0.1:18553/cancel', json={'request_id': request_id})
        response.raise_for_status()
        save('cancel_on_exit.json', response.json())
        for attempt in range(attempts):
            response = await client.post('http://127.0.0.1:18553/status', json={'request_id': request_id})
            response.raise_for_status()
            observed = response.json()
            save('exit_operation_audit.json', observed)
            audit['operation'] = observed
            if observed.get('status') in ('succeeded', 'failed', 'canceled', 'timed_out'):
                audit['stopped_confirmed'] = observed.get('stopped_confirmed') is True
                break
            await asyncio.sleep(interval)
    except Exception as error:
        audit['error'] = f'{type(error).__name__}: {error}'
    save('exit_cleanup_result.json', audit)
    return audit
