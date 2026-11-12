import uuid

from unique_identifier_msgs.msg import UUID


TERMINAL_STATUSES = frozenset({4, 5, 6})


class GoalAcceptanceUnknown(RuntimeError):
    pass


def goal_id_text(handle):
    return bytes(handle.goal_id.uuid).hex()


def record_terminal(store, request_id, handle, response):
    status = int(response.status)
    result = response.result
    data = {'goal_uuid': goal_id_text(handle), 'status': status,
            'terminal_confirmed': status in TERMINAL_STATUSES}
    if hasattr(result, 'error_code'):
        data['error_code'] = int(result.error_code)
        data['error_string'] = str(result.error_string)
    store.event(request_id, 'controller_result', data)
    if status not in TERMINAL_STATUSES:
        raise RuntimeError(f'Controller terminal status unknown: {status}')


def send_audited_goal(node, request_id, client, goal, endpoint, phase):
    identifier = UUID(uuid=list(uuid.uuid4().bytes))
    node.store.event(request_id, 'controller_goal_intent', {
        'goal_uuid': bytes(identifier.uuid).hex(), 'endpoint': endpoint, 'phase': phase})
    try:
        sent = client.send_goal_async(goal, goal_uuid=identifier)
    except Exception as error:
        raise GoalAcceptanceUnknown('Unknown goal acceptance; send failed after durable intent') from error

    def acceptance(handle, late=False):
        if handle is None or goal_id_text(handle) != bytes(identifier.uuid).hex():
            raise RuntimeError('Missing or mismatched controller goal acceptance')
        node.store.event(request_id, 'controller_goal_accepted', {
            'goal_uuid': goal_id_text(handle), 'accepted': bool(handle.accepted), 'late': late})

    def late_cancel(done):
        handle = done.result()
        try:
            acceptance(handle, late=True)
        finally:
            if handle is not None and handle.accepted:
                result = handle.get_result_async()
                result.add_done_callback(lambda finished: record_terminal(
                    node.store, request_id, handle, finished.result()))
                handle.cancel_goal_async()

    try:
        handle = node.await_future(sent, 3)
    except Exception as error:
        sent.add_done_callback(late_cancel)
        raise GoalAcceptanceUnknown('Unknown goal acceptance; manual stop audit required') from error
    try:
        acceptance(handle)
    except Exception as error:
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
        raise GoalAcceptanceUnknown('Unknown goal acceptance; durable acknowledgement unavailable') from error
    if not handle.accepted:
        raise RuntimeError('Controller goal rejected')
    return handle


def cancel_to_hold_and_audit(node, request_id, handles, hold_mode):
    errors = []
    for handle, future in handles:
        try:
            handle.cancel_goal_async()
        except Exception as error:
            errors.append(f'cancel {goal_id_text(handle)}: {error}')
    try:
        node.set_mode(hold_mode)
    except Exception as error:
        errors.append(f'HOLD: {error}')
    for handle, future in handles:
        try:
            record_terminal(node.store, request_id, handle, node.await_future(future, 5))
        except Exception as error:
            errors.append(f'terminal {goal_id_text(handle)}: {error}')
    if errors:
        raise RuntimeError('; '.join(errors))


def goal_receipt_summary(record):
    goals = {}
    invalid = False
    for event in record.get('events', []):
        kind, data = event.get('kind'), event.get('data', {})
        if kind not in ('controller_goal_intent', 'controller_goal_accepted', 'controller_result'):
            continue
        identifier = data.get('goal_uuid')
        if not isinstance(identifier, str) or len(identifier) != 32:
            invalid = True
            continue
        try:
            bytes.fromhex(identifier)
        except ValueError:
            invalid = True
            continue
        if kind == 'controller_goal_intent':
            if identifier in goals:
                invalid = True
                continue
            goals[identifier] = {'endpoint': data.get('endpoint'), 'phase': data.get('phase'),
                                 'accepted': None, 'terminal_status': None}
        elif identifier not in goals:
            invalid = True
        elif kind == 'controller_goal_accepted':
            accepted = data.get('accepted')
            if type(accepted) is not bool or goals[identifier]['accepted'] is not None:
                invalid = True
            else:
                goals[identifier]['accepted'] = accepted
        else:
            status = data.get('status')
            if type(status) is not int or status not in TERMINAL_STATUSES:
                invalid = True
            elif goals[identifier]['accepted'] is not True:
                invalid = True
            elif goals[identifier]['terminal_status'] not in (None, status):
                invalid = True
            else:
                goals[identifier]['terminal_status'] = status
    unresolved = [identifier for identifier, goal in goals.items()
                  if goal['accepted'] is not False and goal['terminal_status'] not in TERMINAL_STATUSES]
    return {'goals': goals, 'unresolved_goal_uuids': unresolved, 'invalid_receipts': invalid,
            'all_goal_terminals_audited': bool(goals) and not invalid and not unresolved,
            'scope': 'Action receipts only; not proof of measured HOLD or permission to unlock'}
