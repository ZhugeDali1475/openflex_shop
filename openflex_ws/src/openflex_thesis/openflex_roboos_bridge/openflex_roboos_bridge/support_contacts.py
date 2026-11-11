def allow_wheel_floor_support(entry_names, enabled_rows, floor_name):
    names = list(entry_names)
    rows = [list(row) for row in enabled_rows]
    if len(set(names)) != len(names) or len(rows) != len(names) or any(len(row) != len(names) for row in rows):
        raise ValueError('Malformed allowed-collision matrix')
    wheels = ['fl_wheel_link', 'fr_wheel_link', 'bl_wheel_link', 'br_wheel_link']
    for name in [floor_name, *wheels]:
        if name not in names:
            names.append(name)
            for row in rows:
                row.append(False)
            rows.append([False] * len(names))
    floor_index = names.index(floor_name)
    for index, name in enumerate(names):
        rows[index][floor_index] = rows[floor_index][index] = name in wheels
    return names, rows
