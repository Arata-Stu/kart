"""Compare recorded and EVS sensor clock progress without correcting either."""


def progress(records, topic):
    segments, series = [], []
    current = None
    anchor = None
    for row in records:
        if row['topic'] != topic or row.get('error'):
            continue
        timing = row['payload'].get('json', {})
        identity = timing.get('stream_id'), timing.get('sensor_epoch')
        sensor = timing.get('window_end_sensor_us')
        offset = timing.get('sensor_to_ros_offset_ns')
        if identity[0] is None or identity[1] is None or type(sensor) is not int or type(offset) is not int:
            continue
        if current is None or identity != current['identity']:
            current = dict(identity=identity, count=0, first_bag_ns=row['bag_ns'],
                           first_sensor_us=sensor, first_offset_ns=offset)
            segments.append(current)
            anchor = row['bag_ns'], sensor, offset
        recorded_s = (row['bag_ns'] - anchor[0]) / 1e9
        sensor_s = (sensor - anchor[1]) / 1e6
        deficit_ms = (recorded_s - sensor_s) * 1000
        change_ms = (offset - anchor[2]) / 1e6
        current.update(count=current['count'] + 1, last_bag_ns=row['bag_ns'],
                       last_sensor_us=sensor, last_offset_ns=offset,
                       recorded_elapsed_s=recorded_s, sensor_elapsed_s=sensor_s,
                       recorded_minus_sensor_elapsed_ms=deficit_ms,
                       offset_change_ms=change_ms)
        series.append(dict(bag_ns=row['bag_ns'], segment=len(segments),
                           recorded_minus_sensor_elapsed_ms=deficit_ms,
                           offset_change_ms=change_ms))
    return segments, series
