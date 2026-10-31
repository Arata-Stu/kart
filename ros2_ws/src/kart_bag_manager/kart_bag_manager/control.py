"""ROS-free shared recording commands and UI status representation."""


def set_recording(recorder, enabled, label=''):
    if enabled:
        return recorder.start(label)
    recorder.stop()
    return True  # Accepted, including an already stopped/stopping recorder.


def snapshot(recorder, settings, raw_relay=None):
    return dict(
        schema='kart.recording.v1', phase=recorder.phase,
        recording=recorder.recording, current_uri=recorder.current_uri,
        last_event=recorder.last_event,
        writing_unverified=bool(settings.start_paused or settings.snapshot_mode),
        raw=(dict(state=raw_relay.state, last_event=raw_relay.last_event,
                  last_error=raw_relay.last_error, owned_writer=raw_relay.owns_raw,
                  pending=bool(raw_relay.pending)) if raw_relay is not None else None),
    )
