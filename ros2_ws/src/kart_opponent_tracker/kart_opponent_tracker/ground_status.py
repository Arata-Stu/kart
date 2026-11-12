"""ROS-free receive/synchronization/projection diagnostics for the ground node."""
import math
import time


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def projection_issue(tracks, info):
    """Validate the existing fixed RGB/calibration contract before projection."""
    stamp = stamp_ns(tracks.header.stamp)
    if stamp <= 0 or stamp_ns(info.header.stamp) <= 0:
        return 'invalid_stamp'
    if stamp != stamp_ns(info.header.stamp):
        return 'not_synchronized'
    if (not tracks.header.frame_id or not info.header.frame_id or
            tracks.header.frame_id != info.header.frame_id):
        return 'invalid_frame'
    if ((info.width, info.height) != (640, 480) or max(info.binning_x, info.binning_y) > 1 or
            info.roi.x_offset or info.roi.y_offset or
            (info.roi.width, info.roi.height) not in ((0, 0), (640, 480))):
        return 'invalid_image_shape'
    if (info.distortion_model not in ('plumb_bob', 'rational_polynomial') or
            len(info.k) != 9 or not all(math.isfinite(value) for value in info.k) or
            min(info.k[0], info.k[4]) <= 0 or len(info.d) not in (0, 4, 5, 8, 12, 14) or
            not all(math.isfinite(value) for value in info.d)):
        return 'invalid_calibration'
    return None


class GroundStatus:
    MAX_AGE_S = 1.0
    # DiagnosticStatus numeric levels, without importing ROS in this helper.
    REASONS = {
        'waiting_tracks': (1, 'tracks未受信'),
        'tracks_stale': (1, 'tracks入力途絶'),
        'waiting_camera_info': (1, 'CameraInfo未受信'),
        'camera_info_stale': (1, 'CameraInfo入力途絶'),
        'not_synchronized': (1, 'tracksとCameraInfoの撮影stampが完全一致していません'),
        'invalid_stamp': (1, '撮影stampが無効です'),
        'invalid_frame': (1, 'tracksとCameraInfoのframeが空または不一致です'),
        'invalid_image_shape': (1, '画像サイズ・ROI・ビニングが投影条件に合いません'),
        'invalid_calibration': (1, 'CameraInfoの内部校正・歪み設定が投影条件に合いません'),
        'tf_unavailable': (1, '撮影時刻のcameraからtarget frameへのTFがありません'),
        'zero_tracks': (0, '追跡対象0件（路面投影なし）'),
        'all_rejected': (1, '全trackが路面投影で除外されました'),
        'projected': (0, '路面投影に成功しました'),
    }

    def __init__(self):
        self.tracks_received = self.info_received = self.sync_received = 0
        self.projected_points = 0
        self.tracks_at = self.info_at = self.sync_at = None
        self.tracks_stamp = self.info_stamp = self.sync_stamp = None
        self.tracks_frame = self.info_frame = ''
        self.track_count = self.projection_count = 0
        self.reason = 'waiting_tracks'
        self.detail = ''

    def tracks(self, stamp, frame, count, now=None):
        self.tracks_received += 1
        self.tracks_at = time.monotonic() if now is None else now
        self.tracks_stamp, self.tracks_frame = stamp, str(frame)[:256]
        self.track_count = count

    def camera_info(self, stamp, frame, now=None):
        self.info_received += 1
        self.info_at = time.monotonic() if now is None else now
        self.info_stamp, self.info_frame = stamp, str(frame)[:256]

    def synchronized(self, stamp, reason, projected=0, detail='', now=None):
        if reason not in self.REASONS:
            raise ValueError(f'Unknown projection diagnostic reason: {reason}')
        self.sync_received += 1
        self.sync_at = time.monotonic() if now is None else now
        self.sync_stamp = stamp
        self.reason, self.detail = reason, str(detail)[:256]
        self.projection_count = projected
        self.projected_points += projected

    def view(self, now=None):
        now = time.monotonic() if now is None else now

        def fresh(received):
            return received is not None and 0 <= now - received <= self.MAX_AGE_S

        if self.tracks_at is None:
            reason = 'waiting_tracks'
        elif not fresh(self.tracks_at):
            reason = 'tracks_stale'
        elif self.tracks_stamp <= 0:
            reason = 'invalid_stamp'
        elif not self.tracks_frame:
            reason = 'invalid_frame'
        elif self.info_at is None:
            reason = 'waiting_camera_info'
        elif not fresh(self.info_at):
            reason = 'camera_info_stale'
        elif self.info_stamp <= 0:
            reason = 'invalid_stamp'
        elif not self.info_frame or self.info_frame != self.tracks_frame:
            reason = 'invalid_frame'
        elif not fresh(self.sync_at) or self.sync_stamp != self.tracks_stamp:
            reason = 'not_synchronized'
        else:
            reason = self.reason
        level, message = self.REASONS[reason]
        values = dict(reason=reason, tracks_received=self.tracks_received,
                      camera_info_received=self.info_received, synchronized_received=self.sync_received,
                      projected_points=self.projected_points, last_tracks_count=self.track_count,
                      last_projection_count=self.projection_count,
                      tracks_stamp_ns=self.tracks_stamp, camera_info_stamp_ns=self.info_stamp,
                      synchronized_stamp_ns=self.sync_stamp,
                      tracks_frame=self.tracks_frame, camera_info_frame=self.info_frame,
                      tracks_age_s=None if self.tracks_at is None else round(now-self.tracks_at, 3),
                      camera_info_age_s=None if self.info_at is None else round(now-self.info_at, 3),
                      synchronized_age_s=None if self.sync_at is None else round(now-self.sync_at, 3),
                      detail=self.detail if reason == self.reason else '')
        return dict(level=level, message=message,
                    values={key: 'unavailable' if value is None else str(value) for key, value in values.items()})
