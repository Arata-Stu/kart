"""ROS-independent acceptance rules for saved-map snapshot acquisition."""


class LocalizationGate:
    def __init__(self):
        self.stamp = -1
        self.since = None
        self.localized = False
        self.last_receipt = 0.0

    def update(self, stamp, values, receipt):
        if stamp < self.stamp:
            return False
        before = self.localized
        self.stamp, self.last_receipt = stamp, receipt
        self.localized = (
            values.get("localized_in_exist_map") == "Yes"
            and values.get("vo_status") == "OK"
        )
        if not self.localized:
            self.since = None
        elif not before:
            self.since = stamp
        return before != self.localized

    def accepts(self, stamp):
        return self.localized and self.since is not None and stamp >= self.since

    def require_final(self, expected_stamp, tolerance_ns):
        if not self.localized or self.stamp < expected_stamp - tolerance_ns:
            raise ValueError(
                "保存地図へのlocalize未成功・追跡喪失・再生末尾の処理未確認"
            )
