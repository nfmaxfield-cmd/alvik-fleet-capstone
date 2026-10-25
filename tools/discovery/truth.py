"""The hidden answer key: what each Alvik RAW column really is. Used only by disc_score.py, never by the discoverer.

Nominal types follow docs/data-formats.md and firmware/home_lab/rawlog.h. "derived" = worked out on the robot from
other streams (get_pose, get_drive_speed, the roll and pitch of get_orientation). yaw_deg is labelled gyro angle
(the integral of the yaw rate, wrapped to 0-360): it is computed too, but it is the gyro's own angle output, the
type the task names. dC_cm is labelled range although the replay found it is exactly the mean of dCL_cm and
dCR_cm (reported as a finding, not scored as derived).

Scoring rule (stated in the report): a stream that never changes in a case can only be called "constant" from
behaviour, so its truth for that case is "constant" (the switches read -1, "not configured", in every session on
disk; charging and lifted stayed 0)."""

NOMINAL = {
    "run": "id", "ms": "clock", "step": "mode", "raw_n": "counter",
    "wl_deg": "wheel_angle", "wr_deg": "wheel_angle", "wl_rpm": "wheel_speed", "wr_rpm": "wheel_speed",
    "v_cms": "derived", "w_dps": "derived", "x_cm": "derived", "y_cm": "derived", "th_deg": "derived",
    "roll_deg": "derived", "pitch_deg": "derived", "yaw_deg": "gyro_angle",
    "ax": "accel", "ay": "accel", "az": "accel", "gx": "gyro_rate", "gy": "gyro_rate", "gz": "gyro_rate",
    "dL_cm": "range", "dCL_cm": "range", "dC_cm": "range", "dCR_cm": "range", "dR_cm": "range",
    "dtop_cm": "range", "dbot_cm": "range", "lineL": "line", "lineC": "line", "lineR": "line",
    "battery": "battery", "charging": "switch", "lifted": "switch", "swL": "switch", "swR": "switch",
}
COMPUTED = {"v_cms", "w_dps", "x_cm", "y_cm", "th_deg", "roll_deg", "pitch_deg"}


def truth_for(column, is_constant):
    if is_constant:
        return "constant"
    return NOMINAL.get(column, "unknown")
