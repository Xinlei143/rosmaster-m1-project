"""Analyze raw Gazebo GPU-LiDAR JSON echo output.

This module intentionally does not alter scan values.  It only classifies the
values emitted by ``ign topic --json-output`` so the Gazebo-to-ROS boundary can
be located with the same finite/non-finite definitions on both sides.
"""

import argparse
import json
import math
import statistics
import sys


def _coerce_range(value):
    if isinstance(value, str):
        token = value.strip().lower()
        value = {
            "infinity": float("inf"),
            "+infinity": float("inf"),
            "inf": float("inf"),
            "+inf": float("inf"),
            "-infinity": float("-inf"),
            "-inf": float("-inf"),
            "nan": float("nan"),
        }.get(token, value)
    return float(value)


def scan_range_stats(ranges, range_max):
    """Return counts and ratios for one raw scan frame."""
    values = [_coerce_range(value) for value in ranges]
    finite = [value for value in values if math.isfinite(value)]
    positive_inf = sum(
        1 for value in values if math.isinf(value) and value > 0.0)
    negative_inf = sum(
        1 for value in values if math.isinf(value) and value < 0.0)
    nan = sum(1 for value in values if math.isnan(value))
    total = len(values)
    denominator = float(total) if total else 1.0
    return {
        "beam_count": total,
        "finite_count": len(finite),
        "positive_inf_count": positive_inf,
        "negative_inf_count": negative_inf,
        "nan_count": nan,
        "finite_ratio": len(finite) / denominator,
        "positive_inf_ratio": positive_inf / denominator,
        "negative_inf_ratio": negative_inf / denominator,
        "nan_ratio": nan / denominator,
        "min_finite_range": min(finite) if finite else None,
        "max_finite_range": max(finite) if finite else None,
        "range_max": float(range_max),
    }


def iter_json_documents(text):
    """Yield JSON objects from noisy or pretty-printed transport output."""
    decoder = json.JSONDecoder()
    position = 0
    length = len(text)
    while position < length:
        starts = [index for index in (
            text.find("{", position), text.find("[", position))
                  if index >= 0]
        if not starts:
            return
        position = min(starts)
        try:
            document, end = decoder.raw_decode(text, position)
        except json.JSONDecodeError:
            position += 1
            continue
        yield document
        position = end


def _ranges_from_document(document):
    if isinstance(document, dict):
        ranges = document.get("ranges")
        if isinstance(ranges, list):
            return ranges, document.get(
                "range_max", document.get("rangeMax", 12.0))
        for value in document.values():
            found = _ranges_from_document(value)
            if found is not None:
                return found
    return None


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _expected_beam_count(frame_stats, expected_beam_count):
    """Choose the caller's beam contract or the observed modal beam count."""
    if expected_beam_count is not None:
        return int(expected_beam_count)
    counts = [frame["beam_count"] for frame in frame_stats]
    if not counts:
        return None
    return max(set(counts), key=counts.count)


def is_all_negative_frame(frame, expected_beam_count):
    """Return true only for an expected-width frame made entirely of -Inf."""
    return (
        expected_beam_count is not None
        and frame["beam_count"] == int(expected_beam_count)
        and frame["negative_inf_count"] == int(expected_beam_count)
    )


def summarize_transitions(frame_stats, stamp_ns, expected_beam_count=None):
    """Describe whole-frame negative-infinity transitions without clamping data."""
    frames = list(frame_stats)
    expected = _expected_beam_count(frames, expected_beam_count)
    states = [
        "all_negative" if is_all_negative_frame(frame, expected) else "other"
        for frame in frames
    ]
    all_negative_indices = [
        index for index, state in enumerate(states)
        if state == "all_negative"
    ]
    good_to_bad = sum(
        previous != "all_negative" and current == "all_negative"
        for previous, current in zip(states, states[1:]))
    if states and states[0] == "all_negative":
        good_to_bad += 1
    bad_to_good = sum(
        previous == "all_negative" and current != "all_negative"
        for previous, current in zip(states, states[1:]))

    streaks = []
    start = None
    for index, state in enumerate(states + ["other"]):
        if state == "all_negative" and start is None:
            start = index
        elif state != "all_negative" and start is not None:
            streaks.append((start, index - 1))
            start = None

    stamps = list(stamp_ns)

    def elapsed_seconds(first, last):
        if first == last:
            return 0.0
        if first >= len(stamps) or last >= len(stamps):
            return None
        first_stamp = stamps[first]
        last_stamp = stamps[last]
        if first_stamp is None or last_stamp is None:
            return None
        return (int(last_stamp) - int(first_stamp)) / 1e9

    first_time = None
    if all_negative_indices:
        first_time = elapsed_seconds(0, all_negative_indices[0])
    longest = max(streaks, key=lambda interval: interval[1] - interval[0]) if streaks else None
    return {
        "expected_beam_count": expected,
        "all_negative_frame_count": len(all_negative_indices),
        "time_to_first_all_negative_s": first_time,
        "good_to_bad_transition_count": good_to_bad,
        "bad_to_good_recovery_count": bad_to_good,
        "longest_continuous_bad_frames": (
            longest[1] - longest[0] + 1 if longest else 0),
        "longest_continuous_bad_seconds": (
            elapsed_seconds(*longest) if longest else None),
        "terminal_frame_state": states[-1] if states else None,
    }


def _stamp_ns(document):
    if not isinstance(document, dict):
        return None
    header = document.get("header")
    stamp = header.get("stamp") if isinstance(header, dict) else None
    if not isinstance(stamp, dict):
        return None
    try:
        return int(stamp.get("sec", 0)) * 1_000_000_000 + int(
            stamp.get("nsec", stamp.get("nanosec", 0)))
    except (TypeError, ValueError):
        return None


def frame_stats_from_document(document):
    """Return raw range statistics and its simulation timestamp, if present."""
    found = _ranges_from_document(document)
    if found is None:
        return None
    ranges, range_max = found
    return scan_range_stats(ranges, range_max), _stamp_ns(document)


def summarize_frames(frame_stats, stamp_ns=None):
    """Summarize raw frames without hiding missing or malformed frames."""
    frames = list(frame_stats)
    if not frames:
        summary = {
            "frame_count": 0,
            "beam_count_min": None,
            "beam_count_max": None,
            "finite_ratio_mean": None,
            "positive_inf_ratio_mean": None,
            "positive_inf_ratio_max": None,
            "negative_inf_ratio_mean": None,
            "negative_inf_ratio_max": None,
            "nan_ratio_mean": None,
        }
        summary.update(_stamp_summary(stamp_ns or []))
        summary.update(summarize_transitions([], stamp_ns or []))
        return summary

    def mean(key):
        return statistics.fmean(frame[key] for frame in frames)

    summary = {
        "frame_count": len(frames),
        "beam_count_min": min(frame["beam_count"] for frame in frames),
        "beam_count_max": max(frame["beam_count"] for frame in frames),
        "finite_ratio_mean": mean("finite_ratio"),
        "positive_inf_ratio_mean": mean("positive_inf_ratio"),
        "positive_inf_ratio_max": max(
            frame["positive_inf_ratio"] for frame in frames),
        "negative_inf_ratio_mean": mean("negative_inf_ratio"),
        "negative_inf_ratio_max": max(
            frame["negative_inf_ratio"] for frame in frames),
        "nan_ratio_mean": mean("nan_ratio"),
    }
    summary.update(_stamp_summary(stamp_ns or []))
    summary.update(summarize_transitions(frames, stamp_ns or []))
    return summary


def _stamp_summary(stamp_ns):
    stamps = [int(stamp) for stamp in stamp_ns if stamp is not None]
    gaps = [
        (later - earlier) / 1e9
        for earlier, later in zip(stamps, stamps[1:])
        if later >= earlier
    ]
    rate = 0.0
    if len(stamps) >= 2 and stamps[-1] > stamps[0]:
        rate = (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
    return {
        "stamp_count": len(stamps),
        "stamp_gap_p50": _percentile(gaps, 50.0),
        "stamp_gap_p95": _percentile(gaps, 95.0),
        "stamp_gap_max": max(gaps) if gaps else None,
        "stamp_rate_hz": rate,
    }


def analyze_text(text):
    frame_stats = []
    stamp_ns = []
    malformed_documents = 0
    for document in iter_json_documents(text):
        try:
            found = frame_stats_from_document(document)
            if found is None:
                continue
            frame, stamp = found
            frame_stats.append(frame)
            stamp_ns.append(stamp)
        except (TypeError, ValueError):
            malformed_documents += 1
    summary = summarize_frames(frame_stats, stamp_ns)
    summary["malformed_scan_count"] = malformed_documents
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Summarize finite/+Inf/-Inf/NaN in raw Gazebo scan JSON.")
    parser.add_argument("--input", default="-", help="JSON echo file or - for stdin")
    parser.add_argument("--output", default="-", help="summary JSON file or - for stdout")
    args = parser.parse_args(argv)
    if args.input == "-":
        text = sys.stdin.read()
    else:
        with open(args.input, "r", encoding="utf-8") as stream:
            text = stream.read()
    encoded = json.dumps(analyze_text(text), indent=2, sort_keys=True) + "\n"
    if args.output == "-":
        sys.stdout.write(encoded)
    else:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(encoded)


if __name__ == "__main__":
    main()
