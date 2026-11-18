"""Read-only USB/SD evidence audit. Usage: compare_captures.py host.json [sd.aev]."""
import argparse
import collections
import json
import pathlib
import statistics
import struct
import zlib


def sd_records(path):
    pending = bytearray()
    records = []
    kinds = collections.Counter()
    closed = False
    previous = None
    identity = None
    with open(path, "rb") as source:
        index = 0
        while page := source.read(512):
            assert len(page) == 512 and not closed, "partial/trailing page"
            assert page[:8] == b"AEVLOG1\0", "page magic"
            page_identity = (struct.unpack_from("<I", page, 8)[0], struct.unpack_from("<Q", page, 20)[0])
            identity = page_identity if identity is None else identity
            assert page_identity == identity, "page session/nonce continuity"
            assert struct.unpack_from("<I", page, 12)[0] == index, "page order"
            assert zlib.crc32(page[:508]) == struct.unpack_from("<I", page, 508)[0], "page CRC"
            length, flags = struct.unpack_from("<HH", page, 16)
            assert length <= 480 and flags in (0, 1) and not any(page[28+length:508])
            closed = bool(flags)
            pending.extend(page[28:28+length])
            while len(pending) >= 24:
                kind, version, length, seq, at_ns, reserved = struct.unpack_from("<BBHQQI", pending)
                assert version == 1 and length <= 256 and reserved == 0
                if len(pending) < 28 + length:
                    break
                assert zlib.crc32(pending[:24+length]) == struct.unpack_from("<I", pending, 24+length)[0], "record CRC"
                assert previous is None or seq == previous + 1, "record sequence"
                previous = seq
                payload = bytes(pending[24:24+length])
                del pending[:28+length]
                kinds[kind] += 1
                if kind == 23:
                    assert length == 101
                    captured, interval, effective, sigma = struct.unpack_from("<QIQQ", payload)
                    vectors = struct.unpack_from("<9d", payload, 28)
                    assert at_ns == captured and payload[100] in (0, 1)
                    records.append(dict(kind="imu", captured_ns=captured, interval_ns=interval,
                        effective_end_ns=effective, timing_uncertainty_ns=sigma, t_us=captured//1000,
                        dt_us=interval//1000, accel=list(vectors[:3]), gyro=list(vectors[3:6]),
                        aligned_gyro=list(vectors[6:]), valid=bool(payload[100])))
                elif kind == 24:
                    assert length == 115
                    arrival, week, tow = struct.unpack_from("<QII", payload)
                    record = dict(kind="gnss", t_us=arrival, gps_week=week, gps_tow_ms=tow)
                    record.update(zip(("lat", "lon", "alt_m", "speed_mps", "heading_deg", "vertical_mps", "hacc_m", "vacc_m", "sacc_mps"), struct.unpack_from("<9d", payload, 16)))
                    latency, satellites, valid, mapped, epoch, sigma, generation = struct.unpack_from("<IBBBQQI", payload, 88)
                    assert valid in (0, 1) and mapped in (0, 1) and at_ns == arrival*1000
                    assert mapped or (epoch == sigma == generation == 0)
                    record.update(latency_ms=latency, satellites=satellites, valid=bool(valid),
                        mapped_ns=epoch if mapped else None, mapping_uncertainty_ns=sigma if mapped else None,
                        mapping_generation=generation if mapped else None)
                    records.append(record)
                elif kind == 22:
                    assert length == 4
                    records.append(dict(kind="health", dropped=struct.unpack("<I", payload)[0], t_us=at_ns//1000))
            index += 1
    assert not pending, "incomplete final record"
    return records, dict(pages=index, closed=closed, record_kinds=dict(kinds), session_nonce=identity)


def key(record):
    return record["kind"], record.get("captured_ns", record.get("t_us"))


def stats(records):
    out = {}
    for kind in ("gnss", "imu"):
        values = [v for v in records if v.get("kind") == kind]
        stamps = [v["t_us"] for v in values]
        deltas = [b-a for a,b in zip(stamps, stamps[1:])]
        out[kind] = dict(count=len(values), valid=sum(v.get("valid") is True for v in values),
            first_us=min(stamps, default=None), last_us=max(stamps, default=None),
            duplicate_keys=len(values)-len(set(map(key, values))), reversed=sum(d<0 for d in deltas),
            delta_us=dict(min=min(deltas), median=statistics.median(deltas), max=max(deltas)) if deltas else None)
        if kind == "imu":
            out[kind].update(interval_sum_s=sum(v.get("interval_ns",0) for v in values)/1e9,
                gaps_over_10ms=sum(d>10000 for d in deltas), sub_microsecond_timestamps=sum(v.get("captured_ns",0)%1000 != 0 for v in values))
            timed = [v for v in values if 'captured_ns' in v and 'interval_ns' in v]
            intervals = [v['interval_ns'] for v in timed]
            out[kind].update(support_discontinuities=sum(b['captured_ns']-b['interval_ns'] != a['captured_ns'] for a,b in zip(timed,timed[1:])),
                native_interval_ns=dict(min=min(intervals),max=max(intervals)) if intervals else None)
        else:
            epochs = [v['gps_week']*604800000+v['gps_tow_ms'] for v in values if 'gps_week' in v and 'gps_tow_ms' in v]
            epoch_steps = [b-a for a,b in zip(epochs,epochs[1:])]
            out[kind]['receiver_epoch_step_ms'] = dict(collections.Counter(epoch_steps))
    return out


def differences(a, b, field=""):
    if isinstance(a, list) and isinstance(b, list):
        return [v for i,(x,y) in enumerate(zip(a,b)) for v in differences(x,y,f"{field}[{i}]")]
    if isinstance(a, float) and isinstance(b, float):
        if struct.pack("<d", a) == struct.pack("<d", b):
            return []
    elif a == b:
        return []
    return [dict(field=field, usb=a, sd=b)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", type=pathlib.Path)
    parser.add_argument("sd", nargs="?", type=pathlib.Path)
    args = parser.parse_args()
    host = json.loads(args.host.read_text(encoding="utf-8-sig"))
    raw = host["raw_records"]
    diagnostics = [r for r in raw if r.get("kind") not in ("gnss", "imu")]
    sidecar = next((r for r in diagnostics if r.get("kind") == "raw_capture_file"), None)
    evidence_container = None
    if sidecar:
        evidence = pathlib.Path(sidecar["path"])
        if not evidence.is_absolute():
            evidence = args.host.parent / evidence
        raw, evidence_container = sd_records(evidence)
    out = dict(host=str(args.host), bytes=args.host.stat().st_size, host_stats=stats(raw),
        projected_samples=len(host["samples"]), projected_valid=sum(s["valid"] for s in host["samples"]),
        analysis=host["analysis"], diagnostics=diagnostics, evidence_container=evidence_container)
    if args.sd:
        sd, container = sd_records(args.sd)
        out.update(sd=str(args.sd), sd_container=container, sd_stats=stats(sd))
        a = {key(r):r for r in raw if r.get("kind") in ("gnss","imu")}
        b = {key(r):r for r in sd if r.get("kind") in ("gnss","imu")}
        missing_host, missing_sd = b.keys()-a.keys(), a.keys()-b.keys()
        errors = []
        error_count = 0
        fields = collections.Counter()
        for k in sorted(a.keys() & b.keys()):
            for name in b[k]:
                ds = differences(a[k].get(name), b[k][name], name)
                error_count += len(ds)
                fields.update(d["field"] for d in ds)
                errors.extend(dict(key=k, **d) for d in ds[:max(0,10-len(errors))])
        out["comparison"] = dict(common=len(a.keys() & b.keys()), missing_from_host=len(missing_host),
            missing_from_sd=len(missing_sd), missing_host_examples=sorted(missing_host)[:10],
            missing_sd_examples=sorted(missing_sd)[:10], differing_fields=error_count,
            differences_by_field=dict(fields), difference_examples=errors)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
