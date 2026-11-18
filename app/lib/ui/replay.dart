import 'common.dart';

/// An observed sample at or immediately before a replay cursor. Interior gaps
/// and time outside the trace have no observation; never borrow a future fix.
class ReplaySelection {
  const ReplaySelection(this.index, this.sample, this.status);
  final int? index;
  final Map<String, dynamic>? sample;
  final String status;
  bool get valid => status == 'VALID SAMPLE';
}

ReplaySelection replaySelection(
  List<Map<String, dynamic>> samples,
  double time,
) {
  const absent = ReplaySelection(null, null, 'NO SAMPLE AT THIS TIME');
  const tolerance = 1e-9;
  if (samples.isEmpty || !time.isFinite) return absent;
  final first = numeric(samples.first['time_s']);
  final last = numeric(samples.last['time_s']);
  if (first == null ||
      last == null ||
      time < first - tolerance ||
      time > last + tolerance) {
    return absent;
  }
  var low = 0, high = samples.length;
  while (low < high) {
    final middle = (low + high) ~/ 2;
    final at = numeric(samples[middle]['time_s']);
    if (at != null && at <= time + tolerance) {
      low = middle + 1;
    } else {
      high = middle;
    }
  }
  final index = low - 1;
  if (index < 0) return absent;
  final sample = samples[index];
  final at = numeric(sample['time_s']);
  if (sample['valid'] != true || at == null) {
    return ReplaySelection(index, sample, 'INVALID SAMPLE');
  }
  if (time > at + tolerance) {
    if (index + 1 >= samples.length) return absent;
    final next = samples[index + 1];
    final nextTime = numeric(next['time_s']);
    if (next['valid'] != true || nextTime == null || nextTime - at > 1.5) {
      return const ReplaySelection(null, null, 'SIGNAL GAP');
    }
  }
  return ReplaySelection(index, sample, 'VALID SAMPLE');
}
