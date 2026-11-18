import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'ui/common.dart';

typedef RequestTransport = Future<String> Function(String request);

class AeviaController extends ChangeNotifier {
  AeviaController(this.transport, {this.autoPoll = true, this.monotonicNow});
  final RequestTransport transport;
  final bool autoPoll;
  final Duration Function()? monotonicNow;
  final Stopwatch _freshnessClock = Stopwatch()..start();
  Duration? _snapshotAt;
  bool _pollFailed = false;
  Map<String, dynamic> _state = {};
  Map<String, dynamic> get state => _state;
  set state(Map<String, dynamic> value) {
    _state = value;
    _snapshotAt = _now;
    _pollFailed = false;
  }

  Duration get _now => monotonicNow?.call() ?? _freshnessClock.elapsed;
  double get _snapshotAgeMs => _snapshotAt == null
      ? double.infinity
      : (_now - _snapshotAt!).inMicroseconds.clamp(0, 1 << 62) / 1000;
  double? get dataAgeMs => numeric(live['age_ms']) == null
      ? null
      : numeric(live['age_ms'])! + _snapshotAgeMs;
  Map<String, dynamic>? session;
  Map<String, dynamic> ppkStatus = {};
  Map<String, dynamic> ppkPrepared = {};
  final Map<String, String> ppkDraft = {};
  List<Map<String, dynamic>> devices = [];
  List<Map<String, dynamic>> deviceSessions = [];
  final List<Map<String, dynamic>> recent = [];
  Timer? _timer;
  bool busy = false;
  bool _polling = false;
  bool _disposed = false;
  String? error;
  String? notice;
  String? activity;
  int page = 0;
  Map<String, dynamic> get device => object(state['device']);
  Map<String, dynamic> get live => object(state['live']);
  bool get connected => device['connected'] == true;
  bool get recording => live['recording'] == true;
  String? get captureInterruption {
    final value = live['capture_interrupted'];
    return value is String && value.isNotEmpty
        ? value
        : value == true
        ? live['capture_error']?.toString() ??
              'Recording interrupted. Stop and save the received data.'
        : null;
  }

  List<Map<String, dynamic>> get sessions => objects(state['sessions']);
  List<Map<String, dynamic>> get tracks => objects(state['tracks']);
  bool get stale =>
      _pollFailed ||
      live['stale'] == true ||
      (dataAgeMs ?? _snapshotAgeMs) >= 1500;
  bool get valid =>
      connected &&
      !stale &&
      live['valid'] == true &&
      ![
        'none',
        'invalid',
        'no_fix',
        'no fix',
        'stale',
        'unknown',
      ].contains(device['fix_quality']?.toString().toLowerCase()) &&
      numeric(live['speed_mps']) != null;
  void changed() {
    if (!_disposed) notifyListeners();
  }

  Future<dynamic> request(
    String op, [
    Map<String, dynamic> params = const {},
  ]) async {
    final decoded = object(
      jsonDecode(await transport(jsonEncode({'op': op, ...params}))),
    );
    if (decoded['ok'] != true)
      throw StateError(
        decoded['error']?.toString() ?? 'The request could not be completed.',
      );
    return decoded['data'];
  }

  Future<void> initialize() async {
    await action(() async {
      state = object(await request('state'));
      devices = objects(await request('devices'));
    });
    if (autoPoll && !_disposed)
      _timer = Timer.periodic(const Duration(milliseconds: 500), (_) {
        // A blocked native request must not freeze a previously valid value.
        changed();
        unawaited(poll());
      });
  }

  Future<void> poll() async {
    if (_polling || _disposed) return;
    _polling = true;
    try {
      state = object(await request('poll'));
      if (valid && recent.lastOrNull?['time_s'] != live['time_s']) {
        recent.add(Map<String, dynamic>.from(live));
        if (recent.length > 600) recent.removeAt(0);
      }
      changed();
    } catch (e) {
      _pollFailed = true;
      error = cleanError(e);
      changed();
    } finally {
      _polling = false;
    }
  }

  Future<void> action(
    Future<void> Function() task, {
    String? success,
    String? pending,
  }) async {
    if (busy) return;
    busy = true;
    error = null;
    notice = null;
    activity = pending;
    changed();
    try {
      await task();
      notice = success;
    } catch (e) {
      error = cleanError(e);
    } finally {
      busy = false;
      activity = null;
      changed();
    }
  }

  String cleanError(Object e) => e.toString().replaceFirst('Bad state: ', '');
  Future<void> cancelProcessing() async {
    try {
      final result = object(await request('cancel_processing'));
      if (result['cancel_requested'] == true) {
        notice =
            'Cancellation requested. Finishing the current bounded calculation.';
      }
    } catch (e) {
      error = cleanError(e);
    }
    changed();
  }

  Future<void> refreshDevices() => action(() async {
    devices = objects(await request('devices'));
  });
  Future<void> refreshDeviceSessions() => action(() async {
    deviceSessions = objects(await request('device_sessions'));
  }, pending: 'Reading the receiver recording directory…');
  Future<void> downloadDeviceSession(int id) => action(
    () async {
      session = object(await request('download', {'id': id}));
      state = object(await request('state'));
      page = 2;
    },
    pending: 'Downloading receiver recording… Keep the USB cable connected.',
    success: 'Receiver recording downloaded and added to the session library.',
  );
  Future<void> connect(String port) => action(() async {
    await request('connect', {'port': port});
    state = object(await request('state'));
    recent.clear();
    deviceSessions.clear();
  }, success: 'Receiver connected.');
  Future<void> disconnect() => action(() async {
    await request('disconnect');
    state = object(await request('state'));
    deviceSessions.clear();
  }, success: 'Receiver disconnected.');
  Future<void> start({String? name, String? trackId}) => action(() async {
    await request('start', {'name': name, 'track_id': trackId});
    state = object(await request('state'));
  }, success: 'Recording started.');
  Future<void> stop() => action(() async {
    await request('stop');
    state = object(await request('state'));
  }, success: 'Recording saved to the session library.');
  Future<void> openSession(String id) => action(() async {
    session = object(await request('session', {'id': id}));
    page = 2;
  });
  Future<void> importSession(String path) => action(() async {
    final result = object(await request('import', {'path': path}));
    state = object(await request('state'));
    if (result['id'] != null) {
      session = object(await request('session', {'id': result['id']}));
      page = 2;
    }
  }, success: 'Session imported.');
  Future<void> exportSession(String id, String path, String format) async {
    if (busy) return;
    Map<String, dynamic>? result;
    await action(() async {
      result = object(
        await request('export', {'id': id, 'path': path, 'format': format}),
      );
    });
    if (error == null && result != null) {
      final paired = result!['paired_files'];
      final warnings = result!['warnings'];
      notice =
          'Export saved to $path'
          '${paired is List && paired.isNotEmpty ? '. Keep the JSON and its ${paired.length} adjacent raw evidence file(s) together.' : ''}'
          '${warnings is List && warnings.isNotEmpty ? '\n${warnings.join('\n')}' : ''}';
      changed();
    }
  }

  Future<void> checkPpk(String? id) => action(() async {
    ppkStatus = object(await request('ppk_status', {if (id != null) 'id': id}));
  });
  Future<void> preparePpk(Map<String, dynamic> params) => action(() async {
    ppkPrepared = object(await request('prepare_ppk', params));
    for (final key in ['rover_obs', 'rover_nav']) {
      if (ppkPrepared[key] is String) ppkDraft[key] = ppkPrepared[key];
    }
  }, pending: 'Exporting raw observations and converting to RINEX…');
  Future<void> runPpk(Map<String, dynamic> params) => action(
    () async {
      final result = object(await request('run_ppk', params));
      session = object(result['session']);
      state = object(await request('state'));
      page = 2;
    },
    pending: 'Processing rover and base observations…',
    success:
        'PPK result saved as a separate session. Review its solution quality.',
  );
  Future<void> importPpk(String path, String? sourceId) => action(
    () async {
      session = object(
        await request('import_ppk', {
          'path': path,
          if (sourceId != null) 'source_id': sourceId,
        }),
      );
      state = object(await request('state'));
      page = 2;
    },
    pending: 'Importing the PPK solution…',
    success: 'PPK solution imported.',
  );
  Future<void> demo() => action(() async {
    session = object(await request('demo'));
    state = object(await request('state'));
    page = 2;
  }, success: 'Opened a synthetic demonstration session.');
  Future<void> saveTrack(Map<String, dynamic> track) => action(() async {
    await request('save_track', {'track': track});
    state = object(await request('state'));
  }, success: 'Track saved.');
  Future<void> deleteTrack(String id) => action(() async {
    await request('delete_track', {'id': id});
    state = object(await request('state'));
  }, success: 'Track deleted.');
  Future<void> setDrag(Map<String, dynamic> config) => action(() async {
    await request('set_drag', {'config': config});
    state = object(await request('state'));
  }, success: 'Drag test configuration saved.');
  Future<void> reanalyze(String id, String? trackId) => action(() async {
    session = object(
      await request('reanalyze', {
        'id': id,
        'track_id': trackId == '' ? null : trackId,
      }),
    );
    state = object(await request('state'));
  }, success: 'Session analysis updated.');
  void navigate(int value) {
    page = value;
    changed();
  }

  void dismissMessage() {
    error = null;
    notice = null;
    changed();
  }

  @override
  void dispose() {
    _disposed = true;
    _timer?.cancel();
    super.dispose();
  }
}
