import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'ui/common.dart';

typedef RequestTransport = Future<String> Function(String request);
typedef GTrailPoint = ({int tUs, double x, double y});

Map<String, dynamic> _decodeReply(String text) => object(jsonDecode(text));

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
    final next = object(value['device']);
    final nextTime = numeric(object(value['live'])['time_s']);
    final previousTime = numeric(live['time_s']);
    if (!receiverFresh ||
        next['port'] != device['port'] ||
        next['transport'] != device['transport'] ||
        (nextTime != null && previousTime != null && nextTime < previousTime)) {
      _gTrail = const [];
      _batteryDrops.clear();
      _previousPercent = null;
    }
    final sd = object(next['sd']);
    final card = sd.isEmpty
        ? null
        : (
            sd['capacity_bytes'],
            sd['file_id'],
            sd['closed'],
            sd['written_pages'],
          );
    if (next['connected'] != true ||
        next['transport'] != 'USB' ||
        next['port'] != device['port'] ||
        sd['state'] == 'fault' ||
        (card != null && _lastCard != null && card != _lastCard) ||
        (device['recording'] == true && next['recording'] == false)) {
      _clearDirectory();
    }
    if (card != null) _lastCard = card;
    _state = value;
    _snapshotAt = _now;
    _pollFailed = false;
    _updateBatteryTrend();
    _updateGTrail();
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
  List<Map<String, dynamic>>? deviceSessions;
  Map<String, dynamic>? boardPreview;
  Record? _lastCard;
  final _previews = <int, Map<String, dynamic>>{};
  int _directoryGeneration = 0;
  bool readingDirectory = false;
  int? readingPreview;
  final List<Map<String, dynamic>> _recent = [];
  List<Map<String, dynamic>>? _recentSnapshot;
  List<Map<String, dynamic>> get recent =>
      _recentSnapshot ??= List.unmodifiable(_recent);
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
  bool get bluetooth => device['transport'] == 'Bluetooth';
  bool get bluetoothAvailable =>
      !kIsWeb &&
      (defaultTargetPlatform == TargetPlatform.windows ||
          defaultTargetPlatform == TargetPlatform.iOS);
  bool get usbAvailable => defaultTargetPlatform != TargetPlatform.iOS;
  bool get connected => device['connected'] == true;
  bool get usbConnected => connected && device['transport'] == 'USB';
  bool get storageBusy =>
      recording ||
      device['recording'] == true ||
      [
        'opening',
        'recording',
        'stopping',
      ].contains(object(device['sd'])['state']);
  bool get canBrowseBoard => usbConnected && !_pollFailed && !storageBusy;
  static const boardPage = 8;
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

  bool get receiverFresh =>
      connected &&
      !_pollFailed &&
      live['stale'] != true &&
      _snapshotAgeMs < 1500;

  bool get diagnosticsFresh =>
      receiverFresh &&
      device['diagnostics_stale'] != true &&
      (numeric(device['diagnostics_age_ms']) ?? 0) + _snapshotAgeMs <=
          (numeric(device['diagnostics_ttl_ms']) ?? 2000);

  double? sensorAgeMs(String sensor) {
    final ageUs = numeric(object(device[sensor])['age_us']);
    return ageUs == null || ageUs < 0 ? null : ageUs / 1000 + _snapshotAgeMs;
  }

  bool sensorFresh(String sensor) =>
      receiverFresh &&
      (sensorAgeMs(sensor) ?? double.infinity) <
          (sensor == 'gnss' ? 1000 : 1500);

  // Store whole-percent crossings. The first reading can be anywhere within
  // its one-percent band, so only transitions provide comparable endpoints.
  final _batteryDrops = <({Duration at, int percent})>[];
  int? _previousPercent;

  String? get batteryState =>
      diagnosticsFresh ? object(device['battery'])['status']?.toString() : null;

  bool get _canEstimateDischarge => switch (batteryState) {
    'battery' || 'low' => true,
    null => bluetooth, // Display frames carry only the gauge percentage.
    _ => false,
  };

  void _updateBatteryTrend() {
    final value = numeric(device['battery_percent']);
    if (!receiverFresh ||
        !_canEstimateDischarge ||
        value == null ||
        value < 0 ||
        value > 100) {
      _batteryDrops.clear();
      _previousPercent = null;
      return;
    }
    final percent = value.toInt();
    final previous = _previousPercent;
    _previousPercent = percent;
    if (previous == null || previous == percent) return;
    if (previous - percent != 1) {
      _batteryDrops.clear();
      return;
    }
    if (_batteryDrops.length >= 2 && _dischargeTrendExpired) {
      _batteryDrops.clear();
    }
    _batteryDrops.add((at: _snapshotAt!, percent: percent));
    if (_batteryDrops.length > 5) _batteryDrops.removeAt(0);
  }

  double get _msPerPercent {
    final first = _batteryDrops.first, last = _batteryDrops.last;
    return (last.at - first.at).inMilliseconds / (first.percent - last.percent);
  }

  // A flat gauge can mean the receiver has been plugged in over Bluetooth.
  bool get _dischargeTrendExpired =>
      (_now - _batteryDrops.last.at).inMilliseconds > 2 * _msPerPercent;

  Duration? get batteryTimeLeft {
    if (!receiverFresh || _batteryDrops.length < 2) return null;
    final first = _batteryDrops.first, last = _batteryDrops.last;
    if (last.at - first.at < const Duration(minutes: 1) ||
        _dischargeTrendExpired) {
      return null;
    }
    return Duration(minutes: (_msPerPercent * last.percent / 60000).round());
  }

  List<GTrailPoint> _gTrail = const [];
  List<GTrailPoint> get gTrail => _gTrail;
  static const gTrailDurationUs = 3000000;

  void _updateGTrail() {
    final reading = imuAccelerationG;
    final tUs = object(object(device['imu'])['latest'])['t_us'];
    if (reading == null || tUs is! int) {
      _gTrail = const [];
      return;
    }
    final previous = _gTrail.lastOrNull;
    if (previous?.tUs == tUs) return;
    _gTrail = List.unmodifiable([
      if (previous != null && previous.tUs < tUs)
        ..._gTrail
            .skip(_gTrail.length > 15 ? _gTrail.length - 15 : 0)
            .where((p) => tUs - p.tUs < gTrailDurationUs),
      (tUs: tUs, x: reading.x, y: reading.y),
    ]);
  }

  ({double x, double y, double z})? get imuAccelerationG {
    if (!sensorFresh('imu')) return null;
    final sample = object(object(device['imu'])['latest']);
    final acceleration = sample['accel'];
    if (sample['valid'] != true ||
        acceleration is! List ||
        acceleration.length != 3) {
      return null;
    }
    final x = numeric(acceleration[0]);
    final y = numeric(acceleration[1]);
    final z = numeric(acceleration[2]);
    if (x == null || y == null || z == null) return null;

    const standardGravity = 9.80665;
    return (
      x: x / standardGravity,
      y: y / standardGravity,
      z: z / standardGravity,
    );
  }

  void changed() {
    if (!_disposed) notifyListeners();
  }

  Future<dynamic> request(
    String op, [
    Map<String, dynamic> params = const {},
  ]) async {
    final response = await transport(jsonEncode({'op': op, ...params}));
    // Session replies can contain megabytes of samples. Keep small live
    // snapshots on the fast path without starting a worker for each poll.
    final decoded = response.length < 64 * 1024
        ? _decodeReply(response)
        : await compute(
            _decodeReply,
            response,
            debugLabel: 'Decode receiver data',
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
  }

  bool _foreground = true;
  bool _editingTrack = false;

  void setEditingTrack(bool value) {
    _editingTrack = value;
    _schedulePoll();
  }

  void setForeground(bool value) {
    _foreground = value;
    _schedulePoll();
  }

  void _schedulePoll() {
    _timer?.cancel();
    if (!autoPoll || _disposed || !_foreground || _editingTrack || !connected)
      return;
    final livePage = page == 0;
    final board = page == boardPage && usbConnected;
    if (!livePage && page != 5 && page != 7 && !board && !recording) return;
    final interval = livePage
        ? const Duration(milliseconds: 200)
        : Duration(seconds: board ? 5 : 1);
    _timer = Timer.periodic(interval, (_) {
      if (_polling && stale) changed();
      unawaited(poll());
    });
  }

  Future<void> poll({bool liveFix = false}) async {
    if (_polling || _disposed || (page == boardPage && busy)) return;
    _polling = true;
    try {
      final requestedPage = page;
      final view = liveFix
          ? 'live'
          : switch (page) {
              0 => 'live',
              5 => 'receiver',
              7 => 'diagnostics',
              boardPage when usbConnected => 'board_data',
              _ => 'idle',
            };
      final before = _state;
      final update = object(await request('poll', {'view': view}));
      if (_disposed || page != requestedPage || !identical(before, _state))
        return;
      state = {...state, ...update};
      if (!connected && !recording) _timer?.cancel();
      if (valid && _recent.lastOrNull?['time_s'] != live['time_s']) {
        _recentSnapshot = null;
        _recent.add({
          for (final key in ['time_s', 'speed_mps', 'lat', 'lon', 'valid'])
            key: live[key],
        });
        final cutoff = (numeric(live['time_s']) ?? 0) - 300;
        while (_recent.length > 1500 ||
            (numeric(_recent.first['time_s']) ?? 0) < cutoff) {
          _recent.removeAt(0);
        }
      }
      changed();
    } catch (e) {
      _pollFailed = true;
      if (usbConnected) _clearDirectory();
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
      _schedulePoll();
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
  void _clearDirectory() {
    _directoryGeneration++;
    _lastCard = null;
    _previews.clear();
    deviceSessions = null;
    boardPreview = null;
  }

  Future<void> refreshDeviceSessions() async {
    if (!canBrowseBoard || busy) return;
    readingDirectory = true;
    await action(() async {
      final generation = _directoryGeneration;
      try {
        final entries = objects(await request('device_sessions'));
        if (generation == _directoryGeneration && canBrowseBoard) {
          deviceSessions = entries.reversed.toList();
          _previews.clear();
          boardPreview = null;
        }
      } catch (_) {
        _clearDirectory();
        rethrow;
      } finally {
        readingDirectory = false;
      }
    });
  }

  Future<void> previewDeviceSession(int id) async {
    if (!canBrowseBoard || busy) return;
    final cached = _previews[id];
    if (cached != null) {
      error = null;
      notice = null;
      boardPreview = cached;
      changed();
      return;
    }
    readingPreview = id;
    await action(() async {
      final generation = _directoryGeneration;
      try {
        final preview = object(
          await request('device_session_preview', {'id': id}),
        );
        if (generation == _directoryGeneration && canBrowseBoard) {
          _previews[id] = preview;
          boardPreview = preview;
        }
      } finally {
        readingPreview = null;
      }
    });
  }

  Future<void> downloadDeviceSession(int id) async {
    if (!canBrowseBoard) return;
    await action(
      () async {
        session = object(await request('download', {'id': id}));
        state = object(await request('state'));
        if (page == boardPage) page = 2;
      },
      pending: 'Downloading receiver recording… Keep the USB cable connected.',
      success:
          'Receiver recording downloaded and added to the session library.',
    );
  }

  Future<void> connect(String port) => action(
    () async {
      final connectedState = object(await request('connect', {'port': port}));
      _clearDirectory();
      state = connectedState;
      _recent.clear();
      _recentSnapshot = null;
    },
    pending: port == 'ble' ? 'Looking for a nearby V2 Mini…' : null,
    success: 'Receiver connected.',
  );
  Future<void> disconnect() => action(() async {
    state = object(await request('disconnect'));
    _clearDirectory();
  }, success: 'Receiver disconnected.');
  Future<void> start({String? name, String? trackId, bool fullRate = false}) =>
      action(() async {
        state = object(
          await request('start', {
            'name': name,
            'track_id': trackId,
            'display_only': !fullRate,
          }),
        );
      }, success: 'Recording started.');
  Future<void> stop() => action(
    () async {
      state = object(await request('stop'));
    },
    success: recording
        ? 'Recording saved to the session library.'
        : 'Receiver recording stopped.',
  );
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
  Future<bool> saveTrack(Map<String, dynamic> track) async {
    var saved = false;
    await action(() async {
      await request('save_track', {'track': track});
      state = object(await request('state'));
      saved = true;
    }, success: 'Track saved.');
    return saved;
  }

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
    _schedulePoll();
    if (autoPoll &&
        _foreground &&
        connected &&
        ([0, 5, 7].contains(page) || (page == boardPage && usbConnected)))
      unawaited(poll());
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
