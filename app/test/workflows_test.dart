import 'dart:convert';
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:app/controller.dart';
import 'package:app/ui/app.dart';
import 'package:app/ui/common.dart';
import 'package:app/ui/charts.dart';

class FakeBackend {
  final calls = <Map<String, dynamic>>[];
  Map<String, dynamic> state = {
    'device': {'connected': false, 'fix_quality': 'no fix'},
    'live': {'recording': false, 'valid': false},
    'sessions': <Map<String, dynamic>>[],
    'tracks': <Map<String, dynamic>>[],
    'drag': {
      'start_kph': 0,
      'end_kph': 100,
      'distance_m': 402.336,
      'rollout': false,
    },
    'storage_path': 'C:\\Telemetry',
  };
  List<Map<String, dynamic>> devices = [];
  List<Map<String, dynamic>> deviceSessions = [];
  late Map<String, dynamic> session = {
    'id': 'demo-1',
    'name': 'Demonstration circuit',
    'created_ms': 1700000000000,
    'duration_s': 20,
    'sample_count': 201,
    'distance_m': 380,
    'max_speed_mps': 30,
    'source': 'synthetic demo',
    'samples': List.generate(
      201,
      (i) => {
        'time_s': i / 10,
        'lat': -33.0 + i / 100000,
        'lon': 151.0 + i / 100000,
        'speed_mps': 20 + i / 20,
        'valid': true,
        'altitude_m': 40,
        'accel_long_g': .2,
        'accel_lat_g': .4,
      },
    ),
    'drag': {
      'start_kph': 0,
      'end_kph': 100,
      'distance_m': 402.336,
      'rollout': false,
    },
    'analysis': {
      'engine': 'test fixture',
      'valid_samples': 201,
      'best_lap_s': 10,
      'theoretical_best_s': 10,
      'laps': [
        {
          'number': 1,
          'start_s': 0,
          'end_s': 10,
          'duration_s': 10,
          'valid': true,
          'sectors_s': [5, 5],
        },
        {
          'number': 2,
          'start_s': 10,
          'end_s': 20,
          'duration_s': 10,
          'valid': true,
          'sectors_s': [5, 5],
        },
      ],
      'drag_runs': [],
      'warnings': [],
    },
  };
  Future<String> request(String input) async {
    final q = Map<String, dynamic>.from(jsonDecode(input));
    calls.add(q);
    dynamic result;
    switch (q['op']) {
      case 'state':
      case 'poll':
        result = state;
      case 'devices':
        result = devices;
      case 'device_sessions':
        result = deviceSessions;
      case 'download':
        session['source'] = 'V2 Mini SD';
        state['sessions'] = [session];
        result = session;
      case 'demo':
        state['sessions'] = [session];
        result = session;
      case 'session':
        result = session;
      case 'ppk_status':
        result = {
          'tools': {'convbin': 'convbin.exe', 'rnx2rtkp': 'rnx2rtkp.exe'},
          'raw_available': q['id'] != null,
          'warnings': [],
        };
      case 'prepare_ppk':
        result = {
          'rover_obs': 'C:\\ppk\\rover.obs',
          'rover_nav': 'C:\\ppk\\rover.nav',
          'ready': true,
          'warnings': [],
        };
      case 'run_ppk':
        session = {
          ...session,
          'id': 'ppk-1',
          'source': 'PPK',
          'provenance': {
            'kind': 'ppk',
            'parent_session_id': q['source_id'],
            'quality_counts': {'fixed': 12, 'float': 3},
          },
        };
        state['sessions'] = [session];
        result = {'session': session, 'warnings': []};
      case 'import_ppk':
        result = session;
      case 'connect':
        state['device'] = {
          'connected': true,
          'port': q['port'],
          'fix_quality': 'no fix',
        };
        result = state;
      case 'start':
        (state['live'] as Map)['recording'] = true;
        result = state;
      case 'stop':
        (state['live'] as Map)['recording'] = false;
        result = state;
      case 'set_drag':
        state['drag'] = q['config'];
        result = state;
      default:
        result = state;
    }
    return jsonEncode({'ok': true, 'data': result});
  }
}

Future<AeviaController> mount(
  WidgetTester tester,
  FakeBackend backend, {
  Size size = const Size(1360, 1000),
}) async {
  tester.view.physicalSize = size;
  tester.view.devicePixelRatio = 1;
  addTearDown(tester.view.resetPhysicalSize);
  addTearDown(tester.view.resetDevicePixelRatio);
  final controller = AeviaController(backend.request, autoPoll: false);
  await tester.pumpWidget(AeviaApp(controller: controller));
  await tester.pumpAndSettle();
  return controller;
}

void main() {
  testWidgets(
    'changing PPK source clears generated rover files from the old source',
    (tester) async {
      final backend = FakeBackend();
      backend.state['sessions'] = [
        {'id': 'a', 'name': 'Source A'},
        {'id': 'b', 'name': 'Source B'},
      ];
      final c = await mount(tester, backend);
      c.ppkDraft.addAll({
        'source_id': 'a',
        'rover_obs': 'C:\\A.obs',
        'rover_nav': 'C:\\A.nav',
      });
      c.ppkPrepared = {'rover_obs': 'C:\\A.obs', 'rover_nav': 'C:\\A.nav'};
      c.navigate(6);
      await tester.pumpAndSettle();
      await tester.tap(find.byKey(const ValueKey('ppk_source_a')));
      await tester.pumpAndSettle();
      await tester.tap(find.text('Source B').last);
      await tester.pumpAndSettle();
      expect(c.ppkDraft['source_id'], 'b');
      expect(
        tester
            .widget<TextField>(find.byKey(const ValueKey('ppk_rover_obs')))
            .controller!
            .text,
        isEmpty,
      );
      expect(
        tester
            .widget<TextField>(find.byKey(const ValueKey('ppk_rover_nav')))
            .controller!
            .text,
        isEmpty,
      );
      expect(tester.takeException(), isNull);
    },
  );

  testWidgets('PPK replay shows float quality with absent velocity channels', (
    tester,
  ) async {
    final backend = FakeBackend();
    final c = await mount(tester, backend);
    c.session = {
      ...backend.session,
      'source': 'RTKLIB PPK result',
      'max_speed_mps': null,
      'provenance': {
        'kind': 'ppk',
        'quality_counts': {'float': 2},
        'solution_file': {'path': 'C:\\ppk\\solution.pos'},
      },
      'samples': [
        for (var i = 0; i < 2; i++)
          {
            'time_s': i.toDouble(),
            'lat': -33.0 + i * .00001,
            'lon': 151.0,
            'altitude_m': 30.0,
            'valid': true,
            'speed_mps': null,
            'ppk_quality': {
              'q': 2,
              'status': 'float',
              'sd_n_m': .2,
              'sd_e_m': .3,
              'sd_u_m': .5,
              'ratio': 1.8,
            },
          },
      ],
      'analysis': {
        'warnings': ['Position-only result: velocity unavailable.'],
        'laps': [],
        'drag_runs': [],
      },
    };
    c.navigate(2);
    await tester.pumpAndSettle();
    expect(find.text('PPK solution quality'), findsOneWidget);
    expect(find.text('Solution: C:\\ppk\\solution.pos'), findsOneWidget);
    expect(find.text('PPK FLOAT'), findsOneWidget);
    expect(find.textContaining('Ratio 1.8'), findsOneWidget);
    expect(tester.takeException(), isNull);
  });

  test(
    'PPK preparation and processing retain explicit inputs and parent lineage',
    () async {
      final backend = FakeBackend();
      final c = AeviaController(backend.request, autoPoll: false);
      await c.initialize();
      await c.checkPpk('raw-1');
      expect(c.ppkStatus['raw_available'], isTrue);
      await c.preparePpk({'id': 'raw-1', 'output_dir': 'C:\\ppk'});
      expect(c.ppkDraft['rover_obs'], 'C:\\ppk\\rover.obs');
      await c.runPpk({
        'source_id': 'raw-1',
        'rover_obs': c.ppkDraft['rover_obs'],
        'base_obs': 'C:\\base.obs',
        'nav_paths': [c.ppkDraft['rover_nav']],
        'output_dir': 'C:\\ppk',
      });
      expect(c.page, 2);
      expect(c.session?['id'], 'ppk-1');
      expect(object(c.session?['provenance'])['parent_session_id'], 'raw-1');
      expect(
        backend.calls.singleWhere((q) => q['op'] == 'run_ppk')['base_obs'],
        'C:\\base.obs',
      );
      c.dispose();
    },
  );

  testWidgets(
    'PPK workspace requires base inputs and does not present USB telemetry as raw',
    (tester) async {
      final backend = FakeBackend();
      final c = await mount(tester, backend);
      c.navigate(6);
      await tester.pumpAndSettle();
      expect(
        find.textContaining('USB telemetry alone is not a PPK input'),
        findsOneWidget,
      );
      final prepare = tester.widget<FilledButton>(
        find.widgetWithText(FilledButton, 'Prepare RINEX'),
      );
      expect(prepare.onPressed, isNull);
      final run = find.widgetWithText(FilledButton, 'Run PPK');
      await tester.ensureVisible(run);
      expect(tester.widget<FilledButton>(run).onPressed, isNull);
      expect(tester.takeException(), isNull);
    },
  );

  testWidgets('cached live values expire while a poll remains pending', (
    tester,
  ) async {
    var now = Duration.zero;
    final pending = Completer<String>();
    final fresh = {
      'device': {'connected': true, 'fix_quality': 'valid'},
      'live': {'valid': true, 'stale': false, 'age_ms': 0, 'speed_mps': 30},
    };
    final controller = AeviaController((input) async {
      final op = jsonDecode(input)['op'];
      if (op == 'poll') return pending.future;
      return jsonEncode({'ok': true, 'data': op == 'devices' ? [] : fresh});
    }, monotonicNow: () => now);
    tester.view.physicalSize = const Size(1360, 1000);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.resetPhysicalSize);
    addTearDown(tester.view.resetDevicePixelRatio);
    await tester.pumpWidget(AeviaApp(controller: controller));
    await tester.pumpAndSettle();
    expect(find.text('108'), findsOneWidget);
    now = const Duration(seconds: 2);
    await tester.pump(const Duration(seconds: 2));
    expect(find.text('STALE · WAITING FOR DATA'), findsOneWidget);
    expect(find.text('108'), findsNothing);
    pending.complete(jsonEncode({'ok': true, 'data': fresh}));
    await tester.pumpAndSettle();
    expect(find.text('LIVE · VALID FIX'), findsOneWidget);
    expect(find.text('108'), findsOneWidget);
    await tester.pumpWidget(const SizedBox.shrink());
  });
  test(
    'failed polling invalidates cached values until a fresh response',
    () async {
      var failed = false;
      var now = Duration.zero;
      final controller = AeviaController(
        (input) async {
          final op = jsonDecode(input)['op'];
          if (op == 'poll' && failed)
            return jsonEncode({'ok': false, 'error': 'Transport failed'});
          return jsonEncode({
            'ok': true,
            'data': op == 'devices'
                ? []
                : {
                    'device': {'connected': true, 'fix_quality': 'valid'},
                    'live': {
                      'valid': true,
                      'stale': false,
                      'age_ms': 750,
                      'speed_mps': 30,
                    },
                  },
          });
        },
        autoPoll: false,
        monotonicNow: () => now,
      );
      await controller.initialize();
      expect(controller.valid, isTrue);
      now = const Duration(milliseconds: 800);
      expect(
        controller.valid,
        isFalse,
        reason: 'Device age and local transport age must add together',
      );
      await controller.poll();
      expect(controller.valid, isTrue);
      failed = true;
      await controller.poll();
      expect(controller.valid, isFalse);
      expect(controller.stale, isTrue);
      failed = false;
      await controller.poll();
      expect(controller.valid, isTrue);
      controller.dispose();
    },
  );
  testWidgets(
    'replay leaves gaps and time beyond a selected trace unavailable',
    (tester) async {
      final backend = FakeBackend();
      backend.session['samples'] = [
        for (final t in [0.0, 1.0, 10.0, 15.0])
          {
            'time_s': t,
            'lat': -33 + t / 100000,
            'lon': 151.0,
            'speed_mps': 30.0 + t,
            'accel_long_g': 0.3,
            'accel_lat_g': 0.2,
            'altitude_m': 40.0,
            'heading_deg': 10.0,
            'valid': true,
          },
      ];
      backend.session['analysis']['laps'] = [
        {
          'number': 1,
          'start_s': 0.0,
          'end_s': 1.0,
          'duration_s': 1.0,
          'valid': true,
          'sectors_s': [],
        },
        {
          'number': 2,
          'start_s': 10.0,
          'end_s': 15.0,
          'duration_s': 5.0,
          'valid': true,
          'sectors_s': [],
        },
      ];
      final controller = await mount(tester, backend);
      await controller.demo();
      await tester.pumpAndSettle();
      void seek(double value) =>
          tester.widget<Slider>(find.byType(Slider).first).onChanged!(value);
      String metric(String label) => tester
          .widgetList<Metric>(find.byType(Metric))
          .firstWhere((m) => m.label == label)
          .value;
      MapPainter map() => tester
          .widgetList<CustomPaint>(find.byType(CustomPaint))
          .map((w) => w.painter)
          .whereType<MapPainter>()
          .first;
      seek(0.5);
      await tester.pumpAndSettle();
      expect(find.text('SIGNAL GAP'), findsOneWidget);
      expect(metric('Speed'), '—');
      expect(metric('Longitudinal'), '—');
      expect(map().cursor, isNull);
      seek(10 / 15);
      await tester.pumpAndSettle();
      expect(find.text('VALID SAMPLE'), findsOneWidget);
      expect(metric('Speed'), '144.0');
      (controller.session!['samples'] as List)[1]['valid'] = false;
      seek(1 / 15);
      await tester.pumpAndSettle();
      expect(find.text('INVALID SAMPLE'), findsOneWidget);
      expect(metric('Speed'), '—');
      expect(map().cursor, isNull);
      (controller.session!['samples'] as List)[1]['valid'] = true;
      tester
          .widget<DropdownButtonFormField<int>>(
            find.byType(DropdownButtonFormField<int>).at(0),
          )
          .onChanged!(0);
      await tester.pumpAndSettle();
      tester
          .widget<DropdownButtonFormField<int>>(
            find.byType(DropdownButtonFormField<int>).at(1),
          )
          .onChanged!(1);
      await tester.pumpAndSettle();
      seek(0.8);
      await tester.pumpAndSettle();
      expect(find.text('NO SAMPLE AT THIS TIME'), findsOneWidget);
      expect(metric('Speed'), '—');
      expect(map().cursor, isNull);
    },
  );
  testWidgets(
    'interrupted capture remains saveable and is never labelled recording',
    (tester) async {
      final backend = FakeBackend();
      backend.state['device'] = {'connected': true, 'fix_quality': 'no fix'};
      backend.state['live'] = {
        'recording': true,
        'valid': false,
        'capture_interrupted': true,
        'capture_error': 'Capture capacity reached.',
      };
      backend.state['sessions'] = [
        {...backend.session, 'incomplete_reason': 'Capture capacity reached.'},
      ];
      final controller = await mount(
        tester,
        backend,
        size: const Size(390, 844),
      );
      expect(find.text('CAPTURE INTERRUPTED'), findsOneWidget);
      expect(find.text('RECORDING'), findsNothing);
      expect(find.textContaining('Capture capacity reached.'), findsOneWidget);
      expect(
        tester
            .widget<FilledButton>(
              find.widgetWithText(FilledButton, 'Stop & save session'),
            )
            .onPressed,
        isNotNull,
      );
      expect(tester.takeException(), isNull);
      controller.navigate(1);
      await tester.pumpAndSettle();
      expect(find.text('INCOMPLETE CAPTURE'), findsOneWidget);
      expect(tester.takeException(), isNull);
    },
  );
  test(
    'a long import leaves status polling and cancellation responsive',
    () async {
      final pendingImport = Completer<String>();
      final calls = <String>[];
      final controller = AeviaController((input) async {
        final op = jsonDecode(input)['op'] as String;
        calls.add(op);
        if (op == 'import') return pendingImport.future;
        if (op == 'cancel_processing') {
          pendingImport.complete(
            jsonEncode({'ok': false, 'error': 'Analysis cancelled'}),
          );
          return jsonEncode({
            'ok': true,
            'data': {'cancel_requested': true},
          });
        }
        return jsonEncode({
          'ok': true,
          'data': {'processing': 'import'},
        });
      }, autoPoll: false);
      final operation = controller.importSession('capture.aev');
      expect(controller.busy, isTrue);
      await controller.poll();
      expect(controller.state['processing'], 'import');
      await controller.cancelProcessing();
      await operation;
      expect(
        calls,
        containsAllInOrder(['import', 'poll', 'cancel_processing']),
      );
      expect(controller.error, 'Analysis cancelled');
      expect(controller.busy, isFalse);
      controller.dispose();
    },
  );
  testWidgets('offline driving display never fabricates live telemetry', (
    tester,
  ) async {
    final backend = FakeBackend();
    (backend.state['live'] as Map)['stale'] = true;
    await mount(tester, backend);
    expect(find.text('DISCONNECTED'), findsOneWidget);
    expect(find.text('Waiting for receiver'), findsOneWidget);
    expect(find.text('Last fix is stale'), findsNothing);
    final record = tester.widget<FilledButton>(
      find.widgetWithText(FilledButton, 'Start recording'),
    );
    expect(record.onPressed, isNull);
    expect(tester.takeException(), isNull);
  });
  testWidgets('stale fix hides numerical speed and recovery cannot connect', (
    tester,
  ) async {
    final backend = FakeBackend();
    backend.state['device'] = {
      'connected': true,
      'port': 'COM9',
      'fix_quality': 'stale',
    };
    backend.state['live'] = {
      'speed_mps': 99,
      'valid': true,
      'stale': true,
      'recording': false,
    };
    final controller = await mount(tester, backend);
    expect(find.text('STALE · WAITING FOR DATA'), findsOneWidget);
    expect(find.text('356'), findsNothing);
    backend.state['device'] = {'connected': false};
    backend.devices = [
      {'port': 'COM7', 'name': 'Aevia Loader', 'recovery': true},
    ];
    await controller.poll();
    await controller.refreshDevices();
    controller.navigate(5);
    await tester.pumpAndSettle();
    expect(
      find.text('Recovery loader · install application firmware first'),
      findsOneWidget,
    );
    expect(
      tester
          .widget<FilledButton>(find.widgetWithText(FilledButton, 'Connect'))
          .onPressed,
      isNull,
    );
    expect(tester.takeException(), isNull);
  });
  testWidgets(
    'workspace navigation keeps scrolling independent between analysis and receiver',
    (tester) async {
      final controller = await mount(
        tester,
        FakeBackend(),
        size: const Size(1360, 700),
      );
      await controller.demo();
      await tester.pumpAndSettle();
      ScrollPosition position(int page) => tester
          .state<ScrollableState>(
            find
                .descendant(
                  of: find.byKey(PageStorageKey('workspace-$page')),
                  matching: find.byType(Scrollable),
                )
                .first,
          )
          .position;
      position(2).jumpTo(500);
      await tester.pumpAndSettle();
      expect(position(2).pixels, 500);
      controller.navigate(5);
      await tester.pumpAndSettle();
      expect(position(5).pixels, 0);
      expect(find.text('V2 Mini connection').hitTestable(), findsOneWidget);
      position(5).jumpTo(200);
      await tester.pumpAndSettle();
      controller.navigate(2);
      await tester.pumpAndSettle();
      expect(position(2).pixels, 500);
      controller.navigate(5);
      await tester.pumpAndSettle();
      expect(position(5).pixels, 200);
    },
  );
  testWidgets(
    'demo opens synchronized analysis with explicit synthetic label',
    (tester) async {
      final backend = FakeBackend();
      final controller = await mount(tester, backend);
      controller.navigate(1);
      await tester.pumpAndSettle();
      await tester.tap(find.text('Explore demo'));
      await tester.pumpAndSettle();
      expect(find.text('SYNTHETIC DEMO'), findsOneWidget);
      expect(find.text('Replay & comparison'), findsOneWidget);
      expect(find.text('Elapsed-time alignment'), findsOneWidget);
      final slider = find.byType(Slider).first;
      await tester.ensureVisible(slider);
      await tester.drag(slider, const Offset(120, 0));
      await tester.pumpAndSettle();
      expect(tester.takeException(), isNull);
      expect(backend.calls.any((q) => q['op'] == 'demo'), isTrue);
    },
  );
  testWidgets('all workspaces fit a narrow viewport without layout errors', (
    tester,
  ) async {
    final backend = FakeBackend();
    backend.state['device'] = {
      'connected': true,
      'fix_quality': 'no fix',
      'battery': {'status': 'charging', 'millivolts': 3800},
      'navigation': {
        'phase': 'waiting-for-fix',
        'qualified': false,
        'timing': 'arrival-only-development',
      },
      'gnss': {'errors': 0},
      'gnss_uart': {'last_error': 'fifo-overflow'},
      'imu': {'errors': 12},
      'imu_diagnostics': {'last_error': 'CaptureTiming'},
      'capture_timing': {
        'pps_locked': false,
        'imu_timing_rejected': 12,
        'capture_errors': 0,
      },
      'display': 'not-fitted',
      'accessories': {'buttons_fitted': false},
    };
    final controller = await mount(tester, backend, size: const Size(390, 844));
    for (final page in [0, 1, 3, 4, 6, 5]) {
      controller.navigate(page);
      await tester.pumpAndSettle();
      expect(tester.takeException(), isNull, reason: 'page $page');
    }
    expect(find.text('UNQUALIFIED'), findsOneWidget);
    expect(
      find.text('Sensor errors since boot: GNSS 0 · IMU 12'),
      findsOneWidget,
    );
    expect(find.text('Last IMU diagnostic: CaptureTiming'), findsOneWidget);
    expect(
      find.text('Last GNSS transport diagnostic: fifo-overflow'),
      findsOneWidget,
    );
    expect(
      find.text('Extra buttons disabled · use the app controls'),
      findsOneWidget,
    );
    await controller.demo();
    await tester.pumpAndSettle();
    expect(tester.takeException(), isNull, reason: 'analysis');
  });
  testWidgets('drag configuration validates threshold order before sending', (
    tester,
  ) async {
    final backend = FakeBackend();
    final controller = await mount(tester, backend);
    controller.navigate(3);
    await tester.pumpAndSettle();
    await tester.enterText(
      find.widgetWithText(TextFormField, 'Starting speed'),
      '150',
    );
    await tester.tap(find.text('Save measurement configuration'));
    await tester.pumpAndSettle();
    expect(find.text('Must exceed starting speed'), findsOneWidget);
    expect(backend.calls.any((q) => q['op'] == 'set_drag'), isFalse);
    await tester.enterText(
      find.widgetWithText(TextFormField, 'Ending speed'),
      '200',
    );
    await tester.tap(find.text('Save measurement configuration'));
    await tester.pumpAndSettle();
    expect(
      backend.calls.lastWhere(
        (q) => q['op'] == 'set_drag',
      )['config']['end_kph'],
      200,
    );
  });
  test(
    'transport errors remain visible and never invent successful state',
    () async {
      final controller = AeviaController(
        (_) async => jsonEncode({'ok': false, 'error': 'USB disconnected'}),
        autoPoll: false,
      );
      await controller.initialize();
      expect(controller.connected, isFalse);
      expect(controller.error, 'USB disconnected');
      expect(controller.busy, isFalse);
      controller.dispose();
    },
  );
  testWidgets(
    'incomplete device recordings are recoverable while active capture prevents download',
    (tester) async {
      final backend = FakeBackend();
      backend.state['device'] = {
        'connected': true,
        'port': 'COM9',
        'fix_quality': 'no fix',
      };
      backend.deviceSessions = [
        {'id': 7, 'bytes': 2048, 'closed': true},
        {'id': 8, 'bytes': 512, 'closed': false},
      ];
      final controller = await mount(tester, backend);
      controller.navigate(5);
      await tester.pumpAndSettle();
      await tester.ensureVisible(find.text('Refresh'));
      await tester.tap(find.text('Refresh'));
      await tester.pumpAndSettle();
      expect(find.text('Recording 7'), findsOneWidget);
      expect(find.text('Recording 8'), findsOneWidget);
      final downloads = find.widgetWithText(
        OutlinedButton,
        'Download & analyze',
      );
      expect(
        tester.widget<OutlinedButton>(downloads.at(0)).onPressed,
        isNotNull,
      );
      expect(
        tester.widget<OutlinedButton>(downloads.at(1)).onPressed,
        isNotNull,
      );
      expect(find.textContaining('Incomplete · recoverable'), findsOneWidget);
      (backend.state['live'] as Map)['recording'] = true;
      await controller.poll();
      await tester.pumpAndSettle();
      expect(tester.widget<OutlinedButton>(downloads.at(0)).onPressed, isNull);
      expect(tester.widget<OutlinedButton>(downloads.at(1)).onPressed, isNull);
      (backend.state['live'] as Map)['recording'] = false;
      (backend.state['device'] as Map)['sd'] = {'state': 'recording'};
      await controller.poll();
      await tester.pumpAndSettle();
      expect(tester.widget<OutlinedButton>(downloads.at(1)).onPressed, isNull);
      (backend.state['device'] as Map)['sd'] = {'state': 'ready'};
      await controller.poll();
      await tester.pumpAndSettle();
      await tester.ensureVisible(downloads.at(1));
      await tester.tap(downloads.at(1));
      await tester.pumpAndSettle();
      expect(backend.calls.lastWhere((q) => q['op'] == 'download')['id'], 8);
      expect(find.text('V2 MINI SD'), findsOneWidget);
      expect(controller.page, 2);
      expect(tester.takeException(), isNull);
    },
  );
}
