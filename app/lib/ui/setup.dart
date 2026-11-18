import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';
import 'track_editor.dart';

class ReceiverView extends StatefulWidget {
  const ReceiverView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<ReceiverView> createState() => _ReceiverViewState();
}

class _ReceiverViewState extends State<ReceiverView> {
  final manualPort = TextEditingController();
  @override
  void dispose() {
    manualPort.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final c = widget.controller, device = widget.controller.device;
    final battery = object(device['battery']);
    final sd = object(device['sd']);
    final nav = object(device['navigation']);
    final timing = object(device['capture_timing']);
    final gnss = object(device['gnss']);
    final gnssUart = object(device['gnss_uart']);
    final imu = object(device['imu']);
    final imuDiagnostics = object(device['imu_diagnostics']);
    final accessories = object(device['accessories']);
    final profile = object(device['acquisition_profile']);
    final ppk = object(device['ppk']);
    final stale = !c.connected || c.stale;
    final deviceRecording =
        c.recording ||
        device['recording'] == true ||
        ['recording', 'stopping'].contains(sd['state']);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Section(
          title: 'V2 Mini connection',
          trailing: Status(
            c.connected ? 'CONNECTED' : 'DISCONNECTED',
            color: c.connected ? accent : muted,
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Text(
                'Connect the V2 Mini USB data port. Only one application can open the receiver at a time.',
                style: TextStyle(color: muted),
              ),
              const SizedBox(height: 16),
              if (c.connected) ...[
                MetricGrid(
                  children: [
                    Metric('Port', device['port']?.toString() ?? '—'),
                    Metric('Firmware', device['firmware']?.toString() ?? '—'),
                    Metric(
                      'Battery',
                      stale ? '—' : number(device['battery_percent'], 0),
                      unit: '%',
                    ),
                    Metric('GNSS rate', number(device['gnss_hz']), unit: 'Hz'),
                    Metric('IMU rate', number(device['imu_hz']), unit: 'Hz'),
                  ],
                ),
                const SizedBox(height: 14),
                Align(
                  alignment: Alignment.centerLeft,
                  child: OutlinedButton(
                    onPressed: c.busy || c.recording ? null : c.disconnect,
                    child: const Text('Disconnect receiver'),
                  ),
                ),
                if (c.recording)
                  const Padding(
                    padding: EdgeInsets.only(top: 8),
                    child: Text(
                      'Stop and save the recording before disconnecting.',
                      style: TextStyle(color: muted),
                    ),
                  ),
              ],
              if (!c.connected) ...[
                for (final d in c.devices)
                  Padding(
                    padding: const EdgeInsets.only(bottom: 8),
                    child: Container(
                      decoration: BoxDecoration(
                        border: Border.all(color: line),
                        borderRadius: BorderRadius.circular(4),
                      ),
                      padding: const EdgeInsets.all(12),
                      child: Row(
                        children: [
                          const Icon(Icons.usb, color: muted, size: 22),
                          const SizedBox(width: 12),
                          Expanded(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  '${d['name'] ?? 'Serial receiver'} · ${d['port']}',
                                ),
                                Text(
                                  d['recovery'] == true
                                      ? 'Recovery loader · install application firmware first'
                                      : 'USB serial receiver',
                                  style: const TextStyle(
                                    color: muted,
                                    fontSize: 11,
                                  ),
                                ),
                              ],
                            ),
                          ),
                          FilledButton(
                            onPressed: c.busy || d['recovery'] == true
                                ? null
                                : () => c.connect(d['port'].toString()),
                            child: const Text('Connect'),
                          ),
                        ],
                      ),
                    ),
                  ),
                if (c.devices.isEmpty)
                  const Padding(
                    padding: EdgeInsets.symmetric(vertical: 16),
                    child: Text(
                      'No serial receivers found. Check the USB data cable and refresh.',
                      style: TextStyle(color: muted),
                    ),
                  ),
                const SizedBox(height: 8),
                Wrap(
                  spacing: 12,
                  runSpacing: 12,
                  crossAxisAlignment: WrapCrossAlignment.center,
                  children: [
                    OutlinedButton.icon(
                      onPressed: c.busy ? null : c.refreshDevices,
                      icon: const Icon(Icons.refresh, size: 17),
                      label: const Text('Refresh devices'),
                    ),
                    SizedBox(
                      width: 180,
                      child: TextField(
                        controller: manualPort,
                        decoration: const InputDecoration(
                          labelText: 'Manual serial port',
                          hintText: 'COM3 / /dev/ttyACM0',
                        ),
                      ),
                    ),
                    OutlinedButton(
                      onPressed: c.busy
                          ? null
                          : () {
                              if (manualPort.text.trim().isNotEmpty)
                                c.connect(manualPort.text.trim());
                            },
                      child: const Text('Connect port'),
                    ),
                  ],
                ),
              ],
            ],
          ),
        ),
        const SizedBox(height: 16),
        Section(
          title: 'Receiver diagnostics',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              MetricGrid(
                children: [
                  Metric(
                    'Fix quality',
                    device['fix_quality']?.toString() ?? '—',
                  ),
                  Metric('Satellites', '${device['satellites'] ?? '—'}'),
                  Metric('Data age', number(c.dataAgeMs, 0), unit: 'ms'),
                  Metric('Sequence', '${c.live['sequence'] ?? '—'}'),
                ],
              ),
              const SizedBox(height: 12),
              SelectableText(
                'Status: ${device['status'] ?? 'No receiver connected'}\nLatitude: ${number(c.live['lat'], 7)}°\nLongitude: ${number(c.live['lon'], 7)}°',
                style: const TextStyle(color: muted, height: 1.8),
              ),
              if (device['imu_status'] != null)
                Text(
                  'IMU: ${device['imu_status']}',
                  style: const TextStyle(color: muted),
                ),
              if (!stale && profile.isNotEmpty)
                Text(
                  'Acquisition: ${profile['gnss_log'] ?? 'GNSS'} ${profile['gnss_requested_hz'] ?? '—'} Hz requested · IMU ${profile['imu_logging'] ?? 'unknown'} · Navigation averages ${profile['imu_navigation_average'] ?? '—'} IMU intervals',
                  style: const TextStyle(color: muted),
                ),
              if (!stale && ppk.isNotEmpty)
                Padding(
                  padding: const EdgeInsets.only(top: 8),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'PPK raw logging: ${ppk['storage'] ?? 'SD'} · ${ppk['requested_hz'] ?? '—'} Hz requested · ${ppk['observation_frames'] ?? '—'} observation frames',
                        style: const TextStyle(color: muted),
                      ),
                      Text(
                        'Latest epoch: ${ppk['last_observation_count'] ?? '—'} signal observations · ${ppk['ephemeris_frames'] ?? '—'} ephemeris frames · ${ppk['dropped'] ?? '—'} dropped chunks · ${ppk['crc_failures'] ?? '—'} CRC failures',
                        style: const TextStyle(color: muted),
                      ),
                      if (ppk['last_observation_count'] == 0)
                        const Text(
                          'No tracked raw observations yet. Empty observation frames cannot produce a PPK solution.',
                          style: TextStyle(color: warning),
                        ),
                      Text(
                        ppk['configured'] != true ||
                                (numeric(ppk['age_ms']) ?? double.infinity) >
                                    1500
                            ? 'Raw observation stream is unavailable or stale.'
                            : 'Last raw observation epoch: ${number(ppk['age_ms'], 0)} ms ago',
                        style: TextStyle(
                          color:
                              ppk['configured'] != true ||
                                  (numeric(ppk['age_ms']) ?? double.infinity) >
                                      1500
                              ? warning
                              : muted,
                        ),
                      ),
                      TextButton(
                        onPressed: () => c.navigate(6),
                        child: const Text('Open PPK workspace'),
                      ),
                    ],
                  ),
                ),
              if (gnss.isNotEmpty || imu.isNotEmpty)
                Text(
                  stale
                      ? 'Sensor counters: telemetry unavailable'
                      : 'Sensor errors since boot: GNSS ${gnss['errors'] ?? '—'} · IMU ${imu['errors'] ?? '—'}',
                  style: const TextStyle(color: muted),
                ),
              if (timing.isNotEmpty)
                Text(
                  stale
                      ? 'Capture timing: telemetry unavailable'
                      : 'IMU timing rejections: ${timing['imu_timing_rejected'] ?? '—'} · Capture errors: ${timing['capture_errors'] ?? '—'}',
                  style: const TextStyle(color: muted),
                ),
              if (!stale && imuDiagnostics['last_error'] != null)
                Text(
                  'Last IMU diagnostic: ${imuDiagnostics['last_error']}',
                  style: const TextStyle(color: warning),
                ),
              if (!stale && gnssUart['last_error'] != null)
                Text(
                  'Last GNSS transport diagnostic: ${gnssUart['last_error']}',
                  style: const TextStyle(color: warning),
                ),
              if (device['storage_status'] != null)
                Text(
                  'Device storage: ${sd['state'] ?? device['storage_status']}${sd['error'] == null ? '' : ' · ${sd['error']}'}',
                  style: const TextStyle(color: muted),
                ),
              if (battery.isNotEmpty)
                Text(
                  stale
                      ? 'Power: telemetry unavailable'
                      : 'Power: ${battery['status'] ?? 'unknown'} · ${number(battery['millivolts'], 0)} mV',
                  style: const TextStyle(color: muted),
                ),
              if (device['display'] != null)
                Text(
                  'Display: ${device['display']}',
                  style: const TextStyle(color: muted),
                ),
              if (accessories['buttons_fitted'] == false)
                const Text(
                  'Extra buttons disabled · use the app controls',
                  style: TextStyle(color: muted),
                ),
              if (c.connected && device['fix_quality'] == 'no fix')
                const Padding(
                  padding: EdgeInsets.only(top: 12),
                  child: Text(
                    'No GNSS fix. This is expected indoors; sensor health and recording remain available.',
                    style: TextStyle(color: warning),
                  ),
                ),
            ],
          ),
        ),
        if (nav.isNotEmpty) ...[
          const SizedBox(height: 16),
          Section(
            title: 'On-device navigation',
            trailing: Status(
              nav['qualified'] == true ? 'QUALIFIED' : 'UNQUALIFIED',
              color: nav['qualified'] == true ? accent : warning,
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'State: ${stale ? 'telemetry unavailable' : nav['phase'] ?? 'unknown'}',
                ),
                if (nav['qualified'] != true)
                  const Padding(
                    padding: EdgeInsets.symmetric(vertical: 8),
                    child: Text(
                      'Development estimator. Sensor mounting and timing calibration are still required before using fused performance results.',
                      style: TextStyle(color: warning),
                    ),
                  ),
                Text(
                  'IMU accepted: ${nav['imu_accepted'] ?? '—'} · GNSS fused: ${nav['gnss_fused'] ?? '—'} · Rejected: ${nav['rejected'] ?? '—'}',
                  style: const TextStyle(color: muted),
                ),
                Text(
                  'Timing: ${nav['timing'] ?? 'unknown'} · PPS ${timing['pps_locked'] == true ? 'locked' : 'unlocked'}',
                  style: const TextStyle(color: muted),
                ),
                if (nav['error'] != null)
                  Text(
                    'Navigation diagnostic: ${nav['error']}',
                    style: const TextStyle(color: warning),
                  ),
              ],
            ),
          ),
        ],
        const SizedBox(height: 16),
        const Section(
          title: 'Device LED guide',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'D32 · GNSS: blue configuring, amber no fix, green valid fix, red communication fault',
              ),
              SizedBox(height: 8),
              Text(
                'D33 · IMU: amber initializing or recent sample error, green healthy, red sensor fault',
              ),
              SizedBox(height: 8),
              Text(
                'D34 · Recording: blue idle, blinking green recording, blinking amber saving, red storage fault',
              ),
              SizedBox(height: 8),
              Text(
                'D35 · Power: green charging, blue USB power, purple battery power, red low battery, off unknown',
              ),
            ],
          ),
        ),
        const SizedBox(height: 16),
        Section(
          title: 'Device recordings',
          trailing: OutlinedButton.icon(
            onPressed: c.busy || !c.connected || deviceRecording
                ? null
                : c.refreshDeviceSessions,
            icon: const Icon(Icons.refresh, size: 16),
            label: const Text('Refresh'),
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Text(
                'Download saved SD recordings into the session library. Stop recording before refreshing or downloading.',
                style: TextStyle(color: muted, fontSize: 12),
              ),
              if (c.activity != null)
                Padding(
                  padding: const EdgeInsets.only(top: 14),
                  child: Text(
                    c.activity!,
                    style: const TextStyle(color: accent),
                  ),
                ),
              if (c.deviceSessions.isEmpty && c.activity == null)
                Padding(
                  padding: const EdgeInsets.only(top: 18),
                  child: Text(
                    c.connected
                        ? 'Refresh to list recordings on the receiver.'
                        : 'Connect a receiver to view its recordings.',
                    style: const TextStyle(color: muted),
                  ),
                ),
              for (final recording in c.deviceSessions)
                Padding(
                  padding: const EdgeInsets.only(top: 16),
                  child: Wrap(
                    spacing: 16,
                    runSpacing: 10,
                    crossAxisAlignment: WrapCrossAlignment.center,
                    children: [
                      SizedBox(
                        width: 150,
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              'Recording ${recording['id']}',
                              style: const TextStyle(
                                fontWeight: FontWeight.w600,
                              ),
                            ),
                            Text(
                              '${number((numeric(recording['bytes']) ?? 0) / 1024, 1)} KiB · ${recording['closed'] == true ? 'Saved' : 'Incomplete · recoverable'}',
                              style: const TextStyle(
                                color: muted,
                                fontSize: 12,
                              ),
                            ),
                          ],
                        ),
                      ),
                      OutlinedButton.icon(
                        onPressed:
                            c.busy ||
                                !c.connected ||
                                deviceRecording ||
                                recording['id'] is! num
                            ? null
                            : () => c.downloadDeviceSession(
                                (recording['id'] as num).toInt(),
                              ),
                        icon: const Icon(Icons.download_outlined, size: 16),
                        label: const Text('Download & analyze'),
                      ),
                    ],
                  ),
                ),
            ],
          ),
        ),
        const SizedBox(height: 16),
        const Section(
          title: 'Bluetooth',
          child: Text(
            'Bluetooth transport is not available in this build. USB provides live telemetry and session recording.',
            style: TextStyle(color: muted),
          ),
        ),
      ],
    );
  }
}

class DragView extends StatefulWidget {
  const DragView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<DragView> createState() => _DragViewState();
}

class _DragViewState extends State<DragView> {
  final form = GlobalKey<FormState>();
  late final TextEditingController start, end, distance;
  bool rollout = false;
  @override
  void initState() {
    super.initState();
    final config = object(widget.controller.state['drag']);
    start = TextEditingController(text: number(config['start_kph'] ?? 0, 0));
    end = TextEditingController(text: number(config['end_kph'] ?? 100, 0));
    distance = TextEditingController(
      text: number(config['distance_m'] ?? 402.336, 3),
    );
    rollout = config['rollout'] == true;
  }

  @override
  void dispose() {
    start.dispose();
    end.dispose();
    distance.dispose();
    super.dispose();
  }

  String? positive(String? value, {bool zero = false}) {
    final n = double.tryParse(value ?? '');
    return n == null || !n.isFinite || (zero ? n < 0 : n <= 0)
        ? 'Enter a ${zero ? 'nonnegative' : 'positive'} number'
        : null;
  }

  @override
  Widget build(BuildContext context) {
    final c = widget.controller;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        const Text(
          'Set the speed interval and distance to measure in each run. Results appear in session analysis after recording.',
          style: TextStyle(color: muted),
        ),
        const SizedBox(height: 20),
        Section(
          title: 'Measurement configuration',
          child: Form(
            key: form,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: [
                    for (final preset in const [
                      [0, 100],
                      [100, 200],
                      [80, 120],
                      [60, 130],
                    ])
                      ActionChip(
                        label: Text('${preset[0]}–${preset[1]} km/h'),
                        onPressed: c.recording
                            ? null
                            : () => setState(() {
                                start.text = '${preset[0]}';
                                end.text = '${preset[1]}';
                              }),
                      ),
                    ActionChip(
                      label: const Text('0–60 mph'),
                      onPressed: c.recording
                          ? null
                          : () => setState(() {
                              start.text = '0';
                              end.text = '96.56064';
                            }),
                    ),
                  ],
                ),
                const SizedBox(height: 22),
                Wrap(
                  spacing: 16,
                  runSpacing: 16,
                  children: [
                    SizedBox(
                      width: 220,
                      child: TextFormField(
                        controller: start,
                        enabled: !c.recording,
                        keyboardType: const TextInputType.numberWithOptions(
                          decimal: true,
                        ),
                        decoration: const InputDecoration(
                          labelText: 'Starting speed',
                          suffixText: 'km/h',
                        ),
                        validator: (v) => positive(v, zero: true),
                      ),
                    ),
                    SizedBox(
                      width: 220,
                      child: TextFormField(
                        controller: end,
                        enabled: !c.recording,
                        keyboardType: const TextInputType.numberWithOptions(
                          decimal: true,
                        ),
                        decoration: const InputDecoration(
                          labelText: 'Ending speed',
                          suffixText: 'km/h',
                        ),
                        validator: (v) {
                          final error = positive(v);
                          if (error != null) return error;
                          return double.parse(v!) <=
                                  (double.tryParse(start.text) ?? 0)
                              ? 'Must exceed starting speed'
                              : null;
                        },
                      ),
                    ),
                    SizedBox(
                      width: 220,
                      child: TextFormField(
                        controller: distance,
                        enabled: !c.recording,
                        keyboardType: const TextInputType.numberWithOptions(
                          decimal: true,
                        ),
                        decoration: const InputDecoration(
                          labelText: 'Distance',
                          suffixText: 'm',
                        ),
                        validator: (v) => positive(v),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 14),
                Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: [
                    for (final preset in const {
                      '60 ft': 18.288,
                      '⅛ mile': 201.168,
                      '¼ mile': 402.336,
                      '½ mile': 804.672,
                      '1000 m': 1000.0,
                    }.entries)
                      ActionChip(
                        label: Text(preset.key),
                        onPressed: c.recording
                            ? null
                            : () => setState(
                                () => distance.text = preset.value.toString(),
                              ),
                      ),
                  ],
                ),
                const SizedBox(height: 16),
                SwitchListTile(
                  contentPadding: EdgeInsets.zero,
                  title: const Text('One-foot rollout'),
                  subtitle: const Text(
                    'Exclude the first 0.3048 m of a standing-start run.',
                    style: TextStyle(color: muted, fontSize: 12),
                  ),
                  value: rollout,
                  onChanged: c.recording
                      ? null
                      : (value) => setState(() => rollout = value),
                ),
                const SizedBox(height: 12),
                Align(
                  alignment: Alignment.centerLeft,
                  child: FilledButton(
                    onPressed: c.busy || c.recording
                        ? null
                        : () {
                            if (form.currentState!.validate())
                              c.setDrag({
                                'start_kph': double.parse(start.text),
                                'end_kph': double.parse(end.text),
                                'distance_m': double.parse(distance.text),
                                'rollout': rollout,
                              });
                          },
                    child: const Text('Save measurement configuration'),
                  ),
                ),
                if (c.recording)
                  const Padding(
                    padding: EdgeInsets.only(top: 12),
                    child: Text(
                      'Stop recording before changing the test configuration.',
                      style: TextStyle(color: warning),
                    ),
                  ),
              ],
            ),
          ),
        ),
        const SizedBox(height: 16),
        Section(
          title: 'Recording a run',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text(
                '1. Save the measurement configuration.\n2. Connect the receiver and wait for a valid GNSS fix.\n3. Start a recording in Live, then complete your runs.\n4. Stop and save. Open the session to inspect timing, speed and validity.',
                style: TextStyle(height: 1.9),
              ),
              const SizedBox(height: 18),
              OutlinedButton(
                onPressed: () => c.navigate(0),
                child: const Text('Open live telemetry'),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

class TracksView extends StatelessWidget {
  const TracksView({super.key, required this.controller});
  final AeviaController controller;
  @override
  Widget build(BuildContext context) {
    final c = controller;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Align(
          alignment: Alignment.centerLeft,
          child: FilledButton.icon(
            onPressed: c.busy ? null : () => editTrack(context, c),
            icon: const Icon(Icons.add_location_alt_outlined, size: 18),
            label: const Text('Create track'),
          ),
        ),
        const SizedBox(height: 20),
        if (c.tracks.isEmpty)
          const Section(
            title: 'Track library',
            child: EmptyState(
              'Add your first circuit',
              'Draw your start/finish and sector gates on the map, then select the track in Live to time laps.',
              icon: Icons.route,
            ),
          ),
        for (final track in c.tracks)
          Padding(
            padding: const EdgeInsets.only(bottom: 16),
            child: Section(
              title: track['name']?.toString() ?? 'Unnamed track',
              trailing: Text(
                '${objects(track['sectors']).length} sector gates',
                style: const TextStyle(color: muted, fontSize: 12),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  SelectableText(
                    'Start/finish  ${number(object(track['start'])['lat'], 6)}°, ${number(object(track['start'])['lon'], 6)}°\nTravel heading  ${number(object(track['start'])['heading_deg'], 0)}°   ·   Gate width  ${number(object(track['start'])['width_m'], 0)} m',
                    style: const TextStyle(color: muted, height: 1.8),
                  ),
                  const SizedBox(height: 16),
                  Wrap(
                    spacing: 10,
                    children: [
                      OutlinedButton(
                        onPressed: c.busy
                            ? null
                            : () => editTrack(context, c, track: track),
                        child: const Text('Edit track'),
                      ),
                      TextButton(
                        onPressed: c.busy
                            ? null
                            : () async {
                                final yes = await showDialog<bool>(
                                  context: context,
                                  builder: (context) => AlertDialog(
                                    title: const Text('Delete track?'),
                                    content: Text(
                                      'Remove ${track['name']} from the track library? Recorded sessions keep their own track definition.',
                                    ),
                                    actions: [
                                      TextButton(
                                        onPressed: () =>
                                            Navigator.pop(context, false),
                                        child: const Text('Cancel'),
                                      ),
                                      FilledButton(
                                        onPressed: () =>
                                            Navigator.pop(context, true),
                                        child: const Text('Delete track'),
                                      ),
                                    ],
                                  ),
                                );
                                if (yes == true)
                                  c.deleteTrack(track['id'].toString());
                              },
                        child: const Text('Delete'),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
      ],
    );
  }
}
