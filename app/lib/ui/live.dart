import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';
import 'charts.dart';
import 'track_editor.dart';

class LiveView extends StatefulWidget {
  const LiveView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<LiveView> createState() => _LiveViewState();
}

class _LiveViewState extends State<LiveView> {
  String? trackId;
  bool fullRate = false;
  final name = TextEditingController();
  @override
  void dispose() {
    name.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final c = widget.controller, live = widget.controller.live;
    final good = c.valid;
    final drag = object(live['drag_config']);
    final speedTarget = drag.isEmpty
        ? 'Speed target'
        : "${number(drag['start_kph'], 0)}–${number(drag['end_kph'], 0)} km/h";
    final distanceTarget = drag.isEmpty
        ? 'Distance target'
        : "${number(drag['distance_m'], 3)} m";
    final delta = c.stale ? null : numeric(live['delta_s']);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Wrap(
          spacing: 10,
          runSpacing: 10,
          crossAxisAlignment: WrapCrossAlignment.center,
          children: [
            if (c.device['recording'] == true && c.bluetooth && !c.recording)
              OutlinedButton(
                onPressed: c.busy ? null : c.stop,
                child: const Text('Stop receiver recording'),
              ),
            Status(
              c.connected
                  ? c.stale
                        ? 'STALE · WAITING FOR DATA'
                        : good
                        ? switch (live['motion_source']) {
                            'gnss' => 'LIVE · GPS ONLY',
                            'inertial' => 'LIVE · IMU COASTING',
                            'held' => 'LIVE · LAST READING',
                            'position_degraded' =>
                              'FUSED · LOW POSITION ACCURACY',
                            _ =>
                              live['degraded'] == true
                                  ? 'LIVE · LIMITED'
                                  : 'LIVE · FUSED',
                          }
                        : 'CONNECTED · WAITING FOR MOTION'
                  : 'DISCONNECTED',
              color: good
                  ? accent
                  : c.connected
                  ? warning
                  : muted,
            ),
            if (c.receiverFresh && live['imu_clipping'] == true)
              const Status('IMU CLIPPING', color: warning),
            Text(
              c.connected
                  ? '${c.device['satellites'] ?? '—'} satellites  ·  ${number(c.device['gnss_hz'], 1)} Hz  ·  ${c.device['fix_quality'] ?? 'Unknown fix'}'
                  : 'Connect your V2 Mini to receive live telemetry.',
              style: const TextStyle(color: muted),
            ),
            if (!c.connected)
              OutlinedButton.icon(
                onPressed: () => c.navigate(5),
                icon: const Icon(Icons.usb, size: 16),
                label: const Text('Connect receiver'),
              ),
          ],
        ),
        if (c.captureInterruption != null)
          Padding(
            padding: const EdgeInsets.only(top: 12),
            child: Text(
              '${c.captureInterruption} Stop and save to retain the received session.',
              style: const TextStyle(color: warning),
            ),
          ),
        const SizedBox(height: 20),
        Section(
          title: 'Driving display',
          trailing: Text(
            c.captureInterruption != null
                ? 'CAPTURE STOPPED'
                : c.recording
                ? 'SESSION ${elapsed(live['elapsed_s'])}'
                : c.connected
                ? 'READY TO RECORD'
                : 'RECEIVER REQUIRED',
            style: TextStyle(
              color: c.captureInterruption != null
                  ? warning
                  : c.recording
                  ? accent
                  : muted,
              fontSize: 11,
            ),
          ),
          child: LayoutBuilder(
            builder: (context, box) {
              final metrics = MetricGrid(
                minimumWidth: 185,
                children: [
                  Metric(
                    'Speed',
                    good ? number(numeric(live['speed_mps'])! * 3.6, 0) : '—',
                    unit: 'km/h',
                    large: true,
                    note: good
                        ? 'Fused horizontal speed'
                        : !c.connected
                        ? 'Waiting for receiver'
                        : c.stale
                        ? 'Estimate is stale'
                        : 'Waiting for a fused estimate',
                  ),
                  Metric(
                    'Current lap',
                    elapsed(c.stale ? null : live['current_lap_s']),
                    large: true,
                    note: 'Lap ${live['lap'] ?? '—'}',
                  ),
                  Metric(
                    'Last lap vs best',
                    delta == null
                        ? '—'
                        : '${delta >= 0 ? '+' : ''}${number(delta, 3)}',
                    unit: 's',
                    large: true,
                    color: delta == null
                        ? ink
                        : delta <= 0
                        ? accent
                        : warning,
                    note: delta == null
                        ? 'Waiting for a completed lap'
                        : delta <= 0
                        ? 'Best lap'
                        : 'Behind best',
                  ),
                ],
              );
              final meter = LiveGMeter(
                acceleration: c.vehicleAccelerationG,
                connected: c.connected,
                trail: c.gTrail,
              );
              if (box.maxWidth >= 820) {
                return Row(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Expanded(child: metrics),
                    const SizedBox(width: 16),
                    SizedBox(width: 236, child: meter),
                  ],
                );
              }
              return Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [metrics, const SizedBox(height: 12), meter],
              );
            },
          ),
        ),
        const SizedBox(height: 16),
        Section(
          title: 'Session controls',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Wrap(
                spacing: 12,
                runSpacing: 12,
                crossAxisAlignment: WrapCrossAlignment.center,
                children: [
                  SizedBox(
                    width: 240,
                    child: TextField(
                      controller: name,
                      enabled: !c.recording,
                      decoration: const InputDecoration(
                        labelText: 'Session name',
                        hintText: 'Optional',
                      ),
                    ),
                  ),
                  SizedBox(
                    width: 250,
                    child: DropdownButtonFormField<String>(
                      key: ValueKey(trackId),
                      initialValue: trackId,
                      isExpanded: true,
                      decoration: const InputDecoration(labelText: 'Track'),
                      items: [
                        const DropdownMenuItem(
                          value: '',
                          child: Text('No track · telemetry only'),
                        ),
                        ...c.tracks.map(
                          (t) => DropdownMenuItem(
                            value: t['id']?.toString(),
                            child: Text(
                              t['name']?.toString() ?? 'Unnamed track',
                              overflow: TextOverflow.ellipsis,
                            ),
                          ),
                        ),
                      ],
                      onChanged: c.recording
                          ? null
                          : (value) => setState(() => trackId = value),
                    ),
                  ),
                  if (!c.recording)
                    OutlinedButton.icon(
                      onPressed: c.busy
                          ? null
                          : () async {
                              final id = await editTrack(context, c);
                              if (mounted && id != null) {
                                setState(() => trackId = id);
                              }
                            },
                      icon: const Icon(
                        Icons.add_location_alt_outlined,
                        size: 18,
                      ),
                      label: const Text('Create track on map'),
                    ),
                  FilledButton.icon(
                    onPressed: c.busy || (!c.connected && !c.recording)
                        ? null
                        : c.recording
                        ? c.stop
                        : () => c.start(
                            name: name.text.trim().isEmpty
                                ? null
                                : name.text.trim(),
                            trackId: trackId?.isEmpty == true ? null : trackId,
                            fullRate: fullRate,
                          ),
                    icon: Icon(
                      c.recording ? Icons.stop : Icons.fiber_manual_record,
                      size: 16,
                    ),
                    label: Text(
                      c.recording ? 'Stop & save session' : 'Start recording',
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 12),
              if (c.device['display_v2'] == true)
                CheckboxListTile(
                  value: c.device['mounted'] == true,
                  onChanged: c.storageBusy || c.busy
                      ? null
                      : (v) => c.confirmMount(v == true),
                  title: const Text('Vehicle axes aligned'),
                  subtitle: const Text(
                    'Confirm the configured mounting has +X forward, +Y left and +Z up to enable vehicle G and heading.',
                  ),
                ),
              if (c.bluetooth)
                const Text(
                  'Recording saves full-rate data on the receiver SD card and a 5 Hz session in the app.',
                )
              else if (c.device['display_v2'] == true ||
                  c.device['display_v1'] == true)
                CheckboxListTile(
                  value: c.recording
                      ? live['full_rate_copy'] == true
                      : fullRate,
                  onChanged: c.recording || c.busy
                      ? null
                      : (v) => setState(() => fullRate = v == true),
                  title: const Text('Copy full-rate raw data over USB'),
                  subtitle: const Text(
                    'The receiver records full-rate data to SD. By default the app saves a 5 Hz session for live timing and replay.',
                  ),
                )
              else
                const Text(
                  'Recording keeps raw receiver evidence. A saved track enables lap and sector timing.',
                  style: TextStyle(color: muted, fontSize: 12),
                ),
            ],
          ),
        ),
        const SizedBox(height: 16),
        MetricGrid(
          minimumWidth: 150,
          children: [
            Metric(
              speedTarget,
              elapsed(live['drag_speed_s']),
              note: drag['rollout'] == true
                  ? 'Estimated · 1 ft rollout'
                  : 'Estimated time',
            ),
            Metric(
              distanceTarget,
              elapsed(live['drag_distance_s']),
              note: 'Estimated time',
            ),
            Metric('Last sector', elapsed(live['last_sector_s'])),
            Metric(
              'Peak speed',
              number(
                numeric(live['max_speed_mps']) == null
                    ? null
                    : numeric(live['max_speed_mps'])! * 3.6,
                1,
              ),
              unit: 'km/h',
            ),
            Metric('Best lap', elapsed(live['best_lap_s'])),
            Metric('Last lap', elapsed(live['last_lap_s'])),
            Metric(
              'Session distance',
              number(live['distance_m'], 0),
              unit: 'm',
            ),
            Metric(
              'Average speed',
              number(
                numeric(live['average_speed_mps']) == null
                    ? null
                    : numeric(live['average_speed_mps'])! * 3.6,
                1,
              ),
              unit: 'km/h',
            ),
            Metric(
              'Altitude',
              good ? number(live['altitude_m']) : '—',
              unit: 'm',
            ),
            Metric(
              'Course over ground',
              good ? number(live['course_deg'], 0) : '—',
              unit: '°',
            ),
            Metric(
              'Receiver battery',
              number(c.batteryPercent, 0),
              unit: '%',
              note: _batteryNote(c),
            ),
          ],
        ),
        const SizedBox(height: 16),
        LayoutBuilder(
          builder: (context, box) {
            final chart = Section(
              title: 'Recent speed',
              trailing: const Text(
                'km/h · last 300 s maximum',
                style: TextStyle(color: muted, fontSize: 11),
              ),
              child: TraceChart(samples: c.recent, cursor: 1),
            );
            final map = Section(
              title: 'Live position',
              child: TrackMap(samples: c.recent, cursor: 1, height: 220),
            );
            return box.maxWidth > 950
                ? Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Expanded(flex: 3, child: chart),
                      const SizedBox(width: 16),
                      Expanded(flex: 2, child: map),
                    ],
                  )
                : Column(children: [chart, const SizedBox(height: 16), map]);
          },
        ),
      ],
    );
  }

  String? _batteryNote(AeviaController c) {
    if (!c.receiverFresh) return null;
    final full = estimatedBatteryTime(c.batteryTimeToFull);
    final left = estimatedBatteryTime(c.batteryTimeLeft);
    final label = switch (c.batteryState) {
      'charging' => 'Charging',
      'bypass'
          when c.batteryPercent == 100 &&
              c.battery['charge_stage'] == 'not_charging' =>
        'Fully charged',
      'bypass' => 'External power',
      'low' => 'Low battery',
      'battery' => 'On battery',
      _ => null,
    };
    final parts = [
      if (label != null) label,
      if (c.battery['power_source'] == 'external' &&
          c.temperatureZoneLabel != null &&
          c.battery['temperature_zone'] != 'normal')
        '${c.temperatureZoneLabel} · full-charge ETA unavailable',
      if (signedBatteryRate(c.batteryRate) case final rate?) rate,
      if (full != null) '$full to full at this rate',
      if (left != null) '$left left',
    ];
    if (parts.isEmpty) {
      return c.bluetooth && c.batteryPercent != null
          ? 'Waiting for discharge trend'
          : null;
    }
    return parts.join(' · ');
  }
}
