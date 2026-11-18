import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';
import 'charts.dart';

class LiveView extends StatefulWidget {
  const LiveView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<LiveView> createState() => _LiveViewState();
}

class _LiveViewState extends State<LiveView> {
  String? trackId;
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
    final delta = c.stale ? null : numeric(live['delta_s']);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Wrap(
          spacing: 10,
          runSpacing: 10,
          crossAxisAlignment: WrapCrossAlignment.center,
          children: [
            Status(
              c.connected
                  ? c.stale
                        ? 'STALE · WAITING FOR DATA'
                        : good
                        ? 'LIVE · VALID FIX'
                        : 'CONNECTED · NO VALID FIX'
                  : 'DISCONNECTED',
              color: good
                  ? accent
                  : c.connected
                  ? warning
                  : muted,
            ),
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
          child: MetricGrid(
            minimumWidth: 200,
            children: [
              Metric(
                'Speed',
                good ? number((numeric(live['speed_mps']) ?? 0) * 3.6, 0) : '—',
                unit: 'km/h',
                large: true,
                note: good
                    ? 'GNSS speed'
                    : !c.connected
                    ? 'Waiting for receiver'
                    : c.stale
                    ? 'Last fix is stale'
                    : 'Waiting for a valid fix',
              ),
              Metric(
                'Current lap',
                elapsed(c.stale ? null : live['current_lap_s']),
                large: true,
                note: 'Lap ${live['lap'] ?? '—'}',
              ),
              Metric(
                'Delta to best',
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
                    ? 'Predictive timing unavailable'
                    : delta <= 0
                    ? 'Ahead of reference'
                    : 'Behind reference',
              ),
            ],
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
                  FilledButton.icon(
                    onPressed: c.busy || !c.connected
                        ? null
                        : c.recording
                        ? c.stop
                        : () => c.start(
                            name: name.text.trim().isEmpty
                                ? null
                                : name.text.trim(),
                            trackId: trackId?.isEmpty == true ? null : trackId,
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
            Metric('Best lap', elapsed(live['best_lap_s'])),
            Metric(
              'Longitudinal',
              good ? number(live['accel_long_g'], 2) : '—',
              unit: 'g',
            ),
            Metric(
              'Lateral',
              good ? number(live['accel_lat_g'], 2) : '—',
              unit: 'g',
            ),
            Metric(
              'Altitude',
              good ? number(live['altitude_m']) : '—',
              unit: 'm',
            ),
            Metric(
              'Heading',
              good ? number(live['heading_deg'], 0) : '—',
              unit: '°',
            ),
            Metric(
              'Receiver battery',
              c.stale ? '—' : number(c.device['battery_percent'], 0),
              unit: '%',
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
              child: TraceChart(samples: List.of(c.recent), cursor: 1),
            );
            final map = Section(
              title: 'Live position',
              child: TrackMap(
                samples: List.of(c.recent),
                cursor: 1,
                height: 220,
              ),
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
}
