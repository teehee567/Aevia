import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';
import 'charts.dart';
import 'replay.dart';
import 'sessions.dart';
import 'track_editor.dart';

class AnalysisView extends StatefulWidget {
  const AnalysisView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<AnalysisView> createState() => _AnalysisViewState();
}

class _AnalysisViewState extends State<AnalysisView> {
  double cursor = 0;
  double playbackRate = 1;
  int lapIndex = -1;
  int referenceIndex = -1;
  String channel = 'speed_mps';
  String? sessionId;
  String? trackId;
  Timer? replay;
  bool get playing => replay?.isActive == true;
  @override
  void dispose() {
    replay?.cancel();
    super.dispose();
  }

  void togglePlay(double duration) {
    if (playing) {
      replay?.cancel();
      setState(() {});
      return;
    }
    if (cursor >= 1) cursor = 0;
    replay = Timer.periodic(const Duration(milliseconds: 50), (_) {
      setState(() {
        cursor = math.min(
          1,
          cursor + .05 * playbackRate / math.max(duration, .05),
        );
        if (cursor >= 1) replay?.cancel();
      });
    });
    setState(() {});
  }

  void seek(double value) {
    setState(() => cursor = value);
  }

  List<Map<String, dynamic>> lapSamples(
    List<Map<String, dynamic>> all,
    List<Map<String, dynamic>> laps,
    int index,
  ) {
    if (index < 0 || index >= laps.length) return all;
    final from = numeric(laps[index]['start_s']) ?? 0,
        to = numeric(laps[index]['end_s']) ?? 0;
    return all
        .where(
          (sample) =>
              (numeric(sample['time_s']) ?? -1) >= from &&
              (numeric(sample['time_s']) ?? double.infinity) <= to,
        )
        .toList();
  }

  @override
  Widget build(BuildContext context) {
    final c = widget.controller;
    final session = c.session;
    if (session == null)
      return Section(
        title: 'Analysis workspace',
        child: EmptyState(
          'Select a session to begin',
          'Inspect the recorded line, replay samples, compare laps and review drag measurements.',
          action: Wrap(
            spacing: 12,
            runSpacing: 12,
            children: [
              FilledButton(
                onPressed: () => c.navigate(1),
                child: const Text('Open session library'),
              ),
              OutlinedButton(
                onPressed: c.busy ? null : c.demo,
                child: const Text('Explore demo'),
              ),
            ],
          ),
        ),
      );
    if (sessionId != session['id']) {
      sessionId = session['id']?.toString();
      cursor = 0;
      lapIndex = -1;
      referenceIndex = -1;
      replay?.cancel();
      trackId = object(session['track'])['id']?.toString();
    }
    final analysis = object(session['analysis']);
    final provenance = object(session['provenance']);
    final laps = objects(analysis['laps']);
    if (lapIndex >= laps.length) lapIndex = -1;
    if (referenceIndex >= laps.length) referenceIndex = -1;
    final all = objects(session['samples']);
    final samples = lapSamples(all, laps, lapIndex);
    final reference = referenceIndex < 0
        ? <Map<String, dynamic>>[]
        : lapSamples(all, laps, referenceIndex);
    final start = samples.isEmpty ? 0.0 : numeric(samples.first['time_s']) ?? 0;
    final duration = samples.isEmpty
        ? 0.0
        : (numeric(samples.last['time_s']) ?? start) - start;
    final referenceDuration = reference.isEmpty
        ? 0.0
        : (numeric(reference.last['time_s']) ?? 0) -
              (numeric(reference.first['time_s']) ?? 0);
    final displayDuration = math.max(duration, referenceDuration);
    final time = start + cursor * displayDuration;
    final selection = replaySelection(samples, time);
    final current = selection.valid ? selection.sample! : <String, dynamic>{};
    final mapCursor = !selection.valid
        ? null
        : samples.length < 2
        ? 0.0
        : selection.index! / (samples.length - 1);
    final demo =
        session['source']?.toString().toLowerCase().contains('demo') == true;
    final warnings = analysis['warnings'] is List
        ? (analysis['warnings'] as List).map((e) => e.toString()).toList()
        : <String>[];
    final selectedChannel = const {
      'speed_mps': ['Speed', 'km/h', 3.6],
      'accel_long_g': ['Longitudinal acceleration', 'g', 1.0],
      'accel_lat_g': ['Lateral acceleration', 'g', 1.0],
      'altitude_m': ['Altitude', 'm', 1.0],
    }[channel]!;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Wrap(
          spacing: 12,
          runSpacing: 12,
          crossAxisAlignment: WrapCrossAlignment.center,
          children: [
            SizedBox(
              width: 320,
              child: DropdownButtonFormField<String>(
                key: ValueKey(sessionId),
                initialValue: c.sessions.any((s) => s['id'] == sessionId)
                    ? sessionId
                    : null,
                isExpanded: true,
                decoration: const InputDecoration(labelText: 'Session'),
                items: c.sessions
                    .map(
                      (s) => DropdownMenuItem(
                        value: s['id']?.toString(),
                        child: Text(
                          s['name']?.toString() ?? '',
                          overflow: TextOverflow.ellipsis,
                        ),
                      ),
                    )
                    .toList(),
                onChanged: c.busy
                    ? null
                    : (id) {
                        if (id != null) c.openSession(id);
                      },
              ),
            ),
            OutlinedButton.icon(
              onPressed: c.busy
                  ? null
                  : () => showExportDialog(context, c, session),
              icon: const Icon(Icons.file_download_outlined, size: 17),
              label: const Text('Export session'),
            ),
            Status(
              demo
                  ? 'SYNTHETIC DEMO'
                  : session['source']?.toString().toUpperCase() ?? 'RECORDED',
              color: demo ? warning : muted,
            ),
          ],
        ),
        const SizedBox(height: 12),
        Text(
          '${sessionDate(session['created_ms'])}  ·  ${session['sample_count'] ?? all.length} samples  ·  ${analysis['valid_samples'] ?? '—'} valid',
          style: const TextStyle(color: muted, fontSize: 12),
        ),
        if (demo)
          const Padding(
            padding: EdgeInsets.only(top: 12),
            child: Text(
              'Demonstration data. These results are generated for exploring the analysis tools.',
              style: TextStyle(color: warning, fontSize: 12),
            ),
          ),
        for (final warningText in warnings)
          Padding(
            padding: const EdgeInsets.only(top: 8),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Icon(Icons.info_outline, color: warning, size: 15),
                const SizedBox(width: 8),
                Expanded(
                  child: Text(
                    warningText,
                    style: const TextStyle(color: muted, fontSize: 12),
                  ),
                ),
              ],
            ),
          ),
        const SizedBox(height: 18),
        if (provenance['kind'] == 'ppk') ...[
          Section(
            title: 'PPK solution quality',
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Wrap(
                  spacing: 18,
                  runSpacing: 8,
                  children: [
                    for (final quality in object(
                      provenance['quality_counts'],
                    ).entries)
                      Text('${quality.key}: ${quality.value} epochs'),
                  ],
                ),
                const SizedBox(height: 8),
                const Text(
                  'Fixed and float solutions have different uncertainty. Inspect the solution report before using the result for precision measurements.',
                  style: TextStyle(color: muted),
                ),
                if (provenance['parent_session_id'] != null)
                  SelectableText(
                    'Source session: ${provenance['parent_session_id']}',
                  ),
                if (provenance['solution_file'] != null)
                  SelectableText(
                    'Solution: ${object(provenance['solution_file'])['path'] ?? provenance['solution_file']}',
                  ),
              ],
            ),
          ),
          const SizedBox(height: 18),
        ],
        MetricGrid(
          minimumWidth: 145,
          children: [
            Metric('Duration', elapsed(session['duration_s'])),
            Metric(
              'Distance',
              number(
                provenance['kind'] == 'ppk'
                    ? null
                    : (numeric(session['distance_m']) ?? 0) / 1000,
                2,
              ),
              unit: 'km',
            ),
            Metric(
              'Maximum speed',
              number(
                numeric(session['max_speed_mps']) == null
                    ? null
                    : numeric(session['max_speed_mps'])! * 3.6,
                1,
              ),
              unit: 'km/h',
            ),
            Metric('Best lap', elapsed(analysis['best_lap_s'])),
            Metric('Theoretical best', elapsed(analysis['theoretical_best_s'])),
            Metric('Average lap', elapsed(analysis['average_lap_s'])),
          ],
        ),
        const SizedBox(height: 18),
        Section(
          title: 'Replay & comparison',
          trailing: const Text(
            'Elapsed-time alignment',
            style: TextStyle(color: muted, fontSize: 11),
          ),
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
                    child: DropdownButtonFormField<int>(
                      isExpanded: true,
                      initialValue: lapIndex,
                      key: ValueKey('lap-$sessionId-$lapIndex'),
                      decoration: const InputDecoration(
                        labelText: 'Trace · mint',
                      ),
                      items: [
                        const DropdownMenuItem(
                          value: -1,
                          child: Text('Complete session'),
                        ),
                        for (int i = 0; i < laps.length; i++)
                          DropdownMenuItem(
                            value: i,
                            child: Text(
                              'Lap ${laps[i]['number']} · ${elapsed(laps[i]['duration_s'])}',
                            ),
                          ),
                      ],
                      onChanged: (value) => setState(() {
                        lapIndex = value ?? -1;
                        cursor = 0;
                        replay?.cancel();
                      }),
                    ),
                  ),
                  SizedBox(
                    width: 240,
                    child: DropdownButtonFormField<int>(
                      isExpanded: true,
                      initialValue: referenceIndex,
                      key: ValueKey('ref-$sessionId-$referenceIndex'),
                      decoration: const InputDecoration(
                        labelText: 'Reference · amber',
                      ),
                      items: [
                        const DropdownMenuItem(
                          value: -1,
                          child: Text('No reference'),
                        ),
                        for (int i = 0; i < laps.length; i++)
                          DropdownMenuItem(
                            value: i,
                            child: Text(
                              'Lap ${laps[i]['number']} · ${elapsed(laps[i]['duration_s'])}',
                            ),
                          ),
                      ],
                      onChanged: (value) =>
                          setState(() => referenceIndex = value ?? -1),
                    ),
                  ),
                  if (lapIndex >= 0 && referenceIndex >= 0)
                    Status(
                      'Δ ${(numeric(laps[lapIndex]['duration_s'])! - numeric(laps[referenceIndex]['duration_s'])!) >= 0 ? '+' : ''}${number(numeric(laps[lapIndex]['duration_s'])! - numeric(laps[referenceIndex]['duration_s'])!, 3)} s',
                      color:
                          numeric(laps[lapIndex]['duration_s'])! <=
                              numeric(laps[referenceIndex]['duration_s'])!
                          ? accent
                          : warning,
                    ),
                ],
              ),
              const SizedBox(height: 16),
              LayoutBuilder(
                builder: (context, box) {
                  final map = TrackMap(
                    samples: samples,
                    reference: reference,
                    cursor: mapCursor,
                    track: object(session['track']),
                  );
                  final telemetry = Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      MetricGrid(
                        minimumWidth: 120,
                        children: [
                          Metric('Replay time', elapsed(time - start)),
                          Metric(
                            'Speed',
                            current['valid'] == false
                                ? '—'
                                : number(
                                    numeric(current['speed_mps']) == null
                                        ? null
                                        : numeric(current['speed_mps'])! * 3.6,
                                    1,
                                  ),
                            unit: 'km/h',
                          ),
                          Metric(
                            'Longitudinal',
                            number(current['accel_long_g'], 2),
                            unit: 'g',
                          ),
                          Metric(
                            'Lateral',
                            number(current['accel_lat_g'], 2),
                            unit: 'g',
                          ),
                        ],
                      ),
                      const SizedBox(height: 10),
                      Status(
                        selection.status,
                        color: selection.valid ? accent : warning,
                      ),
                      if (current['ppk_quality'] != null) ...[
                        const SizedBox(height: 8),
                        Status(
                          'PPK ${object(current['ppk_quality'])['status']?.toString().toUpperCase() ?? 'UNKNOWN'}',
                          color: object(current['ppk_quality'])['q'] == 1
                              ? accent
                              : warning,
                        ),
                        const SizedBox(height: 6),
                        Text(
                          'Reported σ N/E/U: ${number(object(current['ppk_quality'])['sd_n_m'], 3)} / ${number(object(current['ppk_quality'])['sd_e_m'], 3)} / ${number(object(current['ppk_quality'])['sd_u_m'], 3)} m · Ratio ${number(object(current['ppk_quality'])['ratio'], 1)}',
                          style: const TextStyle(color: muted, fontSize: 11),
                        ),
                      ],
                      if (selection.valid)
                        Padding(
                          padding: const EdgeInsets.only(top: 6),
                          child: Text(
                            'Observed sample at ${elapsed(numeric(current['time_s'])! - start)}',
                            style: const TextStyle(color: muted, fontSize: 11),
                          ),
                        ),
                      const SizedBox(height: 14),
                      Text(
                        'Position  ${number(current['lat'], 6)}°, ${number(current['lon'], 6)}°\nAltitude  ${number(current['altitude_m'])} m\nHeading  ${number(current['heading_deg'], 0)}°',
                        style: const TextStyle(
                          color: muted,
                          fontSize: 12,
                          height: 1.8,
                        ),
                      ),
                      const SizedBox(height: 10),
                      OutlinedButton.icon(
                        onPressed:
                            current['valid'] == true &&
                                numeric(current['lat']) != null
                            ? () => editTrack(context, c, position: current)
                            : null,
                        icon: const Icon(
                          Icons.add_location_alt_outlined,
                          size: 17,
                        ),
                        label: const Text('Create track at cursor'),
                      ),
                    ],
                  );
                  return box.maxWidth > 850
                      ? Row(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Expanded(flex: 3, child: map),
                            const SizedBox(width: 24),
                            Expanded(flex: 2, child: telemetry),
                          ],
                        )
                      : Column(
                          children: [
                            map,
                            const SizedBox(height: 16),
                            telemetry,
                          ],
                        );
                },
              ),
              const SizedBox(height: 16),
              Row(
                children: [
                  IconButton(
                    tooltip: playing ? 'Pause replay' : 'Play replay',
                    onPressed: samples.isEmpty
                        ? null
                        : () => togglePlay(displayDuration),
                    icon: Icon(
                      playing
                          ? Icons.pause_circle_outline
                          : Icons.play_circle_outline,
                      size: 28,
                    ),
                  ),
                  Expanded(
                    child: Slider(
                      value: cursor,
                      onChanged: samples.isEmpty ? null : seek,
                      label: elapsed(displayDuration * cursor),
                    ),
                  ),
                  DropdownButton<double>(
                    value: playbackRate,
                    items: const [
                      DropdownMenuItem(value: 1, child: Text('1×')),
                      DropdownMenuItem(value: 2, child: Text('2×')),
                      DropdownMenuItem(value: 4, child: Text('4×')),
                      DropdownMenuItem(value: 10, child: Text('10×')),
                    ],
                    onChanged: (v) => setState(() => playbackRate = v ?? 1),
                  ),
                  const SizedBox(width: 12),
                  Text(
                    elapsed(displayDuration),
                    style: const TextStyle(color: muted, fontSize: 12),
                  ),
                ],
              ),
              const Divider(),
              const SizedBox(height: 16),
              Wrap(
                spacing: 8,
                runSpacing: 8,
                children: [
                  for (final item in const {
                    'speed_mps': 'Speed',
                    'accel_long_g': 'Acceleration',
                    'accel_lat_g': 'Lateral g',
                    'altitude_m': 'Altitude',
                  }.entries)
                    ChoiceChip(
                      label: Text(item.value),
                      selected: channel == item.key,
                      onSelected: (_) => setState(() => channel = item.key),
                    ),
                ],
              ),
              const SizedBox(height: 18),
              TraceChart(
                samples: samples,
                reference: reference,
                channel: channel,
                unit: selectedChannel[1] as String,
                factor: selectedChannel[2] as double,
                cursor: cursor,
                onSeek: seek,
                height: 240,
              ),
              const SizedBox(height: 8),
              const Text(
                'Charts break at invalid samples or gaps over 1.5 s. The cursor is shared with the position replay.',
                style: TextStyle(color: muted, fontSize: 11),
              ),
            ],
          ),
        ),
        const SizedBox(height: 16),
        Section(
          title: 'Lap & sector timing',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Wrap(
                spacing: 12,
                runSpacing: 12,
                crossAxisAlignment: WrapCrossAlignment.center,
                children: [
                  SizedBox(
                    width: 280,
                    child: DropdownButtonFormField<String>(
                      isExpanded: true,
                      key: ValueKey('track-$sessionId-$trackId'),
                      initialValue: c.tracks.any((t) => t['id'] == trackId)
                          ? trackId
                          : '',
                      decoration: const InputDecoration(
                        labelText: 'Track definition',
                      ),
                      items: [
                        const DropdownMenuItem(
                          value: '',
                          child: Text('No track'),
                        ),
                        ...c.tracks.map(
                          (t) => DropdownMenuItem(
                            value: t['id']?.toString(),
                            child: Text(t['name']?.toString() ?? ''),
                          ),
                        ),
                      ],
                      onChanged: (value) => setState(() => trackId = value),
                    ),
                  ),
                  OutlinedButton(
                    onPressed: c.busy
                        ? null
                        : () => c.reanalyze(sessionId!, trackId),
                    child: const Text('Apply track & reanalyze'),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              if (laps.isEmpty)
                const EmptyState(
                  'No complete laps',
                  'Select a track with a start/finish gate and reanalyze. Two valid crossings are required for a complete lap.',
                  icon: Icons.flag_outlined,
                )
              else
                SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: DataTable(
                    showCheckboxColumn: false,
                    columnSpacing: 25,
                    horizontalMargin: 8,
                    columns: [
                      const DataColumn(label: Text('Lap')),
                      const DataColumn(label: Text('Time'), numeric: true),
                      const DataColumn(label: Text('To best'), numeric: true),
                      ...List.generate(
                        laps.fold<int>(
                          0,
                          (max, lap) => math.max(
                            max,
                            (lap['sectors_s'] as List? ?? []).length,
                          ),
                        ),
                        (i) => DataColumn(
                          label: Text('Sector ${i + 1}'),
                          numeric: true,
                        ),
                      ),
                      const DataColumn(label: Text('Status')),
                    ],
                    rows: laps.asMap().entries.map((entry) {
                      final lap = entry.value,
                          sectors = lap['sectors_s'] as List? ?? [];
                      final best = numeric(analysis['best_lap_s']),
                          lapTime = numeric(lap['duration_s']);
                      final count = laps.fold<int>(
                        0,
                        (max, lap) => math.max(
                          max,
                          (lap['sectors_s'] as List? ?? []).length,
                        ),
                      );
                      return DataRow(
                        selected: entry.key == lapIndex,
                        onSelectChanged: (_) => setState(() {
                          lapIndex = entry.key;
                          cursor = 0;
                        }),
                        cells: [
                          DataCell(Text('${lap['number']}')),
                          DataCell(
                            Text(
                              elapsed(lapTime),
                              style: TextStyle(
                                color: lapTime == best ? accent : ink,
                              ),
                            ),
                          ),
                          DataCell(
                            Text(
                              best == null || lapTime == null
                                  ? '—'
                                  : '+${number(lapTime - best, 3)}',
                            ),
                          ),
                          ...List.generate(
                            count,
                            (i) => DataCell(
                              Text(
                                i < sectors.length ? elapsed(sectors[i]) : '—',
                              ),
                            ),
                          ),
                          DataCell(
                            Text(
                              lap['valid'] == true ? 'Valid' : 'Invalid',
                              style: TextStyle(
                                color: lap['valid'] == true ? muted : warning,
                              ),
                            ),
                          ),
                        ],
                      );
                    }).toList(),
                  ),
                ),
            ],
          ),
        ),
        const SizedBox(height: 16),
        LayoutBuilder(
          builder: (context, box) {
            final drag = Section(
              title: 'Drag measurements',
              child: DragResults(session: session),
            );
            final gg = Section(
              title: 'G-G diagram',
              child: GGPlot(samples: samples),
            );
            return box.maxWidth > 1000
                ? Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Expanded(flex: 3, child: drag),
                      const SizedBox(width: 16),
                      Expanded(flex: 2, child: gg),
                    ],
                  )
                : Column(children: [drag, const SizedBox(height: 16), gg]);
          },
        ),
        const SizedBox(height: 14),
        Text(
          'Analysis: ${analysis['engine'] ?? 'Unavailable'}',
          style: const TextStyle(color: muted, fontSize: 11),
        ),
      ],
    );
  }
}

class DragResults extends StatelessWidget {
  const DragResults({super.key, required this.session});
  final Map<String, dynamic> session;
  @override
  Widget build(BuildContext context) {
    final runs = objects(object(session['analysis'])['drag_runs']),
        config = object(session['drag']);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Text(
          '${number(config['start_kph'], 0)}–${number(config['end_kph'], 0)} km/h  ·  ${number(config['distance_m'], 3)} m  ·  ${config['rollout'] == true ? '1 ft rollout' : 'No rollout'}',
          style: const TextStyle(color: muted, fontSize: 12),
        ),
        const SizedBox(height: 16),
        if (runs.isEmpty)
          const EmptyState(
            'No completed drag tests',
            'A valid run must cross the configured start and end thresholds.',
          )
        else
          SingleChildScrollView(
            scrollDirection: Axis.horizontal,
            child: DataTable(
              columnSpacing: 20,
              horizontalMargin: 6,
              columns: const [
                DataColumn(label: Text('Run')),
                DataColumn(label: Text('Speed time'), numeric: true),
                DataColumn(label: Text('Distance time'), numeric: true),
                DataColumn(label: Text('End speed'), numeric: true),
                DataColumn(label: Text('Slope'), numeric: true),
                DataColumn(label: Text('Status')),
              ],
              rows: runs
                  .map(
                    (r) => DataRow(
                      cells: [
                        DataCell(Text('${r['number']}')),
                        DataCell(Text('${number(r['speed_time_s'], 3)} s')),
                        DataCell(Text('${number(r['distance_time_s'], 3)} s')),
                        DataCell(
                          Text(
                            '${number(numeric(r['trap_speed_mps']) == null ? null : numeric(r['trap_speed_mps'])! * 3.6)} km/h',
                          ),
                        ),
                        DataCell(Text('${number(r['slope_percent'], 2)}%')),
                        DataCell(
                          Tooltip(
                            message: r['reason']?.toString() ?? '',
                            child: Text(
                              r['valid'] == true ? 'Valid' : 'Invalid',
                              style: TextStyle(
                                color: r['valid'] == true ? muted : warning,
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  )
                  .toList(),
            ),
          ),
        for (final run in runs.where((r) => r['reason'] != null))
          Padding(
            padding: const EdgeInsets.only(top: 8),
            child: Text(
              'Run ${run['number']}: ${run['reason']}',
              style: const TextStyle(color: warning, fontSize: 11),
            ),
          ),
      ],
    );
  }
}
