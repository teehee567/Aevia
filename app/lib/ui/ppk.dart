import 'package:file_selector/file_selector.dart';
import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';

class PpkView extends StatefulWidget {
  const PpkView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<PpkView> createState() => _PpkViewState();
}

class _PpkViewState extends State<PpkView> {
  AeviaController get c => widget.controller;
  final fields = <String, TextEditingController>{};
  String? source;
  @override
  void initState() {
    super.initState();
    source = c.ppkDraft['source_id'];
    if (source == '') source = null;
    for (final key in [
      'output_dir',
      'rover_obs',
      'base_obs',
      'rover_nav',
      'convbin_path',
      'rnx2rtkp_path',
      'config_path',
      'base_x',
      'base_y',
      'base_z',
      'solution',
    ]) {
      fields[key] = TextEditingController(text: c.ppkDraft[key] ?? '');
    }
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted && !c.busy) c.checkPpk(source);
    });
  }

  @override
  void dispose() {
    for (final field in fields.values) {
      field.dispose();
    }
    super.dispose();
  }

  String value(String key) => fields[key]!.text.trim();
  void remember(String key, String text) {
    setState(() {
      c.ppkDraft[key] = text;
    });
  }

  Future<void> browse(
    String key, {
    bool directory = false,
    bool multiple = false,
  }) async {
    try {
      final String? selected;
      if (directory) {
        selected = await getDirectoryPath();
      } else if (multiple) {
        final files = await openFiles();
        selected = files.isEmpty ? null : files.map((f) => f.path).join('\n');
      } else {
        selected = (await openFile())?.path;
      }
      if (!mounted || selected == null) return;
      fields[key]!.text = selected;
      remember(key, selected);
    } catch (error) {
      if (!mounted) return;
      c.error =
          'File browser unavailable: $error. Enter the path in the field.';
      c.changed();
    }
  }

  Widget path(
    String key,
    String label, {
    bool directory = false,
    bool multiple = false,
  }) => Padding(
    padding: const EdgeInsets.only(top: 12),
    child: Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Expanded(
          child: TextField(
            key: ValueKey('ppk_$key'),
            controller: fields[key],
            enabled: !c.busy,
            maxLines: multiple ? 3 : 1,
            onChanged: (text) => remember(key, text),
            decoration: InputDecoration(
              labelText: label,
              helperText: multiple ? 'One navigation file per line' : null,
            ),
          ),
        ),
        const SizedBox(width: 8),
        IconButton(
          tooltip: 'Browse $label',
          onPressed: c.busy
              ? null
              : () => browse(key, directory: directory, multiple: multiple),
          icon: const Icon(Icons.folder_open_outlined),
        ),
      ],
    ),
  );

  Widget warnings(dynamic values) => Column(
    crossAxisAlignment: CrossAxisAlignment.start,
    children: [
      if (values is List)
        for (final warning in values)
          Padding(
            padding: const EdgeInsets.only(top: 8),
            child: Text(
              '$warning',
              style: const TextStyle(color: compareColor),
            ),
          ),
    ],
  );

  Future<void> prepare() async {
    await c.preparePpk({
      'id': source,
      'output_dir': value('output_dir'),
      if (value('convbin_path').isNotEmpty)
        'convbin_path': value('convbin_path'),
    });
    if (!mounted || c.error != null) return;
    setState(() {
      for (final key in ['rover_obs', 'rover_nav']) {
        if (c.ppkDraft[key] != null) fields[key]!.text = c.ppkDraft[key]!;
      }
    });
  }

  Future<void> run() async {
    final coordinates = ['base_x', 'base_y', 'base_z'].map(value).toList();
    final xyz = coordinates.map(double.tryParse).toList();
    if (coordinates.any((v) => v.isNotEmpty) &&
        xyz.any((v) => v == null || !v.isFinite)) {
      c.error =
          'Enter all three finite base ECEF coordinates, or leave all three blank to use the base observation header.';
      c.changed();
      return;
    }
    await c.runPpk({
      if (source != null) 'source_id': source,
      'rover_obs': value('rover_obs'),
      'base_obs': value('base_obs'),
      'nav_paths': value(
        'rover_nav',
      ).split('\n').map((s) => s.trim()).where((s) => s.isNotEmpty).toList(),
      'output_dir': value('output_dir'),
      if (coordinates.every((v) => v.isNotEmpty)) 'base_ecef_m': xyz,
      for (final key in ['rnx2rtkp_path', 'config_path'])
        if (value(key).isNotEmpty) key: value(key),
    });
  }

  @override
  Widget build(BuildContext context) {
    final sessions = c.sessions;
    final selected = sessions.any((s) => s['id'] == source) ? source : null;
    final tools = object(c.ppkStatus['tools']);
    final canRun =
        !c.busy &&
        !c.recording &&
        [
          'rover_obs',
          'base_obs',
          'rover_nav',
          'output_dir',
        ].every((k) => value(k).isNotEmpty);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        const Text(
          'Post-process a rover recording with observations from a stationary base covering the same time. PPK requires satellite tracking and carrier-phase data; an indoor no-fix recording cannot demonstrate a corrected position.',
          style: TextStyle(color: muted),
        ),
        const SizedBox(height: 18),
        Section(
          title: '1 · Prepare rover observations',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Text(
                'Record on the receiver, stop, then download the SD recording from Receiver. Raw GNSS observations are stored on SD. USB telemetry alone is not a PPK input.',
              ),
              const SizedBox(height: 14),
              DropdownButtonFormField<String>(
                key: ValueKey('ppk_source_$selected'),
                initialValue: selected,
                isExpanded: true,
                decoration: const InputDecoration(
                  labelText: 'Source recording (optional for external RINEX)',
                ),
                items: [
                  const DropdownMenuItem(
                    value: '',
                    child: Text('External RINEX files'),
                  ),
                  for (final session in sessions)
                    DropdownMenuItem(
                      value: '${session['id']}',
                      child: Text(
                        '${session['name'] ?? session['id']}',
                        overflow: TextOverflow.ellipsis,
                      ),
                    ),
                ],
                onChanged: c.busy
                    ? null
                    : (id) {
                        setState(() {
                          source = id == '' ? null : id;
                          c.ppkDraft['source_id'] = source ?? '';
                          // A generated rover file belongs to its original
                          // source; never attach a newly selected parent to it.
                          for (final key in ['rover_obs', 'rover_nav']) {
                            if (value(key) == c.ppkPrepared[key]) {
                              fields[key]!.clear();
                              c.ppkDraft.remove(key);
                            }
                          }
                          c.ppkStatus = {};
                          c.ppkPrepared = {};
                        });
                        c.checkPpk(source);
                      },
              ),
              path('output_dir', 'Output folder', directory: true),
              const SizedBox(height: 12),
              Wrap(
                spacing: 12,
                runSpacing: 12,
                crossAxisAlignment: WrapCrossAlignment.center,
                children: [
                  FilledButton.icon(
                    onPressed:
                        c.busy ||
                            c.recording ||
                            selected == null ||
                            value('output_dir').isEmpty ||
                            c.ppkStatus['raw_available'] != true
                        ? null
                        : prepare,
                    icon: const Icon(Icons.transform_outlined, size: 18),
                    label: const Text('Prepare RINEX'),
                  ),
                  OutlinedButton(
                    onPressed: c.busy ? null : () => c.checkPpk(source),
                    child: const Text('Check inputs & tools'),
                  ),
                  Text(
                    c.ppkStatus.isEmpty
                        ? 'Checking raw observations…'
                        : c.ppkStatus['raw_available'] == true
                        ? 'Raw observations available'
                        : 'Choose a downloaded SD recording with raw observations',
                    style: const TextStyle(color: muted),
                  ),
                ],
              ),
              warnings(c.ppkStatus['warnings']),
              if (c.recording)
                const Padding(
                  padding: EdgeInsets.only(top: 8),
                  child: Text(
                    'Stop and save the current recording before post-processing.',
                    style: TextStyle(color: warning),
                  ),
                ),
              warnings(c.ppkPrepared['warnings']),
              if (c.ppkPrepared.isNotEmpty)
                Padding(
                  padding: const EdgeInsets.only(top: 12),
                  child: Align(
                    alignment: Alignment.centerLeft,
                    child: Status(
                      c.ppkPrepared['ready'] == true
                          ? 'ROVER DATA AVAILABLE'
                          : 'REVIEW ROVER INPUTS',
                      color: c.ppkPrepared['ready'] == true ? accent : warning,
                    ),
                  ),
                ),
              if (c.ppkPrepared['rover_raw'] != null)
                Padding(
                  padding: const EdgeInsets.only(top: 12),
                  child: SelectableText(
                    'Raw export: ${c.ppkPrepared['rover_raw']}',
                  ),
                ),
              if (c.ppkPrepared['manifest'] != null)
                SelectableText('Manifest: ${c.ppkPrepared['manifest']}'),
            ],
          ),
        ),
        const SizedBox(height: 18),
        Section(
          title: '2 · Process with a base station',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              const Text(
                'Supply rover and base RINEX observations plus broadcast navigation files. Confirm the base antenna coordinates. Fixed, float and single solutions remain distinguishable in the result.',
              ),
              path('rover_obs', 'Rover RINEX observations'),
              path('base_obs', 'Base RINEX observations'),
              path('rover_nav', 'RINEX navigation files', multiple: true),
              const SizedBox(height: 12),
              ExpansionTile(
                tilePadding: EdgeInsets.zero,
                title: const Text('Base coordinates and processing tools'),
                children: [
                  const Align(
                    alignment: Alignment.centerLeft,
                    child: Text(
                      'Optional base ECEF position in metres. Leave all three blank to use the observation header.',
                      style: TextStyle(color: muted),
                    ),
                  ),
                  const SizedBox(height: 12),
                  Row(
                    children: [
                      for (final axis in ['x', 'y', 'z'])
                        Expanded(
                          child: Padding(
                            padding: const EdgeInsets.only(right: 8),
                            child: TextField(
                              controller: fields['base_$axis'],
                              enabled: !c.busy,
                              onChanged: (s) => remember('base_$axis', s),
                              decoration: InputDecoration(
                                labelText: 'ECEF ${axis.toUpperCase()} (m)',
                              ),
                            ),
                          ),
                        ),
                    ],
                  ),
                  path('config_path', 'Optional RTKLIB configuration'),
                  Align(
                    alignment: Alignment.centerLeft,
                    child: SelectableText(
                      'Converter: ${tools['convbin'] ?? 'not detected'}\nProcessor: ${tools['rnx2rtkp'] ?? 'not detected'}',
                      style: const TextStyle(color: muted),
                    ),
                  ),
                  path('convbin_path', 'Override convbin executable'),
                  path('rnx2rtkp_path', 'Override rnx2rtkp executable'),
                ],
              ),
              const SizedBox(height: 12),
              Align(
                alignment: Alignment.centerLeft,
                child: FilledButton.icon(
                  onPressed: canRun ? run : null,
                  icon: const Icon(Icons.play_arrow, size: 18),
                  label: const Text('Run PPK'),
                ),
              ),
              const SizedBox(height: 10),
              const Text(
                'The processed result is saved as a separate session with input provenance and solution quality. A position-only solution does not provide measured speed or acceleration.',
                style: TextStyle(color: muted),
              ),
            ],
          ),
        ),
        const SizedBox(height: 18),
        Section(
          title: 'Import an existing solution',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              path('solution', 'RTKLIB .pos solution'),
              const SizedBox(height: 12),
              Align(
                alignment: Alignment.centerLeft,
                child: OutlinedButton(
                  onPressed: c.busy || value('solution').isEmpty
                      ? null
                      : () => c.importPpk(value('solution'), source),
                  child: const Text('Import PPK solution'),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
