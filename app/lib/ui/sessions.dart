import 'package:flutter/material.dart';
import 'package:file_selector/file_selector.dart';
import '../controller.dart';
import 'common.dart';

String sessionDate(dynamic ms) {
  if (ms is! num) return 'Unknown date';
  final date = DateTime.fromMillisecondsSinceEpoch(ms.toInt()).toLocal();
  return '${date.year}-${date.month.toString().padLeft(2, '0')}-${date.day.toString().padLeft(2, '0')}  ${date.hour.toString().padLeft(2, '0')}:${date.minute.toString().padLeft(2, '0')}';
}

Future<void> showImportDialog(BuildContext context, AeviaController c) async {
  final path = TextEditingController();
  final value = await showDialog<String>(
    context: context,
    builder: (context) => AlertDialog(
      title: const Text('Import a session'),
      content: SizedBox(
        width: 520,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Choose an Aevia JSON session, telemetry CSV, .aev recording, or recoverable .capture.jsonl file.',
              style: TextStyle(color: muted),
            ),
            const SizedBox(height: 20),
            OutlinedButton.icon(
              onPressed: () async {
                try {
                  final file = await openFile(
                    acceptedTypeGroups: const [
                      XTypeGroup(
                        label: 'Aevia sessions',
                        extensions: ['json', 'csv', 'jsonl', 'aev'],
                        uniformTypeIdentifiers: [
                          'public.json',
                          'public.comma-separated-values-text',
                          'public.text',
                        ],
                      ),
                    ],
                  );
                  if (context.mounted && file != null) path.text = file.path;
                } catch (error) {
                  if (context.mounted)
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text(
                          'File browser unavailable: $error. Enter a path below.',
                        ),
                      ),
                    );
                }
              },
              icon: const Icon(Icons.folder_open_outlined, size: 17),
              label: const Text('Browse files'),
            ),
            const SizedBox(height: 14),
            TextField(
              controller: path,
              autofocus: true,
              decoration: const InputDecoration(
                labelText: 'File path',
                hintText: 'C:\\Telemetry\\session.json',
              ),
              onSubmitted: (value) {
                if (value.trim().isNotEmpty)
                  Navigator.pop(context, value.trim());
              },
            ),
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('Cancel'),
        ),
        FilledButton(
          onPressed: () {
            if (path.text.trim().isNotEmpty)
              Navigator.pop(context, path.text.trim());
          },
          child: const Text('Import session'),
        ),
      ],
    ),
  );
  path.dispose();
  if (value != null) await c.importSession(value);
}

Future<void> showExportDialog(
  BuildContext context,
  AeviaController c,
  Map<String, dynamic> session,
) async {
  final path = TextEditingController(
    text:
        '${c.state['storage_path'] ?? ''}${Theme.of(context).platform == TargetPlatform.windows ? '\\' : '/'}${session['id']}.csv',
  );
  String format = 'csv';
  final value = await showDialog<Map<String, String>>(
    context: context,
    builder: (context) => StatefulBuilder(
      builder: (context, setState) => AlertDialog(
        title: const Text('Export session'),
        content: SizedBox(
          width: 520,
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              DropdownButtonFormField<String>(
                isExpanded: true,
                initialValue: format,
                decoration: const InputDecoration(labelText: 'Format'),
                items: const [
                  DropdownMenuItem(
                    value: 'csv',
                    child: Text('CSV · telemetry channels'),
                  ),
                  DropdownMenuItem(
                    value: 'gpx',
                    child: Text('GPX · position trace'),
                  ),
                  DropdownMenuItem(
                    value: 'json',
                    child: Text('Aevia JSON · complete session'),
                  ),
                ],
                onChanged: (value) {
                  if (value != null)
                    setState(() {
                      format = value;
                      final dot = path.text.lastIndexOf('.');
                      path.text =
                          '${dot < 0 ? path.text : path.text.substring(0, dot)}.$format';
                    });
                },
              ),
              const SizedBox(height: 20),
              if ([
                TargetPlatform.windows,
                TargetPlatform.linux,
                TargetPlatform.macOS,
              ].contains(Theme.of(context).platform)) ...[
                OutlinedButton.icon(
                  onPressed: () async {
                    try {
                      final location = await getSaveLocation(
                        suggestedName: '${session['id']}.$format',
                        acceptedTypeGroups: [
                          XTypeGroup(
                            label: format.toUpperCase(),
                            extensions: [format],
                          ),
                        ],
                      );
                      if (context.mounted && location != null)
                        path.text = location.path;
                    } catch (error) {
                      if (context.mounted)
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Text(
                              'File browser unavailable: $error. Enter a path below.',
                            ),
                          ),
                        );
                    }
                  },
                  icon: const Icon(Icons.folder_open_outlined, size: 17),
                  label: const Text('Choose destination'),
                ),
                const SizedBox(height: 14),
              ],
              TextField(
                controller: path,
                decoration: const InputDecoration(
                  labelText: 'Destination file path',
                ),
              ),
              const SizedBox(height: 12),
              const Text(
                'CSV includes SI units in column names. JSON preserves the complete session for reimport.',
                style: TextStyle(color: muted, fontSize: 12),
              ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
          FilledButton(
            onPressed: () {
              if (path.text.trim().isNotEmpty)
                Navigator.pop(context, {
                  'path': path.text.trim(),
                  'format': format,
                });
            },
            child: const Text('Export file'),
          ),
        ],
      ),
    ),
  );
  path.dispose();
  if (value != null)
    await c.exportSession(
      session['id'].toString(),
      value['path']!,
      value['format']!,
    );
}

class SessionsView extends StatefulWidget {
  const SessionsView({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<SessionsView> createState() => _SessionsViewState();
}

class _SessionsViewState extends State<SessionsView> {
  String query = '';
  @override
  Widget build(BuildContext context) {
    final c = widget.controller;
    final sessions = c.sessions
        .where(
          (s) => '${s['name']} ${s['source']}'.toLowerCase().contains(
            query.toLowerCase(),
          ),
        )
        .toList();
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Wrap(
          spacing: 12,
          runSpacing: 12,
          crossAxisAlignment: WrapCrossAlignment.center,
          children: [
            SizedBox(
              width: 300,
              child: TextField(
                decoration: const InputDecoration(
                  labelText: 'Search sessions',
                  prefixIcon: Icon(Icons.search, size: 19),
                ),
                onChanged: (value) => setState(() => query = value),
              ),
            ),
            OutlinedButton.icon(
              onPressed: c.busy ? null : () => showImportDialog(context, c),
              icon: const Icon(Icons.file_open_outlined, size: 17),
              label: const Text('Import session'),
            ),
            OutlinedButton.icon(
              onPressed: c.busy ? null : c.demo,
              icon: const Icon(Icons.play_circle_outline, size: 17),
              label: const Text('Explore demo'),
            ),
          ],
        ),
        const SizedBox(height: 20),
        Section(
          title:
              '${sessions.length} recorded session${sessions.length == 1 ? '' : 's'}',
          padding: 0,
          child: sessions.isEmpty
              ? EmptyState(
                  query.isEmpty
                      ? 'Your session library is empty'
                      : 'No matching sessions',
                  query.isEmpty
                      ? 'Record with your V2 Mini, import a capture, or explore a clearly labelled demonstration session.'
                      : 'Try a different name or source.',
                  action: query.isEmpty
                      ? FilledButton(
                          onPressed: () => c.navigate(0),
                          child: const Text('Open live telemetry'),
                        )
                      : null,
                  icon: Icons.folder_open_outlined,
                )
              : LayoutBuilder(
                  builder: (context, box) => SingleChildScrollView(
                    scrollDirection: Axis.horizontal,
                    child: ConstrainedBox(
                      constraints: BoxConstraints(minWidth: box.maxWidth),
                      child: DataTable(
                        showCheckboxColumn: false,
                        headingRowHeight: 42,
                        dataRowMinHeight: 60,
                        dataRowMaxHeight: 72,
                        horizontalMargin: 18,
                        columnSpacing: 28,
                        columns: const [
                          DataColumn(label: Text('Session')),
                          DataColumn(label: Text('Recorded')),
                          DataColumn(label: Text('Duration'), numeric: true),
                          DataColumn(label: Text('Distance'), numeric: true),
                          DataColumn(label: Text('Max speed'), numeric: true),
                          DataColumn(label: Text('Samples'), numeric: true),
                          DataColumn(label: Text('')),
                        ],
                        rows: sessions
                            .map(
                              (s) => DataRow(
                                onSelectChanged: (_) =>
                                    c.openSession(s['id'].toString()),
                                cells: [
                                  DataCell(
                                    Column(
                                      mainAxisAlignment:
                                          MainAxisAlignment.center,
                                      crossAxisAlignment:
                                          CrossAxisAlignment.start,
                                      children: [
                                        Text(
                                          s['name']?.toString() ??
                                              'Unnamed session',
                                          style: const TextStyle(
                                            fontWeight: FontWeight.w600,
                                          ),
                                        ),
                                        Text(
                                          s['source']?.toString() ??
                                              'Unknown source',
                                          style: TextStyle(
                                            fontSize: 11,
                                            color:
                                                s['source']
                                                        ?.toString()
                                                        .contains('demo') ==
                                                    true
                                                ? warning
                                                : muted,
                                          ),
                                        ),
                                        if (s['incomplete_reason'] != null)
                                          Tooltip(
                                            message: s['incomplete_reason']
                                                .toString(),
                                            child: const Text(
                                              'INCOMPLETE CAPTURE',
                                              style: TextStyle(
                                                color: warning,
                                                fontSize: 10,
                                              ),
                                            ),
                                          ),
                                      ],
                                    ),
                                  ),
                                  DataCell(
                                    Text(
                                      sessionDate(s['created_ms']),
                                      style: const TextStyle(color: muted),
                                    ),
                                  ),
                                  DataCell(Text(elapsed(s['duration_s']))),
                                  DataCell(
                                    Text(
                                      '${number(object(s['provenance'])['kind'] == 'ppk' ? null : (numeric(s['distance_m']) ?? 0) / 1000, 2)} km',
                                    ),
                                  ),
                                  DataCell(
                                    Text(
                                      '${number(numeric(s['max_speed_mps']) == null ? null : numeric(s['max_speed_mps'])! * 3.6, 0)} km/h',
                                    ),
                                  ),
                                  DataCell(Text('${s['sample_count'] ?? '—'}')),
                                  DataCell(
                                    IconButton(
                                      tooltip: 'Export ${s['name']}',
                                      onPressed: c.busy
                                          ? null
                                          : () =>
                                                showExportDialog(context, c, s),
                                      icon: const Icon(
                                        Icons.file_download_outlined,
                                        size: 19,
                                      ),
                                    ),
                                  ),
                                ],
                              ),
                            )
                            .toList(),
                      ),
                    ),
                  ),
                ),
        ),
        const SizedBox(height: 14),
        SelectableText(
          'Library location: ${c.state['storage_path'] ?? 'Loading…'}',
          style: const TextStyle(color: muted, fontSize: 11),
        ),
      ],
    );
  }
}
