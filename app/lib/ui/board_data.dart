import 'dart:convert';
import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';

String _size(dynamic bytes) {
  if (bytes is! int || bytes < 0) return '—';
  if (bytes < 1024) return '$bytes B';
  if (bytes < 1024 * 1024) return '${number(bytes / 1024)} KiB';
  if (bytes < 1024 * 1024 * 1024) return '${number(bytes / (1024 * 1024))} MiB';
  return '${number(bytes / (1024 * 1024 * 1024), 2)} GiB';
}

class BoardDataView extends StatelessWidget {
  const BoardDataView({super.key, required this.controller});
  final AeviaController controller;

  @override
  Widget build(BuildContext context) {
    final c = controller;
    if (!c.usbConnected) {
      return EmptyState(
        'Board data requires USB',
        'Connect the V2 Mini to the app by USB to browse its SD card.',
        icon: Icons.usb_outlined,
        action: OutlinedButton(
          onPressed: () => c.navigate(5),
          child: const Text('Open Receiver'),
        ),
      );
    }
    final sd = object(c.device['sd']);
    final files = c.deviceSessions;
    final sizes = files?.map((file) => file['bytes']).whereType<int>();
    final total = sizes != null && sizes.length == files!.length
        ? sizes.fold<int>(0, (a, b) => a + b)
        : null;
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Section(
          title: 'Receiver SD card',
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              MetricGrid(
                children: [
                  Metric('Card state', sd['state']?.toString() ?? '—'),
                  Metric(
                    'Card capacity',
                    (numeric(sd['capacity_bytes']) ?? 0) > 0
                        ? _size(sd['capacity_bytes'])
                        : '—',
                  ),
                  Metric('Recording files', files?.length.toString() ?? '—'),
                  Metric('Committed data', _size(total)),
                ],
              ),
              if (sd['error'] != null)
                Text('${sd['error']}', style: const TextStyle(color: warning)),
            ],
          ),
        ),
        const SizedBox(height: 16),
        Section(
          title: 'Recording history',
          trailing: OutlinedButton.icon(
            onPressed: c.busy || !c.canBrowseBoard
                ? null
                : c.refreshDeviceSessions,
            icon: const Icon(Icons.refresh, size: 16),
            label: const Text('Refresh'),
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              if (c.storageBusy)
                const Text(
                  'Stop recording and wait for the SD card to finish saving to browse its files.',
                )
              else if (files == null)
                const Text('Refresh to read the recording directory over USB.')
              else if (files.isEmpty)
                const Text('No recordings on this SD card.')
              else
                const Text(
                  'Preview file details or download a recording to open it in Analysis.',
                  style: TextStyle(color: muted),
                ),
              for (final file in files ?? <Map<String, dynamic>>[]) ...[
                const SizedBox(height: 14),
                const Divider(),
                const SizedBox(height: 12),
                _Recording(file: file, controller: c),
              ],
            ],
          ),
        ),
      ],
    );
  }
}

class _Recording extends StatelessWidget {
  const _Recording({required this.file, required this.controller});
  final Map<String, dynamic> file;
  final AeviaController controller;

  @override
  Widget build(BuildContext context) {
    final c = controller;
    final id = file['id'];
    final hasData = (numeric(file['bytes']) ?? 0) > 0;
    final enabled = id is int && hasData && !c.busy && c.canBrowseBoard;
    final state = !hasData
        ? 'No committed data'
        : file['closed'] == true
        ? 'Saved'
        : 'Incomplete · recoverable';
    final preview = c.boardPreview;
    final metadata = object(preview?['metadata']);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text('Recording $id', style: Theme.of(context).textTheme.titleMedium),
        Text(
          '${_size(file['bytes'])} · $state',
          style: const TextStyle(color: muted),
        ),
        const SizedBox(height: 8),
        Wrap(
          spacing: 10,
          runSpacing: 8,
          children: [
            OutlinedButton(
              onPressed: enabled ? () => c.previewDeviceSession(id) : null,
              child: const Text('Preview'),
            ),
            OutlinedButton.icon(
              onPressed: enabled ? () => c.downloadDeviceSession(id) : null,
              icon: const Icon(Icons.download_outlined, size: 16),
              label: const Text('Download & analyze'),
            ),
          ],
        ),
        if (preview != null && preview['id'] == id && c.canBrowseBoard) ...[
          MetricGrid(
            children: [
              Metric(
                'Start (board uptime)',
                elapsed(preview['start_uptime_s']),
              ),
              Metric('File duration', elapsed(preview['duration_s'])),
              Metric(
                'SD queue drops at close',
                preview['dropped']?.toString() ?? '—',
              ),
              if (metadata['gnss_log'] != null)
                Metric(
                  'GNSS log',
                  metadata['gnss_log'].toString(),
                  note: metadata['gnss_hz'] == null
                      ? null
                      : '${metadata['gnss_hz']} Hz requested',
                ),
              if (metadata['imu_logging'] != null)
                Metric('IMU logging', metadata['imu_logging'].toString()),
              if (metadata['ppk_format'] != null)
                Metric(
                  'Raw GNSS format',
                  metadata['ppk_format'].toString(),
                  note: metadata['ppk_requested_hz'] == null
                      ? null
                      : '${metadata['ppk_requested_hz']} Hz requested',
                ),
              if (metadata['previous_file_id'] != null)
                Metric(
                  'Previous file',
                  'Recording ${metadata['previous_file_id']}',
                ),
            ],
          ),
          if (metadata.isNotEmpty)
            ExpansionTile(
              title: const Text('Recorded metadata'),
              tilePadding: EdgeInsets.zero,
              expandedAlignment: Alignment.centerLeft,
              children: [
                SelectableText(
                  const JsonEncoder.withIndent(
                    '  ',
                  ).convert(preview['metadata']),
                ),
              ],
            ),
        ],
      ],
    );
  }
}
