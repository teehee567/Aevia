import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';
import 'diagnostics.dart';
import 'board_data.dart';
import 'live.dart';
import 'sessions.dart';
import 'analysis.dart';
import 'setup.dart';
import 'ppk.dart';

class AeviaApp extends StatefulWidget {
  const AeviaApp({super.key, required this.controller});
  final AeviaController controller;
  @override
  State<AeviaApp> createState() => _AeviaAppState();
}

class _AeviaAppState extends State<AeviaApp> with WidgetsBindingObserver {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    widget.controller.initialize();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    widget.controller.setForeground(state == AppLifecycleState.resumed);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    widget.controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) => MaterialApp(
    debugShowCheckedModeBanner: false,
    title: 'Aevia',
    theme: aeviaTheme(),
    home: AeviaShell(controller: widget.controller),
  );
}

class AeviaShell extends StatefulWidget {
  const AeviaShell({super.key, required this.controller});
  final AeviaController controller;
  static const titles = [
    'Live telemetry',
    'Sessions',
    'Session analysis',
    'Drag testing',
    'Tracks',
    'Receiver',
    'Post-processing',
    'Receiver diagnostics',
    'Board data',
  ];
  static const labels = [
    'Live',
    'Sessions',
    'Analysis',
    'Drag testing',
    'Tracks',
    'Receiver',
    'PPK',
    'Diagnostics',
    'Board data',
  ];
  static const icons = [
    Icons.speed_outlined,
    Icons.folder_outlined,
    Icons.insights_outlined,
    Icons.flag_outlined,
    Icons.route_outlined,
    Icons.usb_outlined,
    Icons.satellite_alt_outlined,
    Icons.monitor_heart_outlined,
    Icons.sd_storage_outlined,
  ];
  @override
  State<AeviaShell> createState() => _AeviaShellState();
}

class _AeviaShellState extends State<AeviaShell> {
  AeviaController get controller => widget.controller;
  // The active workspace listens to all data changes. Navigation and window
  // layout only depend on this small set of visible connection and task states.
  Record get chrome => (
    controller.page,
    controller.connected,
    controller.bluetooth,
    controller.device['port'],
    controller.recording,
    controller.captureInterruption,
    [0, 5, 7].contains(controller.page) && controller.stale,
    controller.busy,
    controller.activity,
    controller.state['processing'],
    controller.error,
    controller.notice,
    boardStatus,
  );
  Record? get boardStatus {
    if (controller.page != AeviaController.boardPage) return null;
    final sd = object(controller.device['sd']);
    return (
      controller.usbConnected,
      controller.storageBusy,
      controller.canBrowseBoard,
      sd['state'],
      sd['capacity_bytes'],
      sd['error'],
      controller.deviceSessions,
      controller.boardPreview,
    );
  }

  late Record lastChrome;

  @override
  void initState() {
    super.initState();
    lastChrome = chrome;
    controller.addListener(updateChrome);
  }

  @override
  void didUpdateWidget(AeviaShell oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.controller != controller) {
      oldWidget.controller.removeListener(updateChrome);
      controller.addListener(updateChrome);
      lastChrome = chrome;
    }
  }

  void updateChrome() {
    final next = chrome;
    if (lastChrome != next) setState(() => lastChrome = next);
  }

  @override
  void dispose() {
    controller.removeListener(updateChrome);
    super.dispose();
  }

  Widget navigation(BuildContext context, {bool drawer = false}) => Container(
    width: 184,
    decoration: const BoxDecoration(
      color: panel,
      border: Border(right: BorderSide(color: line)),
    ),
    child: Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        const Padding(
          padding: EdgeInsets.fromLTRB(22, 28, 16, 5),
          child: Text(
            'AEVIA',
            style: TextStyle(
              fontSize: 22,
              letterSpacing: 5,
              fontWeight: FontWeight.w600,
            ),
          ),
        ),
        const Padding(
          padding: EdgeInsets.fromLTRB(23, 0, 12, 28),
          child: Text(
            'MOTORSPORT TELEMETRY',
            style: TextStyle(fontSize: 8, letterSpacing: 1.1, color: muted),
          ),
        ),
        Expanded(
          child: ListView(
            padding: EdgeInsets.zero,
            children: [
              for (int i = 0; i < AeviaShell.labels.length; i++)
                Padding(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 10,
                    vertical: 2,
                  ),
                  child: Material(
                    color: controller.page == i
                        ? accent.withValues(alpha: .09)
                        : Colors.transparent,
                    borderRadius: BorderRadius.circular(4),
                    child: ListTile(
                      dense: true,
                      selected: controller.page == i,
                      selectedColor: accent,
                      leading: Icon(AeviaShell.icons[i], size: 19),
                      title: Text(
                        AeviaShell.labels[i],
                        style: const TextStyle(fontSize: 13),
                      ),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(4),
                      ),
                      onTap: () {
                        controller.navigate(i);
                        if (drawer) Navigator.pop(context);
                      },
                    ),
                  ),
                ),
            ],
          ),
        ),
        const Divider(),
        Padding(
          padding: const EdgeInsets.all(18),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                controller.connected
                    ? 'V2 Mini · ${controller.bluetooth ? 'Bluetooth' : controller.device['port'] ?? 'USB'}'
                    : 'No receiver connected',
                style: const TextStyle(fontSize: 12),
              ),
              const SizedBox(height: 6),
              Text(
                controller.recording
                    ? '● Recording session'
                    : controller.connected
                    ? (controller.bluetooth
                          ? 'Bluetooth connected'
                          : 'USB connected')
                    : 'Connect in Receiver',
                style: TextStyle(
                  fontSize: 11,
                  color: controller.recording ? accent : muted,
                ),
              ),
            ],
          ),
        ),
      ],
    ),
  );
  @override
  Widget build(BuildContext context) {
    final compact = MediaQuery.sizeOf(context).width < 880;
    return Scaffold(
      drawer: compact
          ? Drawer(
              width: 230,
              child: SafeArea(child: navigation(context, drawer: true)),
            )
          : null,
      body: SafeArea(
        child: Row(
          children: [
            if (!compact) navigation(context),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Container(
                    height: 72,
                    padding: EdgeInsets.symmetric(
                      horizontal: compact ? 12 : 26,
                    ),
                    decoration: const BoxDecoration(
                      border: Border(bottom: BorderSide(color: line)),
                    ),
                    child: Row(
                      children: [
                        if (compact)
                          Builder(
                            builder: (context) => IconButton(
                              tooltip: 'Open navigation',
                              onPressed: () =>
                                  Scaffold.of(context).openDrawer(),
                              icon: const Icon(Icons.menu),
                            ),
                          ),
                        Expanded(
                          child: Text(
                            AeviaShell.titles[controller.page],
                            style: Theme.of(context).textTheme.titleLarge,
                          ),
                        ),
                        if (controller.recording)
                          Status(
                            controller.captureInterruption == null
                                ? 'RECORDING'
                                : 'CAPTURE INTERRUPTED',
                            color: controller.captureInterruption == null
                                ? accent
                                : warning,
                          ),
                        if (!compact) ...[
                          const SizedBox(width: 12),
                          Status(
                            controller.connected
                                ? [0, 5, 7].contains(controller.page) &&
                                          controller.stale
                                      ? 'STALE DATA'
                                      : controller.bluetooth
                                      ? 'BLUETOOTH CONNECTED'
                                      : 'USB CONNECTED'
                                : 'OFFLINE',
                            color: controller.connected
                                ? [0, 5, 7].contains(controller.page) &&
                                          controller.stale
                                      ? warning
                                      : accent
                                : muted,
                          ),
                        ],
                      ],
                    ),
                  ),
                  if (controller.busy)
                    const LinearProgressIndicator(minHeight: 2),
                  if (controller.activity != null ||
                      controller.state['processing'] != null)
                    Padding(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 24,
                        vertical: 8,
                      ),
                      child: Row(
                        children: [
                          Expanded(
                            child: Text(
                              controller.activity ??
                                  'Processing session data...',
                            ),
                          ),
                          if (controller.state['processing'] != null)
                            TextButton(
                              onPressed: controller.cancelProcessing,
                              child: const Text('Cancel processing'),
                            ),
                        ],
                      ),
                    ),
                  if (controller.error != null || controller.notice != null)
                    Container(
                      color: (controller.error != null ? warning : accent)
                          .withValues(alpha: .08),
                      padding: const EdgeInsets.fromLTRB(24, 8, 10, 8),
                      child: Row(
                        children: [
                          Icon(
                            controller.error != null
                                ? Icons.error_outline
                                : Icons.check_circle_outline,
                            color: controller.error != null ? warning : accent,
                            size: 18,
                          ),
                          const SizedBox(width: 10),
                          Expanded(
                            child: Text(
                              controller.error ?? controller.notice!,
                              style: TextStyle(
                                color: controller.error != null ? warning : ink,
                              ),
                            ),
                          ),
                          IconButton(
                            tooltip: 'Dismiss message',
                            onPressed: controller.dismissMessage,
                            icon: const Icon(Icons.close, size: 17),
                          ),
                        ],
                      ),
                    ),
                  Expanded(
                    child: controller.page == AeviaController.boardPage
                        ? BoardDataView(
                            key: const PageStorageKey('workspace-8'),
                            controller: controller,
                            padding: EdgeInsets.all(compact ? 16 : 26),
                          )
                        : ListenableBuilder(
                            listenable: controller,
                            builder: (context, _) {
                              final key = PageStorageKey(
                                'workspace-${controller.page}',
                              );
                              final padding = EdgeInsets.all(compact ? 16 : 26);
                              return SingleChildScrollView(
                                key: key,
                                padding: padding,
                                child: switch (controller.page) {
                                  0 => LiveView(controller: controller),
                                  1 => SessionsView(controller: controller),
                                  2 => AnalysisView(controller: controller),
                                  3 => DragView(controller: controller),
                                  4 => TracksView(controller: controller),
                                  5 => ReceiverView(controller: controller),
                                  6 => PpkView(controller: controller),
                                  7 => DiagnosticsView(controller: controller),
                                  _ => throw StateError('Unknown workspace'),
                                },
                              );
                            },
                          ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
