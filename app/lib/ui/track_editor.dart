import 'dart:convert';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';

import '../controller.dart';
import 'common.dart';
import 'track_gate.dart';
import 'track_map.dart';

Future<String?> editTrack(
  BuildContext context,
  AeviaController controller, {
  Map<String, dynamic>? track,
  Map<String, dynamic>? position,
}) async {
  controller.setEditingTrack(true);
  try {
    if (position == null && controller.connected && !controller.valid) {
      await controller.poll(liveFix: true);
    }
    if (!context.mounted) return null;
    final fix = position ?? (controller.valid ? controller.live : null);
    return await Navigator.of(context).push<String>(
      MaterialPageRoute(
        fullscreenDialog: true,
        builder: (context) => _TrackEditor(
          track: track,
          position: fix,
          onSave: (value) async => await controller.saveTrack(value)
              ? null
              : controller.error ?? 'Could not save the track. Try again.',
        ),
      ),
    );
  } finally {
    controller.setEditingTrack(false);
  }
}

class _GateDraft {
  _GateDraft([Map<String, dynamic> values = const {}])
    : lat = TextEditingController(text: values['lat']?.toString() ?? ''),
      lon = TextEditingController(text: values['lon']?.toString() ?? ''),
      heading = TextEditingController(
        text: values['heading_deg']?.toString() ?? '0',
      ),
      width = TextEditingController(
        text: values['width_m']?.toString() ?? '20',
      );

  final TextEditingController lat, lon, heading, width;

  TrackGate? get gate {
    final center = trackPosition(
      double.tryParse(lat.text),
      double.tryParse(lon.text),
    );
    final direction = double.tryParse(heading.text);
    final metres = double.tryParse(width.text);
    if (center == null ||
        direction == null ||
        !direction.isFinite ||
        metres == null ||
        !metres.isFinite ||
        metres < minGateWidth ||
        metres > maxGateWidth) {
      return null;
    }
    return TrackGate(center, direction, metres);
  }

  void place(TrackGate gate) {
    lat.text = gate.center.latitude.toStringAsFixed(8);
    lon.text = gate.center.longitude.toStringAsFixed(8);
    heading.text = gate.heading.toStringAsFixed(3);
    width.text = gate.width.toStringAsFixed(3);
  }

  void dispose() {
    lat.dispose();
    lon.dispose();
    heading.dispose();
    width.dispose();
  }
}

class _TrackEditor extends StatefulWidget {
  const _TrackEditor({
    required this.track,
    required this.position,
    required this.onSave,
  });
  final Map<String, dynamic>? track, position;
  final Future<String?> Function(Map<String, dynamic>) onSave;

  @override
  State<_TrackEditor> createState() => _TrackEditorState();
}

class _TrackEditorState extends State<_TrackEditor> {
  final map = MapController();
  final panelScroll = ScrollController();
  final gateScroll = ScrollController();
  final nameFocus = FocusNode();
  late final String id;
  late final TextEditingController name;
  late final List<_GateDraft> gates;
  late final LatLng? reference;
  late final LatLng initialCenter;
  late final double initialZoom;
  int selected = 0;
  bool drawing = false, precision = false, validate = false, saving = false;
  LatLng? firstEndpoint;
  String? drawError, saveError;

  @override
  void initState() {
    super.initState();
    id =
        widget.track?['id']?.toString() ??
        'track-${DateTime.now().millisecondsSinceEpoch}';
    name = TextEditingController(text: widget.track?['name']?.toString() ?? '');
    gates = [
      _GateDraft(object(widget.track?['start'])),
      ...objects(widget.track?['sectors']).map(_GateDraft.new),
    ];
    reference = trackPosition(
      numeric(widget.position?['lat']),
      numeric(widget.position?['lon']),
    );
    final start = gates.first.gate?.center ?? reference;
    initialCenter = start == null ? const LatLng(12, 0) : mapPosition(start);
    initialZoom = start == null ? 2 : 18;

    if (gates.length > 1) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (mounted) _fitGates();
      });
    }
  }

  @override
  void dispose() {
    name.dispose();
    nameFocus.dispose();
    panelScroll.dispose();
    gateScroll.dispose();
    for (final gate in gates) {
      gate.dispose();
    }
    map.dispose();
    super.dispose();
  }

  void _focusGate(TrackGate gate) =>
      map.move(mapPosition(gate.center), math.max(18, map.camera.zoom));

  void _fitGates() {
    final placed = gates.map((draft) => draft.gate).nonNulls.toList();
    if (placed.isEmpty) return;
    final coordinates = placed
        .expand((gate) => gate.endpoints)
        .map(mapPosition)
        .toList();
    final bounds = LatLngBounds.fromPoints(coordinates);
    if (placed.length == 1 ||
        bounds.longitudeWidth > 180 ||
        map.camera.size.height < 220) {
      map.move(mapPosition(placed.first.center), 18);
      return;
    }
    map.fitCamera(
      CameraFit.coordinates(
        coordinates: coordinates,
        padding: const EdgeInsets.fromLTRB(30, 78, 66, 83),
        maxZoom: 19,
      ),
    );
  }

  void _clearDrawing() {
    drawing = false;
    firstEndpoint = null;
    drawError = null;
  }

  void _select(int index, {bool focus = false}) {
    FocusScope.of(context).unfocus();
    setState(() {
      selected = index;
      _clearDrawing();
    });
    final gate = gates[index].gate;
    if (focus && gate != null) _focusGate(gate);
  }

  void _draw() {
    FocusScope.of(context).unfocus();
    setState(() {
      _clearDrawing();
      drawing = true;
    });
  }

  void _place(LatLng point) {
    final first = firstEndpoint;
    if (first == null) {
      setState(() => firstEndpoint = point);
      return;
    }
    final gate = TrackGate.across(first, point);
    setState(() {
      if (gate == null) {
        drawError = 'Gate width must be 1–500 m. Try another point.';
      } else {
        gates[selected].place(gate);
        _clearDrawing();
        saveError = null;
      }
    });
  }

  void _addSector() {
    FocusScope.of(context).unfocus();
    setState(() {
      gates.add(_GateDraft());
      selected = gates.length - 1;
      _clearDrawing();
      drawing = true;
    });
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (mounted && gateScroll.hasClients) {
        gateScroll.jumpTo(gateScroll.position.maxScrollExtent);
      }
    });
  }

  void _removeSector() {
    FocusScope.of(context).unfocus();
    final removed = gates[selected];
    setState(() {
      gates.removeAt(selected);
      selected = math.min(selected, gates.length - 1);
      _clearDrawing();
    });
    WidgetsBinding.instance.addPostFrameCallback((_) => removed.dispose());
  }

  void _moveSector(int change) {
    setState(() {
      final gate = gates.removeAt(selected);
      selected += change;
      gates.insert(selected, gate);
    });
  }

  void _reverse() {
    final draft = gates[selected];
    final gate = draft.gate;
    if (gate == null) return;
    setState(
      () => draft.heading.text = normalizeHeading(
        normalizeHeading(gate.heading) + 180,
      ).toString(),
    );
  }

  void _copyPosition() {
    _select(0);
    final position = reference!;
    final draft = gates.first;
    setState(() {
      draft.lat.text = position.latitude.toString();
      draft.lon.text = position.longitude.toString();
      draft.heading.text = (numeric(widget.position?['heading_deg']) ?? 0)
          .toString();
    });
    map.move(mapPosition(position), 19);
  }

  String? get _nameError {
    final value = name.text.trim();
    if (value.isEmpty) return 'Enter a track name';
    if (utf8.encode(value).length > 128) return 'Shorten the track name';
    return null;
  }

  Future<void> _save() async {
    if (saving) return;
    final values = gates.map((draft) => draft.gate).toList();
    final missing = values.indexOf(null);
    if (_nameError != null || missing != -1) {
      if (missing != -1 && missing != selected) _select(missing);
      setState(() {
        validate = true;
        saveError = missing == -1
            ? null
            : 'Place ${gateLabel(missing).toLowerCase()} or enter its coordinates.';
      });
      if (_nameError != null) nameFocus.requestFocus();
      return;
    }
    FocusScope.of(context).unfocus();
    setState(() {
      saving = true;
      saveError = null;
    });
    final error = await widget.onSave({
      'id': id,
      'name': name.text.trim(),
      'start': values.first!.toJson(),
      'sectors': values.skip(1).map((gate) => gate!.toJson()).toList(),
    });
    if (!mounted) return;
    if (error == null) {
      Navigator.pop(context, id);
    } else {
      setState(() {
        saving = false;
        saveError = error;
      });
    }
  }

  Widget _numberField(
    TextEditingController controller,
    String label,
    String unit,
    double? low,
    double? high,
  ) {
    final value = double.tryParse(controller.text);
    final invalid =
        value == null ||
        !value.isFinite ||
        (low != null && value < low) ||
        (high != null && value > high);
    return TextField(
      controller: controller,
      keyboardType: const TextInputType.numberWithOptions(
        decimal: true,
        signed: true,
      ),
      onChanged: (_) => setState(() => saveError = null),
      decoration: InputDecoration(
        labelText: label,
        suffixText: unit,
        errorText: invalid && (validate || controller.text.isNotEmpty)
            ? low == null
                  ? 'Enter a number'
                  : '$low to $high'
            : null,
      ),
    );
  }

  Widget _editorPanel({required bool wide}) {
    final draft = gates[selected];
    final gate = draft.gate;
    final nameField = TextField(
      controller: name,
      focusNode: nameFocus,
      maxLength: 128,
      onChanged: (_) => setState(() {}),
      decoration: InputDecoration(
        labelText: 'Track name',
        counterText: '',
        errorText: validate ? _nameError : null,
      ),
    );
    return SingleChildScrollView(
      controller: panelScroll,
      padding: EdgeInsets.all(wide ? 20 : 12),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          if (wide) ...[nameField, const SizedBox(height: 18)],
          Row(
            children: [
              Expanded(
                child: Text(
                  'Timing gates',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
              ),
              TextButton.icon(
                onPressed: gates.length >= 16 ? null : _addSector,
                icon: const Icon(Icons.add, size: 18),
                label: const Text('Add sector'),
              ),
            ],
          ),
          SingleChildScrollView(
            controller: gateScroll,
            scrollDirection: Axis.horizontal,
            child: Row(
              children: [
                for (var i = 0; i < gates.length; i++)
                  Padding(
                    padding: const EdgeInsets.only(right: 6),
                    child: ChoiceChip(
                      label: Text(gateLabel(i)),
                      selected: i == selected,
                      onSelected: (_) => _select(i, focus: true),
                    ),
                  ),
              ],
            ),
          ),
          const SizedBox(height: 10),
          Row(
            children: [
              Expanded(
                child: Text(
                  gate == null
                      ? 'Gate unplaced'
                      : '${gate.width.toStringAsFixed(1)} m wide · ${normalizeHeading(gate.heading).toStringAsFixed(0)}° travel',
                  style: const TextStyle(color: muted),
                ),
              ),
              if (selected > 0)
                PopupMenuButton<String>(
                  tooltip: 'Sector options',
                  onSelected: (option) {
                    switch (option) {
                      case 'earlier':
                        _moveSector(-1);
                      case 'later':
                        _moveSector(1);
                      case 'remove':
                        _removeSector();
                    }
                  },
                  itemBuilder: (_) => [
                    PopupMenuItem(
                      value: 'earlier',
                      enabled: selected > 1,
                      child: const Text('Move earlier'),
                    ),
                    PopupMenuItem(
                      value: 'later',
                      enabled: selected < gates.length - 1,
                      child: const Text('Move later'),
                    ),
                    const PopupMenuItem(
                      value: 'remove',
                      child: Text('Remove sector'),
                    ),
                  ],
                ),
            ],
          ),
          Wrap(
            spacing: 8,
            children: [
              TextButton.icon(
                onPressed: gate == null || drawing ? null : _reverse,
                icon: const Icon(Icons.swap_horiz, size: 18),
                label: const Text('Reverse direction'),
              ),
              if (gates.length > 1)
                TextButton.icon(
                  onPressed: gates.any((draft) => draft.gate != null)
                      ? _fitGates
                      : null,
                  icon: const Icon(Icons.fit_screen_outlined, size: 18),
                  label: const Text('Fit gates'),
                ),
            ],
          ),
          if (!wide) ...[const SizedBox(height: 10), nameField],
          const SizedBox(height: 10),
          const Divider(),
          TextButton.icon(
            onPressed: () => setState(() => precision = !precision),
            icon: Icon(
              precision ? Icons.expand_less : Icons.expand_more,
              size: 18,
            ),
            label: const Text('Gate coordinates'),
            style: TextButton.styleFrom(alignment: Alignment.centerLeft),
          ),
          if (precision)
            Padding(
              key: ObjectKey(draft),
              padding: const EdgeInsets.symmetric(vertical: 8),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Row(
                    children: [
                      Expanded(
                        child: _numberField(
                          draft.lat,
                          'Latitude',
                          '°',
                          -90,
                          90,
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: _numberField(
                          draft.lon,
                          'Longitude',
                          '°',
                          -180,
                          180,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 12),
                  Row(
                    children: [
                      Expanded(
                        child: _numberField(
                          draft.heading,
                          'Travel heading',
                          '°',
                          null,
                          null,
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: _numberField(
                          draft.width,
                          'Gate width',
                          'm',
                          minGateWidth,
                          maxGateWidth,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 10),
                  const Text(
                    'The arrow points in the driving direction. Heading is clockwise from north.',
                    style: TextStyle(color: muted),
                  ),
                ],
              ),
            ),
          if (selected == 0 && reference != null)
            TextButton.icon(
              onPressed: _copyPosition,
              icon: const Icon(Icons.my_location, size: 17),
              label: const Text('Use this position for start / finish'),
              style: TextButton.styleFrom(alignment: Alignment.centerLeft),
            ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) => PopScope(
    canPop: !saving,
    child: Scaffold(
      appBar: AppBar(
        title: Text(widget.track == null ? 'Create track' : 'Edit track'),
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 12),
            child: FilledButton(
              onPressed: saving ? null : _save,
              child: const Text('Save track'),
            ),
          ),
        ],
      ),
      body: SafeArea(
        top: false,
        child: Column(
          children: [
            if (saving) const LinearProgressIndicator(minHeight: 2),
            if (saveError case final message?)
              Material(
                color: panel,
                child: Padding(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 14,
                    vertical: 6,
                  ),
                  child: Semantics(
                    liveRegion: true,
                    child: Text(
                      message,
                      style: const TextStyle(color: warning),
                    ),
                  ),
                ),
              ),
            Expanded(
              child: AbsorbPointer(
                absorbing: saving,
                child: LayoutBuilder(
                  builder: (context, constraints) {
                    final wide =
                        constraints.maxWidth >= 760 ||
                        (constraints.maxWidth >= 640 &&
                            constraints.maxHeight < 440);
                    final workspace = TrackEditorMap(
                      controller: map,
                      initialCenter: initialCenter,
                      initialZoom: initialZoom,
                      gates: gates.map((draft) => draft.gate).toList(),
                      selected: selected,
                      drawing: drawing,
                      firstEndpoint: firstEndpoint,
                      drawError: drawError,
                      onDraw: _draw,
                      onCancel: () => setState(_clearDrawing),
                      onPlace: _place,
                      onSelect: _select,
                    );
                    return Flex(
                      direction: wide ? Axis.horizontal : Axis.vertical,
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Expanded(flex: wide ? 1 : 6, child: workspace),
                        if (wide)
                          const VerticalDivider(width: 1)
                        else
                          const Divider(),
                        if (wide)
                          SizedBox(width: 354, child: _editorPanel(wide: true))
                        else
                          Expanded(flex: 4, child: _editorPanel(wide: false)),
                      ],
                    );
                  },
                ),
              ),
            ),
          ],
        ),
      ),
    ),
  );
}
