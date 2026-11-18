import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';
import 'package:url_launcher/url_launcher.dart';

import 'common.dart';
import 'track_gate.dart';

const _satelliteTiles = String.fromEnvironment('AEVIA_SATELLITE_TILES');
const _satelliteCredit = String.fromEnvironment('AEVIA_SATELLITE_ATTRIBUTION');
const _customSatellite = _satelliteTiles != '' && _satelliteCredit != '';
const _eox = 'https://tiles.maps.eox.at/wmts/1.0.0';
const _nsw = 'https://maps.six.nsw.gov.au/arcgis/rest/services/public';
const _mapZoomLimit = 23.0;

String gateLabel(int index) => index == 0 ? 'Start / finish' : 'Sector $index';

class TrackEditorMap extends StatefulWidget {
  const TrackEditorMap({
    super.key,
    required this.controller,
    required this.initialCenter,
    required this.initialZoom,
    required this.gates,
    required this.selected,
    required this.drawing,
    required this.firstEndpoint,
    required this.drawError,
    required this.onDraw,
    required this.onCancel,
    required this.onPlace,
    required this.onSelect,
  });

  final MapController controller;
  final LatLng initialCenter;
  final double initialZoom;
  final List<TrackGate?> gates;
  final int selected;
  final bool drawing;
  final LatLng? firstEndpoint;
  final String? drawError;
  final VoidCallback onDraw, onCancel;
  final ValueChanged<LatLng> onPlace;
  final ValueChanged<int> onSelect;

  @override
  State<TrackEditorMap> createState() => _TrackEditorMapState();
}

class _TrackEditorMapState extends State<TrackEditorMap> {
  bool streets = false, tileError = false;
  int tileRevision = 0;
  late List<TileLayer> tiles = _buildTiles();

  void _reload({required bool showStreets}) {
    setState(() {
      streets = showStreets;
      tileError = false;
      tileRevision++;
      tiles = _buildTiles();
    });
  }

  List<TileLayer> _buildTiles() {
    final revision = tileRevision;
    return [
      TileLayer(
        key: ValueKey('base-$revision'),
        urlTemplate: streets
            ? '$_eox/osm_3857/default/g/{z}/{y}/{x}.png'
            : _customSatellite
            ? _satelliteTiles
            : '$_eox/s2cloudless-2025_3857/default/g/{z}/{y}/{x}.jpg',
        maxNativeZoom: streets ? 18 : (_customSatellite ? 19 : 14),
        userAgentPackageName: 'open-race.aevia',
        panBuffer: 0,
        errorTileCallback: (_, _, _) {
          if (tileError || revision != tileRevision) return;
          WidgetsBinding.instance.addPostFrameCallback((_) {
            if (mounted && !tileError && revision == tileRevision) {
              setState(() => tileError = true);
            }
          });
        },
      ),
      if (!streets && !_customSatellite)
        TileLayer(
          key: ValueKey('nsw-$revision'),
          urlTemplate:
              '$_nsw/NSW_Imagery/MapServer/tile/{z}/{y}/{x}?blankTile=false',
          minZoom: 14,
          maxNativeZoom: 21,
          tileProvider: NetworkTileProvider(silenceExceptions: true),
          tileBounds: LatLngBounds(
            const LatLng(-37.6, 140.9),
            const LatLng(-28, 153.7),
          ),
          userAgentPackageName: 'open-race.aevia',
          panBuffer: 0,
        ),
    ];
  }

  void _zoom(double change) {
    final camera = widget.controller.camera;
    widget.controller.move(camera.center, camera.zoom + change);
  }

  Color _gateColor(int index) => index == widget.selected
      ? accent
      : index == 0
      ? Colors.white
      : warning;

  Widget _gates() {
    final selected = widget.selected;
    final indices = [
      for (var i = 0; i < widget.gates.length; i++)
        if (i != selected && widget.gates[i] != null) i,
      if (widget.gates[selected] != null) selected,
    ];
    return Stack(
      fit: StackFit.expand,
      children: [
        IgnorePointer(
          child: PolylineLayer(
            polylines: [
              for (final index in indices)
                Polyline(
                  points: widget.gates[index]!.endpoints
                      .map(mapPosition)
                      .toList(),
                  strokeWidth: index == selected ? 4 : 3,
                  borderStrokeWidth: 2,
                  borderColor: Colors.black,
                  color: _gateColor(index),
                ),
            ],
          ),
        ),
        MarkerLayer(
          markers: [
            for (final index in indices) ...[
              Marker(
                point: mapPosition(widget.gates[index]!.center),
                width: 26,
                height: 26,
                child: IgnorePointer(
                  child: Transform.rotate(
                    angle: widget.gates[index]!.headingRadians,
                    child: Icon(
                      Icons.navigation_rounded,
                      color: _gateColor(index),
                      size: 24,
                      shadows: const [
                        Shadow(blurRadius: 5, color: Colors.black),
                      ],
                    ),
                  ),
                ),
              ),
              Marker(
                point: mapPosition(widget.gates[index]!.center),
                width: 40,
                height: 40,
                alignment: Alignment.bottomCenter,
                child: Padding(
                  padding: const EdgeInsets.only(top: 10),
                  child: IgnorePointer(
                    ignoring: widget.drawing,
                    child: Tooltip(
                      message: gateLabel(index),
                      child: TextButton(
                        onPressed: () => widget.onSelect(index),
                        style: TextButton.styleFrom(
                          backgroundColor: const Color(0xee0e1218),
                          foregroundColor: _gateColor(index),
                          padding: EdgeInsets.zero,
                          shape: RoundedRectangleBorder(
                            side: BorderSide(color: _gateColor(index)),
                            borderRadius: BorderRadius.circular(5),
                          ),
                        ),
                        child: Text(index == 0 ? 'S/F' : 'S$index'),
                      ),
                    ),
                  ),
                ),
              ),
            ],
            if (widget.firstEndpoint case final point?)
              Marker(
                point: point,
                width: 20,
                height: 20,
                child: IgnorePointer(
                  child: DecoratedBox(
                    decoration: BoxDecoration(
                      shape: BoxShape.circle,
                      color: accent,
                      border: Border.all(color: Colors.black, width: 2),
                    ),
                  ),
                ),
              ),
          ],
        ),
      ],
    );
  }

  void _showCredits() => showDialog<void>(
    context: context,
    builder: (context) => AlertDialog(
      title: const Text('Map credits'),
      content: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            if (streets) const Text('Street rendering © EOX and MapServer'),
            if (!streets && _customSatellite) const Text(_satelliteCredit),
            if (!streets && !_customSatellite) ...[
              const Text('Contains modified Copernicus Sentinel data 2025.'),
              _creditLink(
                'EOxCloudless by EOX IT Services',
                'https://cloudless.eox.at/',
              ),
              _creditLink(
                'NSW Imagery © NSW Spatial Services, CC BY 3.0',
                'https://data.nsw.gov.au/data/dataset/nsw-imagery',
              ),
            ],
            if (streets || !_customSatellite)
              _creditLink('EOX::Maps', 'https://maps.eox.at/'),
            _creditLink(
              'OpenStreetMap contributors',
              'https://www.openstreetmap.org/copyright',
            ),
            _creditLink('Place search by Photon', 'https://photon.komoot.io/'),
          ],
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('Close'),
        ),
      ],
    ),
  );

  Widget _creditLink(String label, String url) => TextButton(
    onPressed: () => launchUrl(Uri.parse(url)),
    child: Text(label),
  );

  @override
  Widget build(BuildContext context) => LayoutBuilder(
    builder: (context, constraints) => Stack(
      fit: StackFit.expand,
      children: [
        FlutterMap(
          mapController: widget.controller,
          options: MapOptions(
            initialCenter: widget.initialCenter,
            initialZoom: widget.initialZoom,
            minZoom: 2,
            maxZoom: _mapZoomLimit,
            backgroundColor: const Color(0xff1c2a2b),
            interactionOptions: InteractionOptions(
              cursorKeyboardRotationOptions:
                  CursorKeyboardRotationOptions.disabled(),
              flags:
                  InteractiveFlag.all &
                  ~InteractiveFlag.rotate &
                  ~InteractiveFlag.doubleTapZoom &
                  ~InteractiveFlag.doubleTapDragZoom,
            ),
            onTap: (_, point) {
              if (widget.drawing) widget.onPlace(mapPosition(point));
            },
          ),
          children: [...tiles, _gates()],
        ),
        if (constraints.maxHeight > 216)
          Positioned(
            right: 12,
            top: 68,
            child: Material(
              color: panel,
              borderRadius: BorderRadius.circular(6),
              elevation: 3,
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  IconButton(
                    tooltip: 'Zoom in',
                    onPressed: () => _zoom(1),
                    icon: const Icon(Icons.add),
                  ),
                  IconButton(
                    tooltip: 'Zoom out',
                    onPressed: () => _zoom(-1),
                    icon: const Icon(Icons.remove),
                  ),
                  IconButton(
                    tooltip: streets
                        ? 'Show satellite imagery'
                        : 'Show street map',
                    onPressed: () => _reload(showStreets: !streets),
                    icon: Icon(
                      streets
                          ? Icons.satellite_alt_outlined
                          : Icons.map_outlined,
                    ),
                  ),
                ],
              ),
            ),
          ),
        if (constraints.maxHeight > 126)
          Positioned(
            bottom: 35,
            left: 12,
            right: constraints.maxHeight > 216 ? 66 : 12,
            child: Align(
              alignment: Alignment.centerLeft,
              child: widget.drawing
                  ? Material(
                      color: panel,
                      elevation: 3,
                      borderRadius: BorderRadius.circular(6),
                      child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Flexible(
                            child: Padding(
                              padding: const EdgeInsets.fromLTRB(12, 6, 0, 6),
                              child: Semantics(
                                liveRegion: true,
                                child: Text(
                                  widget.drawError ??
                                      (widget.firstEndpoint == null
                                          ? 'Mark one edge of the track'
                                          : 'Mark the opposite edge'),
                                  style: TextStyle(
                                    color: widget.drawError == null
                                        ? ink
                                        : warning,
                                  ),
                                ),
                              ),
                            ),
                          ),
                          IconButton(
                            tooltip: 'Cancel drawing',
                            onPressed: widget.onCancel,
                            icon: const Icon(Icons.close, size: 18),
                          ),
                        ],
                      ),
                    )
                  : FilledButton.icon(
                      onPressed: widget.onDraw,
                      icon: const Icon(
                        Icons.add_location_alt_outlined,
                        size: 18,
                      ),
                      label: Text(
                        'Draw ${gateLabel(widget.selected).toLowerCase()}',
                      ),
                    ),
            ),
          ),
        Positioned(
          right: 8,
          bottom: 3,
          child: Material(
            color: const Color(0xee0e1218),
            borderRadius: BorderRadius.circular(4),
            child: InkWell(
              onTap: _showCredits,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 7, vertical: 4),
                child: Text(
                  streets
                      ? '© EOX · © OpenStreetMap'
                      : _customSatellite
                      ? _satelliteCredit
                      : '© EOX · © NSW Spatial Services',
                  style: const TextStyle(fontSize: 10),
                ),
              ),
            ),
          ),
        ),
        if (tileError && constraints.maxHeight > 245)
          Positioned(
            top: 68,
            left: 12,
            right: 66,
            child: Align(
              alignment: Alignment.centerLeft,
              child: TextButton.icon(
                style: TextButton.styleFrom(
                  backgroundColor: panel,
                  foregroundColor: warning,
                ),
                onPressed: () => _reload(showStreets: streets),
                icon: const Icon(Icons.refresh, size: 17),
                label: const Text('Retry map tiles'),
              ),
            ),
          ),
        Positioned(
          top: 12,
          left: 12,
          right: 12,
          child: Align(
            alignment: Alignment.topLeft,
            child: ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 460),
              child: _LocationSearch(
                resultsHeight: (constraints.maxHeight - 72).clamp(40, 228),
                onSelect: (point) =>
                    widget.controller.move(mapPosition(point), 16),
              ),
            ),
          ),
        ),
      ],
    ),
  );
}

class _Place {
  const _Place(this.label, this.position);
  final String label;
  final LatLng position;
}

class _LocationSearch extends StatefulWidget {
  const _LocationSearch({required this.resultsHeight, required this.onSelect});
  final double resultsHeight;
  final ValueChanged<LatLng> onSelect;

  @override
  State<_LocationSearch> createState() => _LocationSearchState();
}

class _LocationSearchState extends State<_LocationSearch> {
  final query = TextEditingController();
  final client = http.Client();
  List<_Place> results = [];
  String? message;
  bool searching = false;
  int searchId = 0;

  @override
  void dispose() {
    query.dispose();
    client.close();
    super.dispose();
  }

  void _reset() {
    setState(() {
      searchId++;
      searching = false;
      message = null;
      results = [];
    });
  }

  Future<void> _search() async {
    final input = query.text.trim();
    if (input.isEmpty || searching) return;
    FocusScope.of(context).unfocus();
    final coordinates = RegExp(
      r'^([+-]?\d+(?:\.\d+)?)\s*[, ]\s*([+-]?\d+(?:\.\d+)?)$',
    ).firstMatch(input);
    if (coordinates != null) {
      final point = trackPosition(
        double.tryParse(coordinates[1]!),
        double.tryParse(coordinates[2]!),
      );
      _reset();
      if (point == null || point.latitude.abs() > maxMapLatitude) {
        setState(
          () => message =
              'Use a latitude from −85° to 85° and longitude from −180° to 180°.',
        );
      } else {
        widget.onSelect(point);
      }
      return;
    }
    final requested = ++searchId;
    setState(() {
      searching = true;
      results = [];
      message = null;
    });
    try {
      final response = await client
          .get(
            Uri.https('photon.komoot.io', '/api/', {'q': input, 'limit': '5'}),
            headers: const {
              'Accept': 'application/json',
              if (!kIsWeb) 'User-Agent': 'Aevia/1.0 (open-race track editor)',
            },
          )
          .timeout(const Duration(seconds: 10));
      if (response.statusCode != 200)
        throw const FormatException('Place search failed');
      final found = <_Place>[];
      for (final feature in objects(
        object(jsonDecode(utf8.decode(response.bodyBytes)))['features'],
      )) {
        final coordinates = object(feature['geometry'])['coordinates'];
        if (coordinates is! List || coordinates.length < 2) continue;
        final position = trackPosition(
          numeric(coordinates[1]),
          numeric(coordinates[0]),
        );
        if (position == null || position.latitude.abs() > maxMapLatitude)
          continue;
        final properties = object(feature['properties']);
        final address = [
          properties['street'],
          properties['housenumber'],
        ].whereType<String>().join(' ');
        final label = {
          properties['name'] ?? address,
          properties['district'],
          properties['city'],
          properties['state'],
          properties['country'],
        }.whereType<String>().where((part) => part.isNotEmpty).join(', ');
        if (label.isNotEmpty) found.add(_Place(label, position));
      }
      if (!mounted || searchId != requested) return;
      setState(() {
        searching = false;
        results = found;
        if (results.isEmpty)
          message = 'No matches. Try a nearby town or coordinates.';
      });
    } on Exception {
      if (!mounted || searchId != requested) return;
      setState(() {
        searching = false;
        message = 'Place search is unavailable. You can enter coordinates.';
      });
    }
  }

  @override
  Widget build(BuildContext context) => Material(
    color: panel,
    borderRadius: BorderRadius.circular(6),
    clipBehavior: Clip.antiAlias,
    elevation: 3,
    child: Column(
      mainAxisSize: MainAxisSize.min,
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        TextField(
          controller: query,
          textInputAction: TextInputAction.search,
          onSubmitted: (_) => _search(),
          onChanged: (_) => _reset(),
          decoration: InputDecoration(
            hintText: 'Search circuit, address or lat, lon',
            contentPadding: const EdgeInsets.symmetric(
              horizontal: 12,
              vertical: 14,
            ),
            suffixIcon: searching
                ? const Padding(
                    padding: EdgeInsets.all(14),
                    child: SizedBox.square(
                      dimension: 18,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    ),
                  )
                : IconButton(
                    tooltip: 'Search location',
                    onPressed: _search,
                    icon: const Icon(Icons.search),
                  ),
          ),
        ),
        if (message case final text?)
          Padding(
            padding: const EdgeInsets.all(10),
            child: Semantics(liveRegion: true, child: Text(text)),
          ),
        if (results.isNotEmpty)
          ConstrainedBox(
            constraints: BoxConstraints(maxHeight: widget.resultsHeight),
            child: ListView(
              shrinkWrap: true,
              padding: EdgeInsets.zero,
              children: [
                for (final result in results)
                  ListTile(
                    leading: const Icon(Icons.place_outlined, size: 20),
                    title: Text(result.label),
                    onTap: () {
                      query.text = result.label;
                      _reset();
                      widget.onSelect(result.position);
                    },
                  ),
              ],
            ),
          ),
      ],
    ),
  );
}
