import 'dart:math' as math;
import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';

class TraceChart extends StatefulWidget {
  const TraceChart({
    super.key,
    required this.samples,
    this.reference = const [],
    this.channel = 'speed_mps',
    this.unit = 'km/h',
    this.factor = 3.6,
    this.cursor = 0,
    this.onSeek,
    this.height = 220,
  });
  final List<Map<String, dynamic>> samples, reference;
  final String channel, unit;
  final double factor, cursor, height;
  final ValueChanged<double>? onSeek;
  @override
  State<TraceChart> createState() => _TraceChartState();
}

class _TraceChartState extends State<TraceChart> {
  (double, double)? range() {
    double low = double.infinity, high = double.negativeInfinity;
    for (final series in [widget.samples, widget.reference]) {
      for (final point in series) {
        final value = numeric(point[widget.channel]);
        if (point['valid'] == false || value == null) continue;
        low = math.min(low, value * widget.factor);
        high = math.max(high, value * widget.factor);
      }
    }
    return low.isFinite ? (low, high) : null;
  }

  late var limits = range();
  @override
  void didUpdateWidget(TraceChart old) {
    super.didUpdateWidget(old);
    if (old.samples != widget.samples ||
        old.reference != widget.reference ||
        old.channel != widget.channel ||
        old.factor != widget.factor)
      limits = range();
  }

  @override
  Widget build(BuildContext context) {
    final onSeek = widget.onSeek;
    return widget.samples.isEmpty
        ? const EmptyState(
            'No trace yet',
            'Valid samples will appear here as the receiver sends them.',
          )
        : LayoutBuilder(
            builder: (context, box) => GestureDetector(
              behavior: HitTestBehavior.opaque,
              onTapDown: onSeek == null
                  ? null
                  : (event) => onSeek(
                      ((event.localPosition.dx - 48) / (box.maxWidth - 62))
                          .clamp(0, 1),
                    ),
              onHorizontalDragUpdate: onSeek == null
                  ? null
                  : (event) => onSeek(
                      ((event.localPosition.dx - 48) / (box.maxWidth - 62))
                          .clamp(0, 1),
                    ),
              child: Semantics(
                label:
                    '${widget.unit} over elapsed time chart. Drag to move the replay cursor.',
                child: CustomPaint(
                  foregroundPainter: limits == null
                      ? null
                      : _TraceCursor(widget.cursor),
                  child: RepaintBoundary(
                    child: CustomPaint(
                      size: Size(box.maxWidth, widget.height),
                      painter: TracePainter(
                        widget.samples,
                        widget.reference,
                        widget.channel,
                        widget.unit,
                        widget.factor,
                        limits,
                      ),
                    ),
                  ),
                ),
              ),
            ),
          );
  }
}

void chartText(
  Canvas canvas,
  String text,
  Offset offset, {
  Color color = muted,
  double size = 10,
  TextAlign align = TextAlign.left,
}) {
  final painter = TextPainter(
    text: TextSpan(
      text: text,
      style: TextStyle(color: color, fontSize: size, fontFamily: 'Segoe UI'),
    ),
    textDirection: TextDirection.ltr,
    textAlign: align,
  )..layout();
  painter.paint(
    canvas,
    Offset(
      offset.dx -
          (align == TextAlign.right
              ? painter.width
              : align == TextAlign.center
              ? painter.width / 2
              : 0),
      offset.dy,
    ),
  );
}

class TracePainter extends CustomPainter {
  TracePainter(
    this.samples,
    this.reference,
    this.channel,
    this.unit,
    this.factor,
    this.limits,
  );
  final List<Map<String, dynamic>> samples, reference;
  final String channel, unit;
  final double factor;
  final (double, double)? limits;
  @override
  void paint(Canvas canvas, Size size) {
    final area = Rect.fromLTRB(48, 18, size.width - 14, size.height - 28);
    final values = limits;
    if (values == null || area.width <= 0) {
      chartText(canvas, 'Channel unavailable', const Offset(48, 70));
      return;
    }
    double low = math.min(0, values.$1);
    double high = math.max(1, values.$2);
    final margin = (high - low) * .08;
    high += margin;
    if (low < 0) low -= margin;
    final span = high - low;
    final grid = Paint()
      ..color = line
      ..strokeWidth = 1;
    final origin = numeric(samples.first['time_s']) ?? 0;
    final referenceOrigin = reference.isEmpty
        ? 0.0
        : numeric(reference.first['time_s']) ?? 0;
    final end = math.max(
      1.0,
      math.max(
        (numeric(samples.last['time_s']) ?? origin) - origin,
        reference.isEmpty
            ? 0
            : (numeric(reference.last['time_s']) ?? referenceOrigin) -
                  referenceOrigin,
      ),
    );
    for (int i = 0; i <= 4; i++) {
      final y = area.top + area.height * i / 4;
      canvas.drawLine(Offset(area.left, y), Offset(area.right, y), grid);
      chartText(
        canvas,
        (high - span * i / 4).toStringAsFixed(span < 10 ? 1 : 0),
        Offset(area.left - 8, y - 7),
        align: TextAlign.right,
      );
      final x = area.left + area.width * i / 4;
      chartText(
        canvas,
        (end * i / 4).toStringAsFixed(end < 20 ? 1 : 0),
        Offset(x, area.bottom + 7),
        align: TextAlign.center,
      );
    }
    chartText(canvas, unit, const Offset(0, 0));
    chartText(
      canvas,
      's',
      Offset(size.width - 2, area.bottom + 7),
      align: TextAlign.right,
    );
    void trace(List<Map<String, dynamic>> points, Color color, double start) {
      final path = Path();
      bool open = false;
      double? lastTime;
      for (final point in points) {
        final value = numeric(point[channel]);
        final time = numeric(point['time_s']);
        if (point['valid'] == false || value == null || time == null) {
          open = false;
          continue;
        }
        if (lastTime != null && time - lastTime > 1.5) open = false;
        final p = Offset(
          area.left + (time - start) / end * area.width,
          area.bottom - (value * factor - low) / span * area.height,
        );
        if (open) {
          path.lineTo(p.dx, p.dy);
        } else {
          path.moveTo(p.dx, p.dy);
        }
        open = true;
        lastTime = time;
      }
      canvas.save();
      canvas.clipRect(area);
      canvas.drawPath(
        path,
        Paint()
          ..color = color
          ..strokeWidth = 1.6
          ..style = PaintingStyle.stroke,
      );
      canvas.restore();
    }

    if (reference.isNotEmpty) trace(reference, compareColor, referenceOrigin);
    trace(samples, accent, origin);
  }

  @override
  bool shouldRepaint(covariant TracePainter old) =>
      old.samples != samples ||
      old.reference != reference ||
      old.channel != channel ||
      old.unit != unit ||
      old.factor != factor ||
      old.limits != limits;
}

class _TraceCursor extends CustomPainter {
  const _TraceCursor(this.cursor);
  final double cursor;
  @override
  void paint(Canvas canvas, Size size) {
    final area = Rect.fromLTRB(48, 18, size.width - 14, size.height - 28);
    if (area.isEmpty) return;
    final x = area.left + cursor.clamp(0, 1) * area.width;
    canvas.drawLine(
      Offset(x, area.top),
      Offset(x, area.bottom),
      Paint()
        ..color = ink.withValues(alpha: .6)
        ..strokeWidth = 1,
    );
  }

  @override
  bool shouldRepaint(covariant _TraceCursor old) => old.cursor != cursor;
}

class TrackMap extends StatefulWidget {
  const TrackMap({
    super.key,
    required this.samples,
    this.reference = const [],
    this.cursor = 0,
    this.track,
    this.height = 320,
  });
  final List<Map<String, dynamic>> samples, reference;
  final double? cursor;
  final double height;
  final Map<String, dynamic>? track;
  @override
  State<TrackMap> createState() => _TrackMapState();
}

class _TrackMapState extends State<TrackMap> {
  late var bounds = _RouteBounds(widget.samples, widget.reference);
  @override
  void didUpdateWidget(TrackMap old) {
    super.didUpdateWidget(old);
    if (old.samples != widget.samples || old.reference != widget.reference) {
      bounds = _RouteBounds(widget.samples, widget.reference);
    }
  }

  @override
  Widget build(BuildContext context) => SizedBox(
    height: widget.height,
    child: !bounds.hasRoute
        ? const EmptyState(
            'No position trace',
            'A position fix is required to draw the recorded racing line.',
            icon: Icons.route,
          )
        : Stack(
            children: [
              Positioned.fill(
                child: ClipRect(
                  child: InteractiveViewer(
                    minScale: 1,
                    maxScale: 8,
                    child: CustomPaint(
                      foregroundPainter: MapCursorPainter._(
                        widget.samples,
                        widget.cursor,
                        bounds,
                      ),
                      child: RepaintBoundary(
                        child: CustomPaint(
                          painter: _RoutePainter(
                            widget.samples,
                            widget.reference,
                            widget.track,
                            bounds,
                          ),
                          child: const SizedBox.expand(),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
              const Positioned(
                left: 8,
                top: 5,
                child: Text(
                  'N ↑   LOCAL TRACK VIEW',
                  style: TextStyle(fontSize: 10, color: muted),
                ),
              ),
              const Positioned(
                right: 4,
                bottom: 4,
                child: Text(
                  'Scroll / pinch to zoom · drag to pan',
                  style: TextStyle(fontSize: 10, color: muted),
                ),
              ),
            ],
          ),
  );
}

class _RouteBounds {
  double lat0 = 0, lon0 = 0, lonScale = 0;
  double minX = 0, maxX = 0, minY = 0, maxY = 0, maxSpeed = 1;
  bool hasRoute = false;

  _RouteBounds(
    List<Map<String, dynamic>> samples,
    List<Map<String, dynamic>> reference,
  ) {
    var count = 0;
    for (final (series, primary) in [(samples, true), (reference, false)]) {
      for (final p in series) {
        final lat = numeric(p['lat']), lon = numeric(p['lon']);
        if (p['valid'] == false || lat == null || lon == null) continue;
        if (count == 0) {
          lat0 = lat;
          lon0 = lon;
          lonScale = 111320 * math.cos(lat0 * math.pi / 180);
        }
        final point = meters(lat, lon);
        minX = math.min(minX, point.dx);
        maxX = math.max(maxX, point.dx);
        minY = math.min(minY, point.dy);
        maxY = math.max(maxY, point.dy);
        maxSpeed = math.max(maxSpeed, numeric(p['speed_mps']) ?? 0);
        count++;
      }
      if (primary) hasRoute = count >= 2;
    }
  }

  Offset meters(double lat, double lon) =>
      Offset((lon - lon0) * lonScale, -(lat - lat0) * 111320);
  double scale(Size size) => math.min(
    (size.width - 70) / math.max(20, maxX - minX),
    (size.height - 70) / math.max(20, maxY - minY),
  );
  Offset project(Offset p, Size size, double scale) => Offset(
    size.width / 2 + (p.dx - (minX + maxX) / 2) * scale,
    size.height / 2 + (p.dy - (minY + maxY) / 2) * scale,
  );
}

class _RoutePainter extends CustomPainter {
  _RoutePainter(this.samples, this.reference, this.track, this.bounds);
  final List<Map<String, dynamic>> samples, reference;
  final Map<String, dynamic>? track;
  final _RouteBounds bounds;
  @override
  void paint(Canvas canvas, Size size) {
    final scale = bounds.scale(size);
    if (scale <= 0) return;
    Offset project(Offset point) => bounds.project(point, size, scale);
    final grid = Paint()..color = line.withValues(alpha: .4);
    for (double x = 0; x < size.width; x += 40) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), grid);
    }
    for (double y = 0; y < size.height; y += 40) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), grid);
    }
    void route(List<Map<String, dynamic>> points, bool isReference) {
      final stroke = Paint()
        ..strokeWidth = isReference ? 2 : 3
        ..strokeCap = StrokeCap.round;
      Offset? previous;
      double? previousTime;
      for (final p in points) {
        if (p['valid'] == false ||
            numeric(p['lat']) == null ||
            numeric(p['lon']) == null) {
          previous = null;
          continue;
        }
        final next = project(
          bounds.meters(numeric(p['lat'])!, numeric(p['lon'])!),
        );
        final time = numeric(p['time_s']);
        if (previous != null &&
            (time == null ||
                previousTime == null ||
                time - previousTime <= 1.5)) {
          final color = isReference
              ? compareColor.withValues(alpha: .6)
              : Color.lerp(
                  const Color(0xff4588ac),
                  accent,
                  (numeric(p['speed_mps']) ?? 0) / bounds.maxSpeed,
                )!;
          canvas.drawLine(previous, next, stroke..color = color);
        }
        previous = next;
        previousTime = time;
      }
    }

    route(reference, true);
    route(samples, false);
    void gate(Map<String, dynamic> gate, String label) {
      if (numeric(gate['lat']) == null || numeric(gate['lon']) == null) return;
      final p = bounds.meters(numeric(gate['lat'])!, numeric(gate['lon'])!);
      final heading = (numeric(gate['heading_deg']) ?? 0) * math.pi / 180;
      final half = (numeric(gate['width_m']) ?? 20) / 2;
      final direction = Offset(
        math.cos(heading) * half,
        math.sin(heading) * half,
      );
      canvas.drawLine(
        project(p - direction),
        project(p + direction),
        Paint()
          ..color = ink
          ..strokeWidth = 2,
      );
      chartText(canvas, label, project(p) + const Offset(6, -18), color: ink);
    }

    if (track != null) {
      gate(object(track!['start']), 'S/F');
      final sectors = objects(track!['sectors']);
      for (int i = 0; i < sectors.length; i++) {
        gate(sectors[i], 'S${i + 1}');
      }
    }
    final scaleM = math
        .pow(10, (math.log(80 / scale) / math.ln10).floor())
        .toDouble();
    canvas.drawLine(
      Offset(8, size.height - 20),
      Offset(8 + scaleM * scale, size.height - 20),
      Paint()
        ..color = muted
        ..strokeWidth = 2,
    );
    chartText(canvas, '${number(scaleM, 0)} m', Offset(8, size.height - 36));
  }

  @override
  bool shouldRepaint(covariant _RoutePainter old) =>
      old.bounds != bounds || old.track != track;
}

class MapCursorPainter extends CustomPainter {
  MapCursorPainter._(this.samples, this.cursor, this._bounds);
  final List<Map<String, dynamic>> samples;
  final double? cursor;
  final _RouteBounds _bounds;
  @override
  void paint(Canvas canvas, Size size) {
    if (cursor == null || samples.isEmpty) return;
    final sample =
        samples[((samples.length - 1) * cursor!.clamp(0, 1)).round()];
    final lat = numeric(sample['lat']), lon = numeric(sample['lon']);
    if (sample['valid'] == false || lat == null || lon == null) return;
    final p = _bounds.project(
      _bounds.meters(lat, lon),
      size,
      _bounds.scale(size),
    );
    canvas.drawCircle(p, 6, Paint()..color = ink);
    canvas.drawCircle(p, 3, Paint()..color = panel);
  }

  @override
  bool shouldRepaint(covariant MapCursorPainter old) =>
      old._bounds != _bounds || old.cursor != cursor;
}

class LiveGMeter extends StatelessWidget {
  const LiveGMeter({
    super.key,
    required this.acceleration,
    required this.connected,
    this.trail = const [],
  });

  final ({double lateral, double longitudinal})? acceleration;
  final bool connected;
  final List<GTrailPoint> trail;

  @override
  Widget build(BuildContext context) {
    final reading = acceleration;
    return DecoratedBox(
      decoration: BoxDecoration(
        color: Theme.of(context).scaffoldBackgroundColor,
        border: Border.all(color: line),
        borderRadius: BorderRadius.circular(4),
      ),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Expanded(
                  child: Text(
                    'G-meter',
                    style: Theme.of(context).textTheme.titleMedium,
                  ),
                ),
                const Text(
                  '2 g scale',
                  style: TextStyle(color: muted, fontSize: 11),
                ),
              ],
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                SizedBox.square(
                  dimension: 96,
                  child: CustomPaint(
                    painter: _LiveGMeterPainter(
                      reading == null
                          ? null
                          : Offset(-reading.lateral, -reading.longitudinal),
                      trail,
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      _axisValue('Lateral', reading?.lateral),
                      _axisValue('Longitudinal', reading?.longitudinal),
                    ],
                  ),
                ),
              ],
            ),
            const SizedBox(height: 6),
            Text(
              reading != null
                  ? 'Positive left and forward'
                  : connected
                  ? 'Waiting for vehicle alignment'
                  : 'Connect the receiver to see G-force',
              style: const TextStyle(color: muted, fontSize: 11),
            ),
          ],
        ),
      ),
    );
  }

  Widget _axisValue(String axis, double? value) => Text(
    '$axis  ${number(value, 2)} g',
    style: const TextStyle(
      fontSize: 14,
      fontFeatures: [FontFeature.tabularFigures()],
    ),
  );
}

class _LiveGMeterPainter extends CustomPainter {
  const _LiveGMeterPainter(this.point, this.trail);

  final Offset? point;
  final List<GTrailPoint> trail;

  @override
  void paint(Canvas canvas, Size size) {
    final center = size.center(Offset.zero);
    final radius = size.shortestSide / 2 - 4;
    _drawGGrid(canvas, center, radius);
    if (point case final reading?) {
      Offset project(Offset g) =>
          center + g / math.max(2.0, g.distance) * radius;
      final paint = Paint();
      for (var i = 0; i + 1 < trail.length; i++) {
        final p = trail[i];
        final age = (trail.last.tUs - p.tUs) / AeviaController.gTrailDurationUs;
        paint.color = accent.withValues(alpha: .22 * (1 - age).clamp(0, 1));
        canvas.drawCircle(project(Offset(p.x, -p.y)), 2.5, paint);
      }
      final dot = project(reading);
      canvas.drawCircle(dot, 8, Paint()..color = accent.withValues(alpha: .18));
      canvas.drawCircle(dot, 4, Paint()..color = accent);
    }
  }

  @override
  bool shouldRepaint(covariant _LiveGMeterPainter oldDelegate) =>
      point != oldDelegate.point || trail != oldDelegate.trail;
}

void _drawGGrid(Canvas canvas, Offset center, double radius) {
  final grid = Paint()
    ..color = line
    ..style = PaintingStyle.stroke;
  canvas.drawCircle(center, radius, grid);
  canvas.drawCircle(center, radius / 2, grid);
  canvas.drawLine(center - Offset(radius, 0), center + Offset(radius, 0), grid);
  canvas.drawLine(center - Offset(0, radius), center + Offset(0, radius), grid);
}

class GGPlot extends StatelessWidget {
  const GGPlot({super.key, required this.samples});
  final List<Map<String, dynamic>> samples;
  @override
  Widget build(BuildContext context) => SizedBox(
    height: 240,
    child: RepaintBoundary(
      child: CustomPaint(
        painter: GGPainter(samples),
        child: const SizedBox.expand(),
      ),
    ),
  );
}

class GGPainter extends CustomPainter {
  GGPainter(this.samples);
  final List<Map<String, dynamic>> samples;
  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = math.min(size.width, size.height) / 2 - 32;
    final points = samples
        .where(
          (s) =>
              s['valid'] != false &&
              numeric(s['accel_lat_g']) != null &&
              numeric(s['accel_long_g']) != null,
        )
        .toList();
    final extent = points
        .fold<double>(
          1,
          (max, s) => math.max(
            max,
            math.max(
              numeric(s['accel_lat_g'])!.abs(),
              numeric(s['accel_long_g'])!.abs(),
            ),
          ),
        )
        .ceilToDouble();
    _drawGGrid(canvas, center, radius);
    chartText(
      canvas,
      '+${number(extent, 1)} g acceleration',
      Offset(center.dx, 2),
      align: TextAlign.center,
    );
    chartText(
      canvas,
      'Braking',
      Offset(center.dx, size.height - 16),
      align: TextAlign.center,
    );
    chartText(
      canvas,
      'Lateral g',
      Offset(size.width - 2, center.dy + 8),
      align: TextAlign.right,
    );
    for (final s in points) {
      canvas.drawCircle(
        center +
            Offset(
              numeric(s['accel_lat_g'])! / extent * radius,
              -numeric(s['accel_long_g'])! / extent * radius,
            ),
        1.6,
        Paint()..color = accent.withValues(alpha: .45),
      );
    }
    if (points.isEmpty)
      chartText(
        canvas,
        'G-force unavailable',
        Offset(center.dx, center.dy - 6),
        align: TextAlign.center,
      );
  }

  @override
  bool shouldRepaint(covariant GGPainter old) => old.samples != samples;
}
