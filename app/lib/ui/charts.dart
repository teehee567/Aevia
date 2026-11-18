import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'common.dart';

class TraceChart extends StatelessWidget {
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
  Widget build(BuildContext context) => samples.isEmpty
      ? const EmptyState(
          'No trace yet',
          'Valid samples will appear here as the receiver sends them.',
        )
      : LayoutBuilder(
          builder: (context, box) => GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTapDown: onSeek == null
                ? null
                : (event) => onSeek!(
                    ((event.localPosition.dx - 48) / (box.maxWidth - 62)).clamp(
                      0,
                      1,
                    ),
                  ),
            onHorizontalDragUpdate: onSeek == null
                ? null
                : (event) => onSeek!(
                    ((event.localPosition.dx - 48) / (box.maxWidth - 62)).clamp(
                      0,
                      1,
                    ),
                  ),
            child: Semantics(
              label:
                  '$unit over elapsed time chart. Drag to move the replay cursor.',
              child: CustomPaint(
                size: Size(box.maxWidth, height),
                painter: TracePainter(
                  samples,
                  reference,
                  channel,
                  unit,
                  factor,
                  cursor,
                ),
              ),
            ),
          ),
        );
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
    this.cursor,
  );
  final List<Map<String, dynamic>> samples, reference;
  final String channel, unit;
  final double factor, cursor;
  @override
  void paint(Canvas canvas, Size size) {
    final area = Rect.fromLTRB(48, 18, size.width - 14, size.height - 28);
    final values = [...samples, ...reference]
        .where((e) => e['valid'] != false)
        .map((e) => numeric(e[channel]))
        .whereType<double>()
        .map((v) => v * factor)
        .toList();
    if (values.isEmpty || area.width <= 0) {
      chartText(canvas, 'Channel unavailable', const Offset(48, 70));
      return;
    }
    double low = math.min(0, values.reduce(math.min));
    double high = math.max(1, values.reduce(math.max));
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
  bool shouldRepaint(covariant TracePainter oldDelegate) => true;
}

class TrackMap extends StatelessWidget {
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
  Widget build(BuildContext context) => SizedBox(
    height: height,
    child:
        samples
                .where(
                  (e) =>
                      e['valid'] != false &&
                      numeric(e['lat']) != null &&
                      numeric(e['lon']) != null,
                )
                .length <
            2
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
                      painter: MapPainter(samples, reference, cursor, track),
                      child: const SizedBox.expand(),
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

class MapPainter extends CustomPainter {
  MapPainter(this.samples, this.reference, this.cursor, this.track);
  final List<Map<String, dynamic>> samples, reference;
  final double? cursor;
  final Map<String, dynamic>? track;
  @override
  void paint(Canvas canvas, Size size) {
    final valid = [...samples, ...reference]
        .where(
          (s) =>
              s['valid'] != false &&
              numeric(s['lat']) != null &&
              numeric(s['lon']) != null,
        )
        .toList();
    if (valid.length < 2) return;
    final lat0 = numeric(valid.first['lat'])!;
    final lon0 = numeric(valid.first['lon'])!;
    Offset meters(Map<String, dynamic> p) => Offset(
      ((numeric(p['lon']) ?? lon0) - lon0) *
          111320 *
          math.cos(lat0 * math.pi / 180),
      -((numeric(p['lat']) ?? lat0) - lat0) * 111320,
    );
    final positions = valid.map(meters).toList();
    final minX = positions.map((e) => e.dx).reduce(math.min),
        maxX = positions.map((e) => e.dx).reduce(math.max);
    final minY = positions.map((e) => e.dy).reduce(math.min),
        maxY = positions.map((e) => e.dy).reduce(math.max);
    final scale = math.min(
      (size.width - 70) / math.max(20, maxX - minX),
      (size.height - 70) / math.max(20, maxY - minY),
    );
    Offset project(Offset p) => Offset(
      size.width / 2 + (p.dx - (minX + maxX) / 2) * scale,
      size.height / 2 + (p.dy - (minY + maxY) / 2) * scale,
    );
    final grid = Paint()..color = line.withValues(alpha: .4);
    for (double x = 0; x < size.width; x += 40) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), grid);
    }
    for (double y = 0; y < size.height; y += 40) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), grid);
    }
    final maxSpeed = math.max(
      1.0,
      valid.map((e) => numeric(e['speed_mps']) ?? 0).reduce(math.max),
    );
    void route(List<Map<String, dynamic>> points, bool isReference) {
      Offset? previous;
      double? previousTime;
      for (final p in points) {
        if (p['valid'] == false ||
            numeric(p['lat']) == null ||
            numeric(p['lon']) == null) {
          previous = null;
          continue;
        }
        final next = project(meters(p));
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
                  (numeric(p['speed_mps']) ?? 0) / maxSpeed,
                )!;
          canvas.drawLine(
            previous,
            next,
            Paint()
              ..color = color
              ..strokeWidth = isReference ? 2 : 3
              ..strokeCap = StrokeCap.round,
          );
        }
        previous = next;
        previousTime = time;
      }
    }

    route(reference, true);
    route(samples, false);
    final index = cursor == null
        ? null
        : ((samples.length - 1) * cursor!.clamp(0, 1)).round();
    if (index != null &&
        samples.isNotEmpty &&
        numeric(samples[index]['lat']) != null &&
        samples[index]['valid'] != false) {
      final p = project(meters(samples[index]));
      canvas.drawCircle(p, 6, Paint()..color = ink);
      canvas.drawCircle(p, 3, Paint()..color = panel);
    }
    void gate(Map<String, dynamic> gate, String label) {
      if (numeric(gate['lat']) == null || numeric(gate['lon']) == null) return;
      final p = meters(gate);
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
  bool shouldRepaint(covariant MapPainter oldDelegate) => true;
}

class GGPlot extends StatelessWidget {
  const GGPlot({super.key, required this.samples});
  final List<Map<String, dynamic>> samples;
  @override
  Widget build(BuildContext context) => SizedBox(
    height: 240,
    child: CustomPaint(
      painter: GGPainter(samples),
      child: const SizedBox.expand(),
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
    final grid = Paint()
      ..color = line
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(center, radius, grid);
    canvas.drawCircle(center, radius / 2, grid);
    canvas.drawLine(
      center - Offset(radius, 0),
      center + Offset(radius, 0),
      grid,
    );
    canvas.drawLine(
      center - Offset(0, radius),
      center + Offset(0, radius),
      grid,
    );
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
  bool shouldRepaint(covariant GGPainter oldDelegate) => true;
}
