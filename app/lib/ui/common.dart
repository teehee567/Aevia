import 'dart:math' as math;
import 'package:flutter/material.dart';

const ink = Color(0xffe5e9ee);
const muted = Color(0xff9ba5b3);
const panel = Color(0xff151a21);
const line = Color(0xff2a323d);
const accent = Color(0xff7cdbbb);
const compareColor = Color(0xffffc078);
const warning = Color(0xffffc078);

ThemeData aeviaTheme() => ThemeData(
  brightness: Brightness.dark,
  useMaterial3: true,
  scaffoldBackgroundColor: const Color(0xff0e1218),
  colorScheme: const ColorScheme.dark(
    primary: accent,
    surface: panel,
    onSurface: ink,
    outline: line,
  ),
  fontFamily: 'Segoe UI',
  textTheme: const TextTheme(
    bodyMedium: TextStyle(fontSize: 13, height: 1.4),
    bodySmall: TextStyle(fontSize: 12, color: muted),
    titleLarge: TextStyle(fontSize: 24, fontWeight: FontWeight.w600),
    titleMedium: TextStyle(fontSize: 15, fontWeight: FontWeight.w600),
  ),
  dividerTheme: const DividerThemeData(color: line, space: 1),
  inputDecorationTheme: InputDecorationTheme(
    isDense: true,
    filled: true,
    fillColor: const Color(0xff0e1218),
    border: OutlineInputBorder(
      borderRadius: BorderRadius.circular(4),
      borderSide: const BorderSide(color: line),
    ),
  ),
  filledButtonTheme: FilledButtonThemeData(
    style: FilledButton.styleFrom(
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(4)),
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 16),
    ),
  ),
  outlinedButtonTheme: OutlinedButtonThemeData(
    style: OutlinedButton.styleFrom(
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(4)),
      side: const BorderSide(color: line),
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 16),
    ),
  ),
  chipTheme: ChipThemeData(
    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(3)),
    side: const BorderSide(color: line),
  ),
);

String number(dynamic value, [int digits = 1]) =>
    value is num && value.isFinite ? value.toStringAsFixed(digits) : '—';
String elapsed(dynamic value) {
  if (value is! num || !value.isFinite) return '—';
  final minutes = value ~/ 60;
  return '$minutes:${(value % 60).toStringAsFixed(3).padLeft(6, '0')}';
}

Map<String, dynamic> object(dynamic value) =>
    value is Map ? Map<String, dynamic>.from(value) : {};
List<Map<String, dynamic>> objects(dynamic value) => value is List
    ? value.whereType<Map>().map((e) => Map<String, dynamic>.from(e)).toList()
    : [];
double? numeric(dynamic value) =>
    value is num && value.isFinite ? value.toDouble() : null;

class Section extends StatelessWidget {
  const Section({
    super.key,
    required this.title,
    required this.child,
    this.trailing,
    this.padding = 18,
  });
  final String title;
  final Widget child;
  final Widget? trailing;
  final double padding;
  @override
  Widget build(BuildContext context) => Material(
    color: panel,
    shape: RoundedRectangleBorder(
      side: const BorderSide(color: line),
      borderRadius: BorderRadius.circular(4),
    ),
    child: Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Padding(
          padding: EdgeInsets.symmetric(horizontal: padding, vertical: 12),
          child: Row(
            children: [
              Expanded(
                child: Text(
                  title,
                  style: Theme.of(context).textTheme.titleMedium,
                ),
              ),
              ?trailing,
            ],
          ),
        ),
        const Divider(),
        Padding(padding: EdgeInsets.all(padding), child: child),
      ],
    ),
  );
}

class Metric extends StatelessWidget {
  const Metric(
    this.label,
    this.value, {
    super.key,
    this.unit = '',
    this.large = false,
    this.color = ink,
    this.note,
  });
  final String label, value, unit;
  final bool large;
  final Color color;
  final String? note;
  @override
  Widget build(BuildContext context) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 4),
    child: Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(label, style: const TextStyle(color: muted, fontSize: 12)),
        const SizedBox(height: 6),
        Wrap(
          crossAxisAlignment: WrapCrossAlignment.end,
          spacing: 7,
          children: [
            Text(
              value,
              style: TextStyle(
                fontSize: large ? 60 : 27,
                color: color,
                fontWeight: FontWeight.w500,
                height: 1.1,
                fontFeatures: const [FontFeature.tabularFigures()],
              ),
            ),
            if (unit.isNotEmpty)
              Padding(
                padding: const EdgeInsets.only(bottom: 3),
                child: Text(unit, style: const TextStyle(color: muted)),
              ),
          ],
        ),
        if (note != null)
          Padding(
            padding: const EdgeInsets.only(top: 4),
            child: Text(
              note!,
              style: const TextStyle(color: muted, fontSize: 11),
            ),
          ),
      ],
    ),
  );
}

class Status extends StatelessWidget {
  const Status(this.text, {super.key, this.color = muted});
  final String text;
  final Color color;
  @override
  Widget build(BuildContext context) => Container(
    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 5),
    decoration: BoxDecoration(
      border: Border.all(color: color.withValues(alpha: .35)),
      borderRadius: BorderRadius.circular(3),
    ),
    child: Text(
      text,
      style: TextStyle(color: color, fontSize: 11, fontWeight: FontWeight.w600),
    ),
  );
}

class EmptyState extends StatelessWidget {
  const EmptyState(
    this.title,
    this.detail, {
    super.key,
    this.action,
    this.icon = Icons.show_chart,
  });
  final String title, detail;
  final Widget? action;
  final IconData icon;
  @override
  Widget build(BuildContext context) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 26, horizontal: 24),
    child: Column(
      children: [
        Icon(icon, color: muted, size: 30),
        const SizedBox(height: 16),
        Text(title, style: Theme.of(context).textTheme.titleMedium),
        const SizedBox(height: 8),
        ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 430),
          child: Text(
            detail,
            textAlign: TextAlign.center,
            style: const TextStyle(color: muted),
          ),
        ),
        if (action != null)
          Padding(padding: const EdgeInsets.only(top: 20), child: action),
      ],
    ),
  );
}

class MetricGrid extends StatelessWidget {
  const MetricGrid({
    super.key,
    required this.children,
    this.minimumWidth = 155,
  });
  final List<Widget> children;
  final double minimumWidth;
  @override
  Widget build(BuildContext context) => LayoutBuilder(
    builder: (context, box) {
      final count = math.max(1, (box.maxWidth / minimumWidth).floor());
      return Wrap(
        spacing: 16,
        runSpacing: 12,
        children: children
            .map(
              (child) => SizedBox(
                width: (box.maxWidth - (count - 1) * 16) / count,
                child: child,
              ),
            )
            .toList(),
      );
    },
  );
}
