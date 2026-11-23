import 'package:flutter/material.dart';
import '../controller.dart';
import 'common.dart';

double? _scaled(dynamic value, double factor) {
  final n = numeric(value);
  return n == null ? null : n * factor;
}

String _reading(dynamic value, String unit, [int digits = 1]) {
  final n = numeric(value);
  return n == null ? '—' : '${number(n, digits)} $unit';
}

String _error(Map<String, dynamic> source, String key) {
  if (!source.containsKey(key)) return '—';
  final value = source[key];
  if (value is Map && value.length == 1) {
    return '${value.keys.single} · ${value.values.single}';
  }
  return value?.toString() ?? 'None';
}

String _uptime(dynamic seconds) {
  final value = numeric(seconds);
  if (value == null || value < 0) return '—';
  final duration = Duration(seconds: value.floor());
  return '${duration.inHours}h ${duration.inMinutes % 60}m ${duration.inSeconds % 60}s';
}

class _Readings extends StatelessWidget {
  const _Readings(this.rows);
  final List<(String, dynamic)> rows;

  @override
  Widget build(BuildContext context) => Table(
    columnWidths: const {0: FlexColumnWidth(1.15), 1: FlexColumnWidth()},
    children: [
      for (final (label, value) in rows)
        TableRow(
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(0, 4, 12, 4),
              child: Text(label, style: const TextStyle(color: muted)),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 4),
              child: Text(
                value?.toString() ?? '—',
                textAlign: TextAlign.end,
                style: const TextStyle(
                  fontFeatures: [FontFeature.tabularFigures()],
                ),
              ),
            ),
          ],
        ),
    ],
  );
}

class DiagnosticsView extends StatelessWidget {
  const DiagnosticsView({super.key, required this.controller});
  final AeviaController controller;

  @override
  Widget build(BuildContext context) {
    final c = controller;
    if (!c.connected) {
      return EmptyState(
        'No receiver connected',
        'Connect the V2 Mini over USB to inspect its sensors and power supply.',
        icon: Icons.usb_outlined,
        action: OutlinedButton(
          onPressed: () => c.navigate(5),
          child: const Text('Connect receiver'),
        ),
      );
    }

    final device = c.device;
    final recordingMax = c.recording
        ? object(c.live['analysis'])['max_speed_mps']
        : null;
    // Compact drains update sensor samples between full diagnostic reports.
    final status = c.diagnosticsFresh ? device : <String, dynamic>{};
    final battery = c.battery;
    final temperatures = battery['temperatures_c'];
    final imuHealth = object(status['imu']);
    final imuDiagnostics = object(status['imu_diagnostics']);
    final timing = object(status['capture_timing']);
    final ppk = object(status['ppk']);
    final uart = object(status['gnss_uart']);
    final sd = object(status['sd']);

    final lastImu = object(object(device['imu'])['latest']);
    final imuFresh = c.sensorFresh('imu') && lastImu.isNotEmpty;
    final imuValid = imuFresh && lastImu['valid'] == true;
    final imu = imuValid ? lastImu : <String, dynamic>{};

    final lastGnss = object(object(device['gnss'])['latest']);
    final gnssFresh = c.sensorFresh('gnss') && lastGnss.isNotEmpty;
    final epoch = gnssFresh ? lastGnss : <String, dynamic>{};
    final fixValid = epoch['valid'] == true && c.live['valid'] == true;
    final fix = fixValid ? epoch : <String, dynamic>{};

    final fault = battery['faults'];
    final power = Section(
      title: 'Battery & power',
      child: _Readings([
        (
          'Cell voltage',
          _reading(_scaled(battery['millivolts'], 0.001), 'V', 3),
        ),
        ('State of charge', _reading(c.batteryPercent, '%', 0)),
        (
          'Power state',
          switch (battery['status']) {
            'charging' => 'Charging',
            'bypass' => 'External power',
            'battery' => 'On battery',
            'low' => 'Low battery',
            final value => value,
          },
        ),
        (
          'Power source',
          switch (battery['power_source']) {
            'external' => 'External input',
            'battery' => 'Battery',
            _ => null,
          },
        ),
        ('Charger stage', c.chargeStageLabel),
        ('Charger temperature zone', c.temperatureZoneLabel),
        if (temperatures is List && temperatures.length == 3) ...[
          (
            'Charger chip temperature',
            temperatures[0] == null
                ? 'USB bench sample needed'
                : _reading(temperatures[0], '°C', 0),
          ),
          ('IMU temperature', _reading(temperatures[1], '°C', 0)),
          ('Processor temperature', _reading(temperatures[2], '°C', 0)),
        ],
        ('State-of-charge rate', signedBatteryRate(c.batteryRate)),
        (
          'Time to full at current rate',
          estimatedBatteryTime(c.batteryTimeToFull),
        ),
        ('Estimated runtime', estimatedBatteryTime(c.batteryTimeLeft)),
        if (!c.bluetooth) ...[
          ('Power read errors', battery['errors']),
          (
            'Charger fault register',
            fault is int
                ? '0x${fault.toRadixString(16).padLeft(2, '0').toUpperCase()}'
                : null,
          ),
          ('Charger error', _error(battery, 'charger_error')),
          ('Fuel gauge error', _error(battery, 'gauge_error')),
        ],
      ]),
    );

    if (c.bluetooth) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          power,
          const SizedBox(height: 16),
          EmptyState(
            'Full diagnostics use USB',
            'Open Live for driving telemetry.',
            icon: Icons.bluetooth_connected,
            action: OutlinedButton(
              onPressed: () => c.navigate(0),
              child: const Text('Open Live'),
            ),
          ),
        ],
      );
    }

    final logging = Section(
      title: 'Timing & recording',
      child: _Readings([
        (
          'PPS clock',
          switch (timing['pps_locked']) {
            true => 'Locked',
            false => 'Unlocked',
            _ => null,
          },
        ),
        ('Unsynchronized GNSS epochs', timing['gnss_unsynchronized']),
        ('Signals in last raw epoch', ppk['last_observation_count']),
        ('Last raw epoch age', _reading(ppk['age_ms'], 'ms', 0)),
        ('Ephemeris frames', ppk['ephemeris_frames']),
        ('Raw GNSS CRC failures', ppk['crc_failures']),
        ('SD card state', sd['state']),
        ('USB dropped samples', status['dropped']),
        ('SD dropped samples', sd['dropped']),
      ]),
    );

    final gnss = Section(
      title: 'GNSS receiver',
      trailing: Status(
        !gnssFresh
            ? (lastGnss.isEmpty ? 'WAITING' : 'STALE')
            : fixValid
            ? 'VALID FIX'
            : 'NO FIX',
        color: fixValid ? accent : warning,
      ),
      child: _Readings([
        ('Receiver status', object(status['gnss'])['status']),
        ('Sample age', _reading(c.sensorAgeMs('gnss'), 'ms', 0)),
        ('Satellites used', epoch['satellites']),
        ('Latitude', _reading(fix['lat'], '°', 7)),
        ('Longitude', _reading(fix['lon'], '°', 7)),
        ('Ellipsoid height', _reading(fix['alt_m'], 'm', 2)),
        ('Ground speed', _reading(_scaled(fix['speed_mps'], 3.6), 'km/h')),
        ('Course', _reading(fix['heading_deg'], '°')),
        ('Vertical speed', _reading(fix['vertical_mps'], 'm/s', 2)),
        ('Horizontal uncertainty (1σ)', _reading(fix['hacc_m'], 'm', 2)),
        ('Vertical uncertainty (1σ)', _reading(fix['vacc_m'], 'm', 2)),
        ('Speed uncertainty (1σ)', _reading(fix['sacc_mps'], 'm/s', 2)),
        ('Velocity latency', _reading(fix['latency_ms'], 'ms', 0)),
        ('GPS week', epoch['gps_week']),
        ('GPS time of week', _reading(epoch['gps_tow_ms'], 'ms', 0)),
        ('GNSS errors since boot', object(status['gnss'])['errors']),
        ('Last UART error', _error(uart, 'last_error')),
      ]),
    );

    final inertial = Section(
      title: 'IMU',
      trailing: Status(
        !imuFresh
            ? (lastImu.isEmpty ? 'WAITING' : 'STALE')
            : imuValid
            ? 'VALID SAMPLE'
            : 'INVALID',
        color: imuValid ? accent : warning,
      ),
      child: Column(
        children: [
          _ImuAxes(accel: imu['accel'], gyro: imu['gyro']),
          const SizedBox(height: 12),
          const Divider(),
          const SizedBox(height: 8),
          _Readings([
            ('Sensor status', imuHealth['status']),
            ('Sample age', _reading(c.sensorAgeMs('imu'), 'ms')),
            (
              'Sample interval',
              _reading(
                _scaled(imu['interval_ns'], 0.000001) ??
                    _scaled(imu['dt_us'], 0.001),
                'ms',
                3,
              ),
            ),
            ('Samples since boot', imuHealth['count']),
            ('Errors since boot', imuHealth['errors']),
            ('Timing rejections', timing['imu_timing_rejected']),
            ('Raw queue drops', imuDiagnostics['raw_queue_dropped']),
            ('Last IMU error', _error(imuDiagnostics, 'last_error')),
          ]),
        ],
      ),
    );

    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Section(
          title: 'This connection',
          trailing: c.diagnosticsFresh
              ? null
              : const Status('STATUS STALE', color: warning),
          child: MetricGrid(
            children: [
              Metric(
                'Recording maximum',
                number(_scaled(recordingMax, 3.6)),
                unit: 'km/h',
                note: c.recording
                    ? 'Traj maximum speed'
                    : 'Requires an active recording',
                color: accent,
              ),
              Metric(
                'Receiver uptime',
                _uptime(c.receiverFresh ? c.live['time_s'] : null),
              ),
              Metric('GNSS rate', number(status['gnss_hz']), unit: 'Hz'),
              Metric('IMU rate', number(status['imu_hz']), unit: 'Hz'),
            ],
          ),
        ),
        const SizedBox(height: 16),
        LayoutBuilder(
          builder: (context, box) {
            if (box.maxWidth < 750) {
              return Column(
                children: [
                  power,
                  const SizedBox(height: 16),
                  inertial,
                  const SizedBox(height: 16),
                  gnss,
                  const SizedBox(height: 16),
                  logging,
                ],
              );
            }
            return Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Expanded(
                  child: Column(
                    children: [power, const SizedBox(height: 16), gnss],
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: Column(
                    children: [inertial, const SizedBox(height: 16), logging],
                  ),
                ),
              ],
            );
          },
        ),
      ],
    );
  }
}

class _ImuAxes extends StatelessWidget {
  const _ImuAxes({required this.accel, required this.gyro});
  final dynamic accel, gyro;

  String component(dynamic vector, int axis) =>
      vector is List && vector.length == 3 ? number(vector[axis], 3) : '—';

  @override
  Widget build(BuildContext context) => Table(
    columnWidths: const {
      0: FlexColumnWidth(.8),
      1: FlexColumnWidth(1.4),
      2: FlexColumnWidth(1.4),
    },
    children: [
      const TableRow(
        children: [
          Text('Sensor axis', style: TextStyle(color: muted)),
          Text(
            'Accel (m/s²)',
            textAlign: TextAlign.end,
            style: TextStyle(color: muted),
          ),
          Text(
            'Gyro (rad/s)',
            textAlign: TextAlign.end,
            style: TextStyle(color: muted),
          ),
        ],
      ),
      for (int i = 0; i < 3; i++)
        TableRow(
          children: [
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 3),
              child: Text(['X', 'Y', 'Z'][i]),
            ),
            Text(component(accel, i), textAlign: TextAlign.end),
            Text(component(gyro, i), textAlign: TextAlign.end),
          ],
        ),
    ],
  );
}
