import 'dart:math' as math;

import 'package:latlong2/latlong.dart';

const maxMapLatitude = 85.05112878;
const minGateWidth = 1.0;
const maxGateWidth = 500.0;

LatLng? trackPosition(double? lat, double? lon) {
  if (lat == null || lon == null || !lat.isFinite || !lon.isFinite) return null;
  if (lat.abs() > 90 || lon.abs() > 180) return null;
  return LatLng(lat, lon);
}

LatLng mapPosition(LatLng point) => LatLng(
  point.latitude.clamp(-maxMapLatitude, maxMapLatitude),
  point.longitude,
);

double normalizeHeading(double degrees) => degrees % 360;

class TrackGate {
  const TrackGate(this.center, this.heading, this.width);

  final LatLng center;
  final double heading, width;

  static const _distance = Distance(roundResult: false);

  static TrackGate? across(LatLng first, LatLng second) {
    // Reject distant taps before calculating the bearing of a short gate.
    const spherical = DistanceHaversine(roundResult: false);
    if (spherical(first, second) > maxGateWidth * 1.01) return null;
    final width = _distance(first, second);
    if (!width.isFinite || width < minGateWidth || width > maxGateWidth) {
      return null;
    }
    final midpoint = _offset(
      first,
      width / 2,
      _distance.bearing(first, second),
    );
    final heading = normalizeHeading(_distance.bearing(midpoint, second) - 90);
    return TrackGate(midpoint, heading, width);
  }

  static LatLng _offset(LatLng from, double metres, double bearing) {
    final point = _distance.offset(from, metres, bearing);
    return LatLng(point.latitude, (point.longitude + 180) % 360 - 180);
  }

  List<LatLng> get endpoints {
    final direction = normalizeHeading(heading);
    return [
      _offset(center, width / 2, direction - 90),
      _offset(center, width / 2, direction + 90),
    ];
  }

  double get headingRadians => normalizeHeading(heading) * math.pi / 180;

  Map<String, dynamic> toJson() => {
    'lat': center.latitude,
    'lon': center.longitude,
    'heading_deg': heading,
    'width_m': width,
  };
}
