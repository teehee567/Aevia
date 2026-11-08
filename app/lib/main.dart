import 'package:flutter/material.dart';
import 'package:app/src/rust/frb_generated.dart';

Future<void> main() async {
  await RustLib.init();
  runApp(const MaterialApp(home: Scaffold()));
}
