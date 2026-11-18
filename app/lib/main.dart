import 'package:flutter/material.dart';
import 'src/rust/frb_generated.dart';
import 'src/rust/api/simple.dart' as native;
import 'controller.dart';
import 'ui/app.dart';
import 'ui/common.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  try {
    await RustLib.init();
    runApp(
      AeviaApp(
        controller: AeviaController(
          (json) => native.request(requestJson: json),
        ),
      ),
    );
  } catch (error) {
    runApp(
      MaterialApp(
        theme: aeviaTheme(),
        home: Scaffold(
          body: Center(
            child: ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 600),
              child: Padding(
                padding: const EdgeInsets.all(32),
                child: SelectableText(
                  'Aevia could not start its analysis engine.\n\n$error\n\nRebuild the application with its Rust library and try again.',
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
