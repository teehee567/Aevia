"""Exercise the actual native API and pinned external solver with upstream data.

Requires aevia_cli and the pinned RTKLIB source checkout under target/ppk-tools.
This is a processor integration fixture, not an Aevia receiver accuracy test.
"""
import json
import os
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[3]
data = root / 'target/ppk-tools/RTKLIB-v2.5.1/test/data/rinex'
output = root / 'target/ppk-smoke'
output.mkdir(exist_ok=True)
cli = root / 'target/release/examples/aevia_cli.exe'
query = {'op': 'run_ppk', 'rover_obs': str(data / '07590920.05o'),
         'base_obs': str(data / '30400920.05o'),
         'nav_paths': [str(data / '07590920.05n'), str(data / '30400920.05n')],
         'output_dir': str(output)}
env = dict(os.environ, AEVIA_DATA_DIR=str(output / 'sessions'))
result = subprocess.run([str(cli)], input=json.dumps(query) + '\n', text=True,
                        capture_output=True, env=env, timeout=120, check=True)
response = json.loads(result.stdout)
(output / 'processor-smoke.json').write_text(json.dumps(response, indent=2))
assert response['ok'], response
session = response['data']['session']
assert len(session['samples']) == 120
assert all(s['speed_mps'] is None and s['heading_deg'] is None for s in session['samples'])
assert session['provenance']['processing']['process']['exit_code'] == 0
assert session['provenance']['quality_counts'].get('fixed', 0) > 0
assert session['provenance']['quality_counts'].get('float', 0) > 0
assert not session['analysis']['drag_runs'] and not session['analysis']['laps']
print(json.dumps({'samples': len(session['samples']),
                  'quality_counts': session['provenance']['quality_counts'],
                  'report': response['data']['report_path']}))
