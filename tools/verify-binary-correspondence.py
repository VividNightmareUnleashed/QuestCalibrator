#!/usr/bin/env python3
"""Run V09 instruction comparisons and bind their intended failure controls."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    root, out = args.source_root.resolve(), args.output_dir.resolve()
    virtual = root / 'VirtualQuest'
    spec = importlib.util.spec_from_file_location('assurance', root / 'tools/verify-assurance.py')
    assurance = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assurance)
    identity = assurance.source_identity(root, virtual)
    if out.exists() and any(out.iterdir()):
        raise ValueError('Use a new or empty output directory')
    out.mkdir(parents=True, exist_ok=True)
    record = {'schemaVersion': 2, 'sources': identity, 'success': False, 'suites': {}}
    try:
        with (out / 'execution.log').open('w') as log:
            subprocess.run([sys.executable, str(virtual / 'formal/verify-binary-correspondence.py'),
                            '--source-root', str(root), '--output-dir', str(out / 'execution')],
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
        record = json.loads((out / 'execution/result.json').read_text())
        record['success'] = False
        suite = record['suites']['binary-correspondence']
        assurance.validate_binary_controls(suite['negativeControls'])
        if suite['negativeControls'] != record['negativeControls']:
            raise ValueError('Binary execution and accepted controls disagree')
        suite['acceptanceFixtures'] = assurance.binary_negative_control_fixtures()
        record['sourcesUnchanged'] = identity == assurance.source_identity(root, virtual)
        record['success'] = record['sourcesUnchanged']
        assurance.validate(record, identity, {'binary-correspondence': ['V09']},
                           required={'binary-correspondence'}, release=False, require_negative_controls=True)
        print(f"V09 passed for all three supplied pins; {len(suite['negativeControls'])} intended failure controls "
              f"and {suite['acceptanceFixtures']} acceptance fixtures passed.")
    except BaseException:
        record['success'] = False
        raise
    finally:
        (out / 'result.json').write_text(json.dumps(record, indent=2) + '\n')


if __name__ == '__main__':
    main()
