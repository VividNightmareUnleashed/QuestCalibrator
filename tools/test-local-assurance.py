#!/usr/bin/env python3
"""Acceptance fixtures for local evidence, metadata redaction and public scope."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    assurance = module('assurance', root / 'tools/verify-assurance.py')
    local = module('local_assurance', root / 'tools/local-assurance.py')
    expected = assurance.expected_checks(root / 'VirtualQuest', root)
    identity = assurance.source_identity(root, root / 'VirtualQuest')
    required = assurance.REQUIRED_SUITES - {'hub-traces'}
    fixture = {'schemaVersion': 2, 'sources': identity, 'sourcesUnchanged': True,
               'success': True, 'dirty': False, 'fullSuite': False, 'suites': {}}
    for name in required:
        checks = [{'name': check, 'status': 'passed', 'cases': 4577 if name == 'binary-correspondence' else 1,
                   'negative': True} for check in expected[name]]
        fixture['suites'][name] = {'complete': True, 'tools': {'fixture': 'acceptance data, not executed proofs'}, 'checks': checks}
    def cpp(registry):
        return [{'name': name, 'status': 'passed', 'method': 'production mutant',
                 'mutation': path, 'failure': message} for name, (path, message) in registry.items()]
    fixture['suites']['contracts']['negativeControls'] = {'mutants': cpp(assurance.NOW_MUTANTS), 'acceptanceFixtures': 67}
    captures = []
    for name, tests in assurance.CAPTURE_MUTANTS.items():
        log = ''
        for test, message in tests.items():
            log += f'FAIL: {test} (test_quest_pose_debug.PoseAnalysisTests.{test})\n'
            log += 'File "/private_fixture/capture.py", line 1\nPRIVATE_SOURCE_SENTINEL\n'
            log += 'AssertionError: ' + (message or '0.001 not less than 1e-06') + '\n'
        log += f'Ran 19 tests in 0.001s\nFAILED (failures={len(tests)})\n'
        captures.append({'name': name, 'status': 'passed', 'method': 'compiling Python mutant', 'log': log})
    fixture['suites']['inventory-extension']['negativeControls'] = {
        'mutants': cpp(assurance.INVENTORY_MUTANTS), 'acceptanceFixtures': 186,
        'captureMutants': captures, 'captureAcceptanceFixtures': 36}
    fixture['suites']['binary-correspondence'].update(
        negativeControls=[{'name': name, 'status': 'passed', 'intendedFailure': message}
                          for name, message in assurance.BINARY_CONTROLS.items()], acceptanceFixtures=45)
    fixture['privateLog'] = 'PRIVATE_SOURCE_SENTINEL'
    for suite in fixture['suites'].values():
        suite['checks'][0]['rawSource'] = 'PRIVATE_SOURCE_SENTINEL'
    fixture['suites']['contracts']['negativeControls']['mutants'][0]['rawSource'] = 'PRIVATE_SOURCE_SENTINEL'
    fixture['suites']['binary-correspondence']['negativeControls'][0]['rawSource'] = 'PRIVATE_SOURCE_SENTINEL'

    def validate(record):
        assurance.validate(record, identity, expected, required=required, release=False,
                           require_negative_controls=True)
    passed = 0
    validate(fixture)
    safe = local.metadata_only(fixture, assurance)
    validate(safe)
    assert 'PRIVATE_SOURCE_SENTINEL' not in json.dumps(safe)
    assert '/private_fixture/' not in json.dumps(safe)
    assert safe['sources'] == fixture['sources'] and fixture['privateLog'] == 'PRIVATE_SOURCE_SENTINEL'
    passed += 1
    changes = []
    for name in required:
        changes.append(lambda r, name=name: r['suites'].pop(name))
    changes += [
        lambda r: r['sources']['VirtualQuest'].update(commit='stale'),
        lambda r: r['suites']['private-core']['checks'].pop(),
        lambda r: r['suites']['input-validation'].update(tools={}),
        lambda r: r['suites']['inventory-extension']['negativeControls']['mutants'][0].update(failure='unrelated error'),
        lambda r: r['suites']['inventory-extension']['negativeControls']['captureMutants'][0].update(log='RuntimeError: unrelated'),
        lambda r: r['suites']['binary-correspondence']['negativeControls'][0].update(intendedFailure='unrelated'),
        lambda r: r['suites']['inventory-extension'].update(checks=[c for c in r['suites']['inventory-extension']['checks'] if not c['name'].startswith('V')]),
    ]
    for change in changes:
        invalid = copy.deepcopy(safe); change(invalid)
        try:
            validate(invalid)
        except ValueError:
            passed += 1
        else:
            raise AssertionError('Incomplete or unrelated local evidence was accepted')
    try:
        assurance.validate(safe, identity, expected)
    except ValueError:
        passed += 1
    else:
        raise AssertionError('Local Linux evidence satisfied a Windows release')
    public_names = [n for n in expected['inventory-extension'] if not n.startswith('V')]
    public = copy.deepcopy(safe)
    public['suites'] = {'public-inventory-extension': {
        'complete': True, 'tools': {'fixture': 'acceptance data'},
        'checks': [{'name': n, 'status': 'passed', 'cases': 1} for n in public_names],
        'negativeControls': {'mutants': cpp(assurance.PUBLIC_INVENTORY_MUTANTS),
                             'acceptanceFixtures': 144, 'privateSelectionsRefused': 6}}}
    def validate_public(record):
        assurance.validate(record, identity, {'public-inventory-extension': public_names},
                           required={'public-inventory-extension'}, release=False, require_negative_controls=True)
    validate_public(public); passed += 1
    for change in [
        lambda r: r['suites']['public-inventory-extension']['checks'].append({'name': 'V01', 'status': 'passed', 'cases': 1}),
        lambda r: r['suites']['public-inventory-extension']['checks'][0].update(cases=0),
        lambda r: r['suites']['public-inventory-extension']['negativeControls'].update(privateSelectionsRefused=0),
        lambda r: r['suites']['public-inventory-extension']['negativeControls']['mutants'].append(
            next(row for row in cpp(assurance.INVENTORY_MUTANTS) if row['name'] == 'V01')),
    ]:
        invalid = copy.deepcopy(public); change(invalid)
        try:
            validate_public(invalid)
        except ValueError:
            passed += 1
        else:
            raise AssertionError('Private or incomplete public scope was accepted')
    def numeric(*options):
        command = ['pwsh', '-NoProfile', '-File', str(root / 'VirtualQuest/formal/check.ps1'),
                   '-Only', 'input-validation', '-ListChecks', '-SourceRoot', str(root), *options]
        rows = json.loads(subprocess.check_output(command, text=True))
        if isinstance(rows, dict):
            rows = [rows]
        return [row['Name'] for row in rows]
    complete_numeric = numeric()
    public_numeric = numeric('-PublicInputValidation')
    assert len(complete_numeric) == 106 and len(public_numeric) == 102
    assert set(complete_numeric) - set(public_numeric) == {
        'SimulatorController', 'SimulatorCache', 'SimulatorHealth', 'SimulatorScheduler'}
    shards = [numeric('-PublicInputValidation', '-Shard', f'{i}/4') for i in range(1, 5)]
    assert all(shards) and sorted(sum(shards, [])) == sorted(public_numeric)
    assert len(sum(shards, [])) == len(set(sum(shards, [])))
    passed += 1
    print(f'{passed} local-evidence acceptance fixtures passed; fixtures do not certify proof execution.')


if __name__ == '__main__':
    main()
