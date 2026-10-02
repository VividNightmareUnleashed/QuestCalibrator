#!/usr/bin/env python3
"""Acceptance fixtures for local evidence, metadata redaction and public scope.

Also the records private checks pass between public CI jobs: the V09 note, the
reduced inventory extension and tools/ci/run-private-check.py.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


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
    # V09 alone, as published in the private notes and read by the release run.
    binary = {'schemaVersion': 2, 'sources': identity, 'sourcesUnchanged': True, 'success': True,
              'suites': {'binary-correspondence': copy.deepcopy(fixture['suites']['binary-correspondence'])},
              'executionLog': 'PRIVATE_SOURCE_SENTINEL'}
    binary['negativeControls'] = copy.deepcopy(binary['suites']['binary-correspondence']['negativeControls'])
    local.check_binary(binary, identity, assurance)
    safe_binary = local.binary_metadata(binary, assurance)
    local.check_binary(safe_binary, identity, assurance)
    assert 'PRIVATE_SOURCE_SENTINEL' not in json.dumps(safe_binary) and safe_binary['dirty'] is False
    assert local.binary_metadata(safe_binary, assurance) == safe_binary
    passed += 1
    for change in [
        lambda r: r.pop('negativeControls'),
        lambda r: r['negativeControls'].pop(),
        lambda r: r['suites']['binary-correspondence']['checks'][0].update(cases=1),
        lambda r: r['suites']['binary-correspondence'].update(acceptanceFixtures=44),
        lambda r: r['sources']['QuestCalibrator'].update(commit='stale'),
    ]:
        invalid = copy.deepcopy(safe_binary); change(invalid)
        try:
            local.check_binary(invalid, identity, assurance)
        except ValueError:
            passed += 1
        else:
            raise AssertionError('Incomplete or stale V09 evidence was accepted')
    refused = subprocess.run([sys.executable, str(root / 'tools/local-assurance.py'), '--evidence', 'unused.json',
                              '--publish'], capture_output=True)
    assert refused.returncode == 2, 'Only the V09 record may be published'
    passed += 1

    # The complete inventory extension runs on its own runner; what passes to
    # the assembling job must still satisfy every assembly check.
    assemble = module('assemble_assurance', root / 'tools/assemble-assurance.py')
    extension = {key: copy.deepcopy(fixture[key]) for key in
                 ('schemaVersion', 'sources', 'sourcesUnchanged', 'success', 'fullSuite', 'dirty')}
    extension['suites'] = {'inventory-extension': copy.deepcopy(fixture['suites']['inventory-extension'])}
    controls = extension['suites']['inventory-extension']['negativeControls']
    extension['executions'] = {kind: {'success': True, 'sourcesUnchanged': True, 'log': 'PRIVATE_SOURCE_SENTINEL'}
                               for kind in ('inventory', 'portable', 'smoother', 'capture', 'installer')}
    extension['executions']['inventory']['mutants'] = copy.deepcopy(controls['mutants'])
    extension['executions']['capture']['assertionControls'] = copy.deepcopy(controls['captureMutants'])
    extension['executions']['capture']['mutants'] = [{key: row[key] for key in ('name', 'status', 'method')}
                                                     for row in controls['captureMutants']]
    assemble.validate_extension(extension, assurance, identity, expected)
    reduced = local.extension_metadata(extension)
    assemble.validate_extension(reduced, assurance, identity, expected)
    assert 'PRIVATE_SOURCE_SENTINEL' not in json.dumps(reduced) and '/private_fixture/' not in json.dumps(reduced)
    passed += 1
    for change in [
        lambda r: r['executions'].pop('smoother'),
        lambda r: r['executions']['portable'].update(success=False),
        lambda r: r['executions']['inventory']['mutants'].pop(),
        lambda r: r['executions']['capture']['assertionControls'][0].update(log='RuntimeError: unrelated'),
        lambda r: r['executions']['capture']['mutants'][0].update(status='skipped'),
    ]:
        invalid = copy.deepcopy(reduced); change(invalid)
        try:
            assemble.validate_extension(invalid, assurance, identity, expected)
        except ValueError:
            passed += 1
        else:
            raise AssertionError('Incomplete extension evidence passed between jobs')

    # A private check on a public runner: its output and the private parts of
    # its record stay on the runner, and anything short of complete fails.
    runner = root / 'tools/ci/run-private-check.py'
    with tempfile.TemporaryDirectory(prefix='private-check-') as temporary:
        raw, kept = Path(temporary) / 'raw.json', Path(temporary) / 'kept.json'

        def run(record):
            script = (f'import json; print("PRIVATE_SOURCE_SENTINEL"); '
                      f'open({str(raw)!r}, "w").write({json.dumps(json.dumps(record))})')
            return subprocess.run([sys.executable, str(runner), '--label', 'fixture', '--raw', str(raw),
                                   '--result', str(kept), '--', sys.executable, '-c', script],
                                  capture_output=True, text=True, env=dict(os.environ, RUNNER_TEMP=temporary))
        proof = {'schemaVersion': 2, 'complete': True, 'success': True, 'sourcesUnchanged': True, 'expected': ['A'],
                 'sourceHashes': {'VirtualQuest:formal/A.tla': '0' * 64}, 'tools': {'tlc': 'fixture'},
                 'checks': [{'name': 'A', 'tool': 'tlc', 'status': 'passed', 'output': 'PRIVATE_SOURCE_SENTINEL'}]}
        done = run(proof)
        assert done.returncode == 0 and 'PRIVATE_SOURCE_SENTINEL' not in done.stdout + done.stderr
        assert 'PRIVATE_SOURCE_SENTINEL' not in kept.read_text()
        passed += 1
        for change in [
            lambda r: r['checks'][0].update(status='skipped'),
            lambda r: r.update(expected=['A', 'B']),
            lambda r: r.update(complete=False),
            lambda r: r.update(sourcesUnchanged=False),
            lambda r: r.update(checks=[]),
        ]:
            invalid = copy.deepcopy(proof); change(invalid)
            if run(invalid).returncode == 0:
                raise AssertionError('An incomplete private check passed')
            passed += 1

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
