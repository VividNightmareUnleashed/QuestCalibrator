#!/usr/bin/env python3
"""Validate local Linux assurance; publish the V09 metadata in private Git notes."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import tempfile


def policy(root):
    spec = importlib.util.spec_from_file_location('assurance', root / 'tools/verify-assurance.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require_clean_pair(root, virtual, assurance):
    if assurance.git(root, 'rev-parse', 'HEAD:VirtualQuest') != assurance.git(virtual, 'rev-parse', 'HEAD'):
        raise ValueError('Local assurance does not match the submodule pin')
    if any(assurance.git(repo, 'status', '--porcelain') for repo in (root, virtual)):
        raise ValueError('Local assurance requires clean checkouts')


def validate(record, root, assurance, virtual=None):
    virtual = virtual or root / 'VirtualQuest'
    if record.get('dirty') is not False:
        raise ValueError('Local assurance requires committed sources')
    require_clean_pair(root, virtual, assurance)
    expected = assurance.expected_checks(virtual, root)
    assurance.validate(record, assurance.source_identity(root, virtual), expected,
                       required=assurance.REQUIRED_SUITES - {'hub-traces'}, release=False,
                       require_negative_controls=True)


def validate_binary(record, root, assurance, virtual=None):
    """A V09 record for the exact pair, as run (verify-binary-correspondence.py) or as published."""
    virtual = virtual or root / 'VirtualQuest'
    if record.get('dirty', False) is not False:
        raise ValueError('Local assurance requires committed sources')
    require_clean_pair(root, virtual, assurance)
    check_binary(record, assurance.source_identity(root, virtual), assurance)


def check_binary(record, identity, assurance):
    assurance.validate(record, identity, {'binary-correspondence': ['V09']},
                       required={'binary-correspondence'}, release=False, require_negative_controls=True)
    if record.get('negativeControls') != record['suites']['binary-correspondence']['negativeControls']:
        raise ValueError('Binary execution and accepted negative controls disagree')


CAPTURE_LOG_LINE = re.compile(r'^(FAIL: test_|AssertionError: |Ran \d+ tests in |FAILED \(failures=)')


def capture_metadata(control):
    # The acceptance protocol needs assertion identities/results,
    # not Python traceback source lines or absolute local paths.
    result = {key: control[key] for key in ('name', 'status', 'method')}
    result['log'] = '\n'.join(line for line in control['log'].splitlines() if CAPTURE_LOG_LINE.match(line)) + '\n'
    return result


def mutant_metadata(row):
    return {key: row[key] for key in ('name', 'status', 'method', 'mutation', 'failure')}


def suite_metadata(name, original):
    suite = {key: copy.deepcopy(original[key]) for key in ('complete', 'tools')}
    suite['checks'] = [{key: check[key] for key in ('name', 'status', 'cases', 'negative')
                        if key in check} for check in original['checks']]
    if 'acceptanceFixtures' in original:
        suite['acceptanceFixtures'] = original['acceptanceFixtures']
    if 'negativeControls' in original:
        controls = original['negativeControls']
        if name == 'binary-correspondence':
            suite['negativeControls'] = [{key: row[key] for key in ('name', 'status', 'intendedFailure')}
                                         for row in controls]
        else:
            suite['negativeControls'] = {
                'acceptanceFixtures': controls['acceptanceFixtures'],
                'mutants': [mutant_metadata(row) for row in controls['mutants']],
            }
    if name == 'inventory-extension':
        suite['negativeControls']['captureAcceptanceFixtures'] = original['negativeControls']['captureAcceptanceFixtures']
        suite['negativeControls']['captureMutants'] = [
            capture_metadata(row) for row in original['negativeControls']['captureMutants']]
    return suite


def metadata_only(record, assurance, suites=None):
    """Keep verified outcomes and hashes, excluding source snippets and execution logs."""
    result = {key: copy.deepcopy(record[key]) for key in
              ('schemaVersion', 'sources', 'sourcesUnchanged', 'success', 'dirty')}
    result['fullSuite'] = False
    result['suites'] = {name: suite_metadata(name, record['suites'][name])
                        for name in sorted(suites or assurance.REQUIRED_SUITES - {'hub-traces'})}
    return result


def extension_metadata(record):
    """What assembly checks of a complete inventory-extension record, without its logs.

    The extension runs on its own runner; this is the copy that may pass to the
    job assembling the evidence, which checks it as it would the whole record.
    """
    result = {key: copy.deepcopy(record[key]) for key in
              ('schemaVersion', 'sources', 'sourcesUnchanged', 'success', 'fullSuite', 'dirty')}
    result['suites'] = {'inventory-extension': suite_metadata('inventory-extension',
                                                              record['suites']['inventory-extension'])}
    executions = {kind: {key: execution.get(key) for key in ('success', 'sourcesUnchanged')}
                  for kind, execution in record['executions'].items()}
    executions['inventory']['mutants'] = [mutant_metadata(row) for row in record['executions']['inventory']['mutants']]
    capture = record['executions']['capture']
    executions['capture']['mutants'] = [{key: row[key] for key in ('name', 'status', 'method')}
                                        for row in capture['mutants']]
    executions['capture']['assertionControls'] = [capture_metadata(row) for row in capture['assertionControls']]
    result['executions'] = executions
    return result


def binary_metadata(record, assurance):
    """A V09 record without its execution details, for the private notes.

    Only called on a record validate_binary accepted: its sources are the
    committed ones even where the run did not record that itself.
    """
    result = metadata_only(dict(record, dirty=False), assurance, {'binary-correspondence'})
    result['negativeControls'] = copy.deepcopy(result['suites']['binary-correspondence']['negativeControls'])
    return result


def publish(record, root, check):
    virtual = root / 'VirtualQuest'
    quest_commit = record['sources']['QuestCalibrator']['commit']
    virtual_commit = record['sources']['VirtualQuest']['commit']
    ref = 'refs/notes/formal-assurance/' + quest_commit

    def git(*args, data=None):
        return subprocess.check_output(['git', '-C', str(virtual), *args], input=data).decode().strip()

    remote = git('ls-remote', 'origin', ref)
    if remote:
        git('fetch', '--quiet', 'origin', ref)
        existing = json.loads(git('show', 'FETCH_HEAD:' + virtual_commit))
        check(existing)
        print('Valid local assurance is already published for this exact pair.')
        return
    local = subprocess.run(['git', '-C', str(virtual), 'show-ref', '--verify', '--quiet', ref]).returncode
    if local == 0:
        if json.loads(git('show', ref + ':' + virtual_commit)) != record:
            raise ValueError('A different local assurance note already exists; inspect it before publishing')
        git('push', 'origin', ref + ':' + ref)
        print('Published the existing local assurance note.')
        return
    if local != 1:
        raise ValueError('Cannot inspect the local assurance note ref')
    payload = (json.dumps(record, indent=2) + '\n').encode()
    blob = git('hash-object', '-w', '--stdin', data=payload)
    tree = git('mktree', data=f'100644 blob {blob}\t{virtual_commit}\n'.encode())
    with tempfile.TemporaryDirectory(prefix='local-assurance-note-') as temporary:
        message = Path(temporary) / 'message.txt'
        message.write_text('docs(formal): record local assurance for the exact source pair\n\n'
                           'Keep successful local proof outcomes and source hashes available\n'
                           'to the release gate without publishing private source or binaries.\n')
        commit = git('-c', 'user.email=178581564+VividNightmareUnleashed@users.noreply.github.com',
                     'commit-tree', tree, '-F', str(message))
    git('update-ref', ref, commit, '0' * 40)
    git('push', 'origin', ref + ':' + ref)
    print('Published local assurance metadata in private Git notes for ' + quest_commit)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--binary-only', action='store_true',
                        help='The evidence is the V09 record of tools/verify-binary-correspondence.py')
    parser.add_argument('--publish', action='store_true',
                        help='Publish the V09 metadata to private VirtualQuest Git notes (with --binary-only)')
    args = parser.parse_args()
    if args.publish and not args.binary_only:
        # The release run proves every other Linux suite itself and reads only
        # V09, which needs the supplied binaries, from the notes.
        parser.error('only the V09 record is published; pass --binary-only')
    root = args.source_root.resolve()
    assurance = policy(root)
    record = json.loads(args.evidence.read_text(encoding='utf-8-sig'))
    if args.binary_only:
        def check(candidate):
            validate_binary(candidate, root, assurance)
        check(record)
        record = binary_metadata(record, assurance)
    else:
        def check(candidate):
            validate(candidate, root, assurance)
        check(record)
        record = metadata_only(record, assurance)
    check(record)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(record, indent=2) + '\n')
    if args.publish:
        publish(record, root, check)
    print('V09 matches both exact commits; the release run proves the other Linux suites itself.'
          if args.binary_only else
          'Complete local Linux assurance matches both exact commits; Windows traces remain separate.')


if __name__ == '__main__':
    main()
