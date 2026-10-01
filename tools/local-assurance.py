#!/usr/bin/env python3
"""Validate local Linux assurance and optionally publish metadata in private Git notes."""
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


def validate(record, root, assurance, virtual=None):
    virtual = virtual or root / 'VirtualQuest'
    if record.get('dirty') is not False:
        raise ValueError('Local assurance requires committed sources')
    if assurance.git(root, 'rev-parse', 'HEAD:VirtualQuest') != assurance.git(virtual, 'rev-parse', 'HEAD'):
        raise ValueError('Local assurance does not match the submodule pin')
    if any(assurance.git(repo, 'status', '--porcelain') for repo in (root, virtual)):
        raise ValueError('Local assurance requires clean checkouts')
    expected = assurance.expected_checks(virtual, root)
    assurance.validate(record, assurance.source_identity(root, virtual), expected,
                       required=assurance.REQUIRED_SUITES - {'hub-traces'}, release=False,
                       require_negative_controls=True)


def metadata_only(record, assurance):
    """Keep verified outcomes and hashes, excluding source snippets and execution logs."""
    result = {key: copy.deepcopy(record[key]) for key in
              ('schemaVersion', 'sources', 'sourcesUnchanged', 'success', 'dirty')}
    result['fullSuite'] = False
    result['suites'] = {}
    for name in sorted(assurance.REQUIRED_SUITES - {'hub-traces'}):
        original = record['suites'][name]
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
                    'mutants': [{key: row[key] for key in ('name', 'status', 'method', 'mutation', 'failure')}
                                for row in controls['mutants']],
                }
        if name == 'inventory-extension':
            suite['negativeControls']['captureAcceptanceFixtures'] = original['negativeControls']['captureAcceptanceFixtures']
            suite['negativeControls']['captureMutants'] = [
                {key: row[key] for key in ('name', 'status', 'method', 'log')}
                for row in original['negativeControls']['captureMutants']]
            for control in suite['negativeControls']['captureMutants']:
                # The acceptance protocol needs assertion identities/results,
                # not Python traceback source lines or absolute local paths.
                control['log'] = '\n'.join(line for line in control['log'].splitlines()
                                          if re.match(r'^(FAIL: test_|AssertionError: |Ran \d+ tests in |FAILED \(failures=)', line)) + '\n'
        result['suites'][name] = suite
    return result


def publish(record, root, assurance):
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
        validate(existing, root, assurance)
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
    parser.add_argument('--publish', action='store_true', help='Publish metadata to private VirtualQuest Git notes')
    args = parser.parse_args()
    root = args.source_root.resolve()
    assurance = policy(root)
    record = json.loads(args.evidence.read_text(encoding='utf-8-sig'))
    validate(record, root, assurance)
    record = metadata_only(record, assurance)
    validate(record, root, assurance)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(record, indent=2) + '\n')
    if args.publish:
        publish(record, root, assurance)
    print('Complete local Linux assurance matches both exact commits; Windows traces remain separate.')


if __name__ == '__main__':
    main()
