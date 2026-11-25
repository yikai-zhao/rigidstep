"""Adversarial JSONL trace-validator tests: never trust recorder output blindly."""
import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/verify_trace.py'
BASE = {'tick': 0, 'dt': 1/120, 'hash': 'abc', 'contacts': 1,
        'warm_matched': 0, 'warm_applied': 0, 'cached_contacts': 0,
        'edge_contacts': 0, 'face_contacts': 0,
        'bodies': [{'id': 1, 'type': 'sphere', 'p': [0, .6, 0],
                    'v': [0, 0, 0], 'q': [1, 0, 0, 0], 'radius': .5}],
        'contact_snapshot': [{'a': 0, 'b': 1, 'p': [0, 0, 0],
                              'n': [0, 1, 0], 'depth': .1,
                              'feature': 'ground'}]}


class TraceValidationTests(unittest.TestCase):
    def check_trace(self, records, valid):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'test.jsonl'
            path.write_text(''.join(json.dumps(r) + '\n' for r in records))
            proc = subprocess.run([sys.executable, str(SCRIPT), str(path)],
                                  capture_output=True, text=True)
            self.assertEqual(proc.returncode == 0, valid, proc.stdout + proc.stderr)

    def test_valid_snapshot(self):
        self.check_trace([copy.deepcopy(BASE)], True)

    def test_impossible_feature_total(self):
        f = copy.deepcopy(BASE)
        f['edge_contacts'] = 2
        self.check_trace([f], False)

    def test_zero_normal(self):
        f = copy.deepcopy(BASE)
        f['contact_snapshot'][0]['n'] = [0, 0, 0]
        self.check_trace([f], False)

    def test_nan_contact_point(self):
        f = copy.deepcopy(BASE)
        f['contact_snapshot'][0]['p'][0] = float('nan')
        self.check_trace([f], False)

    def test_noncontiguous_frame(self):
        f = copy.deepcopy(BASE)
        second = copy.deepcopy(BASE)
        second['tick'] = 2
        self.check_trace([f, second], False)

    def test_bad_cache_accounting(self):
        f = copy.deepcopy(BASE)
        f['cached_contacts'] = 10
        self.check_trace([f], False)

    def test_unrecognized_feature(self):
        f = copy.deepcopy(BASE)
        f['contact_snapshot'][0]['feature'] = 'imaginary'
        self.check_trace([f], False)


if __name__ == '__main__':
    unittest.main()
