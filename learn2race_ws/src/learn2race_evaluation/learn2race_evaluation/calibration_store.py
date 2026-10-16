"""Persistence for calibration results.

Calibration values belong with the robot they describe, not in a notebook and
not baked into source. Each unit has a parameter file holding its identity and
its measured constants, and that file is under version control, so a robot is
reproducible from a revision plus its own file. This module is the single place
that reads and writes it, which keeps every calibration tool consistent about
where results go and what a result looks like.

Each entry records when it was produced and by which tool. A calibration with no
date is indistinguishable from a guess, and wheel radii change when tyres wear.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: per-robot calibration records
# More details: README.md of this package

import datetime
import os
import shutil

try:
    import yaml
except ImportError:                                   # pragma: no cover
    yaml = None

DEFAULT_PATH = os.path.expanduser('~/learn2race/config/robot.yaml')


class CalibrationStore:

    def __init__(self, path: str = DEFAULT_PATH):
        if yaml is None:
            raise RuntimeError('PyYAML is required to store calibration results')
        self.path = os.path.expanduser(path)
        self.data = self._load()

    def _load(self) -> dict:
        if not os.path.exists(self.path):
            return {'robot': {}, 'calibration': {}}
        with open(self.path) as handle:
            data = yaml.safe_load(handle) or {}
        data.setdefault('robot', {})
        data.setdefault('calibration', {})
        return data

    def get(self, section: str, default=None):
        return self.data['calibration'].get(section, default)

    def update(self, section: str, values: dict, tool: str = ''):
        """Record a calibration section, stamped with its provenance."""
        entry = dict(values)
        entry['_calibrated'] = datetime.datetime.now().isoformat(timespec='seconds')
        if tool:
            entry['_tool'] = tool
        self.data['calibration'][section] = entry

    def save(self):
        """Write the file, keeping one backup.

        A backup matters because a calibration run that goes wrong overwrites a
        good set of values, and the previous set is usually the fastest way back
        to a working robot.
        """
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        if os.path.exists(self.path):
            shutil.copy2(self.path, self.path + '.bak')
        with open(self.path, 'w') as handle:
            yaml.safe_dump(self.data, handle, default_flow_style=False,
                           sort_keys=False)
        return self.path

    def summary(self) -> str:
        lines = [f'{self.path}:']
        for section, values in sorted(self.data['calibration'].items()):
            when = values.get('_calibrated', 'undated')
            fields = ', '.join(f'{k}={v}' for k, v in values.items()
                               if not k.startswith('_'))
            lines.append(f'  {section:24s} {when}  {fields}')
        return '\n'.join(lines)
