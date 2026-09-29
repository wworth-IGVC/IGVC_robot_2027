#!/usr/bin/env python3
"""Check that docs/JAZZY_MIGRATION.md accounts for every ROS package in src/.

Every package.xml under src/ (submodules included) must appear EXACTLY ONCE
as the first cell of a row in the document's package table, the table between
the two marker lines

    <!-- package-table:start -->
    <!-- package-table:end -->

and every row in that table must name a package that exists. A package added
to src/ without a row, a row left behind after a package is removed, or the
same package listed twice all fail this check.

    python3 scripts/check_jazzy_inventory.py      # from the repo root
    python3 scripts/check_jazzy_inventory.py OTHER.md   # check another copy
    exit 0 complete, 1 incomplete, 2 cannot read the inputs

Standard library only, so it runs anywhere Python 3 does, with or without ROS.
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC = os.path.join(REPO, 'docs', 'JAZZY_MIGRATION.md')
START = '<!-- package-table:start -->'
END = '<!-- package-table:end -->'
SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', '.pixi', '.ws'}


def packages_in_src():
    found = {}
    for root, dirs, files in os.walk(os.path.join(REPO, 'src')):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        if 'package.xml' in files:
            path = os.path.join(root, 'package.xml')
            name = ET.parse(path).getroot().findtext('name')
            if not name:
                raise ValueError('no <name> in ' + path)
            found.setdefault(name.strip(), []).append(os.path.relpath(path, REPO))
    return found


def rows_in_doc(doc):
    with open(doc, encoding='utf-8') as f:
        text = f.read()
    if START not in text or END not in text:
        raise ValueError('markers %s / %s not found in %s' % (START, END, doc))
    table = text.split(START, 1)[1].split(END, 1)[0]
    names = []
    for line in table.splitlines():
        line = line.strip()
        if not line.startswith('|') or re.match(r'^\|\s*-{3,}', line):
            continue
        first = line.strip('|').split('|')[0].strip()
        first = first.strip('`* ')
        if not first or first.lower() == 'package':
            continue
        names.append(first)
    return names


def main():
    doc = sys.argv[1] if len(sys.argv) > 1 else DOC
    try:
        src = packages_in_src()
        rows = rows_in_doc(doc)
    except (OSError, ValueError, ET.ParseError) as e:
        print('CANNOT CHECK: %s' % e)
        return 2
    problems = []
    for name, paths in sorted(src.items()):
        if len(paths) > 1:
            problems.append('two package.xml files name %s: %s' % (name, ', '.join(paths)))
        n = rows.count(name)
        if n == 0:
            problems.append('MISSING from the table: %s (%s)' % (name, paths[0]))
        elif n > 1:
            problems.append('LISTED %d TIMES: %s' % (n, name))
    for name in sorted(set(rows) - set(src)):
        problems.append('IN THE TABLE BUT NOT IN src/: %s' % name)
    print('packages in src/: %d   rows in the table: %d' % (len(src), len(rows)))
    if problems:
        for p in problems:
            print('  FAIL  ' + p)
        print('JAZZY INVENTORY: INCOMPLETE')
        return 1
    print('JAZZY INVENTORY: COMPLETE, every package appears exactly once')
    return 0


if __name__ == '__main__':
    sys.exit(main())
