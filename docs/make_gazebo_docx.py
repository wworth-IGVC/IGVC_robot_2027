#!/usr/bin/env python3
"""
make_gazebo_docx.py

Generate docs/IGVC_2027_Gazebo_Setup_Guide.docx from GAZEBO_QUICKSTART.md.

The Markdown is the source of truth. Edit GAZEBO_QUICKSTART.md and re-run this;
do not edit the .docx, because the next run overwrites it.

    python3 docs/make_gazebo_docx.py

Word LOCKS the file while it is open, and the write then fails with a
permission error. Close Word first, or write somewhere else and copy it in:

    DOCX_OUT=/tmp/guide.docx python3 docs/make_gazebo_docx.py

Needs python-docx. On this project that lives in the Git Bash python3, not the
PowerShell one:

    python3 -m pip install python-docx
"""

import os
import re
import sys

try:
    from docx import Document
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    from docx.shared import Pt, RGBColor, Inches
except ImportError:
    sys.exit("python-docx is not installed:  python3 -m pip install python-docx")

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "GAZEBO_QUICKSTART.md")
OUT = os.environ.get("DOCX_OUT") or os.path.join(
    HERE, "IGVC_2027_Gazebo_Setup_Guide.docx")

MONO = "Consolas"
CODE_BG = RGBColor(0x1E, 0x1E, 0x1E)


def add_code(doc, lines):
    """One shaded, monospaced block. Keeps blank lines inside the block."""
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines:
        return
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    p.paragraph_format.space_after = Pt(10)
    p.paragraph_format.space_before = Pt(6)
    run = p.add_run("\n".join(lines))
    run.font.name = MONO
    run.font.size = Pt(9)
    run.font.color.rgb = RGBColor(0x11, 0x33, 0x55)


def add_rich(paragraph, text):
    """Render **bold**, `code` and ~~strike~~ inside one paragraph."""
    for tok in re.split(r"(\*\*.+?\*\*|`[^`]+`|~~.+?~~)", text):
        if not tok:
            continue
        if tok.startswith("**") and tok.endswith("**"):
            paragraph.add_run(tok[2:-2]).bold = True
        elif tok.startswith("~~") and tok.endswith("~~"):
            r = paragraph.add_run(tok[2:-2])
            r.font.strike = True
        elif tok.startswith("`") and tok.endswith("`"):
            r = paragraph.add_run(tok[1:-1])
            r.font.name = MONO
            r.font.size = Pt(9.5)
        else:
            paragraph.add_run(tok)


def split_row(line):
    cells = [c.strip() for c in line.strip().strip("|").split("|")]
    return cells


def main():
    if not os.path.exists(SRC):
        sys.exit("missing source: %s" % SRC)
    lines = open(SRC, encoding="utf-8").read().splitlines()

    doc = Document()
    style = doc.styles["Normal"]
    style.font.name = "Calibri"
    style.font.size = Pt(10.5)

    i = 0
    in_code = False
    code = []
    table_buf = []

    def flush_table():
        if len(table_buf) < 2:
            table_buf.clear()
            return
        header = split_row(table_buf[0])
        body = [split_row(r) for r in table_buf[2:]]
        t = doc.add_table(rows=1, cols=len(header))
        t.style = "Light Grid Accent 1"
        for c, name in zip(t.rows[0].cells, header):
            c.text = ""
            add_rich(c.paragraphs[0], name)
            for r in c.paragraphs[0].runs:
                r.bold = True
        for row in body:
            cells = t.add_row().cells
            for c, val in zip(cells, row):
                c.text = ""
                add_rich(c.paragraphs[0], val)
        doc.add_paragraph()
        table_buf.clear()

    while i < len(lines):
        line = lines[i]

        if line.startswith("```"):
            if in_code:
                add_code(doc, code)
                code = []
            in_code = not in_code
            i += 1
            continue
        if in_code:
            code.append(line)
            i += 1
            continue

        if line.startswith("|"):
            table_buf.append(line)
            i += 1
            continue
        if table_buf:
            flush_table()

        if line.startswith("### "):
            doc.add_heading(line[4:].strip(), level=3)
        elif line.startswith("## "):
            doc.add_heading(line[3:].strip(), level=2)
        elif line.startswith("# "):
            h = doc.add_heading(line[2:].strip(), level=0)
            h.alignment = WD_ALIGN_PARAGRAPH.CENTER
        elif line.strip() == "---":
            doc.add_paragraph()
        elif re.match(r"^\s*[-*] ", line):
            p = doc.add_paragraph(style="List Bullet")
            add_rich(p, re.sub(r"^\s*[-*] ", "", line))
        elif re.match(r"^\s*\d+\. ", line):
            p = doc.add_paragraph(style="List Number")
            add_rich(p, re.sub(r"^\s*\d+\. ", "", line))
        elif line.strip():
            # Join wrapped Markdown lines into one paragraph.
            buf = [line.strip()]
            while (i + 1 < len(lines) and lines[i + 1].strip()
                   and not lines[i + 1].startswith(("#", "|", "```", "---"))
                   and not re.match(r"^\s*([-*]|\d+\.) ", lines[i + 1])):
                i += 1
                buf.append(lines[i].strip())
            add_rich(doc.add_paragraph(), " ".join(buf))
        i += 1

    if table_buf:
        flush_table()
    if code:
        add_code(doc, code)

    try:
        doc.save(OUT)
    except PermissionError:
        sys.exit("cannot write %s - it is open in Word. Close it, or set "
                 "DOCX_OUT to another path." % OUT)
    print("wrote %s" % OUT)


if __name__ == "__main__":
    main()
