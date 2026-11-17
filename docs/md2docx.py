"""Minimal Markdown -> .docx for DOCUMENTATION.md (headings, paragraphs, bullets,
numbered lists, tables, code blocks, quotes, bold/inline code)."""
import re
import sys

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Pt, RGBColor

src, dst = sys.argv[1], sys.argv[2]
lines = open(src, encoding="utf-8").read().splitlines()
doc = Document()
style = doc.styles["Normal"]
style.font.name = "Calibri"
style.font.size = Pt(11)

INLINE = re.compile(r"(\*\*[^*]+\*\*|`[^`]+`|\*[^*]+\*)")


def add_runs(par, text):
    for part in INLINE.split(text):
        if not part:
            continue
        if part.startswith("**") and part.endswith("**"):
            par.add_run(part[2:-2]).bold = True
        elif part.startswith("`") and part.endswith("`"):
            r = par.add_run(part[1:-1])
            r.font.name = "Consolas"
            r.font.size = Pt(10)
        elif part.startswith("*") and part.endswith("*") and len(part) > 2:
            par.add_run(part[1:-1]).italic = True
        else:
            par.add_run(part)


def shade(cell, hex_fill):
    tc = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tc.append(shd)


def cells(row):
    return [c.strip() for c in row.strip().strip("|").split("|")]


i = 0
while i < len(lines):
    line = lines[i]
    if line.startswith("```"):
        block = []
        i += 1
        while i < len(lines) and not lines[i].startswith("```"):
            block.append(lines[i])
            i += 1
        p = doc.add_paragraph()
        r = p.add_run("\n".join(block))
        r.font.name = "Consolas"
        r.font.size = Pt(9)
        i += 1
        continue
    if line.startswith("|") and i + 1 < len(lines) and re.match(r"^\|[-| :]+\|$", lines[i + 1].strip()):
        header = cells(line)
        rows = []
        i += 2
        while i < len(lines) and lines[i].startswith("|"):
            rows.append(cells(lines[i]))
            i += 1
        table = doc.add_table(rows=1 + len(rows), cols=len(header))
        table.style = "Table Grid"
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        for j, text in enumerate(header):
            cell = table.rows[0].cells[j]
            cell.text = ""
            run = cell.paragraphs[0].add_run(text)
            run.bold = True
            run.font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
            shade(cell, "1F2329")
        for r_i, row in enumerate(rows, 1):
            for j, text in enumerate(row[:len(header)]):
                cell = table.rows[r_i].cells[j]
                cell.text = ""
                add_runs(cell.paragraphs[0], text)
        for row in table.rows:
            for cell in row.cells:
                for par in cell.paragraphs:
                    for run in par.runs:
                        run.font.size = Pt(9.5)
        doc.add_paragraph()
        continue
    if line.startswith("#"):
        level = len(line) - len(line.lstrip("#"))
        doc.add_heading(line.lstrip("#").strip(), level=min(level - 1, 3) if level > 1 else 0)
    elif line.strip() == "---" or not line.strip():
        pass
    elif line.startswith("> "):
        p = doc.add_paragraph(style="Intense Quote")
        add_runs(p, line[2:])
    elif re.match(r"^\s*- ", line):
        text = re.sub(r"^\s*- ", "", line)
        while i + 1 < len(lines) and lines[i + 1].startswith("  ") and lines[i + 1].strip() and not lines[i + 1].strip().startswith("-"):
            i += 1
            text += " " + lines[i].strip()
        add_runs(doc.add_paragraph(style="List Bullet"), text)
    elif re.match(r"^\d+\. ", line):
        text = re.sub(r"^\d+\. ", "", line)
        while i + 1 < len(lines) and lines[i + 1].startswith("   ") and lines[i + 1].strip():
            i += 1
            text += " " + lines[i].strip()
        add_runs(doc.add_paragraph(style="List Number"), text)
    else:
        text = line
        while (i + 1 < len(lines) and lines[i + 1].strip() and not lines[i + 1].startswith(("#", "|", "-", "```", ">"))
               and not re.match(r"^\d+\. ", lines[i + 1])):
            i += 1
            text += " " + lines[i].strip()
        add_runs(doc.add_paragraph(), text)
    i += 1

doc.save(dst)
print("wrote", dst)
