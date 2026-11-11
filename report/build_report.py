"""Assemble the team's report from immutable incoming sources."""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import re
from typing import Any
from docx import Document
from docx.document import Document as Doc
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt
from docx.text.paragraph import Paragraph

ROOT = Path(__file__).resolve().parent
CITATION = re.compile(r'\[\d+\](?:\s*[–-]\s*\[\d+\]|\s*,\s*\[\d+\])*')


def find(doc: Doc, prefix: str) -> Paragraph:
    matches = [p for p in doc.paragraphs if p.text.startswith(prefix)]
    assert len(matches) == 1, (prefix, len(matches))
    return matches[0]


def replace(p: Paragraph, text: str, bold_lead: str = '') -> None:
    """Keep paragraph properties and body run formatting; preserve bold leads."""
    runs = p.runs
    props = deepcopy(runs[0]._r.rPr) if runs and runs[0]._r.rPr is not None else None
    if not bold_lead and runs and runs[0].bold and runs[0].text != p.text:
        bold_lead = runs[0].text.strip()
    body_props = next((deepcopy(r._r.rPr) for r in runs if not r.bold and r._r.rPr is not None), props)
    p.clear()
    if bold_lead and text.startswith(bold_lead):
        lead = p.add_run(bold_lead)
        if props is not None:
            lead._r.insert(0, deepcopy(props))
        lead.bold = True
        text = text[len(bold_lead):]
    run = p.add_run(text)
    if body_props is not None:
        run._r.insert(0, deepcopy(body_props))
    if bold_lead:
        run.bold = False


def insert_after(anchor: Paragraph, template: Paragraph, text: str) -> Paragraph:
    element = deepcopy(template._p)
    anchor._p.addnext(element)
    result = Paragraph(element, anchor._parent)
    replace(result, text)
    return result


def remove(p: Any) -> None:
    element = p._element
    element.getparent().remove(element)


def clean_md(text: str) -> str:
    return re.sub(r'\s+', ' ', text.replace('**', '').replace('`', '')).strip()


def md_block(md: str, start: str, end: str) -> str:
    return md.split(start, 1)[1].split(end, 1)[0].strip()


def md_rows(md: str, title: str) -> list[list[str]]:
    block = md.split(title, 1)[1].split('\n\n', 2)
    lines = next(part for part in block if part.lstrip().startswith('|'))
    return [[clean_md(cell).replace('*', '') for cell in line.strip().strip('|').split('|')]
            for line in lines.splitlines() if line.startswith('|') and not re.match(r'\|[-| ]+\|$', line)]


def make_table(doc: Doc, anchor: Paragraph, rows: list[list[str]], template: Any) -> Any:
    table = doc.add_table(rows=len(rows), cols=len(rows[0]))
    table._tbl.remove(table._tbl.tblPr)
    table._tbl.insert(0, deepcopy(template._tbl.tblPr))
    for i, row in enumerate(rows):
        for j, text in enumerate(row):
            target = table.cell(i, j)
            source = template.cell(min(i, len(template.rows)-1), min(j, len(template.columns)-1))
            target._tc.remove(target._tc.tcPr)
            target._tc.insert(0, deepcopy(source._tc.tcPr))
            target._tc.remove(target.paragraphs[0]._p)
            target._tc.append(deepcopy(source.paragraphs[0]._p))
            replace(target.paragraphs[0], text)
    anchor._p.addnext(table._tbl)
    return table


def methodology(doc: Doc) -> None:
    source = Document(ROOT / 'incoming/Group1_Report_with_Abir_Methodology.docx')
    start = next(p for p in source.paragraphs if p.text == 'III. METHODOLOGY' and p.style and p.style.name == 'Heading 1')
    end = find(source, 'IV.')
    dest, stop = find(doc, 'III. METHODOLOGY'), find(doc, 'IV.')
    cursor = dest._p.getnext()
    while cursor is not stop._p:
        following = cursor.getnext()
        cursor.getparent().remove(cursor)
        cursor = following
    cursor, anchor = start._p.getnext(), dest._p
    while cursor is not end._p:
        copied = deepcopy(cursor)
        anchor.addnext(copied)
        anchor, cursor = copied, cursor.getnext()
    for p in list(doc.paragraphs):
        if p.text == 'III. METHODOLOGY' and p._p is not dest._p:
            remove(p)
    table = next(t for t in doc.tables if t.cell(0, 0).text.startswith('CARLA 0.9.16 SIMULATION'))
    element = OxmlElement('w:p')
    table._tbl.addprevious(element)
    p = Paragraph(element, doc._body)
    p.alignment = 1
    columns = doc.sections[-1]._sectPr.xpath('./w:cols')
    two = bool(columns and columns[0].get(qn('w:num')) == '2')
    figure = 'figures/architecture_column.png' if two else 'figures/architecture.png'
    p.add_run().add_picture(str(ROOT / figure), width=Inches(3.4 if two else 6.0))
    remove(table)
    caption = find(doc, 'Fig. X')
    replace(caption, 'Fig. 1. System architecture: perception → decision → control. Every tick is also logged by the test harness.')
    p._p.addnext(caption._p)  # IEEE: figure captions go below the figure


D_TEXT = 'D. Obstacle Distance and Decision Logic. The obstacle-distance interface supplies the bumper-to-bumper gap to the nearest obstacle in the driving path, or no value when the path is clear. The evaluations distinguish two sources (Section V): the LiDAR module described above, and a ground-truth fallback that walks the lane in 1 m steps and measures the gap geometrically from actor positions (resolution about ±1 m). The decision logic compares the gap with the speed-dependent stopping distance d_stop = v·t_react + v²/(2a) + margin, with t_react = 1.0 s, a = 6.0 m/s² and margin = 4.0 m. Below 5 m the vehicle always brakes fully (emergency stop).'
F_TEXT = 'The implemented lateral controller is PD-style: steer = 0.10·e_lat − 1.5·e_heading, limited to ±0.4 during lane changes, where e_lat is the lateral error in metres and e_heading the heading error in radians. The speed controller uses a feed-forward throttle of 0.35 plus a proportional term of 0.15 per m/s of speed error.'
OBSTACLE = 'LiDAR-based obstacle detection usually removes the ground first and then groups the remaining points into objects. Himmelsbach et al. {HIM} fit ground lines in polar segments of the point cloud, and Zermas et al. {ZER} fit ground planes per range segment before clustering scan lines; the Point Cloud Library {PCL} provides Euclidean clustering for the second step. Learned detectors such as VoxelNet {VOX} and PointPillars {PP} predict 3D bounding boxes directly from the point cloud with high accuracy on driving benchmarks, but, like learned lane detectors, they need labelled data and GPU inference. For the single question our controller asks — how far ahead is the nearest object in our lane — a geometric pipeline is sufficient and can be checked against ground truth at every tick.'
NEW_REFS = {
'HIM': 'M. Himmelsbach, F. v. Hundelshausen, and H.-J. Wuensche, “Fast segmentation of 3D point clouds for ground vehicles,” in Proc. IEEE Intell. Vehicles Symp. (IV), 2010, pp. 560–565.',
'ZER': 'D. Zermas, I. Izzat, and N. Papanikolopoulos, “Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications,” in Proc. IEEE Int. Conf. Robotics and Automation (ICRA), 2017, pp. 5067–5073.',
'PCL': 'R. B. Rusu and S. Cousins, “3D is here: Point Cloud Library (PCL),” in Proc. IEEE Int. Conf. Robotics and Automation (ICRA), 2011, pp. 1–4.',
'VOX': 'Y. Zhou and O. Tuzel, “VoxelNet: End-to-end learning for point cloud based 3D object detection,” in Proc. IEEE/CVF Conf. Comput. Vision Pattern Recognit. (CVPR), 2018, pp. 4490–4499.',
'PP': 'A. H. Lang, S. Vora, H. Caesar, L. Zhou, J. Yang, and O. Beijbom, “PointPillars: Fast encoders for object detection from point clouds,” in Proc. IEEE/CVF Conf. Comput. Vision Pattern Recognit. (CVPR), 2019, pp. 12697–12705.'}


def update_method_text(doc: Doc) -> None:
    replace(find(doc, 'C. Perception'), C_TEXT)
    replace(find(doc, 'D. Obstacle Distance'), D_TEXT)
    p = find(doc, 'E. Stop-or-Avoid')
    replace(p, p.text.replace('over the full 60 m detection range', 'using the full configured sensing range and projected closing clearance (Section III-D)'))
    p = find(doc, 'G. Test Methodology.')
    replace(p, p.text.replace('The evaluation comprised 90 runs:', 'The original Run A evaluation comprised 90 runs:'))
    p = find(doc, 'F. Vehicle Control.')
    assert p.text.endswith('before submission.')
    replace(p, p.text.split('Although the project task description')[0] + F_TEXT)
    p = find(doc, 'G. Test Methodology.')
    replace(p, p.text.replace('G. Test Methodology.', 'H. Test Methodology.'))
    nav = insert_after(find(doc, 'F. Vehicle Control.'), find(doc, 'F. Vehicle Control.'), NAV_TEXT)
    replace(nav, NAV_TEXT, 'G. Navigation.')
    p = find(doc, 'At every simulation tick')
    replace(p, p.text.split('These results assess')[0] + G_TEXT)


def literature(doc: Doc) -> None:
    heading = find(doc, 'B. Lateral Control')
    body = Paragraph(heading._p.getnext(), doc._body)
    new = insert_after(body, heading, 'C. Obstacle Detection')
    insert_after(new, body, OBSTACLE)
    replace(find(doc, 'C. End-to-End'), 'D. End-to-End Driving')
    replace(find(doc, 'D. Positioning'), 'E. Positioning of This Work')
    p = find(doc, 'Our lane module')
    text = p.text.replace('so every decision can be traced.', 'so every decision can be traced. The obstacle module follows the geometric route of {HIM}, {ZER}: road removal per range bin and a cluster check inside a lane-shaped corridor taken from the map (Section III-C).')
    text = text.replace('Section V currently uses ground-truth perception, and re-running it with the camera module is the next step.', 'Section V reports results with ground-truth perception and with the LiDAR module; a separate camera-steering evaluation is also reported.')
    replace(p, text)
    for p in doc.paragraphs:
        text = p.text
        for i, key in enumerate(NEW_REFS, 21):
            text = text.replace('{'+key+'}', f'[{i}]')
        if text != p.text:
            replace(p, text)


def expand(text: str) -> list[int]:
    values: list[int] = []
    for part in re.split(r'\s*,\s*', text):
        nums = [int(n) for n in re.findall(r'\d+', part)]
        values.extend(range(nums[0], nums[1]+1) if len(nums) == 2 else nums)
    return values


def compress(values: list[int]) -> str:
    groups: list[list[int]] = []
    for value in values:
        if groups and value == groups[-1][-1]+1:
            groups[-1].append(value)
        else:
            groups.append([value])
    return ', '.join(f'[{g[0]}]–[{g[-1]}]' if len(g) >= 3 else ', '.join(f'[{v}]' for v in g) for g in groups)


def patch_citations(p: Paragraph, mapping: dict[int, int]) -> None:
    """Change only citation spans so all surrounding runs remain untouched."""
    for match in reversed(list(CITATION.finditer(p.text))):
        replacement = compress([mapping[v] for v in expand(match[0])])
        offset = 0
        for run in p.runs:
            start, end = offset, offset + len(run.text)
            offset = end
            if end <= match.start() or start >= match.end():
                continue
            left, right = max(0, match.start()-start), min(len(run.text), match.end()-start)
            run.text = run.text[:left] + (replacement if start <= match.start() < end else '') + run.text[right:]


def body_paragraphs(doc: Doc) -> list[Paragraph]:
    paragraphs: list[Paragraph] = []
    for element in doc._body._element:
        if element.tag == qn('w:p'):
            p = Paragraph(element, doc._body)
            if p.text == 'REFERENCES':
                break
            paragraphs.append(p)
        elif element.tag == qn('w:tbl'):
            paragraphs.extend(Paragraph(p, doc._body) for p in element.xpath('.//w:p'))
    return paragraphs


def references(doc: Doc) -> dict[int, int]:
    refs = {int(m[1]): p for p in doc.paragraphs if (m := re.match(r'^\[(\d+)\]', p.text))}
    assert list(refs) == list(range(1, 21))
    anchor = refs[20]
    for i, text in enumerate(NEW_REFS.values(), 21):
        anchor = insert_after(anchor, refs[20], f'[{i}]\t{text}')
        refs[i] = anchor
    order: list[int] = []
    for p in body_paragraphs(doc):
        for match in CITATION.finditer(p.text):
            for number in expand(match[0]):
                assert number in refs, number
                if number not in order:
                    order.append(number)
    order.extend(n for n in refs if n not in order)
    mapping = {old: new for new, old in enumerate(order, 1)}
    for p in body_paragraphs(doc):
        patch_citations(p, mapping)
    anchor = find(doc, 'REFERENCES')
    for old in order:
        p = refs[old]
        patch_citations(p, {old: mapping[old]})
        anchor._p.addnext(p._p)
        anchor = p
    return mapping


def validate(doc: Doc) -> None:
    headings = [p.text for p in doc.paragraphs if p.style and p.style.name == 'Heading 1']
    expected = ['I.', 'II.', 'III.', 'IV.', 'V.', 'VI.', 'REFERENCES']
    assert len(headings) == 7 and all(h.startswith(e) for h, e in zip(headings, expected)), headings
    text = '\n'.join(p.text for p in doc.paragraphs) + '\n' + '\n'.join(c.text for t in doc.tables for row in t.rows for c in row.cells)
    assert not re.search(r'\[(?:Adil|Abir|Khalid):|\[\[|paste .*here|placeholder', text, re.I)
    iv = text.split('IV. INNOVATION', 1)[1].split('V. FINDINGS', 1)[0]
    assert all(any(r.bold and r.text.startswith(letter + '.') for p in doc.paragraphs for r in p.runs) for letter in 'ABCDEF')
    assert 'Six points stand out.' in iv
    refs = [int(m[1]) for p in doc.paragraphs if (m := re.match(r'^\[(\d+)\]', p.text))]
    assert refs == list(range(1, len(refs)+1)), refs
    assert all(n in refs for p in body_paragraphs(doc) for m in CITATION.finditer(p.text) for n in expand(m[0]))
    blob = (ROOT / 'figures/architecture_column.png').read_bytes()
    ids = {r.rId for r in doc.part.rels.values() if 'image' in r.reltype and r.target_part.blob == blob}
    assert len(ids) == 1
    assert sum(b.get(qn('r:embed')) in ids for b in doc._element.xpath('.//a:blip')) == 1
    assert not any(t.cell(0, 0).text.startswith('CARLA 0.9.16 SIMULATION') for t in doc.tables)
    assert 'Fig. X' not in text and '{HIM}' not in text
    assert '194/200' in text and '17/18' in text
    assert any(len(t.rows) == 26 and t.cell(25, 0).text == '25' for t in doc.tables)



# Final implementation details are kept here so the incoming team documents stay immutable.
C_TEXT = "C. Perception and Sensor Suite. Camera lane detection combines a local-contrast white mask, a yellow mask and curb edges with Canny [4] and Hough lines [5] in a wider region of interest. Boundary pairs are checked for vanishing point, centre and width consistency, then ranked using a lane-width prior. A median of accepted widths supports single-side fallback. A PD-style camera controller, using feedback control principles [9], uses offset and heading, a five-estimate median filter and a steering rate limit of 0.15 per update after warmup. Tuning used labelled CARLA frames with map-derived offset and heading: drives 0–1 were tuning data, while drive 2 was held out; controller state was reset per town/drive and frames were evaluated in tick order (Section V). LiDAR processing uses a map-shaped path corridor, range-bin road-height removal and a cluster check requiring a vertical span ≥0.25 m; adjacent-lane detection uses the neighbouring corridor. Core scenarios use a 32-channel, 70 m roof LiDAR. Moving-traffic scenarios use 64 channels and 185 m range, extending adjacent sensing to 180 m. Approximately 150 m of visible free lane is needed for an overtake against closing traffic; a 60–70 m view can become unsafe during the manoeuvre. Camera and LiDAR reads wait, with a bounded timeout, for the current simulation frame, avoiding braking based on stale sweeps."
D_TEXT += " Obstacle speed is estimated from ego speed plus tracked gap change. FOLLOW adaptive cruise uses a 6 m standstill gap plus a 1.8 s time gap, with TTC braking below 3 s. A possibly moving obstacle cannot be overtaken even before motion confirmation. An adjacent lane is free only when nearest gap − closing speed·9 s >10 m; nearby traffic and unknown object speed block pull-out. Clearance must remain free for 1 s. If the target lane closes during a committed manoeuvre, ABORT brakes and returns behind the obstacle where clearance allows. Stronger low-speed steering and creep recovery support pull-out after waiting."
G_TEXT = "Run A preserves the original 90-run ground-truth evaluation. Final Run B uses LiDAR obstacle and adjacent-lane detection with map-waypoint pure-pursuit steering [11] over 200 runs. Run C uses camera steering plus LiDAR for 18 core-scenario runs. A separate Town04 test adds 0.05 m LiDAR range noise. New curve scenarios accumulate 45–120° heading change. Moving scenarios include a slower lead vehicle, a lead braking at 3 m/s², and an oncoming car at 6–8 m/s. Success requires collision-free lane following, stopping with at least 2 m gap, or completed pass-and-return as appropriate. Lead braking additionally requires final speed below 0.2 m/s; oncoming overtakes must never move more than half a lane over while the oncoming vehicle approaches. Full criteria and coverage are given in Section V."


NAV_TEXT = 'G. Navigation. The planner builds a directed graph from the map topology and uses Dijkstra to find a route. Parallel-lane changes carry a 30 m penalty and are excluded inside junctions. A pure-pursuit route follower [11] uses speed-dependent look-ahead 2 m + 0.5 s·v, clamped to 3–6 m. Speed is limited to 5 m/s near and inside junctions and 4 m/s for sharp turns. The LiDAR corridor follows the route and is cut where the route turns away and forward distance stops increasing. Overtaking is disabled within 25 m of a junction and inside it.'


def innovation(doc: Doc) -> None:
    start, end = find(doc, 'IV.'), find(doc, 'V. FINDINGS')
    template = find(doc, 'C. Perception')
    cursor = start._p.getnext()
    while cursor is not end._p:
        following = cursor.getnext()
        cursor.getparent().remove(cursor)
        cursor = following
    anchor = start
    md = (ROOT / 'section_iv_innovation.md').read_text(encoding='utf-8')
    for chunk in re.split(r'\n\s*\n', md.split('\n', 1)[1].strip()):
        anchor = insert_after(anchor, template, clean_md(chunk))
        lead = re.match(r'\*\*(.*?)\*\*', chunk)
        replace(anchor, clean_md(chunk), lead[1] if lead else '')


def final_sections(doc: Doc, md: str) -> None:
    start, end = find(doc, 'V. FINDINGS'), find(doc, 'REFERENCES')
    h1 = deepcopy(start._p)
    h2 = deepcopy(find(doc, 'A. Test Setup')._p)
    body = deepcopy(find(doc, 'Every tick was logged')._p)
    caption = deepcopy(next(p for p in doc.paragraphs if p.text == 'TABLE I')._p)
    template = deepcopy(doc.tables[0])
    cursor = start._p.getnext()
    while cursor is not end._p:
        following = cursor.getnext()
        # Preserve any layout section break rather than silently dropping it.
        if cursor.xpath('.//w:sectPr'):
            raise AssertionError('Unexpected section break inside results')
        cursor.getparent().remove(cursor)
        cursor = following
    anchor = start._p
    def paragraph(text, model=body):
        nonlocal anchor
        element = deepcopy(model)
        anchor.addnext(element)
        p = Paragraph(element, doc._body)
        replace(p, clean_md(text))
        anchor = element
        return p
    block = md.split('## V.', 1)[1].split('## References', 1)[0]
    block = block.split('\n', 1)[1]
    chunks = re.split(r'\n\s*\n', block)
    for chunk in chunks:
        chunk = chunk.strip()
        if not chunk or chunk == '---':
            continue
        if chunk.startswith('### '):
            paragraph(chunk[4:], h2)
        elif chunk.startswith('## VI.'):
            paragraph('VI. CONCLUSION AND FUTURE WORK', h1)
        elif chunk.startswith('**Table '):
            label, title = clean_md(chunk).split(' – ', 1)
            paragraph(label.upper(), caption)
            paragraph(title.upper(), caption)
        elif chunk.startswith('|'):
            rows = [[clean_md(c) for c in l.strip().strip('|').split('|')]
                    for l in chunk.splitlines() if l.startswith('|') and not re.match(r'\|[-| :]+\|$', l)]
            p = Paragraph(anchor, doc._body)
            t = make_table(doc, p, rows, template)
            t.autofit = False
            count = len(rows[0])
            widths = ([0.76] + [(3.4-0.76)/(count-1)]*(count-1)) if count > 1 else [3.4]
            for col, width in zip(t.columns, widths):
                col.width = Inches(width)
            header = OxmlElement('w:tblHeader')
            t.rows[0]._tr.get_or_add_trPr().append(header)
            for row in t.rows:
                for cell, width in zip(row.cells, widths):
                    cell.width = Inches(width)
                    for cp in cell.paragraphs:
                        cp.paragraph_format.space_after = Pt(1)
                        cp.paragraph_format.space_before = Pt(0)
                        for run in cp.runs:
                            run.font.size = Pt(6 if count >= 8 else 7)
                            run.text = run.text.replace('_', '_\u200b')
            t._tbl.tblPr.find(qn('w:tblW')).set(qn('w:w'), str(int(3.4*1440)))
            t._tbl.tblPr.find(qn('w:tblW')).set(qn('w:type'), 'dxa')
            anchor = t._tbl
        elif chunk.startswith('*Fig. '):
            match = re.match(r'\*Fig\. (\d+) – (figures/[^:]+): (.*)\*', chunk)
            assert match, chunk
            p = paragraph('')
            p.alignment = 1
            p.add_run().add_picture(str(ROOT / match[2]), width=Inches(3.4))
            paragraph(f'Fig. {match[1]}. {match[3]}', caption)
        else:
            paragraph(chunk)


def main() -> None:
    doc = Document(ROOT / 'incoming/Group1_IEEE_Report_LitReview_Adil.docx')
    original_sections = [s._sectPr.xml for s in doc.sections]
    title = [p._p.xml for p in doc.paragraphs[:4]]
    md = (ROOT / 'omar_report_sections.md').read_text(encoding='utf-8')
    methodology(doc)
    update_method_text(doc)
    literature(doc)
    innovation(doc)
    for p in list(doc.paragraphs):
        if p.text.strip() == '[Khalid: keep this list consistent with Section IV.]':
            remove(p)
    final_sections(doc, md)
    mapping = references(doc)
    validate(doc)
    assert original_sections == [s._sectPr.xml for s in doc.sections]
    assert title == [p._p.xml for p in doc.paragraphs[:4]]
    output = ROOT / 'Group1_IEEE_Report.docx'
    doc.save(output)
    validate(Document(output))
    print(f'Built {output.name}; validation passed; old->new: ' + ', '.join(f'{k}->{mapping[k]}' for k in sorted(mapping)))


if __name__ == '__main__':
    main()


