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
from docx.shared import Inches
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


C_TEXT = "C. Perception and Sensor Suite. Lane perception uses a camera-based OpenCV pipeline: an HSV colour mask isolates white lane markings, followed by Canny edge detection [4], a trapezoidal region of interest and probabilistic Hough-line detection [5]. The fitted lane boundaries give the lateral displacement and heading error used by the steering controller. Obstacle perception uses a ray-cast LiDAR mounted centrally on the roof, 2.0 m above the ground, with 32 channels, a vertical field of view from +2° to −24°, a 70 m range and 600,000 points per second. Its rotation frequency equals the simulation rate, so every 0.05 s tick yields one full 360° sweep. The points are transformed into the vehicle frame (x forward, y right, z up) and processed in four steps. (1) A range gate keeps points between the front bumper and 60 m ahead and discards returns from the ego vehicle's own body. (2) A path corridor keeps points within the ego half-width plus 0.3 m of the predicted path; the path follows the shape of the current lane taken from the map, so the corridor bends with curves and ends where the lane turns away. (3) Road removal estimates the road height in 2 m range bins from the lowest returns within 3 m of the corridor, limits the change between bins to a 10 % grade, and keeps only points 0.35–2.5 m above the road, which also removes overhead signs and bridges. (4) A cluster check accepts the nearest group of at least three points within 1 m of depth, which rejects isolated noise returns. The obstacle distance is the gap between the front bumper and that cluster. The same filter, applied over the full width of the neighbouring lane from 15 m behind to 60 m ahead, decides whether the adjacent lane is free; until the first sweep has arrived the lane is reported as not free, so the vehicle never swerves blind."
D_TEXT = 'D. Obstacle Distance and Decision Logic. The obstacle-distance interface supplies the bumper-to-bumper gap to the nearest obstacle in the driving path, or no value when the path is clear. Two sources were evaluated with identical seeds (Section V): the LiDAR module described above, and a ground-truth fallback that walks the lane in 1 m steps and measures the gap geometrically from actor positions (resolution about ±1 m). The decision logic compares the gap with the speed-dependent stopping distance d_stop = v·t_react + v²/(2a) + margin, with t_react = 1.0 s, a = 6.0 m/s² and margin = 4.0 m. Below 5 m the vehicle always brakes fully (emergency stop).'
F_TEXT = 'The implemented lateral controller is PD-style: steer = 0.10·e_lat − 1.5·e_heading, limited to ±0.4 during lane changes, where e_lat is the lateral error in metres and e_heading the heading error in radians. The speed controller uses a feed-forward throttle of 0.35 plus a proportional term of 0.15 per m/s of speed error.'
G_TEXT = 'The full set of 90 runs was executed twice with the same seeds: once with ground-truth perception, which isolates the decision and control logic, and once with the LiDAR module providing both the obstacle distance and the adjacent-lane check. In the LiDAR runs the ground-truth values were logged alongside every tick to measure detection error. Lane steering used the map-waypoint fallback in both runs.'
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
    p = find(doc, 'F. Vehicle Control.')
    assert p.text.endswith('before submission.')
    replace(p, p.text.split('Although the project task description')[0] + F_TEXT)
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
    text = text.replace('Section V currently uses ground-truth perception, and re-running it with the camera module is the next step.', 'Section V reports results with ground-truth perception and with the LiDAR module; re-running them with the camera lane module is the next step.')
    replace(p, text)
    for p in doc.paragraphs:
        text = p.text
        for i, key in enumerate(NEW_REFS, 21):
            text = text.replace('{'+key+'}', f'[{i}]')
        if text != p.text:
            replace(p, text)


def perception(doc: Doc, md: str) -> None:
    p = find(doc, 'Every tick was logged')
    replace(p, p.text.split(' Perception:')[0])
    block = md_block(md, '**Perception sources.**', '### B. Metrics')
    parts = [clean_md(s) for s in re.split(r'\n\s*\n|\n(?=- )', block) if s.strip()]
    anchor = insert_after(p, p, 'Perception sources. ' + parts[0])
    replace(anchor, anchor.text, 'Perception sources.')
    for part in parts[1:]:
        bullet = part.startswith('- ')
        anchor = insert_after(anchor, p, ('• ' + part[2:]) if bullet else part)
        if bullet:
            lead = anchor.text.split(':', 1)[0] + ':'
            replace(anchor, anchor.text, lead)


def results(doc: Doc, md: str) -> None:
    template = doc.tables[0]
    caption = next(p for p in doc.paragraphs if p.text == 'TABLE I')
    subtitle = Paragraph(caption._p.getnext(), doc._body)  # e.g. "OVERALL RESULTS (...)"
    observations = find(doc, 'In S2 the stopping gap')
    anchor = Paragraph(observations._p.getprevious(), doc._body)
    cap = insert_after(anchor, caption, 'TABLE III')
    cap = insert_after(cap, subtitle, 'RUN B: LIDAR PERCEPTION (30 RUNS PER SCENARIO, SAME SEEDS)')
    table = make_table(doc, cap, md_rows(md, '**Table III'), template)
    text = clean_md(md_block(md, 'All six towns:', '**Additional observations'))
    p = insert_after(cap, observations, 'All six towns: ' + text)
    table._tbl.addnext(p._p)
    for p in doc.paragraphs:
        if p.text.startswith('Fig. ') and not p.text.startswith('Fig. 1. System architecture'):
            replace(p, re.sub(r'Fig\. (\d+)', lambda m: f'Fig. {int(m[1])+1}', p.text))
        elif 'Figs. 1–3' in p.text:
            replace(p, p.text.replace('Figs. 1–3', 'Figs. 2–4'))


def iterations(doc: Doc, md: str) -> None:
    heading = find(doc, 'Development iterations.')
    table = make_table(doc, heading, md_rows(md, 'regression-tested (28 automated tests):'), doc.tables[0])
    cursor = table._tbl.getnext()
    while cursor is not None and cursor.tag == qn('w:p'):
        p = Paragraph(cursor, doc._body)
        if p.text.startswith('Limitations.'):
            break
        following = cursor.getnext()
        cursor.getparent().remove(cursor)
        cursor = following
    p = find(doc, 'Limitations.')
    item = clean_md(md_block(md, '1. **Perception is partly idealised.**', '2. **Static obstacles'))
    rest = p.text[p.text.index(' (2)'):]
    replace(p, 'Limitations. (1) Perception is partly idealised. ' + item + rest)
    # Preserve the existing limitation paragraph and bold the requested first-item lead.
    text = p.text
    replace(p, text.split('Perception is partly idealised.')[0])
    r = p.add_run('Perception is partly idealised.')
    if p.runs[0]._r.rPr is not None:
        r._r.insert(0, deepcopy(p.runs[0]._r.rPr))
    r.bold = True
    r = p.add_run(text.split('Perception is partly idealised.', 1)[1])
    if p.runs[0]._r.rPr is not None:
        r._r.insert(0, deepcopy(p.runs[0]._r.rPr))
    remove(find(doc, '[After integration:'))


def conclusion(doc: Doc, md: str) -> None:
    block = md_block(md, '## VI. Conclusion & Future Work', 'Future work:')
    text = clean_md(block.split('\n\n', 1)[1])
    replace(find(doc, 'This project produced'), text)


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
    text = '\n'.join(p.text for p in doc.paragraphs)
    assert '[Adil:' not in text and '[Abir:' not in text
    iv = text.split('IV. INNOVATION', 1)[1].split('V. FINDINGS', 1)[0]
    assert '[Khalid:' in iv
    refs = [int(m[1]) for p in doc.paragraphs if (m := re.match(r'^\[(\d+)\]', p.text))]
    assert refs == list(range(1, len(refs)+1)), refs
    assert all(n in refs for p in body_paragraphs(doc) for m in CITATION.finditer(p.text) for n in expand(m[0]))
    blob = (ROOT / 'figures/architecture_column.png').read_bytes()
    ids = {r.rId for r in doc.part.rels.values() if 'image' in r.reltype and r.target_part.blob == blob}
    assert len(ids) == 1
    assert sum(b.get(qn('r:embed')) in ids for b in doc._element.xpath('.//a:blip')) == 1
    assert not any(t.cell(0, 0).text.startswith('CARLA 0.9.16 SIMULATION') for t in doc.tables)
    assert 'Fig. X' not in text and '{HIM}' not in text
    assert '[[Rerun with Adil\'s OpenCV lane module once integrated.]]' in text
    assert any(len(t.rows) == 13 and t.cell(12, 0).text == '12' for t in doc.tables)


def main() -> None:
    doc = Document(ROOT / 'incoming/Group1_IEEE_Report_LitReview_Adil.docx')
    original_sections = [s._sectPr.xml for s in doc.sections]
    title = [p._p.xml for p in doc.paragraphs[:4]]
    md = (ROOT / 'omar_report_sections.md').read_text(encoding='utf-8')
    methodology(doc)
    update_method_text(doc)
    literature(doc)
    perception(doc, md)
    results(doc, md)
    iterations(doc, md)
    conclusion(doc, md)
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


