#!/usr/bin/env python3
"""Print-ready PDF of AprilTag 36h11 landmark tags, one per page, at an exact size.

Reads the tag codes straight from the apriltag library's tag36h11.c (submodule
src/apriltag) and renders each tag the same way apriltag_to_image() does, as
vector rectangles, so it is crisp at any print resolution. No dependencies.

    python3 Scripts/make_apriltag_pdf.py                       # IDs 0-9, 150 mm, Letter
    python3 Scripts/make_apriltag_pdf.py --ids 0 1 2 --size-mm 200 --page a4

"size" is the edge of the black border square, which is what tag_sizes in
config/apriltag_landmarks.yaml must be set to. Print at 100 % / "actual size",
never "fit to page"; each page has a 100 mm scale bar to check with a ruler.
"""

import argparse
import re
from pathlib import Path

PT_PER_MM = 72 / 25.4
PAGES = {'letter': (612.0, 792.0), 'a4': (595.28, 841.89)}


def load_family(c_file):
    """Return (codes, bit_x, bit_y, width_at_border, total_width) from tagNN.c."""
    src = Path(c_file).read_text()
    codes = [int(h, 16) for h in re.findall(r'0x([0-9a-fA-F]+)UL', src)]
    nbits = int(re.search(r'nbits\s*=\s*(\d+)', src).group(1))
    bit_x = [0] * nbits
    bit_y = [0] * nbits
    for axis, arr in (('x', bit_x), ('y', bit_y)):
        for i, v in re.findall(r'bit_%s\[(\d+)\]\s*=\s*(\d+)' % axis, src):
            arr[int(i)] = int(v)
    wab = int(re.search(r'width_at_border\s*=\s*(\d+)', src).group(1))
    tw = int(re.search(r'total_width\s*=\s*(\d+)', src).group(1))
    if re.search(r'reversed_border\s*=\s*true', src):
        raise SystemExit('reversed-border families are not supported')
    return codes, bit_x, bit_y, wab, tw


def tag_cells(code, bit_x, bit_y, wab, tw):
    """Grid [row][col] of True for black, mirroring apriltag_to_image()."""
    border_start = (tw - wab) // 2
    black = [[False] * tw for _ in range(tw)]
    for r in range(border_start, border_start + wab):
        for c in range(border_start, border_start + wab):
            black[r][c] = True
    nbits = len(bit_x)
    for i in range(nbits):
        if code & (1 << (nbits - i - 1)):
            black[bit_y[i] + border_start][bit_x[i] + border_start] = False
    return black


def pdf_text(s):
    return '(' + s.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)') + ')'


def page_stream(tag_id, black, size_mm, wab, tw, page_w, page_h):
    cell = size_mm / wab * PT_PER_MM
    total = cell * tw
    x0 = (page_w - total) / 2
    y_top = page_h - 60
    # One black square for the whole bordered area, then white cells on top:
    # abutting black rectangles can leave hairline seams on some printers.
    bs = (tw - wab) // 2
    ops = ['0 g', f'{x0 + bs * cell:.3f} {y_top - (bs + wab) * cell:.3f} {wab * cell:.3f} {wab * cell:.3f} re f', '1 g']
    for r in range(bs, bs + wab):
        for c in range(bs, bs + wab):
            if not black[r][c]:
                ops.append(f'{x0 + c * cell:.3f} {y_top - (r + 1) * cell:.3f} {cell:.3f} {cell:.3f} re f')
    ops.append('0 g')
    y = y_top - total - 36
    ops.append(f'BT /F1 22 Tf {x0:.1f} {y:.1f} Td {pdf_text(f"AprilTag 36h11   ID {tag_id}   frame tag_{tag_id}")} Tj ET')
    y -= 22
    ops.append(f'BT /F1 13 Tf {x0:.1f} {y:.1f} Td {pdf_text(f"Black square = {size_mm:g} mm edge  (tag_sizes: {size_mm / 1000:.3f}).  Keep the white margin; mount flat.")} Tj ET')
    y -= 18
    ops.append(f'BT /F1 13 Tf {x0:.1f} {y:.1f} Td {pdf_text("Print at 100% / actual size (not fit-to-page) and check the bar below with a ruler.")} Tj ET')
    # 100 mm scale bar with 10 mm ticks
    y -= 40
    bar = 100 * PT_PER_MM
    ops.append(f'1 w {x0:.1f} {y:.1f} m {x0 + bar:.1f} {y:.1f} l S')
    for k in range(11):
        tx = x0 + k * bar / 10
        h = 10 if k % 5 == 0 else 5
        ops.append(f'{tx:.1f} {y:.1f} m {tx:.1f} {y + h:.1f} l S')
    ops.append(f'BT /F1 10 Tf {x0:.1f} {y - 14:.1f} Td {pdf_text("0")} Tj ET')
    ops.append(f'BT /F1 10 Tf {x0 + bar - 22:.1f} {y - 14:.1f} Td {pdf_text("100 mm")} Tj ET')
    return '\n'.join(ops) + '\n'


def write_pdf(path, pages, page_w, page_h):
    """Minimal PDF 1.4 writer: pages = list of content-stream strings."""
    objs = []  # object bodies, 1-indexed

    def add(body):
        objs.append(body)
        return len(objs)

    font = add('<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>')
    pages_obj = add('')  # placeholder, filled after page ids are known
    page_ids = []
    for stream in pages:
        data = stream.encode('latin-1')
        content = add(f'<< /Length {len(data)} >>\nstream\n'.encode('latin-1') + data + b'\nendstream')
        page_ids.append(add(
            f'<< /Type /Page /Parent {pages_obj} 0 R /MediaBox [0 0 {page_w} {page_h}] '
            f'/Resources << /Font << /F1 {font} 0 R >> >> /Contents {content} 0 R >>'))
    objs[pages_obj - 1] = (f'<< /Type /Pages /Count {len(page_ids)} /Kids ['
                           + ' '.join(f'{i} 0 R' for i in page_ids) + '] >>')
    catalog = add(f'<< /Type /Catalog /Pages {pages_obj} 0 R >>')

    out = bytearray(b'%PDF-1.4\n%\xe2\xe3\xcf\xd3\n')
    offsets = []
    for i, body in enumerate(objs, 1):
        offsets.append(len(out))
        out += f'{i} 0 obj\n'.encode('latin-1')
        out += body if isinstance(body, bytes) else body.encode('latin-1')
        out += b'\nendobj\n'
    xref = len(out)
    out += f'xref\n0 {len(objs) + 1}\n0000000000 65535 f \n'.encode('latin-1')
    for off in offsets:
        out += f'{off:010d} 00000 n \n'.encode('latin-1')
    out += (f'trailer\n<< /Size {len(objs) + 1} /Root {catalog} 0 R >>\n'
            f'startxref\n{xref}\n%%EOF\n').encode('latin-1')
    Path(path).write_bytes(out)


def main():
    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ids', type=int, nargs='+', default=list(range(10)))
    ap.add_argument('--size-mm', type=float, default=150.0, help='black border square edge (default 150)')
    ap.add_argument('--page', choices=PAGES, default='letter')
    ap.add_argument('--family-file', default=str(root / 'src' / 'apriltag' / 'tag36h11.c'))
    ap.add_argument('--out', default=None, help='default documents/apriltag_landmarks_36h11_<size>mm.pdf')
    args = ap.parse_args()

    codes, bit_x, bit_y, wab, tw = load_family(args.family_file)
    page_w, page_h = PAGES[args.page]
    if args.size_mm / wab * tw * PT_PER_MM > page_w - 40:
        raise SystemExit(f'{args.size_mm} mm does not fit on {args.page} with margins')
    pages = []
    for tag_id in args.ids:
        if tag_id >= len(codes):
            raise SystemExit(f'ID {tag_id} out of range (family has {len(codes)} codes)')
        black = tag_cells(codes[tag_id], bit_x, bit_y, wab, tw)
        pages.append(page_stream(tag_id, black, args.size_mm, wab, tw, page_w, page_h))
    out = args.out or root / 'documents' / f'apriltag_landmarks_36h11_{args.size_mm:g}mm.pdf'
    write_pdf(out, pages, page_w, page_h)
    print(f'wrote {out}: {len(pages)} page(s), IDs {args.ids}, {args.size_mm:g} mm, {args.page}')


if __name__ == '__main__':
    main()
