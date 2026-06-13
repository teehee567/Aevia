"""Convert a KiCad/Altium-exported .net file into a short, human-readable summary.

Output sections:
  COMPONENTS  -  ref, short value, footprint
  NETS        -  net name followed by ref.pin tokens
"""

import re
import sys
from pathlib import Path


def tokenize(text):
    tokens = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
        elif c == '(':
            tokens.append('(')
            i += 1
        elif c == ')':
            tokens.append(')')
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while j < n:
                if text[j] == '\\' and j + 1 < n:
                    buf.append(text[j + 1])
                    j += 2
                elif text[j] == '"':
                    break
                else:
                    buf.append(text[j])
                    j += 1
            tokens.append(('STR', ''.join(buf)))
            i = j + 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()':
                j += 1
            tokens.append(('SYM', text[i:j]))
            i = j
    return tokens


def parse(tokens):
    pos = 0

    def parse_expr():
        nonlocal pos
        tok = tokens[pos]
        if tok == '(':
            pos += 1
            items = []
            while tokens[pos] != ')':
                items.append(parse_expr())
            pos += 1
            return items
        elif isinstance(tok, tuple):
            pos += 1
            return tok
        else:
            pos += 1
            return tok

    return parse_expr()


def val(node):
    """Return string value of an atom (SYM/STR tuple) or empty string."""
    if isinstance(node, tuple):
        return node[1]
    return ''


def find_all(tree, key):
    """Yield direct child lists whose head symbol is `key`."""
    for child in tree:
        if isinstance(child, list) and child and isinstance(child[0], tuple) and child[0][1] == key:
            yield child


def find_first(tree, key):
    for c in find_all(tree, key):
        return c
    return None


def get_field(comp, key):
    node = find_first(comp, key)
    if node is None or len(node) < 2:
        return ''
    return val(node[1])


def short_value(comp):
    """Pick the most informative short label for a component."""
    altium = ''
    for field in find_all(find_first(comp, 'fields') or [], 'field'):
        name_attr = None
        for sub in field[1:]:
            if isinstance(sub, list) and sub and val(sub[0]) == 'name':
                name_attr = val(sub[1]) if len(sub) > 1 else ''
        if name_attr == 'ALTIUM_VALUE':
            tail = [x for x in field[1:] if isinstance(x, tuple)]
            if tail:
                altium = tail[-1][1]
    base = get_field(comp, 'value')
    if altium and altium.lower() not in ('', 'not used'):
        if base and base.lower() not in altium.lower():
            return f"{base} {altium}"
        return altium
    return base


def simplify(src_path, out_path):
    text = Path(src_path).read_text(encoding='utf-8', errors='replace')
    tree = parse(tokenize(text))

    components_block = find_first(tree, 'components') or []
    nets_block = find_first(tree, 'nets') or []

    comps = []
    for comp in find_all(components_block, 'comp'):
        ref = get_field(comp, 'ref')
        value = short_value(comp)
        footprint = get_field(comp, 'footprint')
        desc = get_field(comp, 'description')
        comps.append((ref, value, footprint, desc))

    def ref_key(r):
        m = re.match(r'([A-Za-z]+)(\d+)', r)
        return (m.group(1), int(m.group(2))) if m else (r, 0)

    comps.sort(key=lambda c: ref_key(c[0]))

    nets = []
    for net in find_all(nets_block, 'net'):
        name = get_field(net, 'name')
        short = name.rsplit('/', 1)[-1] if name else ''
        nodes = []
        for node in find_all(net, 'node'):
            r = get_field(node, 'ref')
            p = get_field(node, 'pin')
            nodes.append(f"{r}.{p}")
        nodes.sort(key=lambda s: ref_key(s.split('.')[0]))
        nets.append((short, nodes))

    nets.sort(key=lambda n: (-len(n[1]), n[0]))

    lines = []
    lines.append(f"# Simplified netlist  ({len(comps)} components, {len(nets)} nets)")
    lines.append(f"# source: {src_path}")
    lines.append('')
    lines.append('== COMPONENTS ==')
    ref_w = max((len(c[0]) for c in comps), default=4)
    val_w = min(max((len(c[1]) for c in comps), default=4), 24)
    fp_w = min(max((len(c[2]) for c in comps), default=4), 18)
    for ref, value, fp, desc in comps:
        v = (value or '')[:val_w]
        f = (fp or '')[:fp_w]
        d = (desc or '').split(',')[0][:60]
        lines.append(f"{ref:<{ref_w}}  {v:<{val_w}}  {f:<{fp_w}}  {d}")

    lines.append('')
    lines.append('== NETS ==')
    for name, nodes in nets:
        if len(nodes) < 2:
            continue
        lines.append(f"{name}: {' '.join(nodes)}")

    singletons = [n for n in nets if len(n[1]) < 2]
    if singletons:
        lines.append('')
        lines.append(f"# {len(singletons)} single-node nets omitted")

    Path(out_path).write_text('\n'.join(lines), encoding='utf-8')
    print(f"wrote {out_path}  ({len(comps)} comps, {len(nets)} nets)")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: python netlist_simplify.py <input.net> [output.txt]")
        sys.exit(1)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else str(Path(src).with_suffix('.simple.txt'))
    simplify(src, dst)
