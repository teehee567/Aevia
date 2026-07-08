"""Convert a KiCad/Altium-exported .net file into a debug-friendly summary.

The output intentionally keeps schematic/netlist meaning (sheets, fields,
library sources, pin names/types, net classes/codes, and unconnected pins)
while omitting placement/coordinate-style data.
"""

import re
import sys
from pathlib import Path


POSITION_FIELD_NAMES = {
    'at',
    'angle',
    'orientation',
    'pos',
    'position',
    'rotation',
    'x',
    'xy',
    'y',
}


SUMMARY_FIELD_NAMES = {
    'Description',
    'Datasheet',
    'Footprint',
    'Reference',
    'Sheetfile',
    'Sheetname',
    'Value',
}


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


def clean(text):
    """Return one-line text suitable for compact summaries."""
    return re.sub(r'\s+', ' ', text or '').strip()


def is_position_field(name):
    key = clean(name).lower().replace('-', '_').replace(' ', '_')
    return key in POSITION_FIELD_NAMES or key.endswith(('_x', '_y', '_xy'))


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


def node_scalars(node):
    return [val(x) for x in node[1:] if isinstance(x, tuple) and val(x)]


def first_scalar(node):
    scalars = node_scalars(node)
    return scalars[0] if scalars else ''


def get_fields(block):
    """Return non-empty KiCad/Altium field values from a fields block."""
    fields = {}
    for field in find_all(block or [], 'field'):
        name = get_field(field, 'name')
        values = node_scalars(field)
        value = clean(values[-1]) if values else ''
        if name and value and not is_position_field(name):
            fields[name] = value
    return fields


def get_properties(block):
    """Return non-empty KiCad property values."""
    properties = {}
    for prop in find_all(block or [], 'property'):
        name = get_field(prop, 'name')
        value = clean(get_field(prop, 'value'))
        if name and value and not is_position_field(name):
            properties[name] = value
    return properties


def merge_maps(*maps):
    merged = {}
    for data in maps:
        for key, value in data.items():
            if value and (key not in merged or not merged[key]):
                merged[key] = value
    return merged


def libsource_info(comp):
    node = find_first(comp, 'libsource') or []
    lib = get_field(node, 'lib')
    part = get_field(node, 'part')
    desc = get_field(node, 'description')
    return lib, part, desc


def sheetpath_info(comp):
    node = find_first(comp, 'sheetpath') or []
    return get_field(node, 'names'), get_field(node, 'tstamps')


def component_pin_numbers(comp):
    pins = []
    units = find_first(comp, 'units') or []
    for unit in find_all(units, 'unit'):
        pin_block = find_first(unit, 'pins') or []
        for pin in find_all(pin_block, 'pin'):
            num = get_field(pin, 'num')
            if num and num not in pins:
                pins.append(num)
    return pins


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


def ref_key(r):
    m = re.match(r'([A-Za-z]+)(\d+)(.*)', r or '')
    if m:
        suffix = m.group(3) or ''
        return (m.group(1), int(m.group(2)), suffix)
    return (r or '', 0, '')


def pin_key(pin):
    pin = pin or ''
    m = re.match(r'([A-Za-z]*)(\d+)(.*)', pin)
    if m:
        return (m.group(1), int(m.group(2)), m.group(3))
    return (pin, 0, '')


def net_name_parts(name):
    short = name.rsplit('/', 1)[-1] if name else ''
    return name, short or name


def format_kv(key, value):
    return f"{key}={clean(value)}"


def add_wrapped(lines, prefix, items, indent='  ', sep=' ', width=140):
    if not items:
        lines.append(prefix.rstrip())
        return

    current = prefix
    first_on_line = True
    for item in items:
        glue = '' if first_on_line else sep
        if len(current) + len(glue) + len(item) > width and not first_on_line:
            lines.append(current.rstrip())
            current = indent + item
            first_on_line = False
        else:
            current += glue + item
            first_on_line = False
    lines.append(current.rstrip())


def add_kv_line(lines, label, data, omit=()):
    items = []
    for key in sorted(data):
        value = data[key]
        if key in omit or not value or is_position_field(key):
            continue
        items.append(format_kv(key, value))
    if items:
        add_wrapped(lines, f"  {label}: ", items, indent='    ', sep='; ')


def parse_design(design_block):
    sheets = []
    for sheet in find_all(design_block, 'sheet'):
        title_block = find_first(sheet, 'title_block') or []
        comments = []
        for comment in find_all(title_block, 'comment'):
            number = get_field(comment, 'number')
            value = clean(get_field(comment, 'value'))
            if value:
                comments.append(f"{number}:{value}" if number else value)
        sheets.append({
            'number': get_field(sheet, 'number'),
            'name': get_field(sheet, 'name'),
            'tstamps': get_field(sheet, 'tstamps'),
            'title': get_field(title_block, 'title'),
            'company': get_field(title_block, 'company'),
            'rev': get_field(title_block, 'rev'),
            'date': get_field(title_block, 'date'),
            'source': get_field(title_block, 'source'),
            'comments': comments,
        })
    return {
        'source': get_field(design_block, 'source'),
        'date': get_field(design_block, 'date'),
        'tool': get_field(design_block, 'tool'),
        'sheets': sheets,
    }


def parse_libparts(libparts_block):
    libparts = {}
    for libpart in find_all(libparts_block, 'libpart'):
        lib = get_field(libpart, 'lib')
        part = get_field(libpart, 'part')
        key = (lib, part)
        footprints = [first_scalar(fp) for fp in find_all(find_first(libpart, 'footprints') or [], 'fp')]
        pins = {}
        for pin in find_all(find_first(libpart, 'pins') or [], 'pin'):
            num = get_field(pin, 'num')
            if not num:
                continue
            pins[num] = {
                'name': clean(get_field(pin, 'name')),
                'type': clean(get_field(pin, 'type')),
            }
        libparts[key] = {
            'lib': lib,
            'part': part,
            'description': clean(get_field(libpart, 'description')),
            'docs': clean(get_field(libpart, 'docs')),
            'fields': get_fields(find_first(libpart, 'fields') or []),
            'footprints': [fp for fp in footprints if fp],
            'pins': pins,
        }
    return libparts


def parse_components(components_block):
    comps = []
    for comp in find_all(components_block, 'comp'):
        fields = get_fields(find_first(comp, 'fields') or [])
        properties = get_properties(comp)
        extra_properties = {
            key: value for key, value in properties.items()
            if fields.get(key) != value
        }
        all_fields = merge_maps(fields, properties)
        lib, part, lib_desc = libsource_info(comp)
        sheet_name, sheet_tstamps = sheetpath_info(comp)
        sheet_file = all_fields.get('Sheetfile', '')
        desc = clean(get_field(comp, 'description') or all_fields.get('Description') or lib_desc)
        datasheet = clean(get_field(comp, 'datasheet') or all_fields.get('Datasheet'))
        comps.append({
            'ref': get_field(comp, 'ref'),
            'value': clean(short_value(comp)),
            'raw_value': clean(get_field(comp, 'value')),
            'footprint': clean(get_field(comp, 'footprint') or all_fields.get('Footprint')),
            'datasheet': datasheet,
            'description': desc,
            'fields': fields,
            'properties': extra_properties,
            'all_fields': all_fields,
            'lib': lib,
            'part': part,
            'lib_desc': clean(lib_desc),
            'sheet': sheet_name,
            'sheet_tstamps': sheet_tstamps,
            'sheet_file': sheet_file,
            'tstamps': node_scalars(find_first(comp, 'tstamps') or []),
            'pins': component_pin_numbers(comp),
        })
    comps.sort(key=lambda c: ref_key(c['ref']))
    return comps


def parse_nets(nets_block):
    nets = []
    connected = {}
    for net in find_all(nets_block, 'net'):
        name = get_field(net, 'name')
        full_name, short_name = net_name_parts(name)
        nodes = []
        for node in find_all(net, 'node'):
            ref = get_field(node, 'ref')
            pin = get_field(node, 'pin')
            info = {
                'ref': ref,
                'pin': pin,
                'pinfunction': clean(get_field(node, 'pinfunction')),
                'pintype': clean(get_field(node, 'pintype')),
            }
            nodes.append(info)
            if ref and pin:
                connected.setdefault(ref, set()).add(pin)
        nodes.sort(key=lambda n: (ref_key(n['ref']), pin_key(n['pin'])))
        nets.append({
            'code': get_field(net, 'code'),
            'name': full_name,
            'short': short_name,
            'class': get_field(net, 'class'),
            'nodes': nodes,
        })

    def net_sort_key(net):
        return (-len(net['nodes']), net['short'], net['name'])

    nets.sort(key=net_sort_key)
    return nets, connected


def format_node(node):
    token = f"{node['ref']}.{node['pin']}"
    extras = []
    if node['pinfunction']:
        extras.append(node['pinfunction'])
    if node['pintype']:
        extras.append(node['pintype'])
    if extras:
        token += f"({', '.join(extras)})"
    return token


def pin_label(pin, pin_info):
    name = pin_info.get('name', '')
    ptype = pin_info.get('type', '')
    if name and ptype:
        return f"{pin}({name}, {ptype})"
    if name:
        return f"{pin}({name})"
    if ptype:
        return f"{pin}({ptype})"
    return pin


def simplify(src_path, out_path):
    text = Path(src_path).read_text(encoding='utf-8', errors='replace')
    tree = parse(tokenize(text))

    design_block = find_first(tree, 'design') or []
    components_block = find_first(tree, 'components') or []
    libparts_block = find_first(tree, 'libparts') or []
    nets_block = find_first(tree, 'nets') or []

    design = parse_design(design_block)
    libparts = parse_libparts(libparts_block)
    comps = parse_components(components_block)
    nets, connected_pins = parse_nets(nets_block)
    multi_node_nets = [net for net in nets if len(net['nodes']) >= 2]
    single_node_nets = [net for net in nets if len(net['nodes']) < 2]

    lines = []
    lines.append(f"# Simplified netlist  ({len(comps)} components, {len(nets)} nets)")
    lines.append(f"# source: {src_path}")
    if design['source']:
        lines.append(f"# design source: {design['source']}")
    if design['date']:
        lines.append(f"# exported: {design['date']}")
    if design['tool']:
        lines.append(f"# tool: {design['tool']}")
    lines.append("# note: placement/coordinate-style fields are intentionally omitted")
    lines.append('')

    if design['sheets']:
        lines.append('== SHEETS ==')
        for sheet in design['sheets']:
            parts = [
                f"#{sheet['number']}" if sheet['number'] else '',
                sheet['name'] or '/',
                f"file={sheet['source']}" if sheet['source'] else '',
                f"rev={sheet['rev']}" if sheet['rev'] else '',
                f"date={sheet['date']}" if sheet['date'] else '',
                f"tstamps={sheet['tstamps']}" if sheet['tstamps'] else '',
            ]
            line = '  '.join(part for part in parts if part)
            lines.append(line)
            if sheet['title'] or sheet['company'] or sheet['comments']:
                details = []
                if sheet['title']:
                    details.append(format_kv('title', sheet['title']))
                if sheet['company']:
                    details.append(format_kv('company', sheet['company']))
                details.extend(format_kv('comment', comment) for comment in sheet['comments'])
                add_wrapped(lines, '  ', details, indent='    ', sep='; ')
        lines.append('')

    lines.append('== COMPONENTS ==')
    ref_w = max((len(c['ref']) for c in comps), default=4)
    for comp in comps:
        libref = f"{comp['lib']}:{comp['part']}" if comp['lib'] or comp['part'] else ''
        header = [
            f"{comp['ref']:<{ref_w}}",
            f"value={comp['value'] or '-'}",
            f"footprint={comp['footprint'] or '-'}",
            f"lib={libref or '-'}",
            f"sheet={comp['sheet'] or '/'}",
            f"pins={len(comp['pins']) or '-'}",
            f"connected={len(connected_pins.get(comp['ref'], set()))}",
        ]
        if comp['sheet_file']:
            header.append(f"sheetfile={comp['sheet_file']}")
        lines.append('  '.join(header))
        if comp['description']:
            add_wrapped(lines, '  desc: ', [comp['description']], indent='    ')
        if comp['datasheet']:
            add_wrapped(lines, '  datasheet: ', [comp['datasheet']], indent='    ')
        if comp['tstamps']:
            add_wrapped(lines, '  tstamps: ', comp['tstamps'], indent='    ')
        add_kv_line(lines, 'fields', comp['fields'], omit=SUMMARY_FIELD_NAMES)
        add_kv_line(lines, 'properties', comp['properties'], omit=SUMMARY_FIELD_NAMES)

    lines.append('')
    lines.append('== NETS ==')
    for net in multi_node_nets:
        display = net['short']
        if net['name'] and net['name'] != net['short']:
            display += f" ({net['name']})"
        prefix = f"{display}  [code={net['code'] or '-'} class={net['class'] or '-'} nodes={len(net['nodes'])}]: "
        add_wrapped(lines, prefix, [format_node(node) for node in net['nodes']], indent='  ')

    if single_node_nets:
        lines.append('')
        lines.append('== SINGLE-NODE NETS ==')
        for net in sorted(single_node_nets, key=lambda n: (n['short'], n['name'])):
            display = net['short']
            if net['name'] and net['name'] != net['short']:
                display += f" ({net['name']})"
            prefix = f"{display}  [code={net['code'] or '-'} class={net['class'] or '-'}]: "
            add_wrapped(lines, prefix, [format_node(node) for node in net['nodes']], indent='  ')

    unconnected_sections = []
    for comp in comps:
        libkey = (comp['lib'], comp['part'])
        libpins = libparts.get(libkey, {}).get('pins', {})
        pin_nums = comp['pins'] or list(libpins)
        missing = [pin for pin in pin_nums if pin not in connected_pins.get(comp['ref'], set())]
        if missing:
            missing.sort(key=pin_key)
            unconnected_sections.append((comp, missing, libpins))

    if unconnected_sections:
        lines.append('')
        lines.append('== PINS WITHOUT NETS ==')
        for comp, missing, libpins in unconnected_sections:
            labels = [pin_label(pin, libpins.get(pin, {})) for pin in missing]
            add_wrapped(lines, f"{comp['ref']}: ", labels, indent='  ')

    if libparts:
        lines.append('')
        lines.append('== LIBPARTS ==')
        for key in sorted(libparts, key=lambda k: (k[0], k[1])):
            libpart = libparts[key]
            type_counts = {}
            for pin in libpart['pins'].values():
                ptype = pin.get('type') or 'unknown'
                type_counts[ptype] = type_counts.get(ptype, 0) + 1
            bits = [
                f"{libpart['lib']}:{libpart['part']}",
                f"pins={len(libpart['pins'])}",
            ]
            if libpart['description']:
                bits.append(f"desc={libpart['description']}")
            if libpart['docs']:
                bits.append(f"docs={libpart['docs']}")
            lines.append('  '.join(bits))
            if type_counts:
                counts = [f"{name}={type_counts[name]}" for name in sorted(type_counts)]
                add_wrapped(lines, '  pin_types: ', counts, indent='    ', sep='; ')
            if libpart['footprints']:
                add_wrapped(lines, '  footprint_filters: ', libpart['footprints'], indent='    ', sep='; ')
            add_kv_line(lines, 'fields', libpart['fields'], omit=SUMMARY_FIELD_NAMES)

    Path(out_path).write_text('\n'.join(lines), encoding='utf-8')
    print(f"wrote {out_path}  ({len(comps)} comps, {len(nets)} nets)")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: python netlist_simplify.py <input.net> [output.txt]")
        sys.exit(1)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else str(Path(src).with_suffix('.simple.txt'))
    simplify(src, dst)
