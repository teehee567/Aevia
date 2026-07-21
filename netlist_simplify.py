"""Convert a KiCad-exported ``.net`` file into a review-friendly report.

The default output is a compact, problems-first report intended for both
people and AI reviewers. It keeps exact component/pin/net connectivity while
separating real connections, singleton nets, explicit no-connect markers, and
pins that are absent from the exported netlist. ``--detailed`` retains the
larger library/debug-oriented view.
"""

import argparse
import re
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


def component_unit_pins(comp):
    """Return every library unit and its pin numbers for a component."""
    result = {}
    units = find_first(comp, 'units') or []
    for unit in find_all(units, 'unit'):
        unit_name = get_field(unit, 'name') or '?'
        pins = []
        pin_block = find_first(unit, 'pins') or []
        for pin in find_all(pin_block, 'pin'):
            num = get_field(pin, 'num')
            if num and num not in pins:
                pins.append(num)
        result[unit_name] = pins
    return result


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
        unit_pins = component_unit_pins(comp)
        tstamps = node_scalars(find_first(comp, 'tstamps') or [])
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
            'tstamps': tstamps,
            'placed_unit_count': len(tstamps),
            'unit_pins': unit_pins,
            'pins': [pin for pins in unit_pins.values() for pin in pins],
        })
    comps.sort(key=lambda c: ref_key(c['ref']))
    return comps


NO_CONNECT_STATUSES = {'explicit_no_connect', 'intrinsic_no_connect'}
SINGLETON_REVIEW_STATUSES = {'dangling', 'one_pin_auto', 'named_singleton'}


def node_no_connect_kind(node):
    """Return explicit/intrinsic NC kind, or an empty string."""
    pintype = clean(node.get('pintype', ''))
    if pintype == 'no_connect':
        return 'intrinsic_no_connect'
    if 'no_connect' in pintype.split('+'):
        return 'explicit_no_connect'
    return ''


def classify_net(net):
    nc_kinds = {node_no_connect_kind(node) for node in net['nodes']}
    nc_kinds.discard('')
    if nc_kinds:
        if len(net['nodes']) != 1 or not net['name'].startswith('unconnected-('):
            return 'no_connect_conflict'
        return next(iter(nc_kinds))
    if len(net['nodes']) >= 2:
        return 'connected'
    if not net['nodes']:
        return 'empty'
    if net['name'].startswith('unconnected-('):
        return 'dangling'
    if net['name'].startswith('Net-('):
        return 'one_pin_auto'
    return 'named_singleton'


def parse_nets(nets_block):
    nets = []
    emitted_pins = {}
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
                emitted_pins.setdefault(ref, set()).add(pin)
        nodes.sort(key=lambda n: (ref_key(n['ref']), pin_key(n['pin'])))
        parsed_net = {
            'code': get_field(net, 'code'),
            'name': full_name,
            'short': short_name,
            'class': get_field(net, 'class'),
            'nodes': nodes,
        }
        parsed_net['status'] = classify_net(parsed_net)
        nets.append(parsed_net)

    def net_sort_key(net):
        return (-len(net['nodes']), net['short'], net['name'])

    nets.sort(key=net_sort_key)
    return nets, emitted_pins


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


def load_netlist(src_path):
    text = Path(src_path).read_text(encoding='utf-8', errors='replace')
    tree = parse(tokenize(text))
    design = parse_design(find_first(tree, 'design') or [])
    libparts = parse_libparts(find_first(tree, 'libparts') or [])
    comps = parse_components(find_first(tree, 'components') or [])
    nets, emitted_pins = parse_nets(find_first(tree, 'nets') or [])
    return design, libparts, comps, nets, emitted_pins


def analyse_components(comps, nets):
    """Infer instantiated units and classify every emitted component pin."""
    pin_nets = {}
    duplicate_pin_nets = []
    for net in nets:
        for node in net['nodes']:
            key = (node['ref'], node['pin'])
            if key in pin_nets:
                duplicate_pin_nets.append((key, pin_nets[key], net))
            else:
                pin_nets[key] = net

    result = {}
    for comp in comps:
        ref = comp['ref']
        emitted = {
            pin for (node_ref, pin), net in pin_nets.items()
            if node_ref == ref
        }
        unit_pins = {
            name: set(pins) for name, pins in comp['unit_pins'].items()
        }
        candidate_units = [
            name for name, pins in unit_pins.items() if pins & emitted
        ]
        pin_units = {}
        for unit_name, pins in unit_pins.items():
            for pin in pins:
                pin_units.setdefault(pin, []).append(unit_name)
        definite_units = [
            unit_name for unit_name, pins in unit_pins.items()
            if any(
                pin in emitted and len(pin_units.get(pin, [])) == 1
                for pin in pins
            )
        ]
        over_inferred_units = max(
            0, len(definite_units) - comp['placed_unit_count']
        )

        if len(unit_pins) == 1 and comp['placed_unit_count']:
            inferred_units = list(unit_pins)
            ambiguous_units = []
        elif over_inferred_units:
            inferred_units = []
            ambiguous_units = candidate_units
        else:
            # Shared/stacked pins can make multiple library units candidates.
            # Only definite units are safe to use for missing-pin checks.
            inferred_units = definite_units
            ambiguous_units = [
                unit for unit in candidate_units if unit not in definite_units
            ]

        expected = set()
        for unit_name in inferred_units:
            expected.update(unit_pins[unit_name])

        counts = {
            'connected': 0,
            'named_singleton': 0,
            'dangling': 0,
            'one_pin_auto': 0,
            'no_connect': 0,
            'no_connect_conflict': 0,
            'other': 0,
        }
        for pin in emitted:
            status = pin_nets[(ref, pin)]['status']
            if status in NO_CONNECT_STATUSES:
                counts['no_connect'] += 1
            elif status in counts:
                counts[status] += 1
            else:
                counts['other'] += 1

        result[ref] = {
            'emitted': emitted,
            'expected': expected,
            'missing': sorted(expected - emitted, key=pin_key),
            'inferred_units': inferred_units,
            'candidate_units': candidate_units,
            'ambiguous_units': ambiguous_units,
            'unresolved_unit_instances': max(
                0, comp['placed_unit_count'] - len(inferred_units)
            ),
            'over_inferred_units': over_inferred_units,
            'pin_units': pin_units,
            'counts': counts,
        }

    return {
        'components': result,
        'pin_nets': pin_nets,
        'duplicate_pin_nets': duplicate_pin_nets,
    }


def possible_split_parts(comps, comp_analysis):
    """Find multi-unit packages apparently divided across references."""
    groups = {}
    for comp in comps:
        if len(comp['unit_pins']) < 2:
            continue
        key = (comp['lib'], comp['part'], comp['raw_value'], comp['footprint'])
        groups.setdefault(key, []).append(comp)

    findings = []
    for key, group in groups.items():
        if len(group) < 2:
            continue
        available = set(group[0]['unit_pins'])
        inferred_sets = [
            set(comp_analysis[comp['ref']]['inferred_units']) for comp in group
        ]
        if any(not units for units in inferred_sets):
            continue
        covered = set().union(*inferred_sets)
        disjoint = sum(len(units) for units in inferred_sets) == len(covered)
        unresolved = any(
            comp_analysis[comp['ref']]['unresolved_unit_instances']
            for comp in group
        )
        if disjoint and covered == available and not unresolved:
            findings.append({'key': key, 'components': group})
    return findings


def disconnected_label_groups(nets):
    groups = {}
    for net in nets:
        short = net['short']
        if not short or short.startswith(('Net-(', 'unconnected-(')):
            continue
        groups.setdefault(short, []).append(net)
    return [
        (short, group)
        for short, group in sorted(groups.items())
        if len({net['name'] for net in group}) > 1
    ]


def ref_prefix(ref):
    match = re.match(r'([A-Za-z]+)', ref or '')
    return match.group(1).upper() if match else ''


def is_key_component(comp):
    return ref_prefix(comp['ref']) not in {'R', 'C'}


def type_abbreviation(ptype):
    base = clean(ptype).split('+', 1)[0]
    return {
        'bidirectional': 'bi',
        'input': 'in',
        'open_collector': 'oc',
        'open_emitter': 'oe',
        'output': 'out',
        'passive': '',
        'no_connect': 'nc',
        'power_in': 'pwr-in',
        'power_out': 'pwr-out',
        'tri_state': 'tri',
        'unspecified': '?',
    }.get(base, base)


def endpoint_pin_info(node, comp, libparts):
    libpart = libparts.get((comp['lib'], comp['part']), {}) if comp else {}
    info = libpart.get('pins', {}).get(node['pin'], {})
    name = clean(info.get('name') or node.get('pinfunction', ''))
    suffix = f"_{node['pin']}"
    if name.endswith(suffix):
        name = name[:-len(suffix)]
    ptype = clean(node.get('pintype') or info.get('type', ''))
    return name, ptype


def format_compact_node(node, comp_lookup, libparts, comp_analysis):
    comp = comp_lookup.get(node['ref'])
    ref = node['ref']
    if comp and len(comp['unit_pins']) > 1:
        units = comp_analysis[ref]['pin_units'].get(node['pin'], [])
        if len(units) == 1:
            unit = units[0]
            ref += unit if re.fullmatch(r'[A-Za-z]+', unit) else f"[unit {unit}]"
    token = f"{ref}.{node['pin']}"
    name, ptype = endpoint_pin_info(node, comp, libparts)
    prefix = ref_prefix(node['ref'])
    if comp and prefix in {'R', 'C'} and comp['value']:
        return f"{token}[{comp['value']}]"
    details = [part for part in (name, type_abbreviation(ptype)) if part]
    return f"{token}[{':'.join(details)}]" if details else token


def display_units(comp, stats):
    inferred = stats['inferred_units']
    if not inferred:
        return '?'
    if len(inferred) == 1:
        return inferred[0]
    return ','.join(inferred)


def net_component_sheets(net, comp_lookup):
    return {
        comp_lookup[node['ref']]['sheet'] or '/'
        for node in net['nodes']
        if node['ref'] in comp_lookup
    }


def net_group_sheet(net, comp_lookup):
    name = net['name']
    if name.startswith('/') and '/' in name[1:]:
        return name.rsplit('/', 1)[0] + '/'
    sheets = sorted(net_component_sheets(net, comp_lookup))
    if len(sheets) == 1:
        return sheets[0]
    return '/ (global or cross-sheet)'


def sheet_matches(comp, sheet_filter):
    if not sheet_filter:
        return True
    needle = sheet_filter.strip('/').casefold()
    sheet_name = comp['sheet'].strip('/').casefold()
    file_name = comp['sheet_file'].casefold()
    file_stem = Path(comp['sheet_file']).stem.casefold()
    return needle in {sheet_name, file_name, file_stem}


def select_scope(comps, nets, sheet_filter):
    selected_comps = [comp for comp in comps if sheet_matches(comp, sheet_filter)]
    if sheet_filter and not selected_comps:
        available = sorted({comp['sheet'].strip('/') for comp in comps if comp['sheet']})
        raise ValueError(
            f"sheet {sheet_filter!r} did not match; available sheets: "
            + ', '.join(available)
        )
    selected_refs = {comp['ref'] for comp in selected_comps}
    if not sheet_filter:
        return selected_comps, nets, selected_refs
    selected_nets = [
        net for net in nets
        if any(node['ref'] in selected_refs for node in net['nodes'])
    ]
    return selected_comps, selected_nets, selected_refs


def simplify_detailed(src_path, out_path):
    design, libparts, comps, nets, _ = load_netlist(src_path)
    analysis = analyse_components(comps, nets)
    comp_analysis = analysis['components']
    multi_node_nets = [net for net in nets if net['status'] == 'connected']
    review_nets = [
        net for net in nets
        if net['status'] in SINGLETON_REVIEW_STATUSES | {'no_connect_conflict', 'empty'}
    ]
    no_connect_nets = [net for net in nets if net['status'] in NO_CONNECT_STATUSES]

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
        stats = comp_analysis[comp['ref']]
        counts = stats['counts']
        libref = f"{comp['lib']}:{comp['part']}" if comp['lib'] or comp['part'] else ''
        header = [
            f"{comp['ref']:<{ref_w}}",
            f"value={comp['value'] or '-'}",
            f"footprint={comp['footprint'] or '-'}",
            f"lib={libref or '-'}",
            f"sheet={comp['sheet'] or '/'}",
            f"units={display_units(comp, stats)}",
            f"pins={len(stats['expected']) or '-'}",
            f"connected={counts['connected']}",
            f"singleton={counts['named_singleton'] + counts['dangling'] + counts['one_pin_auto']}",
            f"no_connect={counts['no_connect']}",
            f"nc_conflict={counts['no_connect_conflict']}",
            f"missing={len(stats['missing'])}",
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

    if review_nets:
        lines.append('')
        lines.append('== SINGLE-NODE NETS REQUIRING REVIEW ==')
        for net in sorted(review_nets, key=lambda n: (n['short'], n['name'])):
            display = net['short']
            if net['name'] and net['name'] != net['short']:
                display += f" ({net['name']})"
            prefix = f"{display}  [status={net['status']} code={net['code'] or '-'} class={net['class'] or '-'}]: "
            add_wrapped(lines, prefix, [format_node(node) for node in net['nodes']], indent='  ')

    if no_connect_nets:
        lines.append('')
        lines.append('== EXPLICIT OR INTRINSIC NO-CONNECT NETS ==')
        for net in sorted(no_connect_nets, key=lambda n: (n['short'], n['name'])):
            prefix = f"{net['name']}  [status={net['status']}]: "
            add_wrapped(lines, prefix, [format_node(node) for node in net['nodes']], indent='  ')

    unconnected_sections = []
    for comp in comps:
        libkey = (comp['lib'], comp['part'])
        libpins = libparts.get(libkey, {}).get('pins', {})
        missing = comp_analysis[comp['ref']]['missing']
        if missing:
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


def compress_refs(refs):
    """Compress simple consecutive references (C1,C2,C3 -> C1-C3)."""
    simple_groups = {}
    other = []
    for ref in refs:
        match = re.fullmatch(r'([A-Za-z]+)(\d+)', ref or '')
        if match:
            simple_groups.setdefault(match.group(1), []).append(int(match.group(2)))
        else:
            other.append(ref)

    result = []
    for prefix in sorted(simple_groups):
        numbers = sorted(set(simple_groups[prefix]))
        start = previous = numbers[0]
        for number in numbers[1:] + [None]:
            if number is not None and number == previous + 1:
                previous = number
                continue
            length = previous - start + 1
            if length >= 3:
                result.append(f"{prefix}{start}-{prefix}{previous}")
            elif length == 2:
                result.extend((f"{prefix}{start}", f"{prefix}{previous}"))
            else:
                result.append(f"{prefix}{start}")
            if number is not None:
                start = previous = number
    result.extend(sorted(other, key=ref_key))
    return result


def add_net_lines(lines, nets, comp_lookup, libparts, comp_analysis):
    for net in sorted(nets, key=lambda item: (item['name'].startswith('Net-('), item['name'])):
        class_text = ''
        if net['class'] and net['class'] != 'Default':
            class_text = f" class={net['class']}"
        prefix = f"- {net['name']} [{len(net['nodes'])} nodes{class_text}]: "
        endpoints = [
            format_compact_node(node, comp_lookup, libparts, comp_analysis)
            for node in net['nodes']
        ]
        add_wrapped(lines, prefix, endpoints, indent='  ', sep=', ')


def component_status_text(stats):
    counts = stats['counts']
    text = (
        f"connected={counts['connected']} "
        f"named-singleton={counts['named_singleton']} "
        f"auto-singleton={counts['dangling'] + counts['one_pin_auto']} "
        f"no-connect={counts['no_connect']} "
        f"missing={len(stats['missing'])}"
    )
    if counts['no_connect_conflict']:
        text += f" NC-CONFLICT={counts['no_connect_conflict']}"
    return text


def simplify(src_path, out_path, sheet_filter=''):
    design, libparts, comps, nets, _ = load_netlist(src_path)
    analysis = analyse_components(comps, nets)
    comp_analysis = analysis['components']
    comp_lookup = {comp['ref']: comp for comp in comps}
    scope_comps, scope_nets, scope_refs = select_scope(comps, nets, sheet_filter)

    connected_nets = [net for net in scope_nets if net['status'] == 'connected']
    named_singletons = [net for net in scope_nets if net['status'] == 'named_singleton']
    auto_singletons = [
        net for net in scope_nets if net['status'] in {'dangling', 'one_pin_auto'}
    ]
    no_connect_nets = [net for net in scope_nets if net['status'] in NO_CONNECT_STATUSES]
    conflict_nets = [
        net for net in scope_nets if net['status'] in {'no_connect_conflict', 'empty'}
    ]
    cross_sheet_nets = [
        net for net in connected_nets
        if len(net_component_sheets(net, comp_lookup)) > 1
    ]
    duplicate_labels = [
        (name, group) for name, group in disconnected_label_groups(nets)
        if any(net in scope_nets for net in group)
    ]
    split_parts = [
        finding for finding in possible_split_parts(comps, comp_analysis)
        if any(comp['ref'] in scope_refs for comp in finding['components'])
    ]
    missing_footprints = [comp for comp in scope_comps if not comp['footprint']]
    identity_mismatches = [
        comp for comp in scope_comps
        if ref_prefix(comp['ref']) in {'U', 'IC', 'J'}
        and comp['raw_value'] and comp['part']
        and comp['raw_value'] != comp['part']
    ]
    unit_inference_issues = [
        comp for comp in scope_comps
        if comp_analysis[comp['ref']]['unresolved_unit_instances']
        or comp_analysis[comp['ref']]['over_inferred_units']
    ]
    non_default_classes = sorted({
        net['class'] for net in scope_nets
        if net['class'] and net['class'] != 'Default'
    })

    lines = []
    lines.append('# Netlist review report')
    lines.append('')
    lines.append(f"- Source: {Path(src_path).as_posix()}")
    if design['date']:
        lines.append(f"- Exported: {design['date']}")
    if design['tool']:
        lines.append(f"- Tool: {design['tool']}")
    lines.append(f"- Scope: {sheet_filter or 'entire design'}")
    lines.append('- Generated file: do not edit by hand; regenerate it from the KiCad netlist.')
    lines.append('')
    lines.append('## Summary')
    lines.append('')
    lines.append(f"- Components: {len(scope_comps)} of {len(comps)}")
    lines.append(f"- Nets touching scope: {len(scope_nets)} of {len(nets)}")
    lines.append(f"- Connected nets: {len(connected_nets)}")
    lines.append(f"- Labeled single-ended nets requiring review: {len(named_singletons)}")
    lines.append(f"- Automatic/dangling single-ended nets requiring review: {len(auto_singletons)}")
    lines.append(f"- Explicit or intrinsic no-connects: {len(no_connect_nets)}")
    lines.append(f"- Malformed or empty net records: {len(conflict_nets)}")
    lines.append(f"- Missing footprints: {len(missing_footprints)}")
    lines.append('')
    lines.append('Legend: `REF.PIN[FUNCTION:TYPE]`; `in/out/bi` are pin directions; R/C endpoints show values.')
    lines.append('Multi-unit references include the inferred unit, for example `U1D.G3`.')
    lines.append('A single-ended net has exactly one exported endpoint and is not treated as connected.')
    lines.append('This report describes exported connectivity only; it cannot infer design intent or PCB placement.')
    lines.append('')
    lines.append('## Structural findings')
    lines.append('')

    finding_count = 0
    for finding in split_parts:
        components = []
        for comp in finding['components']:
            units = display_units(comp, comp_analysis[comp['ref']])
            components.append(f"{comp['ref']} units={units}")
        value = finding['key'][2] or finding['key'][1]
        lines.append(
            f"- ATTENTION possible split physical part `{value}`: "
            + '; '.join(components)
            + '. These complementary units use different references.'
        )
        finding_count += 1

    for short, group in duplicate_labels:
        lines.append(f"- ATTENTION `{short}` is {len(group)} disconnected hierarchical nets:")
        for net in sorted(group, key=lambda item: item['name']):
            endpoints = ', '.join(
                format_compact_node(node, comp_lookup, libparts, comp_analysis)
                for node in net['nodes']
            ) or '(no endpoints)'
            lines.append(f"  - {net['name']} -> {endpoints}")
        finding_count += 1

    if cross_sheet_nets:
        summaries = [f"{net['name']}({len(net['nodes'])})" for net in cross_sheet_nets]
        add_wrapped(
            lines,
            f"- Cross-sheet connected nets ({len(cross_sheet_nets)}): ",
            summaries,
            indent='  ',
            sep=', ',
        )
    else:
        lines.append('- ATTENTION no connected net crosses between schematic sheets.')
        finding_count += 1

    if identity_mismatches:
        for comp in identity_mismatches:
            lines.append(
                f"- CHECK component identity {comp['ref']}: "
                f"value={comp['raw_value']}; symbol={comp['lib']}:{comp['part']}."
            )
            finding_count += 1

    if unit_inference_issues:
        for comp in unit_inference_issues:
            stats = comp_analysis[comp['ref']]
            candidates = ','.join(stats['candidate_units']) or 'none'
            if stats['over_inferred_units']:
                lines.append(
                    f"- ATTENTION {comp['ref']}: net endpoints prove more units than the "
                    f"{comp['placed_unit_count']} placed instance(s); candidates={candidates}."
                )
            else:
                lines.append(
                    f"- CHECK {comp['ref']}: {comp['placed_unit_count']} placed unit instance(s), "
                    f"but only {len(stats['inferred_units'])} could be inferred; "
                    f"candidates={candidates}."
                )
            finding_count += 1

    if conflict_nets:
        lines.append(f"- ATTENTION malformed/empty net records: {len(conflict_nets)}.")
        finding_count += 1
    if analysis['duplicate_pin_nets']:
        lines.append(
            f"- ATTENTION component pins assigned to multiple nets: "
            f"{len(analysis['duplicate_pin_nets'])}."
        )
        finding_count += 1
    if missing_footprints:
        lines.append(
            f"- CHECK footprint assignment: {len(missing_footprints)} of "
            f"{len(scope_comps)} components have no footprint."
        )
        finding_count += 1
    if not non_default_classes:
        lines.append('- CHECK all nets in scope use the `Default` net class; no electrical constraints are encoded here.')
        finding_count += 1
    else:
        lines.append('- Non-default net classes: ' + ', '.join(non_default_classes))
    if not finding_count:
        lines.append('- No structural findings generated.')

    lines.append('')
    lines.append('## Sheet summary')
    lines.append('')
    scope_sheets = []
    for comp in scope_comps:
        sheet = comp['sheet'] or '/'
        if sheet not in scope_sheets:
            scope_sheets.append(sheet)
    scope_sheets.sort()
    for sheet in scope_sheets:
        refs = {comp['ref'] for comp in scope_comps if (comp['sheet'] or '/') == sheet}
        touched = [
            net for net in scope_nets
            if any(node['ref'] in refs for node in net['nodes'])
        ]
        status_counts = {}
        for net in touched:
            status_counts[net['status']] = status_counts.get(net['status'], 0) + 1
        lines.append(
            f"- {sheet}: parts={len(refs)} "
            f"connected={status_counts.get('connected', 0)} "
            f"named-singleton={status_counts.get('named_singleton', 0)} "
            f"auto-singleton={status_counts.get('dangling', 0) + status_counts.get('one_pin_auto', 0)} "
            f"no-connect={sum(status_counts.get(status, 0) for status in NO_CONNECT_STATUSES)}"
        )

    lines.append('')
    lines.append('## Key components')
    lines.append('')
    key_comps = [comp for comp in scope_comps if is_key_component(comp)]
    for sheet in scope_sheets:
        sheet_comps = [comp for comp in key_comps if (comp['sheet'] or '/') == sheet]
        if not sheet_comps:
            continue
        lines.append(f"### {sheet}")
        lines.append('')
        for comp in sorted(sheet_comps, key=lambda item: ref_key(item['ref'])):
            stats = comp_analysis[comp['ref']]
            parts = [
                f"- {comp['ref']} {comp['value'] or '-'}",
                f"symbol={comp['lib']}:{comp['part']}",
                f"units={display_units(comp, stats)} ({len(stats['inferred_units'])}/{comp['placed_unit_count']} inferred/placed)",
                f"pins: {component_status_text(stats)}",
                f"footprint={comp['footprint'] or 'MISSING'}",
            ]
            if comp['datasheet']:
                parts.append(f"datasheet={comp['datasheet']}")
            add_wrapped(lines, parts[0] + ' | ', parts[1:], indent='  | ', sep=' | ')
        lines.append('')

    lines.append('## Connected nets')
    lines.append('')
    grouped_connected = {}
    for net in connected_nets:
        group = '/ (global or cross-sheet)' if net in cross_sheet_nets else net_group_sheet(net, comp_lookup)
        grouped_connected.setdefault(group, []).append(net)
    for group in sorted(grouped_connected, key=lambda item: (item != '/ (global or cross-sheet)', item)):
        lines.append(f"### {group}")
        lines.append('')
        add_net_lines(lines, grouped_connected[group], comp_lookup, libparts, comp_analysis)
        lines.append('')

    if conflict_nets:
        lines.append('## Malformed or empty net records')
        lines.append('')
        for net in sorted(conflict_nets, key=lambda item: item['name']):
            endpoints = [
                format_compact_node(node, comp_lookup, libparts, comp_analysis)
                for node in net['nodes']
            ]
            prefix = f"- [{net['status']}] {net['name'] or '(unnamed net)'}: "
            add_wrapped(lines, prefix, endpoints, indent='  ', sep=', ')
        lines.append('')

    lines.append('## Labeled single-ended nets requiring review')
    lines.append('')
    grouped_singletons = {}
    for net in named_singletons:
        grouped_singletons.setdefault(net_group_sheet(net, comp_lookup), []).append(net)
    if not grouped_singletons:
        lines.append('- None.')
        lines.append('')
    for group in sorted(grouped_singletons):
        lines.append(f"### {group}")
        lines.append('')
        add_net_lines(lines, grouped_singletons[group], comp_lookup, libparts, comp_analysis)
        lines.append('')

    lines.append('## Automatic/dangling single-ended nets requiring review')
    lines.append('')
    if not auto_singletons:
        lines.append('- None.')
    else:
        for net in sorted(auto_singletons, key=lambda item: item['name']):
            endpoint = format_compact_node(net['nodes'][0], comp_lookup, libparts, comp_analysis)
            lines.append(f"- [{net['status']}] {net['name']} -> {endpoint}")

    lines.append('')
    lines.append('## Explicit and intrinsic no-connect pins')
    lines.append('')
    no_connect_by_ref = {}
    for net in no_connect_nets:
        node = net['nodes'][0]
        no_connect_by_ref.setdefault(node['ref'], []).append((node, net['status']))
    if not no_connect_by_ref:
        lines.append('- None.')
    for ref in sorted(no_connect_by_ref, key=ref_key):
        entries = []
        for node, status in sorted(no_connect_by_ref[ref], key=lambda item: pin_key(item[0]['pin'])):
            endpoint = format_compact_node(node, comp_lookup, libparts, comp_analysis)
            kind = 'intrinsic' if status == 'intrinsic_no_connect' else 'explicit'
            entries.append(f"{endpoint}({kind})")
        add_wrapped(lines, f"- {ref}: ", entries, indent='  ', sep=', ')

    absent = [comp for comp in scope_comps if comp_analysis[comp['ref']]['missing']]
    lines.append('')
    lines.append('## Pins absent from inferred placed units')
    lines.append('')
    if not absent:
        lines.append('- None. Pins belonging only to uninstantiated library units are intentionally excluded.')
    for comp in absent:
        libpins = libparts.get((comp['lib'], comp['part']), {}).get('pins', {})
        labels = [pin_label(pin, libpins.get(pin, {})) for pin in comp_analysis[comp['ref']]['missing']]
        add_wrapped(lines, f"- {comp['ref']}: ", labels, indent='  ', sep=', ')

    lines.append('')
    lines.append('## R/C value index')
    lines.append('')
    for sheet in scope_sheets:
        rc_comps = [
            comp for comp in scope_comps
            if (comp['sheet'] or '/') == sheet and ref_prefix(comp['ref']) in {'R', 'C'}
        ]
        if not rc_comps:
            continue
        lines.append(f"### {sheet}")
        lines.append('')
        for prefix in ('C', 'R'):
            value_groups = {}
            for comp in rc_comps:
                if ref_prefix(comp['ref']) == prefix:
                    value_groups.setdefault(comp['value'] or '-', []).append(comp['ref'])
            items = []
            for value in sorted(value_groups):
                items.append(f"{value}={','.join(compress_refs(value_groups[value]))}")
            if items:
                add_wrapped(lines, f"- {prefix}: ", items, indent='  ', sep='; ')
        lines.append('')

    lines.append('## Missing footprints')
    lines.append('')
    if not missing_footprints:
        lines.append('- None.')
    for sheet in scope_sheets:
        refs = [
            comp['ref'] for comp in missing_footprints
            if (comp['sheet'] or '/') == sheet
        ]
        if refs:
            add_wrapped(lines, f"- {sheet}: ", compress_refs(refs), indent='  ', sep=', ')

    Path(out_path).write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(
        f"wrote {out_path}  "
        f"({len(scope_comps)} comps, {len(scope_nets)} nets, "
        f"{len(named_singletons) + len(auto_singletons)} review singletons)"
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='KiCad-exported .net file')
    parser.add_argument('output', nargs='?', help='output report path')
    parser.add_argument(
        '--sheet',
        help='only report components/nets touching a matching sheet name or file',
    )
    parser.add_argument(
        '--detailed',
        action='store_true',
        help='emit the larger library/debug-oriented report',
    )
    args = parser.parse_args()
    destination = args.output or str(Path(args.input).with_suffix('.simple.txt'))
    if args.detailed and args.sheet:
        parser.error('--sheet cannot be combined with --detailed')
    if args.detailed:
        simplify_detailed(args.input, destination)
    else:
        try:
            simplify(args.input, destination, sheet_filter=args.sheet or '')
        except ValueError as error:
            parser.error(str(error))
