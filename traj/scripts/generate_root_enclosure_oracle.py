#!/usr/bin/env python3
"""Regenerate the independent Decimal90 expression regression corpus.

Only Python's standard library is used. Binary64 inputs are imported exactly;
sin/cos use convergent Decimal Taylor series, sqrt uses Decimal.sqrt, and
derivatives use symmetric differences with a 1e-18 step at 90 digits. This is
a host regression oracle, not an MPFR or hardware qualification attestation.
"""

from decimal import Decimal as D, localcontext
from pathlib import Path


def exact(x):
    return D.from_float(float(x))


def vec(xs):
    return tuple(exact(x) for x in xs)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def dot(a, b):
    return sum((x * y for x, y in zip(a, b)), D(0))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def sin_cos(x):
    sine, cosine = x, D(1)
    st, ct = x, D(1)
    for n in range(1, 180):
        st *= -x*x / D(2*n*(2*n+1))
        ct *= -x*x / D((2*n-1)*2*n)
        sine += st
        cosine += ct
    return sine, cosine


def rotate(s, phi, source):
    norm2 = dot(phi, phi)
    if not norm2:
        return source
    norm = norm2.sqrt()
    sn, cs = sin_cos(s*norm)
    first = cross(phi, source)
    return add(source, add(scale(first, sn/norm), scale(cross(phi, first), (1-cs)/norm2)))


def qrotate(q, source):
    first = cross(q[1:], source)
    return add(source, scale(add(scale(first, q[0]), cross(q[1:], first)), D(2)))


COEFFICIENTS = ((4510000, 20, -3, 1), (451000, -8, 7, -2), (4480000, 5, -4, 2))
ROTATIONS = ((0.4, -0.7, 1.1), (1e-6, -2e-6, 3e-6), (0, 0, 0), (0, 0, 8))
CORRECTIONS = ((-0.2, 0.3, 0.15), (-1e-6, 1e-6, 2e-6), (0, 0, 0), (0.2, -0.1, 0.3))


def equations(case, s):
    a, b = vec(ROTATIONS[case]), vec(CORRECTIONS[case])
    q = vec((0.5, 0.5, 0.5, 0.5))
    lever = vec((1.25, -0.5, 0.125))
    r = lambda source: qrotate(q, rotate(s, a, rotate(s, b, source)))
    lever_velocity = qrotate(q, add(rotate(s, a, cross(a, rotate(s, b, lever))), rotate(s, a, rotate(s, b, cross(b, lever)))))
    p = add(tuple(sum(D(c)*(s**n if n else D(1)) for n, c in enumerate(row)) for row in COEFFICIENTS), r(lever))
    v = scale(add(tuple(sum(D(n*c)*(s**(n-1) if n > 1 else D(1)) for n, c in enumerate(row) if n) for row in COEFFICIENTS), lever_velocity), D(1)/exact(1.25))
    x, y, z = p
    major, inv_f = exact(6378137), exact(298.257223563)
    f = 1/inv_f
    minor = major*(1-f)
    e2 = f*(2-f)
    ep2 = e2/(1-e2)
    horizontal = (x*x+y*y).sqrt()
    ty, tx = z/minor, horizontal/major
    tn = (ty*ty+tx*tx).sqrt()
    ly = z/major + ep2*(minor/major)*(ty/tn)**3
    lx = horizontal/major - e2*(tx/tn)**3
    ln = (ly*ly+lx*lx).sqrt()
    up = (lx/ln*x/horizontal, lx/ln*y/horizontal, ly/ln)
    gate = dot(add(p, scale(vec((4510000, 451000, 4480000)), D(-1))), vec((0.6, -0.8, 0)))
    spatial = dot(v, v)
    horizontal_speed = spatial-dot(v, up)**2
    body = dot(v, r(vec((1, 0, 0))))
    return (gate, spatial, horizontal_speed, body, body*body)


def main():
    output = ["# Decimal90 oracle: case,s, then value/first/second for gate, spatial squared, horizontal squared, body signed, body squared"]
    with localcontext() as context:
        context.prec = 90
        h = D('1e-18')
        for case in range(len(ROTATIONS)):
            for parameter in (0.0, 0.125, 0.5, 0.875, 1.0):
                s = exact(parameter)
                lower, center, upper = (equations(case, t) for t in (s-h, s, s+h))
                row = [str(case), str(parameter)]
                for before, value, after in zip(lower, center, upper):
                    row += [format(value, '.65e'), format((after-before)/(2*h), '.65e'), format((after-2*value+before)/(h*h), '.65e')]
                output.append(','.join(row))
    path = Path(__file__).resolve().parents[1] / 'src/trajectory/tests/fixtures/root_enclosure_decimal90.csv'
    path.write_text('\n'.join(output) + '\n')


if __name__ == '__main__':
    main()
