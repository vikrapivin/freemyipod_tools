"""Physical M2 BCH8 code: identical to M1, verified on 352 p16 raw sectors.

GF(2^13) x^13+x^4+x^3+x+1; generator of degree 104 (roots alpha^1..alpha^16);
message = 512 data bytes then the covered metadata bytes, MSB first; parity MSB first.
"""
GENERATOR = 0x115f914e07b0c138741c5c4fb23


def bch8(message):
    m = int.from_bytes(message, 'big') << 104
    while m.bit_length() > 104:
        m ^= GENERATOR << (m.bit_length() - 105)
    return m.to_bytes(13, 'big')


def _generator(t, poly=0x201b, m=13):
    """Narrow-sense binary BCH generator, roots alpha^1..alpha^2t (bit k = x^k)."""
    n = (1 << m) - 1
    exp, x = [0] * (2 * n), 1
    for i in range(n):
        exp[i] = exp[i + n] = x
        x <<= 1
        if x >> m:
            x ^= poly
    log = {exp[i]: i for i in range(n)}
    seen, g = set(), [1]
    for i in range(1, 2 * t + 1):
        if i in seen:
            continue
        coset, j = [], i
        while j not in coset:
            coset.append(j)
            j = j * 2 % n
        seen.update(coset)
        for c in coset:
            out = [0] * (len(g) + 1)
            for k, coeff in enumerate(g):
                out[k + 1] ^= coeff
                if coeff:
                    out[k] ^= exp[log[coeff] + c]
            g = out
    return sum(c << k for k, c in enumerate(g))


GENERATOR16 = None


def bch16(message):
    """26 parity bytes of the measured BCH16 code (p19)."""
    global GENERATOR16
    if GENERATOR16 is None:
        GENERATOR16 = _generator(16)
    m = int.from_bytes(message, 'big') << 208
    while m.bit_length() > 208:
        m ^= GENERATOR16 << (m.bit_length() - 209)
    return m.to_bytes(26, 'big')
