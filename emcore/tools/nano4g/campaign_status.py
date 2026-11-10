"""Which registry families can still produce new information.

This tree does not ship a capture index, so every family stays open.
"""
import json
from pathlib import Path


def needed(names):
    """names: candidate family names. This tree has no capture index, so every family is open."""
    return {name: 'no capture index' for name in names}


def record_capture(index, capture, results, errors):
    """Retain partial evidence without mistaking it for a completed family.

    A failed repeat does not reopen a previously completed family. A later
    successful attempt clears the incomplete marker.
    """
    families = index.setdefault('families', {})
    incomplete = index.setdefault('incomplete', {})
    for name, result in results.items():
        if not isinstance(result, dict):
            continue
        data = result.get('data')
        partial = result.get('partial') or name in errors
        if partial and (name not in families or name in incomplete):
            incomplete[name] = dict(capture=capture, cases=len(data) if isinstance(data, dict) else 0)
        elif not partial and isinstance(data, dict) and data:
            incomplete.pop(name, None)
        if isinstance(data, dict) and data:
            seen = families.setdefault(name, [])
            if capture not in seen:
                seen.append(capture)


FOCUS = Path(__file__).resolve().parent / 'focus.json'


def write_focus(names):
    """Write nano4g/focus.json from the families that are still needed."""
    import hashlib
    from datetime import datetime, timezone
    why = needed(names)
    FOCUS.write_text(json.dumps({
        'generated_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'registry_sha256': hashlib.sha256('\n'.join(sorted(names)).encode()).hexdigest(),
        'families': why}, indent=1) + '\n')
    return why


def shipped_focus(names):
    """explore --new: the shipped focus list. Refuses when it is missing or was generated for
    another registry, instead of silently selecting everything."""
    import hashlib
    from .nand import ToolError
    try:
        focus = json.loads(FOCUS.read_text())
    except (OSError, ValueError):
        raise ToolError('nano4g/focus.json is missing')
    if focus.get('registry_sha256') != hashlib.sha256('\n'.join(sorted(names)).encode()).hexdigest():
        raise ToolError('nano4g/focus.json does not match this experiment registry')
    unknown = [n for n in focus['families'] if n not in names]
    if unknown:
        raise ToolError('nano4g/focus.json names unknown families %s' % unknown)
    return focus['families']
