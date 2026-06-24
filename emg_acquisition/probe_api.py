"""One-shot probe: dump every public method/property exposed by DelsysAPI's
AeroPy class and flag anything that could plausibly drive a sensor's LED.

Default mode does NOT touch the hardware (the .NET surface is on the type, not
the instance, so reflection works on a fresh AeroPy()). Pass --connect to also
call ValidateBase and re-probe in case the documented public surface differs
from what the assembly actually exports post-init.

Invocation:
    ./venv/bin/python probe_api.py            # offline reflection only
    ./probe.sh --connect                      # full env, touches USB (use only when GUI is stopped)
"""
import re
import sys

from delsys_client import _load_aeropy, _load_credentials


KEYWORDS = (
    "identify", "blink", "locate", "flash", "highlight", "find",
    "wake", "led", "indicate", "ping", "signal", "beacon", "buzz",
)


def reflect_dotnet(obj):
    """Return sorted public method signatures from the .NET type behind obj."""
    import clr
    t = clr.GetClrType(type(obj))
    sigs = set()
    for m in t.GetMethods():
        if not m.IsPublic:
            continue
        name = str(m.Name)
        if name.startswith("get_") or name.startswith("set_") or name.startswith("add_") or name.startswith("remove_"):
            continue
        if name in ("ToString", "GetHashCode", "Equals", "GetType"):
            continue
        params = ", ".join(f"{p.ParameterType.Name} {p.Name}" for p in m.GetParameters())
        sigs.add(f"{m.ReturnType.Name} {name}({params})")
    return sorted(sigs)


def reflect_properties(obj):
    import clr
    t = clr.GetClrType(type(obj))
    return sorted(f"{p.PropertyType.Name} {p.Name}" for p in t.GetProperties() if p.CanRead)


def report(base, header):
    print(f"\n=== {header} ===")
    sigs = reflect_dotnet(base)
    print(f"({len(sigs)} public methods)")
    for s in sigs:
        print(f"  {s}")

    props = reflect_properties(base)
    if props:
        print(f"\n-- Public properties ({len(props)}) --")
        for p in props:
            print(f"  {p}")

    pattern = re.compile("|".join(KEYWORDS), re.IGNORECASE)
    hits = [s for s in sigs if pattern.search(s)] + [p for p in props if pattern.search(p)]
    print(f"\n-- Keyword hits ({len(hits)}) for {KEYWORDS} --")
    if hits:
        for h in hits:
            print(f"  HIT: {h}")
    else:
        print("  (none — DelsysAPI exposes nothing LED/identify-shaped)")


def main():
    print("Loading DelsysAPI assembly...")
    AeroPy = _load_aeropy()
    base = AeroPy()
    print(f"AeroPy instance: {base!r}")
    print(f".NET type:       {type(base).__module__}.{type(base).__name__}")

    report(base, "Offline reflection (no ValidateBase)")

    if "--connect" in sys.argv:
        print("\nConnecting to base (touches USB)...")
        key, lic = _load_credentials()
        base.ValidateBase(key, lic)
        print(f"Receiver: {base.GetTrignoReceiverType()}")
        report(base, "After ValidateBase")


if __name__ == "__main__":
    main()
