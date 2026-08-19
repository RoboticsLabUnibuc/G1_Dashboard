#!/usr/bin/env python3
import inspect
import sys
import traceback

def safe_signature(obj):
    try:
        return str(inspect.signature(obj))
    except Exception as exc:
        return f"<unavailable: {exc}>"

def safe_source_file(obj):
    try:
        return inspect.getfile(obj)
    except Exception as exc:
        return f"<unavailable: {exc}>"

def safe_source(obj):
    try:
        return inspect.getsource(obj)
    except Exception as exc:
        return f"<source unavailable: {exc}>"

def main():
    try:
        import vuer
        import vuer.schemas as schemas
        from televuer.televuer import TeleVuer
    except Exception:
        print("=== Import failure ===")
        traceback.print_exc()
        return 1

    print("=== Python ===")
    print(sys.version)
    print("executable:", sys.executable)

    print("\n=== Versions and paths ===")
    print("vuer version:", getattr(vuer, "__version__", "no __version__"))
    print("vuer module:", safe_source_file(vuer))
    print("vuer.schemas:", safe_source_file(schemas))
    print("TeleVuer:", safe_source_file(TeleVuer))

    print("\n=== Camera/control-related schema exports ===")
    matches = []
    for name in sorted(dir(schemas)):
        if "camera" in name.lower() or "control" in name.lower():
            matches.append(name)
            obj = getattr(schemas, name)
            print(f"\n{name}: {obj!r}")
            print("signature:", safe_signature(obj))
            print("source file:", safe_source_file(obj))
    if not matches:
        print("<none>")

    print("\n=== TeleVuer.on_cam_move ===")
    if hasattr(TeleVuer, "on_cam_move"):
        print(safe_source(TeleVuer.on_cam_move))
    else:
        print("<TeleVuer has no on_cam_move attribute>")

    print("\n=== TeleVuer methods containing pass/main/camera ===")
    found = False
    for name in sorted(dir(TeleVuer)):
        lname = name.lower()
        if any(key in lname for key in ("pass", "main", "camera", "scene")):
            obj = getattr(TeleVuer, name)
            if callable(obj):
                found = True
                print(f"\n--- {name} ---")
                print(safe_source(obj))
    if not found:
        print("<none>")

    print("\n=== TeleVuer.__init__ ===")
    print(safe_source(TeleVuer.__init__))

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
