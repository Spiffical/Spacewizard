"""Exit nonzero when a running rig has stopped updating its health files."""

import sys
import time
from pathlib import Path


def healthy(path: Path, now: float, timeout: float = 10.0) -> bool:
    try:
        return 0 <= now - path.stat().st_mtime <= timeout
    except OSError:
        return False


def main() -> int:
    now = time.time()
    stale = [path for path in sys.argv[1:] if not healthy(Path(path), now)]
    if stale:
        print("SpaceWizard health heartbeat expired: " + ", ".join(stale), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
