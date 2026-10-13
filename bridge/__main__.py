"""Allow running bridge package directly with `python -m bridge`."""

from .app import main


if __name__ == "__main__":
    raise SystemExit(main())

