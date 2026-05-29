#!/usr/bin/env python3
"""Preferred CLI entrypoint for the SpaceWizard OSC bridge."""

from bridge.app import main


if __name__ == "__main__":
    raise SystemExit(main())

