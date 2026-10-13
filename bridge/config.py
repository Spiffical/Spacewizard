"""Configuration loading and mapping compilation helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from typing import Any

from .constants import DEFAULT_MAESTRO_HOST, DEFAULT_MAESTRO_PORT


@dataclass
class ConfigLoadResult:
    """Result of loading bridge configuration files."""

    config: dict[str, Any]
    loaded: bool
    config_path: str | None = None
    mapping_path: str | None = None
    mapping_file: str | None = None
    mapping_profile: str | None = None
    warnings: list[str] = field(default_factory=list)
    mapping_compiled_count: int = 0


def get_default_config() -> dict[str, Any]:
    """Return default configuration if config files are missing."""
    return {
        "maestro_settings": {
            "default_host": DEFAULT_MAESTRO_HOST,
            "default_port": DEFAULT_MAESTRO_PORT,
        },
        "synth_settings": {
            "enabled": False,
            "host": "127.0.0.1",
            "port": 57120,
        },
        "mapping_file": "osc_mappings.json",
        "mapping_profiles": {
            "default": "osc_mappings.json",
        },
        "mpu6050_properties": {},
        "data_processing": {
            "smoothing_factor": 0.1,
            "deadzone_threshold": 0.05,
            "output_rate_hz": 40,
            "test_display_rate_hz": 8,
            "enable_debug_output": False,
        },
    }


def _dedupe_paths(paths: list[str]) -> list[str]:
    seen = set()
    deduped = []
    for path in paths:
        normalized = os.path.abspath(path)
        if normalized not in seen:
            seen.add(normalized)
            deduped.append(path)
    return deduped


def _load_first_json(paths: list[str]) -> tuple[dict[str, Any] | None, str | None, str | None]:
    """
    Try candidate paths until one parses as JSON.

    Returns:
    - parsed data, or None
    - path used, or None
    - warning string when a JSON file is found but invalid
    """
    for path in _dedupe_paths(paths):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f), path, None
        except FileNotFoundError:
            continue
        except json.JSONDecodeError as e:
            return None, path, f"Error parsing JSON file {path}: {e}"
    return None, None, None


def _resolve_main_config_candidates(config_path: str) -> list[str]:
    if os.path.isabs(config_path):
        return [config_path]

    project_root = os.path.dirname(os.path.dirname(__file__))
    return [
        config_path,  # Current working directory
        os.path.join(project_root, config_path),  # Bridge project directory
        os.path.join(os.getcwd(), config_path),  # Explicit cwd
    ]


def _resolve_mapping_candidates(mapping_file: str, config_dir: str) -> list[str]:
    if os.path.isabs(mapping_file):
        return [mapping_file]

    project_root = os.path.dirname(os.path.dirname(__file__))
    return [
        os.path.join(config_dir, mapping_file),  # Next to the loaded config file
        mapping_file,  # Current working directory
        os.path.join(project_root, mapping_file),  # Bridge project directory
        os.path.join(os.getcwd(), mapping_file),  # Explicit cwd
    ]


def _merge_mapping_data(config_dict: dict[str, Any], mapping_dict: dict[str, Any]) -> None:
    for key in ("mapping_constants", "osc_routes", "osc_mappings"):
        if key in mapping_dict:
            config_dict[key] = mapping_dict[key]


def _select_mapping_file(
    config_dict: dict[str, Any],
    mapping_profile: str | None,
    mapping_file_override: str | None,
    warnings: list[str],
) -> tuple[str | None, str | None]:
    if mapping_file_override:
        return mapping_file_override, mapping_profile

    if mapping_profile:
        profiles = config_dict.get("mapping_profiles", {})
        if isinstance(profiles, dict):
            selected = profiles.get(mapping_profile)
            if isinstance(selected, str) and selected.strip():
                return selected, mapping_profile
            available_profiles = ", ".join(sorted(profiles.keys())) if profiles else "none"
            warnings.append(
                f"Mapping profile '{mapping_profile}' not found. "
                f"Available profiles: {available_profiles}. "
                "Falling back to mapping_file."
            )
        else:
            warnings.append(
                "mapping_profiles must be an object if mapping_profile is used. "
                "Falling back to mapping_file."
            )

    mapping_file = config_dict.get("mapping_file")
    if isinstance(mapping_file, str) and mapping_file.strip():
        return mapping_file, None
    return None, None


def apply_named_mapping_routes(config_dict: dict[str, Any]) -> tuple[int, list[str]]:
    """
    Compile user-friendly route mappings into the existing osc_mappings format.

    Supported config keys:
    - mapping_constants.sources: reusable source transforms
    - mapping_constants.osc_addresses: reusable OSC addresses
    - osc_routes: route table tying source constants to OSC constants
    """
    if not isinstance(config_dict, dict) or "osc_routes" not in config_dict:
        return 0, []

    routes = config_dict.get("osc_routes", [])
    constants = config_dict.get("mapping_constants", {})
    source_constants = constants.get("sources", {}) if isinstance(constants, dict) else {}
    address_constants = constants.get("osc_addresses", {}) if isinstance(constants, dict) else {}

    warnings: list[str] = []
    if not isinstance(routes, list):
        return 0, ["osc_routes must be a list; ignoring osc_routes."]
    if not isinstance(source_constants, dict):
        warnings.append("mapping_constants.sources must be an object; ignoring source constants.")
        source_constants = {}
    if not isinstance(address_constants, dict):
        warnings.append("mapping_constants.osc_addresses must be an object; ignoring OSC address constants.")
        address_constants = {}

    compiled = []
    mapping_keys = ("scale_min", "scale_max", "invert", "threshold")

    for index, route in enumerate(routes):
        if not isinstance(route, dict):
            warnings.append(f"osc_routes[{index}] must be an object.")
            continue

        source_ref = route.get("source")
        osc_ref = route.get("osc", route.get("osc_address"))
        if not source_ref:
            warnings.append(f"osc_routes[{index}] is missing 'source'.")
            continue
        if not osc_ref:
            warnings.append(f"osc_routes[{index}] is missing 'osc' (or osc_address).")
            continue

        source_defaults = {}
        if isinstance(source_ref, str) and source_ref in source_constants:
            source_constant = source_constants[source_ref]
            if isinstance(source_constant, str):
                resolved_source = source_constant
            elif isinstance(source_constant, dict):
                resolved_source = source_constant.get("source")
                source_defaults = source_constant
            else:
                warnings.append(f"osc_routes[{index}] source constant '{source_ref}' must be a string or object.")
                continue
        elif isinstance(source_ref, str):
            resolved_source = source_ref
        else:
            warnings.append(f"osc_routes[{index}] source must be a string.")
            continue

        if not isinstance(resolved_source, str) or not resolved_source:
            warnings.append(f"osc_routes[{index}] resolves to an invalid source.")
            continue

        if isinstance(osc_ref, str) and osc_ref in address_constants:
            resolved_osc = address_constants[osc_ref]
        else:
            resolved_osc = osc_ref

        if not isinstance(resolved_osc, str) or not resolved_osc.startswith("/"):
            warnings.append(f"osc_routes[{index}] resolves to invalid OSC address '{resolved_osc}'.")
            continue

        compiled_mapping = {
            "enabled": route.get("enabled", True),
            "source": resolved_source,
            "osc_address": resolved_osc,
        }

        for key in mapping_keys:
            if key in source_defaults:
                compiled_mapping[key] = source_defaults[key]
            if key in route:
                compiled_mapping[key] = route[key]

        description = route.get("description", source_defaults.get("description"))
        if description:
            compiled_mapping["description"] = description

        compiled.append(compiled_mapping)

    config_dict["osc_mappings"] = compiled
    return len(compiled), warnings


def load_config(
    config_path: str = "config/bridge.json",
    mapping_profile: str | None = None,
    mapping_file_override: str | None = None,
) -> ConfigLoadResult:
    """Load main config and optional mapping file, then compile osc routes."""
    warnings: list[str] = []
    config_candidates = _resolve_main_config_candidates(config_path)
    config_dict, loaded_path, config_error = _load_first_json(config_candidates)

    loaded = False
    if config_error:
        warnings.append(config_error)
    if config_dict is None:
        config_dict = get_default_config()
        warnings.append(
            "Configuration file not found. Searched: "
            + ", ".join(_dedupe_paths(config_candidates))
            + ". Using defaults."
        )
    else:
        loaded = True

    mapping_path = None
    selected_mapping_file, selected_profile = _select_mapping_file(
        config_dict,
        mapping_profile,
        mapping_file_override,
        warnings,
    )
    if selected_mapping_file:
        base_dir = os.path.dirname(os.path.abspath(loaded_path)) if loaded_path else os.getcwd()
        mapping_candidates = _resolve_mapping_candidates(selected_mapping_file, base_dir)
        mapping_dict, mapping_loaded_path, mapping_error = _load_first_json(mapping_candidates)
        if mapping_error:
            warnings.append(mapping_error)
        if mapping_dict is not None:
            _merge_mapping_data(config_dict, mapping_dict)
            mapping_path = mapping_loaded_path
        else:
            warnings.append(
                f"Mapping file '{selected_mapping_file}' not found. "
                "Using mappings from the main config (if present)."
            )
    elif mapping_profile:
        warnings.append(
            f"No mapping_file resolved for profile '{mapping_profile}'. "
            "Using mappings from the main config (if present)."
        )

    compiled_count, compile_warnings = apply_named_mapping_routes(config_dict)
    warnings.extend(compile_warnings)

    return ConfigLoadResult(
        config=config_dict,
        loaded=loaded,
        config_path=loaded_path,
        mapping_path=mapping_path,
        mapping_file=selected_mapping_file,
        mapping_profile=selected_profile,
        warnings=warnings,
        mapping_compiled_count=compiled_count,
    )
