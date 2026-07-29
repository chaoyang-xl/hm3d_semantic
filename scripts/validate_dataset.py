#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hm3d_reconstruction.validator import validate_dataset


def main() -> int:
    p = argparse.ArgumentParser(description="Validate HM3D GT dataset")
    p.add_argument("--data-root", required=True, type=Path)
    p.add_argument("--sample-count", type=int, default=50)
    p.add_argument("--strict", action="store_true")
    p.add_argument("--write-preview", action="store_true")
    args = p.parse_args()
    result = validate_dataset(args.data_root, args.sample_count, args.strict, args.write_preview)
    print(json.dumps({"valid": result.valid, "errors": result.errors, "warnings": result.warnings, "checks": result.checks}, indent=2))
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())

