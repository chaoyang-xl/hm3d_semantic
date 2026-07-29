from __future__ import annotations

from typing import Any


def extract_semantic_metadata(scene: Any) -> dict:
    instances = {}
    for fallback, obj in enumerate(getattr(scene, "objects", []) or []):
        if obj is None:
            continue
        instance_id = int(getattr(obj, "semantic_id", fallback))
        category = getattr(obj, "category", None)
        region = getattr(obj, "region", None)
        name = getattr(category, "name", None)
        index = getattr(category, "index", None)
        instances[str(instance_id)] = {
            "instance_id": instance_id,
            "category_name": name() if callable(name) else name,
            "category_index": index() if callable(index) else index,
            "region_id": str(getattr(region, "id", "")),
            "semantic_object_id": str(getattr(obj, "id", instance_id)),
        }
    return {"instances": instances}

