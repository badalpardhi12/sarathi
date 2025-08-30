# sarathi/perception/resolve_object.py
import numpy as np

def resolve_object_ref(text, known_entities):
    """known_entities: dict name->callable() returning world pos (e.g., lambda: box.get_pos())."""
    t = text.lower()
    if "red box" in t and "red_box" in known_entities:
        return np.array(known_entities["red_box"](), dtype=float)
    # fallback: first entity
    name = next(iter(known_entities.keys()))
    return np.array(known_entities[name](), dtype=float)
