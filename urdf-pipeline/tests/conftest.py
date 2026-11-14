import sys
from pathlib import Path

# Make `urdf_pipeline` and `builders` importable without installing the package.
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
