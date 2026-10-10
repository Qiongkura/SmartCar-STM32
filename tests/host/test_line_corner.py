"""Corner regression entry point for the new square-map implementation."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).with_name('test_line_map.py')),run_name='__main__')
