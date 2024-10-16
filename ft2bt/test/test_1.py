"""
========================================
Test 1
========================================

Fault Tree to Behavior Tree Conversion

"""
import os
from pathlib import Path
try:
    import ft2bt
except:
    raise ImportError("ft2bt package not found. Install the package to run the test.")

# Get the current folder path
current_folder = os.path.dirname(os.path.abspath(__file__))

# Get the child folder path for the fault tree XML file
fta_folder = Path(current_folder) / 'fault_trees'
fta_file = fta_folder / 'fta_example.xml'

output_folder = Path(current_folder).parent.parent / 'behavior_trees'


def main():
    """
    Run the ft2bt script with the fault tree XML file.
    """
    os.system(f'ft2bt -f {fta_file} -o {output_folder} -v')

if __name__ == "__main__":
    main()