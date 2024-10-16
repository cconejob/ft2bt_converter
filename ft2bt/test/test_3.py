"""
========================================
Test 3
========================================

Fault Tree and HARA to Behavior Tree Conversion (considering FTA probabilities)

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
hara_folder = Path(current_folder) / 'hara'

fta_file = fta_folder / 'fta_example.xml'
hara_file = hara_folder / 'hara_example.csv'

output_folder = Path(current_folder).parent.parent / 'behavior_trees'

def main():
    """
    Run the ft2bt script with the fault tree XML file.
    """
    os.system(f'ft2bt -f {fta_file} -o {output_folder} -v -H {hara_file} -os -p')

if __name__ == "__main__":
    main()