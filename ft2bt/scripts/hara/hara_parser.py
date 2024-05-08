import csv
from pathlib import Path

"""
===================================
HARA CSV File Parser

Columns:
    - Item_ID
    - Hazard_ID
    - Operating_Scenario_ID
    - ASIL
    - Safety_State_ID
===================================
"""

class HARAParser:
    def __init__(self, hara_file):
        self.hara_file = hara_file
        self.hara_dict = self.parse_hara_file()
        
    def parse_hara_file(self):
        """
        Parse the HARA CSV file and store the data in a dictionary
        """
        hara_dict = dict()
        with open(self.hara_file, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                item_id = row['Item_ID']
                operating_scenario_id = row['Operating_Scenario_ID']
                hazard_id = row['Hazard_ID']
                asil = row['ASIL']
                safety_state_id = row['Safety_State_ID']
                if item_id not in hara_dict:
                    hara_dict[item_id] = dict()
                if operating_scenario_id not in hara_dict[item_id]:
                    hara_dict[item_id][operating_scenario_id] = dict()
                if operating_scenario_id not in hara_dict[item_id][operating_scenario_id]:
                    hara_dict[item_id][operating_scenario_id][hazard_id] = dict()
                hara_dict[item_id][operating_scenario_id][hazard_id] = {
                    'ASIL': asil,
                    'Safety_State_ID': safety_state_id
                }
        return hara_dict