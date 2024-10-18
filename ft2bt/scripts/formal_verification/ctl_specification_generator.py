import pandas as pd


class CTLSpecificationGenerator:
    def __init__(self, hara_file_path):
        self.hara_df = pd.read_csv(hara_file_path)

    def convert_subscripts(self, text):
        # Define a mapping for subscript numbers to regular numbers
        subscripts = str.maketrans("₀₁₂₃₄₅₆₇₈₉", "0123456789")
        return text.translate(subscripts)

    def generate_ctl_specifications(self):
        ctl_output = "-------------------------------------------------------------------------------------------------------------------------\n"
        ctl_output += "-- CTL PROPERTIES\n"
        ctl_output += "-------------------------------------------------------------------------------------------------------------------------\n\n"
        
        # Apply conversion to remove subscripts in all relevant columns
        self.hara_df['Operating_Scenario_ID'] = self.hara_df['Operating_Scenario_ID'].apply(self.convert_subscripts)
        self.hara_df['Hazard_ID'] = self.hara_df['Hazard_ID'].apply(self.convert_subscripts)
        self.hara_df['Safety_State_ID'] = self.hara_df['Safety_State_ID'].apply(self.convert_subscripts)
        
        # Group by 'Item_ID' and 'Operating_Scenario_ID'
        items = self.hara_df['Item_ID'].unique()
        for item in items:
            item_df = self.hara_df[self.hara_df['Item_ID'] == item]
            os_ids = item_df['Operating_Scenario_ID'].unique()
            
            # Proposition 1: Check operating situations until one is successful
            os_enable_conditions = " & ".join([f"{item}.condition_{os_id}.enable" for os_id in os_ids])
            os_success_conditions = " | ".join([f"{item}.condition_{os_id}.output = Success" for os_id in os_ids])
            ctl_output += f"-- PROPOSITION 1\n"
            ctl_output += f"-- Verify that operating situations (os) are being checked until one is successful. \n"
            ctl_output += f"-- Verify that there is always one successful operating situation.\n\n"
            ctl_output += f"CTLSPEC\n    AG ({os_enable_conditions} -> AG({os_success_conditions}));\n\n\n"
            
            # Proposition 2: Check hazards within each operating scenario until one is successful or all fail
            for os_id in os_ids:
                hazards_df = item_df[item_df['Operating_Scenario_ID'] == os_id]
                hazard_success_conditions = " | ".join([f"{item}.{os_id}.{hazard}.output = Success" for hazard in hazards_df['Hazard_ID']])
                hazard_failure_conditions = " & ".join([f"{item}.{os_id}.{hazard}.output = Failure" for hazard in hazards_df['Hazard_ID']])
                ctl_output += f"-- PROPOSITION 2\n"
                ctl_output += f"-- Verify that when an operating situation is found, its corresponding hazards are being checked until one is found. \n"
                ctl_output += f"-- It is not necessary to check all hazards if one is found. If no event is found, any hazard is detected. \n\n"
                ctl_output += f"CTLSPEC\n    AG ({item}.condition_{os_id}.output = Success -> AG({hazard_success_conditions} | {hazard_failure_conditions}));\n\n"
            
            # Proposition 3: Check safety state actions for each hazard and operating scenario
            for os_id in os_ids:
                for index, row in item_df[item_df['Operating_Scenario_ID'] == os_id].iterrows():
                    hazard = row['Hazard_ID']
                    safety_state = row['Safety_State_ID']
                    ctl_output += f"-- PROPOSITION 3\n"
                    ctl_output += f"-- Verify that when a hazard found under the identified operating scenatio, its corresponding safety state action \n"
                    ctl_output += f"-- is running until the safety goal is reached\n\n"
                    ctl_output += f"CTLSPEC\n    AG ({item}.{os_id}.{hazard}.output = Success -> AX( A [{item}.{os_id}.{safety_state}.output = Running U {item}.{os_id}.{safety_state}.goal_reached]));\n\n"
                    
        return ctl_output
    
    def write_ctl_specifications(self, output_file, ctl_specifications):
        with open(output_file, 'a') as file:
            file.write(ctl_specifications)

        print("CTL specifications have been generated and saved to:", output_file)