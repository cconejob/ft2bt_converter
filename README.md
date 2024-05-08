# Fault Tree to Behavior Tree Converter

## Overview

This project focuses on the conversion of fault trees, represented in draw.io diagram XML files, into behavior tree XML files compatible with the BehaviorTree.CPP library. It enables users to transform their fault tree diagrams into actionable behavior trees, facilitating integration with systems that utilize the BehaviorTree.CPP framework for managing complex behaviors.

## Installation

Install with PyPI:

```bash
pip install ft2bt
```

## Usage

The tool is designed to convert fault trees from draw.io diagram XML files into behavior tree XML files compatible with the BehaviorTree.CPP library. Here's how to use it:

### Preparing Your Fault Tree Diagram

1. **Create or Open Your Fault Tree Diagram in Draw.io**:
    * First, visit [draw.io](https://draw.io/) to create or edit your fault tree diagram. You may refer to their [documentation](https://www.drawio.com/doc/) for guidance on using the tool.
2. **Diagram Structure & Symbols**:
    * **Hazards**: Represent hazards using rectangles. This is a required element in your diagram.
    * **Events**: Depict events using circles. These are also required elements.
    * **AND/OR Gates**: Use the respective symbols for AND/OR gates in your diagram. These are required for depicting logical relationships in the fault tree.
    * **Probabilities**: Use text below the events to indicate the correspondent probability. Example: `p = 0.1`. These elements are not required.
3. **Exporting the Diagram as XML**:
    * Once your fault tree diagram is ready, you need to export it in XML format. In draw.io, go to `File` > `Export as` > `XML` to save your diagram as an XML file.

<p align="center">
  <img src="ft2bt/test/fault_trees/fta_example.png" alt="Fault Tree Example">
</p>

**Warning!**: All fault tree elements, with the exception of text probabilities, should be connected by directional arrows. Ensure that each arrow is physically attached to its corresponding elements to maintain clarity and accuracy in the diagram.

### Preparing Your Hazard Analysis and Risk Assessment (Opitonal)

Create a *.csv file with some required column names:

1. **Item_ID**: Identificator of the Item analyzed.
2. **Hazard_ID**:  Identificator of the possible Hazard. The ID must match with the name of the correspondent Hazard in the Fault Tree.
3. **Operating_Scenario_ID**: Identificator of the Operating Scenario.
4. **ASIL**: Automotive Safety Integrity Level. Options: A, B, C, D
5. **Safety_State_ID**: Identificator of the Safety State action.

<p align="center">
  <img src="ft2bt/test/hara/hara_example.png" alt="Fault Tree Example">
</p>

### Running the Conversion Tool

Run the conversion command:

```bash
ft2bt [-h] -f FTA_FILEPATH [-v] [-c] [-r] [-o OUTPUT_FOLDER] [-p] [-H HARA_FILEPATH]
```

Where:

* **-f**: (Required) Specifies the XML global filepath name of the draw.io diagram.
* **-v**: (Optional) Automatically shows and saves the renders. Defaults to false.
* **-c**: (Optional) Generate a cpp ROS node template for the behavior tree. Defaults to false.
* **-r**: (Optional) Replaces current code if previously generated and -c is set to True.
* **-o**: (Optional) Specifies the global folder path, where the behavior tree XML diagram is saved.
* **-p**: (Optional) Probabilities are considered to sort the behavior tree nodes.
* **-H**: (Optional) Specifies the CSV global file name of the Hazard Analysis and Risk Assessment (HARA).

### Output Example: Behavior Tree Diagram

Below is an example of the behavior tree diagrams generated from the fault tree. The XML file is loaded using [Groot](https://github.com/BehaviorTree/Groot):

The order of the events is randomly selected in this version of the software. Future versions will sort the events by probability of occurrence.

<p align="center">
  <img src="ft2bt/test/behavior_trees/render/BT_hz_01.svg" alt="Behavior Tree Conversion Example" height="400"> <!-- or you can set the height instead -->
</p>

## Contact Information and Acknowledgement

For further information regarding this project, please feel free to reach out to Carlos Conejo.

This project was developed at the [Institut de Robòtica i Informàtica Industrial (IRI)](https://www.iri.upc.edu/), a joint university research center of the Polytechnic University of Catalonia (UPC) and the Spanish National Research Council (CSIC).

Research partially funded by the Spanish State Research Agency (AEI) and the European Regional Development Fund (ERFD) through the SaCoAV project (ref. PID2020-114244RB-I00). Also funded by Renault Group through the Industrial Doctorate "Safety of Autonomous Vehicles" (ref. C12507).
