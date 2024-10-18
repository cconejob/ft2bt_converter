# Fault Tree to Behavior Tree Converter

## Overview

This repository focuses on the conversion of fault trees, represented in draw.io diagram XML files, into behavior tree XML files compatible with the BehaviorTree.CPP library. It enables users to transform their fault tree diagrams into actionable behavior trees, facilitating integration with systems that utilize the BehaviorTree.CPP framework for managing complex behaviors.

The project is specifically designed for **Functional Safety (FuSa)** runtime assessment for **autonomous vehicles**, adhering to the **ISO 26262** standard for road vehicles. It allows for the automatic generation of **FuSa Supervisors** from static safety analysis, ensuring compliance with these safety requirements. These supervisors can be formally verified using the **NuSMV 2.6.0** tool.

## Installation

Install with [PyPI](https://pypi.org/project/ft2bt/):

```bash
pip install ft2bt
echo 'export FT2BT_PATH=$(python3 -c "import ft2bt; import os; print(os.path.dirname(ft2bt.__file__))")' >> ~/.bashrc
source ~/.bashrc
```

### Requirements

This project requires **NuSMV version 2.6.0** to be installed, only in the case that the formal verification is required. Follow the steps below to install it:

- Go to the [NuSMV Downloads Page](https://nusmv.fbk.eu/downloads.html).
- Download the **NuSMV 2.6.0** archive file for your operating system.

After downloading, extract the contents of the archive file:

```bash
tar -xvzf NuSMV-2.6.0.tar.gz
cd NuSMV-2.6.0
./configure
make
sudo make install
NuSMV -version
```

You can refer to the [NuSMV Documentation](https://nusmv.fbk.eu/userman/v26/nusmv.pdf) for more details.

The repository has been proven in an Ubuntu 20.04 environment.

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
  <img src="https://raw.githubusercontent.com/cconejob/ft2bt_converter/master/ft2bt/test/fault_trees/fta_example.png" alt="Fault Tree Example">
</p>

**Warning!**: All fault tree elements, with the exception of text probabilities, should be connected by directional arrows. Ensure that each arrow is physically attached to its corresponding elements to maintain clarity and accuracy in the diagram.

### Preparing Your Hazard Analysis and Risk Assessment (Optional)

Create a *.csv file with some required column names:

1. **Item_ID**: Identificator of the Item analyzed.
2. **Hazard_ID**:  Identificator of the possible Hazard. The ID must match with the name of the correspondent Hazard in the Fault Tree.
3. **Operating_Scenario_ID**: Identificator of the Operating Scenario.
4. **ASIL**: Automotive Safety Integrity Level. Options: A, B, C, D.
5. **Safety_Goal_ID**: Identificator of the Safety Goal.
5. **Safety_State_ID**: Identificator of the Safety State action.

<p align="center">
  <img src="https://raw.githubusercontent.com/cconejob/ft2bt_converter/master/ft2bt/test/hara/hara_example.png" alt="HARA Example">
</p>

### Running the Conversion Tool

Run the conversion command:

```bash
ft2bt -f FTA_FILEPATH [-v] [-c] [-r] [-o OUTPUT_FOLDER] [-p] [-H HARA_FILEPATH] [-os] [-ctl]
```

Where:

- **-f**: (Required, str) Specifies the XML global filepath name of the draw.io diagram.
- **-v**: (Optional, bool) Automatically shows and saves the renders. Defaults to False.
- **-c**: (Optional, bool) Generate a cpp ROS node template for the behavior tree. Defaults to False.
- **-r**: (Optional, bool) Replaces current code if previously generated and -c is set to True.
- **-o**: (Optional, str) Specifies the global folder path, where the behavior tree XML diagram is saved.
- **-p**: (Optional, bool) Probabilities are considered to sort the behavior tree nodes. Defaults to False.
- **-H**: (Optional, str) Specifies the CSV global file name of the Hazard Analysis and Risk Assessment (HARA).
- **-os**: (Optional, bool) Generate a FuSa BT that includes events to check the Operating Scenario. Defaults to False
- **-ctl**: (Optional, bool) Formally verify the BT FuSa supervisor with CTL formulation. Defaults to False

### Output Example: Behavior Tree Diagram

Below is an example of the behavior tree diagrams generated from the fault tree and HARA examples. The command used for the generation is:

```bash
ft2bt -os -p -ctl -f $FT2BT_PATH/test/fault_trees/fta_example.xml -H $FT2BT_PATH/test/hara/hara_example.csv -o $FT2BT_PATH/test/behavior_trees
```

The XML file is loaded using [Groot](https://github.com/BehaviorTree/Groot):

The order of the events is sorted by probability of occurrence (**-p** oprion).

<p align="center">
  <img src="https://raw.githubusercontent.com/cconejob/ft2bt_converter/master/ft2bt/test/behavior_trees/render/BT_hz_01.svg" alt="Behavior Tree Conversion Example"> <!-- or you can set the height instead -->
</p>

## Related Research

- Behavior Trees for the Application of ISO 26262 in Field Monitoring Processes for Autonomous Vehicles (Conference Article, IEEE ITSC 2024)
- Behavior Trees in Functional Safety Supervisors for Autonomous Vehicles ([Preprint Article](https://arxiv.org/abs/2410.02469), IEEE ITS)

## Contact Information and Acknowledgement

For further information regarding this project, please feel free to reach out to Carlos Conejo [carlos.conejo@upc.edu](mailto:carlos.conejo@upc.edu).

This project was mainly developed at the [Institut de Robòtica i Informàtica Industrial (IRI)](https://www.iri.upc.edu/), a joint university research center of the Polytechnic University of Catalonia (UPC) and the Spanish National Research Council (CSIC). The automatized formal verification process was developed in collaboration with the  [Cyber Physical Systems Group (TUM)](https://www.ce.cit.tum.de/cps/home/).

Research partially funded by the Spanish State Research Agency (AEI) and the European Regional Development Fund (ERFD) through the SaCoAV project (ref. PID2020-114244RB-I00). Also funded by Renault Group through the Industrial Doctorate "Safety of Autonomous Vehicles" (ref. C12507).
