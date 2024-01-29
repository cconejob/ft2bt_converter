# Fault Tree to Behavior Tree Converter

## Overview

This project focuses on the conversion of fault trees, represented in draw.io diagram XML files, into behavior tree XML files compatible with the BehaviorTree.CPP library. It enables users to transform their fault tree diagrams into actionable behavior trees, facilitating integration with systems that utilize the BehaviorTree.CPP framework for managing complex behaviors.

## Installation

1. Clone the repository to your local machine.

2. Ensure you have Python installed.

3. Install the required dependencies by running the following command in the project directory.

```bash
git clone git@github.com:cconejob/trees.git
cd trees
pip install -r requirements.txt
```

## Usage

The tool is designed to convert fault trees from draw.io diagram XML files into behavior tree XML files compatible with the BehaviorTree.CPP library. Here's how to use it:

### Preparing Your Fault Tree Diagram

1. **Create or Open Your Fault Tree Diagram in Draw.io**:
    * First, visit [draw.io](https://draw.io/) to create or edit your fault tree diagram. You may refer to their [documentation](https://www.drawio.com/doc/) for guidance on using the tool.
2. **Diagram Structure & Symbols**:
    * **Hazards**: Represent hazards using rectangles. This is a required element in your diagram.
    * **Events**: Depict events using circles. These are also required elements.
    * **Actions**: Actions can be represented by processes. Including actions is optional but helpful to clarify how the hazard should be handled.
    * **AND/OR Gates**: Use the respective symbols for AND/OR gates in your diagram. These are required for depicting logical relationships in the fault tree.
3. **Exporting the Diagram as XML**:
    * Once your fault tree diagram is ready, you need to export it in XML format. In draw.io, go to `File` > `Export as` > `XML` to save your diagram as an XML file.
4. **Save the XML File in the `/fault_trees` Folder**:
    * After exporting your diagram as an XML file, save it in the `/fault_trees` folder within the project directory.

**Warning!**: All the fault tree elements need to be connected by directional arrows. Ensure that all of them are physically attached to their related elements.

### Input Example: Fault Tree Diagram

Below is an example of two fault tree diagrams generated in the same Draw.io file:

![Fault Tree Example](fault_trees/example_3.png)

### Running the Conversion Tool

1. Ensure you have the required XML file from draw.io in the /fault_trees folder.
2. Open a terminal or command prompt and navigate to the project directory.
3. Run the conversion command:

    ```bash
    python3 scripts/ft2bt.py [-h] -f FTA_FILENAME [-r] [-v] [-c]
    ```

Where:

* **-f**: (Required) Specifies the XML filename of the draw.io diagram.
* **-r**: (Optional) Enables renderization of the output. Defaults to false.
* **-v**: (Optional) Automatically shows the renders if renderization (**-r**) is true. Defaults to false.
* **-c**: (Optional) Generate a cpp ROS node template for the behavior tree. Defaults to false.

### Output Example: Behavior Tree Diagram

Below is an example of the behavior tree diagrams generated from the two fault trees. The XML file is loaded using [Groot](https://github.com/BehaviorTree/Groot):

The first behavior tree example represents the monitorisation of the control module (AVCM).
![Behavior Tree Example 1](behavior_trees/render/BT_avcm.png)

The second behavior tree represents the monitorisation of the localisation module (AVLS).
![Behavior Tree Example 2](behavior_trees/render/BT_avls.png)

## Contact Information and Acknowledgement

For further information regarding this project, please feel free to reach out to Carlos Conejo.

This project was developed at the [Institut de Robòtica i Informàtica Industrial (IRI)](https://www.iri.upc.edu/), a joint university research center of the Polytechnic University of Catalonia (UPC) and the Spanish National Research Council (CSIC).

Research partially funded by the Spanish State Research Agency (AEI) and the European Regional Development Fund (ERFD) through the SaCoAV project (ref. PID2020-114244RB-I00). Also funded by Renault Group through the Industrial Doctorate "Safety of Autonomous Vehicles" (ref. C12507).
