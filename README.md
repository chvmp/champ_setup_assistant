# CHAMP Setup Assistant
This software auto generates a configuration package containing all the [files](https://github.com/chvmp/champ_setup_assistant/tree/master/templates) necessary to make CHAMP walk. 

![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/setup.png)
## Installation

1. Clone and install all dependencies:

        sudo apt install -y python-rosdep
        cd <your_ws>/src
        git clone https://github.com/chvmp/champ_setup_assistant
        cd ..
        rosdep install --from-paths src --ignore-src -r -y

2.  Build your workspace:

        cd <your_ws>
        catkin_make
        source <your_ws/>/devel/setup.bash

## Quick Start

1. Run the setup assistant:

        roslaunch champ_setup_assistant setup_assistant.launch 

Configure your robot by loading a URDF or manually key in the configuration parameters. If you don't have any URDF now, you can download [ANYmal](https://github.com/chvmp/anymal_b_simple_description/tree/master/urdf) or [SpotMicro](https://github.com/chvmp/spotmicro_description/blob/master/urdf/spotmicroai.urdf) just to try the package.

## Configuration

You can either use a URDF file or manually key in the origin of each actuator in the robot.

1. Using a URDF file:

   1.1 Load the URDF file:
   
   - Click the 'BROWSE URDF' button on the upper right corner and click the URDF file you want to use from the file browser. Double click the file or click 'Open' to load the URDF.

   - Click the 'LOAD' button. This will list down the links of the robot on the left pane and display the robot's meshes on the RVIZ widget. Take note that it may take a while for huge mesh files to load.
   
        ![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/load_urdf.gif)


   1.2 Select the namespace for each leg (auto-configuration):
   
   - Select the correct namespace for each leg on the drop-down menu. 

        ![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/select_namespace.gif)


        This namespace is the unique identifier used by the URDF's author to differentiate each leg. For instance, Anymal's hip links are named as LF_HIP (front left), LH_HIP (rear left), RF_HIP (right front), RH_HIP (rear right). The assistant will parse the namespaces as LF_, LH_, RF_, and RH_. After selecting the namespaces for all the legs, the assistant will automatically drag the links to its respective leg parts, emptying the left pane.

   1.3 Manually adding links (if auto-configuration fails):

   - You can also add a link to each leg part manually by clicking on a leg tab (ie 'Left Front Leg'). Select a link on the left pane and click '>' to add. This would be useful if the assistant fails to parse the namespaces.
   
        ![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/manual_urdf.gif)


   1.4 Verify the configuration:

   - To visualize a configured link for a leg part, click on one of the leg tabs (ie 'Left Front Leg') and select the part. The link should now be highlighted on the RVIZ widget. Check all the leg tabs if the configuration is correct.

        ![CHAMP Setup Assistant](https://raw.githubusercontent.com/chvmp/champ_setup_assistant/master/docs/images/visualize_links.gif)

