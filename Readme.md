AADC 2016 - Team KACADU
===============================================
This is the entry of team KACADU (Karlsruhe Cognitive Autonomous Driving Unit) for the [Audi Autonomous Driving Cup](https://www.audi-autonomous-driving-cup.com/) 2016.

Team KACADU is the 2016 student team supported by the FZI Forschungszentrum Informatik. Besides this code basis a lot of additional material from other years as well as publications describing the code and supplementary simulation frameworks can be found on the official website:

**Please visit [Audi Autonomous Driving Cup at www.fzi.de](https://www.fzi.de/aadc/)**


License
-----------------------------------------------
Almost everything is licensed under BSD (see file [LICENSE](LICENSE)). There are some parts integrated
from oadrive that are licensed under CDDL. See there for more details. Copyright belongs partially to the FZI ([Forschungszentrum Informatik](https://www.fzi.de/startseite/)) or Audi (the
files are marked accordingly).


Structure:
-----------------------------------------------

oadrive:
	This folder holds the data processing and logic of 
        our entry. It can be run from within ADTF, or separately
	(for simulation and offline reconstruction).

aadcUser Folder:
	This is the aadcUser folder as given to us by Audi,
        with our modifications. Building this folder is only
        possible when oadrive has already been built (and installed).

ADTF_Project Folder:
	This folder holds the ADTF project files which are used
        to run the code using the adtf_devenv program.


Setup:
-----------------------------------------------

1. Follow the instructions in the *oadrive* subfolder to set up and install oadrive.
2. Follow the instructions in the *aadcUser* folder to build the filters.
3. Load the project from the *Project* folder, and run it.


Configuration:
-----------------------------------------------

The project needs various configuration files to run properly. Examples of these files
are provided, but to work properly, they need to be adjusted to your specific camera setup.
If you have any questions, don't hesitate to contact us.


Contact:
-----------------------------------------------
* kacadu@lists.kit.edu
* kuhnt@fzi.de
* [Audi Autonomous Driving Cup at FZI website](https://www.fzi.de/aadc/)
