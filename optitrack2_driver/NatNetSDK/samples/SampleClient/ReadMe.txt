Running the SampleClient on Linux
========================================

This Readme file contains first time instructions for building and running
the NatNetSDK for Linux.

Note: 

  * The NatNetSDK for Linux will only work on 64 bit operating systems.
  * For up-to-date information, visit:
	http://wiki.optitrack.com/index.php?title=NatNet:_Sample_Projects


1.[Linux] Install necessary programs and libraries.
In order to build and run the SampleClient application on Linux, there are 
two essential requirements. The gcc/g++ compiler must be installed on the 
machine.

  Ubuntu Terminal Instructions
     - sudo apt-get install build-essential
					
  Fedora Terminal Instructions
     - sudo dnf install gcc-c++



2.[Linux] Navigate to the NatNetSDK directory.
Open a shell prompt and set the directory to the samples/SampleClient folder
in the uncompressed NatNet SDK directory.


3.[Linux] Build the sample. While in the SampleClient directory, enter make 
clean all and compile the sample.

     - make clean
     - make -f makefile

4.[Linux] Once the sample is built, navigate to the build output folder.


5.[Linux] Set an environment variable for the library path.
In order to run compiled NatNetSDK samples, the directory of the NatNet library
(libNatNetLibShared.so)	must be specified. To do this, set up an environment
variable for defining the path to the library file directory:

   - export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{insert lib folder directory here}


6.[Motive] Start an Optitrack server (Motive).
Motive needs to run on a Windows machine and the data needs to be streamed 
through the connected network using the Data Streaming pane. Take a note of 
the server IP address from the Data Streaming pane in Motive. Make sure to 
stream onto a network that the Linux machine is connected to; not local 
loopback nor the camera network.


8.[Linux] Start SampleClient. 
Now, Start the client application from shell. 
Once the application starts, it will search the networks and list out available
tracking servers. From the list, select the Optitrack server from the above step 
to start receiving tracking data. If you built it to be a (.exe) file instead of 
the default file without a file type then run that command instead. 

  - ./SampleClient