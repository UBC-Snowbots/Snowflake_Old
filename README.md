# Snowflake

A basic (work in progress) rebuild of our previous IGVC repository, with better dependency management, and a built-in Vagrant setup.

Current contents: one tutorial package containing a solution to the python fizzbuzzprime with ROS integration. CPP version coming soon.

To Snowbots members: any and all feedback on how to vagrant-up and build this repository is welcome. Please include any observations under Troubleshooting, or if there is an error in the Chef or Vagrant setup, you are welcome to try and commit a fix to that too.

How to build this repository AFTER you've cloned this repository
----------------------------

### Without Vagrant ###

Make the repository into a catkin workspace, and then run catkin_make. (requires ROS Indigo to be installed)

`rosdep` should take care of any ROS-type dependencies that we don't include.

### With Vagrant ###
You will need to install the following before continuing on:

[VirtualBox 4.3](http://www.virtualbox.org/wiki/Download_Old_Builds_4_3) *VirtualBox 5.0 is unfortunity not supported by vagrant at the moment.

[Vagrant](http://www.vagrantup.com/downloads)

1) Change directory using the command `cd` and goto the Snowflake repository 

Ex: If you've cloned your repository onto your desktop you can type `cd Desktop/Snowflake`

2) Once in the repository type `vagrant up` this will start downloading a vagrant box.

`The download can take several minutes depending on your internet connection. Be aware that the download can automatically disconnect (you can tell this if the rate is 0/s). If this happens stop the current download by pressing CTRL C, then resume the download again by either pressing the UP and ENTER.`

Once the download is complete it will automatically install itself; this procress and take around 10+ minutes. You'll know that the installation is done once your `$` prompt comes back on the terminal screen.

3) Type `vagrant ssh`, then you will be inside a fully set up catkin workspace. 

4) Type `catkin_make` to build the repository (note: it builds on start-up, so no need to build again when you've just loaded the VM), and all of the ROS command-line works as well.

Once you have done your testing, keep editing on your host system. Your source folder is linked to the Vagrant workspace, so all your changes will appear automatically inside the Vagrant VM.

Troubleshooting
---------------

Any observations/problems people encounter when trying to build/run go here.

Modifications to the set-up files
---------------------------------

For package management and ROS setup, go to `cookbooks/snowbots/recipes/default.rb`. For more information on how this works, please consult [the Chef docs](http://docs.chef.io/release/12-4/#getting-started).

For VM-related things, take a look at `Vagrantfile`, and the corresponding documentation at [the Vagrant website](https://docs.vagrantup.com/v2/).

If you need a new Chef module, take a look at [Berkshelf](http://berkshelf.com/). It makes sure all your dependencies work, and was used to put the current set-up together.
