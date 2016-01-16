# Snowflake

A basic (work in progress) rebuild of our previous IGVC repository, with better dependency management, and a built-in Vagrant setup.

To Snowbots members: any and all feedback on how to vagrant-up and build this repository is welcome. Please include any observations under Troubleshooting, or if there is an error in the Chef or Vagrant setup, you are welcome to try and commit a fix to that too.

Before continuing onwards
----------------------------
Make sure you've cloned with repository onto your system. For convience you can clone it onto your desktop by opening up a terminal (or [Git Bash](https://git-scm.com/downloads)) and paste the following:

`git clone https://github.com/UBC-Snowbots/Snowflake.git Desktop/Snowflake`

Building the repository AFTER you've cloned it
----------------------------

### Without Vagrant ###

This is a more involved process, so unless you need it using Vagrant is recommended. As of writing, this requires Ubuntu or something close. Try other systems at your own risk.

First, install [Chef](https://www.chef.io/chef/), usually by typing `sudo apt-get install chef`. Then, make sure you have ruby (type `ruby --version` - it should say 1.9.3 or something similar). If not, `sudo apt-get install ruby` should do it.

The next step should set everything else up: `./local_setup`. Enter your password when prompted, and once it completes everything should be in the right places. When we release updates to the build (like adding new libraries such as OpenCV, or fixing bugs), you can safely re-run this script in order to painlessly install those updates.

Note: do not run `sudo ./local_setup`. It will transfer ownership of your catkin workspace to root.

### With Vagrant ###
You will need to install the following before continuing on:

[VirtualBox 4.3](http://www.virtualbox.org/wiki/Download_Old_Builds_4_3) *VirtualBox 5.0 is unfortunity not supported by vagrant at the moment.

[Vagrant](http://www.vagrantup.com/downloads) *Installing this will require you to restart afterwards

1) Change directory using the command `cd` and goto the Snowflake repository 

Ex: If you've cloned your repository onto your desktop you can type `cd Desktop/Snowflake`

2) Once in the repository type `vagrant up` this will start downloading a vagrant box.

`The download can take several minutes depending on your internet connection. Be aware that the download can automatically disconnect (you can tell this if the rate is 0/s). If this happens stop the current download by pressing CTRL C, then resume the download again by pressing the UP and ENTER or retyping the whole command again.`

Once the download is complete it will automatically install itself; this procress and take around 10+ minutes. You'll know that the installation is done once your `$` prompt comes back on the terminal screen.

3) Type `vagrant ssh`, then you will be inside a fully set up catkin workspace. 

If you're in the virtual machine your terminal prompt will be: `vagrant@vagrant-ubuntu-trusty-32:~$`

4) Type `catkin_make` to build the repository (note: it builds on start-up, so no need to build again when you've just loaded the VM), and all of the ROS command-line works as well.

Once you have done your testing, keep editing on your host system. Your source folder is linked to the Vagrant workspace, so all your changes will appear automatically inside the Vagrant VM.

**Concluding remarks**

You can exit your virtual machine with the `exit` command.

In order to get back in again: `vagrant up` to turn on the virtual machine and `vagrant ssh` to login.

If you leave your computer on (haven't shutdown/logout) you'll only have to `vagrant up` once.

Troubleshooting
---------------

Any observations/problems people encounter when trying to build/run go here.

Modifications to the set-up files
---------------------------------

For package management and ROS setup, go to `cookbooks/snowbots/recipes/default.rb`. For more information on how this works, please consult [the Chef docs](http://docs.chef.io/release/12-4/#getting-started).

For VM-related things, take a look at `Vagrantfile`, and the corresponding documentation at [the Vagrant website](https://docs.vagrantup.com/v2/).

If you need a new Chef module, take a look at [Berkshelf](http://berkshelf.com/). It makes sure all your dependencies work, and was used to put the current set-up together.