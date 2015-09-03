# Snowflake

A basic (work in progress) rebuild of our previous IGVC repository, with better dependency management, and a build-in Vagrant setup.

How to build this repository
----------------------------

### Without Vagrant ###

Make the repository into a catkin workspace, and then run catkin_make. (requires ROS Indigo to be installed)

`rosdep` should take care of any ROS-type dependencies that we don't include.

### With Vagrant ###

Get Vagrant and the corresponding Vagrant Box `ubuntu/trusty32`, and then go to the repository and type `vagrant up`. Once this has completed, type `vagrant ssh`, then you will be inside a fully set up catkin workspace. Typing `catkin_make` will build the repository (note: it builds on start-up, so no need to build again when you've just loaded the VM), and all of the ROS command-line works as well.

Once you have done your testing, keep editing on your host system. Your source folder is linked to the Vagrant workspace, so all your changes are will appear automatically inside the Vagrant VM.

Modifications to the set-up files
---------------------------------

For package management and ROS setup, go to `cookbooks/snowbots/recipes/default.rb`. For more information on how this works, please consult [the Chef docs](http://docs.chef.io/release/12-4/#getting-started).

For VM-related things, take a look at `Vagrantfile`, and the corresponding documentation at [the Vagrant website](https://docs.vagrantup.com/v2/).

If you need a new Chef module, take a look at [Berkshelf](http://berkshelf.com/). It makes sure all your dependencies work, and was used to put the current set-up together.
