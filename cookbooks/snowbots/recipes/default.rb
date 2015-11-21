#
# Cookbook Name:: snowbots
# Recipe:: default
#
# Copyright (C) 2015 Snowbots
#
# All rights reserved - Do Not Redistribute
#

snowbots_workspace = node['snowbots']['workspace']
user_name = node['snowbots']['user']
should_make_source_link = node['snowbots']['vagrant_src_link']

ros 'indigo' do
  config 'desktop'
  action [:install, :upgrade]
end

# link to src folder synchronised from the host machine - Vagrant only
link File.join(snowbots_workspace, 'src') do
  to '/vagrant/src'
  only_if { should_make_source_link }
end

catkin 'snowbots' do
  user user_name
  release 'indigo'
  workspace snowbots_workspace
end

apt_package 'gdb'

apt_package 'libgtk2.0-dev'
apt_package 'ros-indigo-hokuyo-node'
# apt_package 'vision_opencv'
apt_package 'x11-xserver-utils'
apt_package 'git'

#might install own version of opencv
#apt_package 'ros-indigo-opencv3'
apt_package 'ros-indigo-hokuyo-node'

catkin_package 'laser_geometry' do
  source_uri 'https://github.com/ros-perception/laser_geometry.git'
end

