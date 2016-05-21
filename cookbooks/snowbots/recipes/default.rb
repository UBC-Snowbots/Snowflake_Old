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

#group 'dialout' do
#  members [user_name]
#  append true
#end

# link to src folder synchronised from the host machine - Vagrant only
link File.join(snowbots_workspace, 'src') do
  to '/vagrant/src'
  only_if { should_make_source_link }
end

apt_package 'gdb'
apt_package "python-pip"

apt_package 'libgtk2.0-dev'
apt_package 'x11-xserver-utils'

apt_package 'libarmadillo-dev'

apt_package 'ros-indigo-vision-opencv'
apt_package 'ros-indigo-hokuyo-node'
apt_package 'ros-indigo-rosbash'
apt_package 'ros-indigo-scan-tools'
apt_package 'ros-indigo-gmapping'
apt_package 'ros-indigo-sicktoolbox-wrapper'
apt_package 'ros-indigo-image-pipeline'
apt_package 'v4l-utils'
apt_package 'ros-indigo-cv-camera'
apt_package 'ros-indigo-tf2-bullet'


python_pip 'docopt'

catkin 'snowbots' do
  user user_name
  release 'indigo'
  workspace snowbots_workspace
end
