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
apt_package 'ros-indigo-opencv3'

