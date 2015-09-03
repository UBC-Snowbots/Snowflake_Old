#
# Cookbook Name:: snowbots
# Recipe:: default
#
# Copyright (C) 2015 Snowbots
#
# All rights reserved - Do Not Redistribute
#

ros 'indigo' do
  config 'ros-base'
  action [:install, :upgrade]
end

snowbots_workspace = "/home/vagrant"

directory snowbots_workspace do
  owner 'vagrant'
end
link "#{snowbots_workspace}/src" do
  to '/vagrant/src'
end

catkin 'snowbots' do
  user 'vagrant'
  release 'indigo'
  workspace snowbots_workspace
end
