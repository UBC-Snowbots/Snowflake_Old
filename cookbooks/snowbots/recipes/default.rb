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

snowbots_workspace = node['snowbots']['workspace']

# link to src folder synchronised from the host machine - Vagrant only
link File.join(snowbots_workspace, 'src') do
  to '/vagrant/src'
  only_if { node['snowbots']['vagrant_src_link'] }
end

catkin 'snowbots' do
  user 'vagrant'
  release 'indigo'
  workspace snowbots_workspace
end
