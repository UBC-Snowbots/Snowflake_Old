# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|

  config.vm.box = "ubuntu/trusty32"
  config.ssh.forward_x11 = true
  config.vm.provision "chef_solo" do |chef|
    chef.roles_path = 'roles'
    chef.add_role 'vagrant'
  end

  config.vm.provider "virtualbox" do |v|
    v.memory = 1024
  end


end

