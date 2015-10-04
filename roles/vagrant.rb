name "vagrant"
description "Custom settings to set up a workspace on Vagrant."
run_list "recipe[snowbots]"

override_attributes(
  :snowbots => {
    :vagrant_src_link => true,
    :workspace => "/home/vagrant"
  }
)
