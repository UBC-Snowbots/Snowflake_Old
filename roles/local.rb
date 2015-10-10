name "local"
description "Custom settings to set up a local workspace."

override_attributes(
  :snowbots => {
    :vagrant_src_link => false,
  }
)

run_list "recipe[snowbots]"

