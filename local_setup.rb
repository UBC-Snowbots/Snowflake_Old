# evil hack to make Chef use the current directory
File.open('local_config.json', 'w') do |file|
  JSON.dump({
    'run_list' => ['role[local]'],
    'snowbots' => {
      'workspace' => Dir.pwd
    }
  }, file)
end

json_attribs File.join(Dir.pwd, 'local_config.json')
role_path File.join(Dir.pwd, 'roles')
cookbook_path File.join(Dir.pwd, 'cookbooks')
