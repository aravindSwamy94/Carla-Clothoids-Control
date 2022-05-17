tab="--tab"
cmd1="bash -c 'cd PythonAPI ; cd util; py35 ;'bash"

cmd2="bash -c 'cd PythonAPI ; cd examples; py35 ;'bash"

cmd3="bash -c 'cd PythonAPI ; cd Motion_planning_Framework/Scenarios; py35 ;'bash"

cmd4="bash -c 'cd PythonAPI ; cd Motion_planning_Framework/framework_fns; py35 ;'bash"

gnome-terminal $tab -e "$cmd1" 

gnome-terminal $tab -e "$cmd2" 

gnome-terminal $tab -e "$cmd3" 

gnome-terminal $tab -e "$cmd4" 

exit 0
