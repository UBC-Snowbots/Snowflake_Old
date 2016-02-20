Written by Maxim Tsai

Welcome to the Snowbots JAUS (Joint Architecture for Unmanned Systems) Branch.
This side project is for the IGVC competition.

Site competition rules: http://www.igvc.org/rules.htm (page 17)

Overall Goal: Make a program that takes in JAUS commands and responds correctly. JAUS commands will be in the form of python byte streams.

How to set up and run stuff:
0. In Git bash set up Vagrant defaults (type in "vagrant up", then type in "vagrant ssh");
0.5. Go to the Jaus branch (type in "git checkout jaus")
0.9. Pull (type in "git pull");

1. Go to src/jaus (type in "cd src/jaus");
2. Install Twisted, (type in "sudo python setup.py develop"); // https://twistedmatrix.com/trac/
3. Begin Twisted. (type in "test_twisted");
4. Open new git bash window (alt+f4);
5. Try out test message (Type in "curl http://localhost:8080");
6. You should receive a message "test"

This message comes from the file src/jaus/jaus/__init__.py

Will continue to update once I figure out how more stuff works.