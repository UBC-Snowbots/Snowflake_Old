Written by Maxim Tsai

# Welcome to the Snowbots JAUS (Joint Architecture for Unmanned Systems) Branch.
This side project is for the IGVC competition.

Site competition rules: http://www.igvc.org/rules.htm (page 17)

## Overall Goal:

Make a program that takes in JAUS commands and responds correctly.
JAUS commands will be in the form of python byte streams received via UDP.

## Terminology
JAUS - Joint architecture for unmanned systems. JAUS is a type of language/specification used for sending messages to robots.
Twisted - Event driven networking engine written in Python. Makes it easy to implement custom network applications.
SDP - Software Defined Protocol
JUDP - JAUS over UDP. Standard of the transmission of JAUS messages over UDP communications links.
UDP - User Datagram Protocol. Unreliable best-effort connectionless message delivery protocol.

## How to set up and run stuff

0. In Git bash set up Vagrant defaults (type in `vagrant up`, then type in `vagrant ssh`);
0.5. Go to the Jaus branch (type in `git checkout jaus`, only needs to be done once until you checkout another branch)
0.9. Pull (type in `git pull`);
1. Go to src/jaus (type in `cd src/jaus`);
2. Install Twisted, (type in `sudo python setup.py develop`); // https://twistedmatrix.com/trac/
3. Begin Twisted server. (type in `test_twisted`);
4. Open new git bash window in src/jaus;
5. In new git bash window, log into vagrant (type in 'vagrant ssh' in new window);
6. Try out test message (Type in `curl http://localhost:8080`);
7. You should receive a message "test". Message does not contain any newlines so it may show up as "testvagrant@vagrant-ubuntuss-trusty-32"

This message comes from the file src/jaus/jaus/__init__.py

## How to run the unit tests

1. Go to src/jaus
2. Type `py.test`
3. Watch them pass (or fail)
