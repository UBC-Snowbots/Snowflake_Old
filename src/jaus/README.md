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

### Set up (git and things)
1. In Git Bash set up Vagrant defaults (type in `vagrant up`, then type in `vagrant ssh`)
--you do the above for anything in Snowflake--

2. Go to the Jaus branch (type in `git checkout jaus`, only needs to be done once until you checkout another branch)
3. Pull (type in `git pull`)
--you only need to do steps 0.2 and 0.3 once--

### Things to do each time you start
1. Go to src/jaus (type in `cd src/jaus`)
2. sudo python3 setup.py develop --do step 2 only once--
3. Everything should now

### How to run the unit tests

1. Go to src/jaus in new window
2. Type `py.test`
3. Watch them pass (or fail)
