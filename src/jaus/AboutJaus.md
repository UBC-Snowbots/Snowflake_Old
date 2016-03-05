Written by Maxim Tsai

## Terminology
JAUS - Joint architecture for unmanned systems. JAUS is a type of language/specification used for sending messages to robots.
Twisted - Event driven networking engine written in Python. Makes it easy to implement custom network applications.
SDP - Software Defined Protocol
JUDP - JAUS over UDP. Standard of the transmission of JAUS messages over UDP communications links.
UDP - User Datagram Protocol. Unreliable best-effort connectionless message delivery protocol.

# JAUS details
Part 1/3: https://slack-files.com/files-pri-safe/T04M11YQ1-F0NAAEX18/as5669a.pdf?c=1457203892-9225a8d9baa28f13a1a8c1eebca064b4acb5c831
Important stuff:
- We are only interested in JUDP
- Messages come in a specific format described on page 10.
- Finn already finished parsing message format. They are now in a class called GeneralTransportHeaderStructure.
