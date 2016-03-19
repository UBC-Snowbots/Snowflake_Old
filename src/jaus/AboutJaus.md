Written by Maxim Tsai
Part 1/3: https://slack-files.com/files-pri-safe/T04M11YQ1-F0NAAEX18/as5669a.pdf?c=1457203892-9225a8d9baa28f13a1a8c1eebca064b4acb5c831
Part 2/3: https://slack-files.com/files-pri-safe/T04M11YQ1-F0TV883MJ/as5710a.pdf?c=1458413840-4427803db8972f68be9e8f99e6c7b3ad716b6613
Part 3/3: https://slack-files.com/files-pri-safe/T04M11YQ1-F0U03C8H3/as6009.pdf?c=1458413760-684a4fe2fcf244bdd055ddc7f1261ecd6d1ed56f

## Tasks remaining
- Implement parsing of JUDP packets (info on page 30, 32, 34...). Look at how GeneralTransportHeaderStructure is implemented for an idea of what to do. 

## Terminology
JAUS - Joint architecture for unmanned systems. JAUS is a type of language/specification used for sending messages to robots.
Twisted - Event driven networking engine written in Python. Makes it easy to implement custom network applications.
SDP - Software Defined Protocol
JUDP - JAUS over UDP. Standard of the transmission of JAUS messages over UDP communications links.
UDP - User Datagram Protocol. Unreliable best-effort connectionless message delivery protocol.

# JAUS details
Important stuff:
- We are only interested in JUDP
- Messages come in a specific format described on page 10. Each message contains the following:
	1. (6 bits) message_type - Type of message that is sent. 
	2. (2 bits) HC_flags - Header Compression flags.
	3. (16 bits) data_size - Size of entire message.
	4. (8 bits) HC_number - Header Compression Header Number field. Enumerates headers.
	5. (8 bits) HC_length - Header Compression Length field. 
	6. (2 bits) priority - low/standard/high/safety critical priority.
	7. (2 bits) broadcast - Broadcast flags used to mark messages intended for multiple destinations.
	8. (2 bits) ack_nack - Acknowledge/Negative Acknowledge behavior
	9. (2 bits) data_flags - If dataset larger than max packet size, parses large datasets into smaller.
	10. (32 bits) destination_id - Globally unique identifier of target of message. Little Endian format.
	11. (32 bits) source_id - Globally unique identifier of source of message. Little-Endian format. 
	... Following parts are not part of the header ...
	12. (N bits) payload - content of the message.
	13. (16 bit) sequence number - Increments for every message from a given source ID.
- Finn already finished parsing message format. They are now in a class called GeneralTransportHeaderStructure inside jaus/jaus/general_transport_header.py.
- Header compression, not too useful.
