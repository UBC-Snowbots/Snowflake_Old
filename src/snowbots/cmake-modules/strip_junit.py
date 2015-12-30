"""
Strips the outermost tag from JUnit-style XML reports so ROS can understand them.

Usage:
	strip_junit.py -h
	strip_junit.py [(-i <input>)] [(-o <output>)]

Options:
	-h --help                       Show this message
	-i <input>, --input <input>     The file to read [default: -]
	-o <output>, --output <output>  The file to write to [default: -]
"""
import xml.etree.ElementTree as ElementTree
import docopt, sys

def main():
	arguments = docopt.docopt(__doc__)
	in_file = sys.stdin
	out_file = sys.stdout
	if arguments['-i']:
		in_file = open(arguments['<input>'], 'r')
	if arguments['-o']:
		out_file = open(arguments['<output>'], 'w')
	
	tree = ElementTree.ElementTree()
	tree.parse(in_file)
	root = tree.getroot()
	if len(root) != 1:
		raise RuntimeError('Wrong number of testSuites (%s) in input file' % len(root))
		
	tree._setroot(root[0])
	tree.write(out_file)

if __name__ == '__main__':
	main()
