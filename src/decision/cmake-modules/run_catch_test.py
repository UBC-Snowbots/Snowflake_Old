"""
Runs a CATCH test under ROSTest

Usage:
	run_catch_test.py -h
	run_catch_test.py --test <test> --post_proc <post_proc> --output <output>

Options:
	-h --help                Show this message
	--test <test>            Path to the test executable
	--post_proc <post_proc>  Path to a post-processing program
	--output <output>        XML file to output to
"""

import docopt, subprocess, sys

def main():
	arguments = docopt.docopt(__doc__)
	
	test = subprocess.Popen([arguments['<test>'], '-r', 'junit'], stdout = subprocess.PIPE)
	
	post_proc = subprocess.Popen([sys.executable, arguments['<post_proc>'], '-o', arguments['<output>']], stdin = test.stdout)
	
	post_proc.wait()
	
	sys.exit(test.returncode)

if __name__ == '__main__':
	main()
