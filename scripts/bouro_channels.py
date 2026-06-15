#!/usr/bin/python3

# This is intended to print a list of the frequencies
# according to their channel numbers as presently displayed
# in Bouro. Probably needs more thorough testing against Bouro.

basefreq = 5600		# Out of band!

def f4chan(basefreq:int, x:int) -> int:
	if x < 0:
		print("No negative channel numbers!")
	elif x < 32:
		return basefreq - (x+1)*0.15625
	elif x == 32:
		print("32 is not a valid channel")
	elif x < 64:
		return basefreq + (63-x)*0.15625
	else:
		print("No channel numbers greater than 63!")

if __name__ == "__main__":
	for chan in range(0,32):
		print(chan, f4chan(basefreq, chan))
	for chan in range(33,64):
		print(chan, f4chan(basefreq, chan))