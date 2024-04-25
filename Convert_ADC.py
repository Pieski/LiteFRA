import sys
import numpy as np

if __name__ == '__main__':
    allText = open(sys.argv[1]).read()
    shortCount = int(len(allText)/6)
    shorts = np.zeros(shortCount)

    for shortIndex in range(0,shortCount):
        shortString = allText[shortIndex*6 : shortIndex*6+6]
        shorts[shortIndex] = int(shortString[3:5]+shortString[0:2], 16)
        shorts[shortIndex] /= 4095
        shorts[shortIndex] *= 3.3
        print(shorts[shortIndex])

    with open(sys.argv[2],'w') as csvFile:
        for short in shorts:
            csvFile.write(str(short)+'\n')
