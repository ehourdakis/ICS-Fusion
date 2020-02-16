from random import sample
import sys
k = int(sys.argv[2])


keypoints_dir = "./data/keypts/"
descr_dir = "./data/descr/"

infile = keypoints_dir +"f_" + sys.argv[1] + "_keypoints.txt"
outfile = keypoints_dir +"f_" + sys.argv[1] + "_keypoints2.txt"

inDescr = descr_dir +"frame" + sys.argv[1] + ".csv"
outDescr = descr_dir +"frame" + sys.argv[1] + "_2.csv"

f = open(infile, "r",errors='replace')
oldKeypts = f.readlines()
f.close()

f = open(inDescr, "r",errors='replace')
odlDescr = f.readlines()
f.close()


size=len(oldKeypts)
print( "Choosing %d keypoints from %d vertexes." % (k, size ) )

xlist = range(size)
r=sample(xlist,k)

f = open(outfile,"w")
descrFile = open(outDescr, "w")
for i in r:
    f.write( oldKeypts[i] + "\n" )
    descrFile.write( odlDescr[i] + "\n" )
f.close() 
descrFile.close()