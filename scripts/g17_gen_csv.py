import subprocess
string=""
itern=500
rerun=50
outfile=open("data/g17_data_01.csv",'w')
print("GENERATING DATA.... Please wait")
for i in range(1,itern+1):
	for j in range(1,rerun+1):
		command='./mybins/cs296_17_exe '+str(i)+' >temp.txt'
		subprocess.call(command, shell=True)
		data=open("temp.txt",'r')
		count=1
		string=str(i)+','+str(j)
		for line in data:
			if count >= 2:
				words=line.split(" ")
				if len(words)>1:
					string+=','+words[-2]
			count+=1
		string+='\n'
		outfile.write(string)		
print("csv file g17_data_01.csv generated successfully")
subprocess.call("rm temp.txt", shell=True)

