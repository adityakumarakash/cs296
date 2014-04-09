import numpy as ny
import matplotlib.pyplot as plt
import math
import decimal
itern=500
rerun=50

data=open("data/g17_data_01.csv")
x=range(1,itern+1)

print("GENERATING PLOTS..")
def bin(x,s):
	rep=s*(x//s)+(s/2.0)
	return round(rep,3)
	
step=[]
randstep=[]
sd_step=[]
lt=[]
colt=[]
velt=[]
post=[]
tott=[]
alldata=[]

bin_width=0.01
processed_data=[]

for i in range(1,itern+1):
	sttime=0
	looptime=0
	coltime=0
	veltime=0
	postime=0
	sttime2=0
	rand=[]
	
	for j in range(1,rerun+1):
		line=data.readline().split(",")
		alldata.append(line)
		sttime+=float(line[2])
		rand.append(float(line[2]))
		sttime2+=float(line[2])*float(line[2])
		coltime+=float(line[3])
		veltime+=float(line[4])
		postime+=float(line[5])
		looptime+=float(line[6])/i
		
		if i==46:
			processed_data.append(bin(float(line[2]),bin_width))
		
		
	ny.random.shuffle(rand)
	randstep.append(sum(rand[0:15])/15)
	sd_step.append(math.sqrt((sttime2-((sttime)*(sttime/rerun)))/rerun))
	step.append(sttime/rerun)
	lt.append(looptime/rerun)
	colt.append(coltime/rerun)
	velt.append(veltime/rerun)
	post.append(postime/rerun)
	tott.append((coltime+veltime+postime)/rerun)
 
# plot 1 generation 
maxlt=max(lt)
minlt=min(lt)
maxt=[maxlt for it in lt]
mint=[minlt for it in lt]
plt.plot(x,lt,label="Step time")
plt.plot(x,maxt,color='r',label="maximum loop time")
plt.plot(x,mint,color='g',label="minimum loop time")
plt.bar(x,step,color='y',label="looptime")

plt.legend()
plt.ylabel('Time in milisecond')
plt.xlabel('Iteration no')
plt.title("Plot of Step time and Loop Time")	
plt.savefig('./plots/g17_plot01.png')
plt.clf()

 # plot 2 generation
plt.plot(x,step,label="step time")
plt.plot(x,colt,label="collision time")
plt.plot(x,velt,label="velocity update time")
plt.plot(x,post,label="position update time")
plt.plot(x,tott,label="total time")
plt.ylabel('Time in milisecond')
plt.xlabel('Iteration no')
plt.title("Plot of Step ,collision velocity & position update time")
plt.legend()
plt.savefig('./plots/g17_plot02.png')
plt.clf()

#plot 3 generation
plt.errorbar(x, step, sd_step,label="step time with errorbars")
plt.ylabel('Time in milisecond')
plt.xlabel('Iteration no')
plt.title("Plot of Step time with error bars")
plt.legend()
plt.savefig('./plots/g17_plot03.png')
plt.clf()


#plot 4 generation
minV=min(processed_data)
maxV=max(processed_data)

freq={}
cumul={}
xdat=[]
y1=[]
y2=[]

i=minV
while i<= maxV:
	freq[i]=0
	cumul[i]=0
	xdat.append(i)
	i=round(i+bin_width,3)

for i in processed_data:
	freq[i]+=1

cumul[minV]=freq[minV]
i=minV+bin_width

while i <= maxV:
	cumul[i]=cumul[round(i-bin_width,3)]+freq[i]
	i=round(i+bin_width,3)

for i in xdat:
	y1.append(freq[i])
	y2.append(cumul[i])

plt.hist(y1,bins=ny.arange(minV,maxV+bin_width,bin_width),label="frequency of step time")
plt.plot(xdat,y2,label="cumulative frequency")
plt.legend()
plt.ylabel('Frequency')
plt.xlabel('step time in ms')
plt.title("The Frequency and Cumulative Frequency of Step Time")
plt.savefig('./plots/g17_plot04.png')
plt.clf()

#plot 5 generatiom
bestfit_par=ny.polyfit(x, step, 1)
bestfit_st=[(bestfit_par[0]*it+bestfit_par[1]) for it in x]

plt.plot(x, step, marker='o',label="step time from all",ls='')
plt.plot(x,bestfit_st,label="best fit line for step time")

bestfit_par=ny.polyfit(x, randstep, 1)
bestfit_st=[(bestfit_par[0]*it+bestfit_par[1]) for it in x]
plt.plot(x, randstep, marker='+',label="step time random",ls='')
plt.plot(x,bestfit_st,label="best fit line for randomstep time")
plt.ylabel('Time in milisecond')
plt.xlabel('Iteration no')
plt.title("Plot of Step time from all data and random with best fit line")
plt.legend()
plt.savefig('./plots/g17_plot05.png')
plt.clf()

print("ALL PLOTS GENERATED in plots folder..")
