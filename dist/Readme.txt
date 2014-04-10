Introduction
============

This is the installed version of a project developed in CS296 course under prof Parag Choudhary , by a group of 3 students from 2nd Year btech in Computer Science. This contains the executable version of cheetah simulation , and also the project report and the documented code fo the project


------------------------------------------------------------------------------------
Team members - 
1. Naveen Sagar 		120050026
2. Prateek Chandan		120050042
3. Aditya Kumar Akash 	120050046

-------------------------------------------------------------------------------------


-------------------------------------------------------------------------------------
The project shows the running simulation of cheetah (a robo representing it) in Box2D.
The simulation as default shows the cheetah running leftwards .
Keyboard controls have been incorporated to make the simulation interactive .
The controls are - 
1. 'a'/'A' - Move cheetah leftwards
2. 'd'/'D' - Move cheetah rightwards
3. 's'/'S' - Make the cheetah still
4. 'w'/'W' - Make the cheetah jump

Other controls are same as provided by the box2d .

--------------------------------------------------------------------------------------
The project folder contains a Makefile which does various operations as - 

1. make	-> gives you instruction of project
					
2. make report	-> The plots are made if not present already and the html as well as
					latex report are made in this section.

3. make data 	-> This generates the data in csv file format

5. make plot	-> This makes all the plots from the data which was generated

6. make clean	-> Removes all the compiled files , execuatble , data , plots
					and the documentation


---------------------------------------------------------------------------------------

CITATIONS - 

The project is made solely by the group and constains no work of plagiarism.
However the following sources have been helpful in making of project -

1. We have borrowed the dimensions for configurations for the crank system used and 	
	reused a little code from the TheoJansen.h code provided in the box2d sample codes 
	int TestBed provided in the box2d external folder . 
	
	This was done because running of the TheoJansen is dependent the dimensions 
	of the basic structure of crank system. The data provided in the TheoJansen.h
	was helpful in this sense.
		
2. The other websites that were helpful in making the project are - 
	1. stackoverflow
	2. online documentation of box2d - http://box2d.org/documentation.html
	3. https://www.iforce2d.net/b2dtut/
	4. google
		
-------------------------------------------------------------------------------------------
