		README for Project cs296 lab
		
------------------------------------------------------------------------------------
Team members - 
1. Naveen Sagar 		120050026
2. Prateek Chandan		120050042
3. Aditya Kumar Akash 	120050046

-------------------------------------------------------------------------------------

The procect is made as a part of cs296 lab .

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

1. make			-> Installs the box2d project if it is not, and makes exe in mybins folder 
					to run the simulations
					
2. make report	-> The plots are made if not present already and the html as well as
					latex report are made in this section.
					
3. make doc 	-> This generates the doxygen documentation

4. make data 	-> This generates the data in csv file format

5. make plot	-> This makes all the plots from the data which was generated

6. make clean	-> Removes all the compiled files , execuatble , data , plots
					and the documentation
		
7. make distclean	-> This uninstalls the box2d and cleans up everything

8. make dist	-> Clean everything up and make a distributable source copy of your project.
					This contains the box2d source and other helper scripts in tar.gz
					
9. make install 	-> This copies - executable, libraries and documentation  - to appropriate folders
					so that the user can run it. The folder name is kept as a variable in 
					Makefile and can be changed depending on the user.



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
