/////////////////////////////////////////////////
Welcome to EGP Graphics Framework!
By Dan Buckstein
/////////////////////////////////////////////////

Windows users: 
The supported IDE is VISUAL STUDIO 2015.

First-time use: 

1.	Close all instances of Visual Studio.
	
2. 	Make sure you have DEV-SDKs on your computer 
	and have run the install batch file therein.
	**YOUR FW WILL NOT WORK IF YOU DON'T DO THIS!

3.	The root directory of the framework is called 
	'EGP Graphics Framework' and contains the 
	following folder structure: 
		-fw_helpers
		-include
			-egpfw
				-egpfw
					-utils
		-project
			-VS2015
				-_SLN
					-EGP-Graphics-FW
				-egpfw
				-egpfwDemoFreeglut
		-resource
		-source
			-egpfw
			-egpfwDemoFreeglut
		-thirdparty (etc.)
	
4.	Open the framework: 
	To open the framework, run the batch file 
	called 'LAUNCH_VS2015' in the root folder 
	after closing all instances if Visual Studio.
	
	
Note the following for all uses of the framework: 

To open the framework, follow step 4 above.

If any Intellisense problems occur, close Visual 
Studio and navigate to the solution directory 
(projects/VS2015/_SLN/EGP-Graphics-FW) and delete
the Intellisense database (.db) file. It is also 
recommended that you run the 'uninstall' batch 
file under 'fw_helpers/win' just to be safe.

The solution contains TWO projects: 
	-egpfw: This is your back-end library. You 
		will need to implement functions here to 
		make things work.
	-egpfwDemoFreeglut: This is your front-end 
		application. Use the file 'egpfwDemo.cpp'
		to test your back-end functions.
	
Press F7 to build the solution or F5 to build and
run. The front-end application will start in the 
active configuration.
	
The solution has FOUR configurations: 
	-Debug: Your front-end application will link 
		against your back-end library with debug 
		mode enabled (you can set breakpoints).
	-Debug_complete: Your application will link 
		against a closed-source, fully-functional
		version of the library for testing your 
		front-end without any unexpected errors. 
		This configuration also has debugging.
	-Release: Your application will link against 
		your back-end library in release mode 
		(release is faster but no debugging).
	-Release_complete: Your application will link
		against the completed library in release 
		mode for testing purposes.
		
Additionally, you can check if you are using one 
of the 'complete' configurations by using these 
preprocessor directives: 
	#ifdef _EGP_COMPLETE
	// code if using 'complete' lib
	#else
	// code if *NOT* using complete lib
	#endif
			
**DO NOT CHANGE PROJECT SETTINGS FOR ANY CONFIG!
	

Note the following when preparing submissions: 

To properly prepare a submission,  execute the 
batch file 'PREPARE_SUBMISSION_<IDE>.bat'. This 
will package up the RELEVANT files in your copy 
of the framework. You will see a new folder 
appear in the root, and another one in there, 
named to your liking; zip it up and submit!
**DO NOT ADD FILES TO THE RESULTING PACKAGE!
**DO NOT MODIFY THE BATCH FILE!


Mac users: 
The supported IDE is XCODE.

Directly open the Xcode workspace in the folder 
project/Xcode/_WORKSPACE/EGP-Graphics-FW. All of 
the above features hold true for Xcode, except 
that the batch files will not work.


That's the gist of it... good luck, have fun, and
learn lots!  :)
-Dan

dbuckstein@champlain.edu
