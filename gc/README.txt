Team Overbot CVS tree layout
John Nagle

This is the layout for module "gc".
See also project "admin", which covers administrative details not
related to the project (building, phones, in house network,
membership, etc.)	
		
gc	Grand Challenge robot vehicle project 
        admin                   	Project specific administration
	notes				Technical notes
        parts				Part information for subsystems (drawings, scanned pages, etc.)
		audible
		awd
		brake
		choke
		fuel_cell
		generator
		gps_ins
		e_stop
		ignition
		laser_range_finder
		motion_control
		radar
		sonar
		speedo
		steering
		templates
		throttle
		top_unit
		transmission
		water_detection			
        procs_and_forms         	Project-specific procedures and forms (not tests)
		generator
		vehicle
	src				Software source code
		common			Support code common to more than one target OS
			include		Include files - in search path for all builds
			lib		Portable libraries (source)
		qnx
			common		Support code common to modules below
				include	Include files - include in all builds for this OS
				lib	OS-specific libraries (source)
			drivers		Device drivers and device interface code
			control		Actuator control
			vision		Camera vision processing
			lidar		Laser rangefinding processing
			auxsensors	Other sensors
			nav		Navigation - where are we?
				gpsins	GPS/INS processing								
				sensormap	Local map-building
				waypoints	waypoint processing
			planning		Route planning
			targets
				chassis		Chassis PC/104 computer
				auxsensors	Aux sensors PC/104 computer
				topunit		Vehicle-top sensor pod PC/104 computer
				backseat	Larger rack-mounted machines
				desktop		Desktop-oriented test builds, demos, etc.
			tools			Tools running on QNX
			tests
			demos			
		win				Code intended to run under MS Windows
			common			Support code common to modules below
				include		Include files - include in all builds for this OS
				lib		OS-specific libraries (source)
			tools			General tools
			tests			Windows-based test harnesses
			demos			Demos
		linux				Code intended to run under Linux.
			common			Support code common to modules below
				include		Include files - include in all builds for this OS
				lib		OS-specific libraries (source)
			tools			General tools
			tests			Linux-based test harnesses
			demos			Demos        
	system                  	System documents
        	boms                	Bills of Materials
        	budgets			Budgets
        	milestones          	Milestones, in various forms
        	specs               	System and subsystem descriptions
    	tests				Test-related documents and data
        	fuel_capacity
        	fuel_consumption				

