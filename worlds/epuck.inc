# epuck.inc
# rough model of the epuck robots

#ranger sensors
define epuck_IR ranger
(
	# number of sonars
	scount 8

	# define the pose of each transducer [xpos ypos heading]
	scount 8
  # define the pose of each transducer [xpos ypos heading]
  spose[0] [0.030  -0.010   342.8]
  spose[1] [0.022  -0.025   314.2]	
  spose[2] [ 0.0    -0.031    270]
  spose[3] [ -0.03   -0.015   208.5]
  spose[4] [ -0.03  0.015   151.5]
  spose[5] [ 0.0   0.031   90]
  spose[6] [0.022 0.025   405.8]
  spose[7] [0.03  0.01    377.2]
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 0.1 10]
  
  # define the size of each transducer [xsize ysize] in meters
  ssize [0.002 0.007]
  
	
		#angles got from the placement diagram on epuck website.

		
	# define the field of view of each transducer [range_min range_max view_angle]
	#sview [0.0064 0.1 10]
		# range min and max are from the epuck website prox sensor graphs.

	# define the size of each transducer [xsize ysize] in meters
	#ssize [0.002 0.002]
		#from IR data sheet
)

# epuck blobfinder
define epuck_cam blobfinder
(
		# number of colours to look for
		colors_count 2
	
		# which colours to look for
		colors ["green" "red"]
	
		# camera parameters
		image [640 480]		#resolution
		
		size [0.001 0.001 0.001]
)

define epuck_bumper ranger
(
  scount 8
  # define the pose of each transducer [xpos ypos heading]
  spose[0] [0.030  -0.010   342.8]
  spose[1] [0.022  -0.025   314.2]	
  spose[2] [ 0.0    -0.031    270]
  spose[3] [ -0.03   -0.015   208.5]
  spose[4] [ -0.03  0.015   151.5]
  spose[5] [ 0.0   0.031   90]
  spose[6] [0.022 0.025   405.8]
  spose[7] [0.03  0.01    377.2]
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 0.1 10]
  
  # define the size of each transducer [xsize ysize] in meters
  ssize [0.002 0.007]
  obstacle_return 0
  ranger_return 1
)
# epuck wheels, just draw the wheels, no other purpose
define epuck_wheel model
(
  size [0.03 0.002 0]
  color "black"
  obstacle_return 0
  ranger_return 1
  blob_return 0 
)


define epuck_turret model
(
   size [0.061 0.046 0]
   polygons 1
   polygon[0].points 11
   polygon[0].point[0] [-0.035 0  0]
   polygon[0].point[1] [-0.028 -0.018  0]
   polygon[0].point[2] [0.011 -0.012  0]
   polygon[0].point[3] [0.026 -0.021  0]
   polygon[0].point[4] [0.027 -0.019  0]
   polygon[0].point[5] [0.015 -0.008  0]
   polygon[0].point[6] [0.015 0.008  0]
   polygon[0].point[7] [0.027 0.019  0]
   polygon[0].point[8] [0.026 0.021  0]
   polygon[0].point[9] [0.011 0.012  0]
   polygon[0].point[10] [-0.028 0.018  0]
   polygon[0].fill 0
   #color "LightGreen"
  
)


# epuck base
define epuck position
(
		
	# actual size
  size [0.07 0.07 0]

  # the epuck's center of rotation is offset from its center of area
  origin [0 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 0.1 
  
  # this polygon approximates the shape of a epuck (R=3.5cm circle)
  polygons 1
  polygon[0].points 18
  polygon[0].point[0] [0 0.035]
  polygon[0].point[1] [0.0119707050160703 0.0328892417276262 ]
  polygon[0].point[2] [0.0224975663384939 0.0268115555096132 ]
  polygon[0].point[3] [0.0303108891319316 0.0175000000009072 ]
  polygon[0].point[4] [0.0344682713551847 0.00607768621971813 ]
  polygon[0].point[5] [0.0344682713557305 -0.00607768621662311 ]
  polygon[0].point[6] [0.0303108891335029 -0.0174999999981855 ] 
  polygon[0].point[7] [0.0224975663409014 -0.026811555507593 ]
  polygon[0].point[8] [0.0119707050190235 -0.0328892417265513 ]
  polygon[0].point[9] [0 -0.035 ]
  polygon[0].point[10] [-0.011970705013117 -0.0328892417287011 ]
  polygon[0].point[11] [-0.0224975663360864 -0.0268115555116333 ]
  polygon[0].point[12] [-0.0303108891303602 -0.017500000003629 ]
  polygon[0].point[13] [-0.034468271354639 -0.00607768622281315 ]
  polygon[0].point[14] [-0.0344682713562762 0.0060776862135281 ]
  polygon[0].point[15] [-0.0303108891350743 0.0174999999954638 ]
  polygon[0].point[16] [-0.0224975663433089 0.0268115555055729 ]
  polygon[0].point[17] [-0.0119707050219767 0.0328892417254764 ]

 
 #Defining the shape of an epuck
	bitmap "bitmaps/circle.png"
	gui_outline 1
	show_blocks 0
 
  #epuck_turret()

  # differential steering model
  drive "diff"
  
  # localization "gps"

  # bumper
	
#left wheel
  epuck_wheel(origin [0 0.026 0 0])
  #right wheel
  epuck_wheel(origin [0 -0.026 0 0])
  #epuck_turret()

  # differential steering model
  drive "diff"
  
  # localization "gps"

  # bumper
  epuck_bumper()  
epuck_turret()
  
  #epuck_turret()
	gui_outline 1
	show_blocks 0

	# differential steering model
	drive "diff"

	# sensors attached to epuck
	epuck_IR()  
	epuck_cam()
) 




