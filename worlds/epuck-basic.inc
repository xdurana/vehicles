define epuck_ir ranger
(
 scount 8

 spose[0] [ 0.22 0.075 0 ]
 spose[1] [ 0.2 0.2 45 ]
 spose[2] [ 0.0 0.23 90 ]
 spose[3] [ -0.2 0.2 150 ]
 spose[4] [ -0.2 -0.2 -150 ]
 spose[5] [ 0.0 -0.23 -90 ]
 spose[6] [ 0.2 -0.2 -45 ]
 spose[7] [ 0.22 -0.075 0 ]
    
 sview [0 5.0 15]
 ssize [0.02 0.005]
)

define epuck position
(
 size [0.4 0.4]
 origin [0.0 0.0 0.0]
 gui_nose 1
 mass 0.6 

 polygons 1
 polygon[0].points 8
 polygon[0].point[0] [  -0.2  0.2 ]
 polygon[0].point[1] [  0.0  0.28 ]
 polygon[0].point[2] [  0.2  0.2 ]
 polygon[0].point[3] [  0.28  0.0 ]
 polygon[0].point[4] [  0.2 -0.2 ]
 polygon[0].point[5] [  0.0 -0.28 ]
 polygon[0].point[6] [  -0.2 -0.2 ]
 polygon[0].point[7] [  -0.28 0.0 ]

 drive "diff"

 epuck_ir()

 ptz
 (
    blobfinder
    ( 
        channel_count 6
        channels [ "red" "blue" "green" "cyan" "yellow" "magenta" ]
        image [100 100]
        range_max 6.0
        ptz [0 0 80.0]
        size [1.0 1.0]
    )
 )
) 