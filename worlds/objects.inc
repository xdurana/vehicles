define carton model
(
	block
	(
		points 4
		point[0] [1 0]
		point[1] [1 1]
		point[2] [0 1]
		point[3] [0 0]
		z [0 1]
	)

	size [0.1 0.1 0.1]
	color "DarkBlue"
)

define obstacle model
(
	block
	(
		points 4
		point[0] [1 0]
		point[1] [1 1]
		point[2] [0 1]
		point[3] [0 0]
		z [0 1]
	)

	size [0.4 0.4 0.4]
	color "Red"
)

carton
(
	name "carton01"
	pose [-0.5 1 0 0]
)

carton
(
	name "carton02"
	pose [-0.8 1.3 0 0]
)

carton
(
	name "carton03"
	pose [-1.7 -0.8 0 0]
)

carton
(
	name "carton04"
	pose [0.8 1.8 0 0]
)

carton
(
	name "carton05"
	pose [2.8 0.8 0 0]
)

carton
(
	name "carton06"
	pose [2.0 -2.2 0 0]
)

obstacle
(
	name "obstacle01"
	pose [0.5 1 0 0]
)

obstacle
(
	name "obstacle02"
	pose [1.5 0 0 0]
	size [0.3 0.3 0.3]
)