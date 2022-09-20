
def fish(h,p):
	return linear_extrude(p)(
create_wire(True)(
bezier([[0,0,0],[1.5*h,h,0], [3*h, h,0]],False),
bezier([[0,0,0],[1.5*h,-h,0],[3*h,-h,0]],False),
bezier([[3*h,  1*h,0],[4.5*h, 1*h,0],[5*h,0.4*h,0]],False),
bezier([[3*h,-1*h,0],[4.5*h,-1*h,0],[5*h,-0.4*h,0]],False),
line([5*h,0.4*h,0],  [6*h, 1*h,0]),
line([5*h,-0.4*h,0],[6*h,-1*h,0]),
line([6*h,-1*h,0],   [6*h, 1*h,0]),
),
).move(0,0,p);

def ecaille(j,h):
	return linear_extrude(h)(
create_wire(True)(
line([-j,j,0],  [-j, -j,0]),
line([-j,j,0],  [j, j,0]),
line([-j,-j,0],  [j, -j,0]),
bezier([[j,j,0],[3*j,j,0],[3*j,0,0]],False),
bezier([[j,-j,0],[3*j,-j,0],[3*j,0,0]],False),
),
);

ecailles = union();
for y in range(-3,4,1):
	for x in range(0,14,1):
		if (x%2 ==0):
			ecailles(ecaille(2, (2.99-x*0.2)).move(10+x*4,-2+y*4, 0))
		else:
			ecailles(ecaille(2, (2.99-x*0.2)).move(10+x*4,y*4, 0))

scene().redraw(
cut()(
	union()(
		intersection()(
			fish(10,2.99),
			cube(15,100,100).yz(),
		),
		intersection()(
			ecailles,
			fish(10,100).move(0,0,-50),
		),
	),
	cylinder(1,10).move(8.6,2,-1),
),
)